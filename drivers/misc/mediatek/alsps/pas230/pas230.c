/******************************************************************************
 * MODULE       : pas230.c
 * FUNCTION     : Driver source for PAS230,
 *              : Proximity Sensor(PS) and Ambient Light Sensor(ALS) IC.
 * AUTHOR       : Seo Ji Won < jiwon.seo@lge.com >
 * MODIFY       : Haein Jeong < haein.jeong@lge.com >
 * PROGRAMMED   : Sensing solution Group, PARTRON CO.,LTD.
 * MODIFICATION : Modified by PARTRON
 * REMARKS      : File : mediatek\custom\common\kernel\alsps\pas230.c
 * COPYRIGHT    : Copyright (C) 2015 PARTRON CO.,LTD.
 *              : This program is free software; you can redistribute it and/or
 *              : modify it under the terms of the GNU General Public License
 *              : as published by the Free Software Foundation; either version 2
 *              : of the License, or (at your option) any later version.
 *              :
 *              : This program is distributed in the hope that it will be useful,
 *              : but WITHOUT ANY WARRANTY; without even the implied warranty of
 *              : MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *              : GNU General Public License for more details.
 *              :
 *              : You should have received a copy of the GNU General Public License
 *              : along with this program; if not, write to the Free Software
 *              : Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *****************************************************************************/

/****************************************************************************
* Include Files
****************************************************************************/
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>

#define MT6735

#ifdef MT6582
#include <mach/devs.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#endif

#ifdef MT6582
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#ifdef MT6735
#define MT65XX_POWER_NONE -1 //temp define
#define POWER_NONE_MACRO MT65XX_POWER_NONE
#endif

#include <hwmsensor.h>
#include <hwmsen_dev.h>
#include <sensors_io.h>
#include "cust_alsps.h"
#include "alsps.h"
#include "pas230.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <asm/io.h>
#include <linux/irqchip/mt-eic.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <cust_alsps.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define FAKE_CROSS_OFFSET
#define PAS230_ALS_SENSOR_ENABLE
#define PAS230_ALS_SENSOR_INT

#define PAS230_I2CADDR    (0x53)

#define CALB_SYSTEM_WAIT     (101)
#define CALB_IC_WAIT         (5)
#define CALB_TIMES           (2)
#define CALB_BOX_TIMES       (20)
#define CALB_REMOVAL_TIME    (5)

#define SET_IC_DISABLE       (0)
#define SET_IC_ENABLE        (1)

#define PAS230_PINT		(1 << 1)
#ifdef PAS230_ALS_SENSOR_INT
#define ALS_INT_MASK	(0x10)
#define ALS_THRESHOLD_MAX	(524287)
#define ALS_THRESHOLD_MIN	(0)
#define ALS_LUX_THRESHOLD	(4)	//Matching the Pocket Detection thresold
#define LUX_VAR_PERCENT		1
//static atomic_t irq_status = ATOMIC_INIT(-1);
#endif
static atomic_t part_id = ATOMIC_INIT(0); //0= "b4" sample, 1= "b1"

#define PS_TH_VAL_MAX          (0x7FF) /* 10bit resolution */
#define PS_TH_VAL_MIN          (0)
enum {
	MAIN_CTRL=0x00,
	PS_LED,
	PS_PULSES,
	PS_MEAS_RATE,
	ALS_CS_MEAS_RATE,
	ALS_CS_GAIN,
	PART_ID,
	MAIN_STATUS,
	PS_DATA=0x08,
	CLEAR_DATA=0x0a,
	GREEN_DATA=0x0d,
	BLUE_DATA=0x10,
	RED_DATA=0x13,
	COMP_DATA=0x16,
	INT_CFG=0x19,
	INT_PST,
	PS_THRES_UP=0x1b,
	PS_THRES_LOW=0x1d,
	PS_CAN=0x1f,
	ALS_THRES_UP=0x21,
	ALS_THRES_LOW=0x24,
	ALS_THRES_VAR=0x27
};

static u8 reg_defaults[40] = {
#ifdef PAS230_ALS_SENSOR_ENABLE
	0x03, /* 0x00_0 : MAIN_CTRL */
#else
	0x01,
#endif
	0x36, /* 0x01_1 : PS_LED */
	0x08, /* 0x02_2 : PS_PULSES */
	0x56, /* 0x03_3 : PS_MEAS_RATE */
	0x23, /* 0x04_4 : ALS_CS_MEAS_RATE */
	0x01, /* 0x05_5 : ALS_CS_GAIN */
	0xb0, /* 0x06_6 : PART_ID */
	0x00, /* 0x07_7 : MAIN_STATUS */
	0x00, 0x00, /* 0x08_8 : PS_DATA */
	0x00, 0x00, 0x00, /* 0x0a_10 : CLEAR_DATA */
	0x00, 0x00, 0x00, /* 0x0d_13 : GREEN_DATA */
	0x00, 0x00, 0x00, /* 0x10_16 : BLUE_DATA */
	0x00, 0x00, 0x00, /* 0x13_19 : RED_DATA */
	0x00, 0x00, 0x00, /* 0x16_22 : COMP_DATA */	
#ifdef PAS230_ALS_SENSOR_INT
	0x15, /* 0x19_25 : INT_CFG (0x15 = ALS & PS INT ENABLE) */
#else
	0x11, /* 0x19_25 : INT_CFG (0x11 = PS INT ONLY)*/
#endif
	0x11, /* 0x1a_26 : INT_PST */
	0x14, 0x00, /* 0x1b_27 : PS_THRES_UP, 2047_80 */
	0x0a, 0x00, /* 0x1d_29 : PS_THRES_LOW, 0_65 */
	0x00, 0x00, /* 0x1f_31 : PS_CAN, 2047_0 */
	0xff, 0xff, 0x0f, /* 0x21_33 : ALS_THRES_UP */
	0x00, 0x00, 0x00, /* 0x24_36 : ALS_THRES_LOW */
	0x00, /* 0x27_39 : ALS_THRES_VAR */
};

#define PS_ON			(reg_defaults[MAIN_CTRL]&0x01)
#define PS_OFF			(reg_defaults[MAIN_CTRL]&(0x01^0xff))
#define ALS_CS_ON		(reg_defaults[MAIN_CTRL]&0x02)
#define ALS_CS_OFF		(reg_defaults[MAIN_CTRL]&(0x02^0xff))
#define ALL_ON			(reg_defaults[MAIN_CTRL]&0x03)
#define ALL_OFF			(reg_defaults[MAIN_CTRL]&(0x03^0xff))

enum {
#ifdef PAS230_ALS_SENSOR_ENABLE
	LIGHT_ENABLED = BIT(0),
#endif
	PROXIMITY_ENABLED = BIT(1),
};

struct pas230_hw {
	u32 near_offset;
	u32 far_offset;
	u32 crosstalk_max;
	u32 als_lux_coeff;
	u32 als_gain;

	unsigned int ppcount;
	unsigned int ps_led_current;
#ifdef PAS230_ALS_SENSOR_INT
	u32 als_up_thres;
	u32 als_low_thres;
#endif
};

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_enable ( struct i2c_client *client, unsigned char enable );
static int pas230_get_alsdata ( struct i2c_client *client, unsigned short *aData );
static long pas230_als_enable ( struct i2c_client *client  );
static long pas230_als_disable ( struct i2c_client *client  );
static long pas230_als_activate ( struct i2c_client *client, int enable );
#endif

static int pas230_set_ps_enable ( struct i2c_client *client, unsigned char enable );
static int pas230_set_ps_led ( struct i2c_client *client, unsigned char ps_led );
static int pas230_get_ps_led ( struct i2c_client *client, unsigned char *ps_led );
static int pas230_set_ps_pulse ( struct i2c_client *client, unsigned char ps_pulse );
static int pas230_get_ps_pulse ( struct i2c_client *client, unsigned char *ps_pulse );
static int pas230_set_ps_meas_rate ( struct i2c_client *client, unsigned char ps_meas_rate );
static int pas230_get_ps_meas_rate ( struct i2c_client *client, unsigned char *ps_meas_rate );
static int pas230_set_ps_int_cfg ( struct i2c_client *client, unsigned char ps_int_cfg );
static int pas230_get_ps_int_cfg ( struct i2c_client *client, unsigned char *ps_int_cfg );
static int pas230_set_ps_int_pst ( struct i2c_client *client, unsigned char int_pst );
static int pas230_get_ps_int_pst ( struct i2c_client *client, unsigned char *int_pst );
static int pas230_set_ps_can_0 ( struct i2c_client *client, unsigned char ps_can );
static int pas230_get_ps_can_0 ( struct i2c_client *client, unsigned char *ps_can );
static int pas230_set_ps_can_1 ( struct i2c_client *client, unsigned char ps_can );
static int pas230_get_ps_can_1 ( struct i2c_client *client, unsigned char *ps_can );
static int pas230_set_pilt ( struct i2c_client *client, unsigned short threshold );
static int pas230_set_piht ( struct i2c_client *client, unsigned short threshold );
#ifdef PAS230_ALS_SENSOR_INT
static int pas230_set_ailt ( struct i2c_client *client, u32 threshold );
static int pas230_set_aiht ( struct i2c_client *client, u32 threshold );
#endif
static int pas230_get_status ( struct i2c_client *client, unsigned char *pData );
static int pas230_get_pdata ( struct i2c_client *client, unsigned short *pData );
static int pas230_get_deivceid( struct i2c_client *client, unsigned char *pData );
static int pas230_clear_interrupt ( struct i2c_client *client );
#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_cs_meas_rate ( struct i2c_client *client, unsigned char als_cs_meas_rate );
static int pas230_set_als_cs_gain ( struct i2c_client *client, unsigned char als_cs_gain );
#endif
static int pas230_decide_ps_state ( struct i2c_client *client, int pdata );
static long pas230_initialize ( struct i2c_client *client  );
static long pas230_ps_enable ( struct i2c_client *client  );
static long pas230_ps_disable ( struct i2c_client *client  );
static void pas230_swap(int *x, int *y);
static int pas230_do_calibration ( struct i2c_client *client, int *value );
static unsigned int pas230_calc_calibration ( struct i2c_client *client );

static long pas230_ps_activate ( struct i2c_client *client, int enable );
static void pas230_eint_func ( void );
static void pas230_eint_work ( struct work_struct *work );
static irqreturn_t pas230_eint_handler(int irq, void *desc);

static ssize_t pas230_show_cali_value ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_cali_value ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_led ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_led ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_pulse ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_pulse ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_ps_meas_rate ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_ps_meas_rate ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_pilt ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_pilt ( struct device_driver *dev, char *buf, size_t count );
static ssize_t pas230_show_piht ( struct device_driver *dev, char *buf );
static ssize_t pas230_store_piht ( struct device_driver *dev, char *buf, size_t count );

static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
/****************************************************************************
* Manifest Constants / Defines
****************************************************************************/
#define PAS230_DEV_NAME     "PAS230"
static DEFINE_MUTEX(pas230_access);

/****************************************************************************
 * Macros
 ****************************************************************************/
#ifdef PAS230_ALS_SENSOR_ENABLE
#define SENSOR_TAG                  "[LGE_PS/ALS]"
#else
#define SENSOR_TAG                  "[LGE_Proximity]"
#endif
#ifdef CONFIG_MT_ENG_BUILD
#define DEBUG 1
#endif

#ifdef DEBUG
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[D]""%s : "fmt, __FUNCTION__,##args)
#else
#define SENSOR_FUN(f)               printk(KERN_NOTICE SENSOR_TAG"[F]""%s\n", __FUNCTION__)
#define SENSOR_ERR(fmt, args...)    printk(KERN_ERR SENSOR_TAG"[E]""%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define SENSOR_LOG(fmt, args...)    printk(KERN_NOTICE SENSOR_TAG"[L]""%s : "fmt, __FUNCTION__,##args)
#define SENSOR_DBG(fmt, args...)    NULL
#endif

/****************************************************************************
* Type Definitions
****************************************************************************/
typedef enum
{
	PS_NEAR = 0,
	PS_FAR = 1,
	PS_UNKNOWN = 2
} PS_STATUS;

/* Maintain alsps cust info here */
struct alsps_hw alsps_cust;
struct pas230_hw local_alsps_cust;
static struct alsps_hw *hw = &alsps_cust;
static struct pas230_hw *proxi_hw = &local_alsps_cust;
struct platform_device *alsps_platf_Dev;

/* For alsp driver get cust info */
struct alsps_hw *get_cust_alsps(void)
{
	return &alsps_cust;
}
struct pas230_hw *get_cust_local_alsps(void)
{
	return &local_alsps_cust;
}

struct pas230_priv
{
	struct alsps_hw  *hw;
	struct pas230_hw *local_hw;
	struct i2c_client *client;
	struct work_struct eint_work;

	unsigned int activate; /* 1 = activate, 0 = deactivate */

	/* variables to store register value - begin */
	unsigned int enable;
	unsigned int pilt;
	unsigned int piht;
    unsigned int interrupt;

    unsigned int ps_led;
    unsigned int ps_pulse;
	unsigned int ps_meas_rate;
	unsigned int ps_int_cfg;
	unsigned int ps_int_pst;
	unsigned int ps_can_0;
	unsigned int ps_can_1;
	unsigned int als_cs_meas_rate;
	unsigned int als_cs_gain;
	/* variables to store register value - end */

	u16 ps;

	struct device_node *irq_node;
	int		irq;

	/* control flag from HAL */
	unsigned int enable_ps_sensor;
	unsigned int enable_als_sensor;

	unsigned int ps_status; /* current status of poximity detection : 0 = near, 1 = far */
	unsigned int ps_th_status; /* current threshold status of poximity detection : 0 = near, 1 = far */

	/* threshold value to detect "near-to-far" event */
	unsigned int ps_th_near_low;
	unsigned int ps_th_near_high;

	/* threshold value to detect "far-to-near" event */
	unsigned int ps_th_far_low;
	unsigned int ps_th_far_high;

	unsigned int ps_cross_talk; /* a result value of calibration. it will be used to compensate threshold value. */

	/* ALS parameters */
	int als_data;
	unsigned int als_poll_delay;	/* needed for light sensor polling : micro-second (us) */

	#if defined(CONFIG_FB)
	struct notifier_block	fb_notif;
	#endif
};
static atomic_t driver_suspend_flag = ATOMIC_INIT(0);
static atomic_t first_int_flag = ATOMIC_INIT(0);
static unsigned int PS_DEFAULT_CAL = 140;/*k430 default cal*/
/****************************************************************************
* Variables
****************************************************************************/
static struct i2c_client *pas230_i2c_client = NULL; /* for general file I/O service. will be init on pas230_i2c_probe() */
static struct pas230_priv *g_pas230_ptr = NULL; /* for interrupt service call. will be init on pas230_i2c_probe() */

/****************************************************************************
* Extern Function Prototypes
****************************************************************************/
extern void mt_eint_unmask ( unsigned int line );
extern void mt_eint_mask ( unsigned int line );
extern void mt_eint_set_polarity ( unsigned int eint_num, unsigned int pol );
extern void mt_eint_set_hw_debounce ( unsigned int eint_num, unsigned int ms );
extern unsigned int mt_eint_set_sens ( unsigned int eint_num, unsigned int sens );
#if !defined(CONFIG_OF)
void mt_eint_registration(unsigned int eint_num, unsigned int flag,
              void (EINT_FUNC_PTR) (void), unsigned int is_auto_umask);
#endif
/****************************************************************************
* Local Function Prototypes
****************************************************************************/
static int pas230_local_init(void);
static int pas230_remove(void);
static int pas230_init_flag = PAS230_INIT_FAIL;
static struct sensor_init_info pas230_init_info = {
	.name = PAS230_DEV_NAME,
	.init = pas230_local_init,
	.uninit = pas230_remove,
};

#if !defined(CONFIG_OF)
#define GPIO_PROXIMITY_INT         GPIO_ALS_EINT_PIN
#define GPIO_PROXIMITY_INT_M_GPIO   GPIO_ALS_EINT_PIN_M_EINT
#define GPIO_PROXIMITY_INT_M_EINT   GPIO_PROXIMITY_INT_M_GPIO

#define CUST_EINT_PROXIMITY_NUM              CUST_EINT_ALS_NUM
#define CUST_EINT_PROXIMITY_DEBOUNCE_CN      0
#define CUST_EINT_PROXIMITY_TYPE			CUST_EINTF_TRIGGER_FALLING
#define CUST_EINT_PROXIMITY_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE
#endif

/****************************************************************************
* Local Functions
****************************************************************************/
struct pas230_hw *get_pas230_dts_func(const char *name, struct pas230_hw *hw)
{
	int i, ret;
	u32 near_offset[] = {0};
	u32 far_offset[] = {0};
	u32 crosstalk_max[] = {0};
	u32 ppcount[] = {0};
	u32 ps_led_current[] = {0};
#ifdef PAS230_ALS_SENSOR_INT
	u32 als_up_thres[] = {0};
	u32 als_low_thres[] = {0};
#endif
	u32 als_lux_coeff[] = {0};
	u32 als_gain[] = {0};

	struct device_node *node = NULL;

	//SENSOR_LOG("Device Tree get alsps info!\n");
	if (name == NULL)
		return NULL;

	node = of_find_compatible_node(NULL, NULL, name);
	if (node) {
		ret = of_property_read_u32_array(node , "near_offset", near_offset, ARRAY_SIZE(near_offset));
		if (ret == 0)
			hw->near_offset =	near_offset[0];

		ret = of_property_read_u32_array(node , "far_offset", far_offset, ARRAY_SIZE(far_offset));
		if (ret == 0)
			hw->far_offset =	far_offset[0];
		
		ret = of_property_read_u32_array(node , "crosstalk_max", crosstalk_max, ARRAY_SIZE(crosstalk_max));
		if (ret == 0)
			hw->crosstalk_max = crosstalk_max[0];
		
		ret = of_property_read_u32_array(node , "ppcount", ppcount, ARRAY_SIZE(ppcount));
		if (ret == 0)
			hw->ppcount =	ppcount[0];
		
		ret = of_property_read_u32_array(node , "ps_led_current", ps_led_current, ARRAY_SIZE(ps_led_current));
		if (ret == 0)
			hw->ps_led_current =	ps_led_current[0];

#ifdef PAS230_ALS_SENSOR_INT
		ret = of_property_read_u32_array(node , "als_up_thres", als_up_thres, ARRAY_SIZE(als_up_thres));
		if (ret == 0)
			hw->als_up_thres =	als_up_thres[0];
		
		ret = of_property_read_u32_array(node , "als_low_thres", als_low_thres, ARRAY_SIZE(als_low_thres));
		if (ret == 0)
			hw->als_low_thres = als_low_thres[0];
		reg_defaults[ALS_THRES_UP] = (u8)hw->als_up_thres;
		reg_defaults[ALS_THRES_UP+1] = (u8)(hw->als_up_thres>>8);
		reg_defaults[ALS_THRES_UP+2] = (u8)((hw->als_up_thres>>16) & 0x07);
		reg_defaults[ALS_THRES_LOW] = (u8)hw->als_low_thres;
		reg_defaults[ALS_THRES_LOW+1] = (u8)(hw->als_low_thres>>8);
		reg_defaults[ALS_THRES_LOW+2] = (u8)((hw->als_low_thres>>16) & 0x07);
#endif
		ret = of_property_read_u32_array(node , "als_lux_coeff", als_lux_coeff, ARRAY_SIZE(als_lux_coeff));
		if (ret == 0)
			hw->als_lux_coeff = als_lux_coeff[0];
		ret = of_property_read_u32_array(node , "als_gain", als_gain, ARRAY_SIZE(als_gain));
		if (ret == 0)
			hw->als_gain = als_gain[0];
		
	} else {
		SENSOR_ERR("Device Tree: can not find alsps node!. Go to use old cust info\n");
		return NULL;
	}
	return hw;
}

//==========================================================
// Platform(AP) dependent functions
//==========================================================
static int pas230_setup_eint ( void )
{
	SENSOR_FUN ();
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	
	g_pas230_ptr->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, als-eint");
	if (g_pas230_ptr->irq_node) {
		of_property_read_u32_array(g_pas230_ptr->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		SENSOR_LOG("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);

		g_pas230_ptr->irq = irq_of_parse_and_map(g_pas230_ptr->irq_node, 0);
		if (g_pas230_ptr->irq < 0) {
			SENSOR_ERR("irq_of_parse_and_map IRQ LINE NOT AVAILABLE!.");
			return -1;
		}

		ret = request_irq(g_pas230_ptr->irq, pas230_eint_handler, IRQF_TRIGGER_FALLING,"ALS-eint", NULL);
		if (ret > 0) {
			SENSOR_ERR("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
	} else {
		SENSOR_ERR("null irq node!!\n");
		return -EINVAL;
	}
	return ret;
}


//==========================================================
// PAS230 Register Read / Write Funtions
//==========================================================
static int pas230_write_byte ( struct i2c_client *client, u8 reg, u8 val )
{
	int res = 0;
	u8 pBuf[2] = { 0 };

    mutex_lock(&pas230_access);

	pBuf[0] = reg;
	pBuf[1] = val;

	res = i2c_master_send ( client, pBuf, 2 );
	if ( res == 2 )
	{
		//SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%02x )\n", reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x, val=0x%02x )\n", res, reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_ERR_I2C;
	}

}


static int pas230_write_word ( struct i2c_client *client, u8 reg, u16 val )
{
	int res = 0;
	u8 pBuf[3] = { 0 };

    mutex_lock(&pas230_access);

	pBuf[0] = reg ;
	pBuf[1] = val & 0xFF ;
	pBuf[2] = ( val >> 8 ) & 0xFF ;

	res = i2c_master_send ( client, pBuf, 3 );
	if ( res == 3 )
	{
		//SENSOR_DBG ( "I2C write ( reg=0x%02x, val=0x%04x )\n", reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_SUCCESS;
	}
	else
	{
		SENSOR_ERR ( "failed to write to PAS230 ( err=%d, reg=0x%02x, val=0x%04x )\n", res, reg, val );
        mutex_unlock(&pas230_access);
		return PAS230_ERR_I2C;
	}

}


static int pas230_read_byte ( struct i2c_client *client, u8 reg, u8 *pVal )
{
	int res = 0;

    mutex_lock(&pas230_access);

	if ( pVal == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "pas230_read_byte error i2c_master_send (1)....\n" );
		goto EXIT_ERR;
	}

	res = i2c_master_recv ( client, pVal, 1 );
	if ( res != 1 )
	{
		SENSOR_ERR ( "pas230_read_byte error i2c_master_recv (2)....\n" );
		goto EXIT_ERR;
	}

	//SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%02x )\n", reg, *pVal );
    mutex_unlock(&pas230_access);
	return PAS230_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&pas230_access);
	return PAS230_ERR_I2C;
}

static int pas230_read_word ( struct i2c_client *client, u8 reg, u16 *Val, u8 cnt )
{
	int res = 0;
	u8 buf[3]={0, };

    mutex_lock(&pas230_access);

	if ( Val == NULL )
	{
		SENSOR_ERR ( "invalid input ( pVal=NULL )" );
		goto EXIT_ERR;
	}

	res = i2c_master_send ( client, &reg, 1 );
	if ( res != 1 )
	{
		goto EXIT_ERR;
	}

	res = i2c_master_recv(client, buf, cnt);
	if(res != cnt)
	{
		goto EXIT_ERR;
	}
	
	*Val = ( ((u16)buf[2] << 16) | ((u16)buf[1] << 8) | buf[0]);
	
	//SENSOR_DBG ( "I2C read ( reg=0x%02x, val=0x%04x )\n", reg, *Val );

    mutex_unlock(&pas230_access);
	return PAS230_SUCCESS;

	EXIT_ERR:
	SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, reg );
    mutex_unlock(&pas230_access);
	return PAS230_ERR_I2C;
}

//==========================================================
// PAS230 Basic Read / Write Funtions
//==========================================================
static int pas230_set_ps_enable ( struct i2c_client *client, unsigned char enable )
{
    struct pas230_priv *obj = i2c_get_clientdata ( client );
    int res;
    u8 tmp;

    //SENSOR_FUN();
   	res = pas230_read_byte ( client, MAIN_CTRL, &tmp );
    if ( res != PAS230_SUCCESS )
  		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
	else {
		SENSOR_DBG("before setting : 0x%02x\n",tmp);
		if(enable == 1) {
			res = pas230_write_byte(client, MAIN_CTRL, tmp | PS_ON);
			SENSOR_LOG("MAIN_CTRL=0x%02x", tmp | PS_ON);
		} else {
			res = pas230_write_byte(client, MAIN_CTRL, tmp & PS_OFF);
			SENSOR_LOG("MAIN_CTRL=0x%02x", tmp & PS_OFF);
			/* to reduce the delay in first interrupt -reduce meas rate */
			pas230_set_ps_meas_rate ( client, (reg_defaults[PS_MEAS_RATE] & 0xf8) | ((u8)0x01));
			atomic_set(&first_int_flag, 0);
			/* to reduce the delay in first interrupt */
		}
		
		if ( res == PAS230_SUCCESS ) {
		   obj->enable_ps_sensor = (unsigned int)enable;
		} else
		{
		   SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
		}
		
	}
    return res;
}
#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_enable ( struct i2c_client *client, unsigned char enable )
{
    struct pas230_priv *obj = i2c_get_clientdata ( client );
    int res;
    u8 tmp;

    //SENSOR_FUN();
   	res = pas230_read_byte ( client, MAIN_CTRL, &tmp );
    if ( res != PAS230_SUCCESS )
  		SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
	else {
		SENSOR_DBG("before setting : 0x%02x\n",tmp);
		if(enable == 1) {
			res = pas230_write_byte(client, MAIN_CTRL, tmp | ALS_CS_ON);
			SENSOR_LOG("MAIN_CTRL=0x%02x\n", tmp | ALS_CS_ON);
		} else {
			res = pas230_write_byte(client, MAIN_CTRL, tmp & ALS_CS_OFF);
			SENSOR_LOG("MAIN_CTRL=0x%02x\n", tmp & ALS_CS_OFF);
		}
		
		if ( res == PAS230_SUCCESS ) {
		   obj->enable_als_sensor = (unsigned int)enable;
		} else
		{
		   SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, MAIN_CTRL );
		}
		
	}
    return res;
}
#endif

static int pas230_set_ps_led ( struct i2c_client *client, unsigned char ps_led )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
    
   	res = pas230_write_byte ( client, PS_LED, ( u8 )ps_led );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_led = (unsigned int)ps_led;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_LED );
    	}

	return res;
}

static int pas230_get_ps_led ( struct i2c_client *client, unsigned char *ps_led )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, PS_LED, ( u8 *)ps_led );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_LED );
    	
	return res;
}


static int pas230_set_ps_pulse ( struct i2c_client *client, unsigned char ps_pulse )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_write_byte ( client, PS_PULSES, ( u8 )ps_pulse );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_pulse = (unsigned int)ps_pulse;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_PULSES );
    	}

	return res;
}

static int pas230_get_ps_pulse ( struct i2c_client *client, unsigned char *ps_pulse )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, PS_PULSES, ( u8 *)ps_pulse );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_PULSES );
    	
	return res;
}


static int pas230_set_ps_meas_rate ( struct i2c_client *client, unsigned char ps_meas_rate )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
	res = pas230_write_byte ( client, PS_MEAS_RATE, ( u8 )ps_meas_rate );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_meas_rate = (unsigned int)ps_meas_rate;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_MEAS_RATE );
    	}

	return res;
}

static int pas230_get_ps_meas_rate ( struct i2c_client *client, unsigned char *ps_meas_rate )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
	res = pas230_read_byte ( client, PS_MEAS_RATE, ( u8 *)ps_meas_rate );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_MEAS_RATE );
    	
	return res;
}

static int pas230_set_ps_int_cfg ( struct i2c_client *client, unsigned char ps_int_cfg )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_write_byte ( client, INT_CFG, ( u8 )ps_int_cfg );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_int_cfg = (unsigned int)ps_int_cfg;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_CFG );
    	}

	return res;
}

static int pas230_get_ps_int_cfg ( struct i2c_client *client, unsigned char *ps_int_cfg )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, INT_CFG, ( u8 *)ps_int_cfg );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_CFG );
    	
	return res;
}

static int pas230_set_ps_int_pst ( struct i2c_client *client, unsigned char int_pst )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_write_byte ( client, INT_PST, ( u8 )int_pst );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_int_pst = (unsigned int)int_pst;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_PST );
    	}

	return res;
}

static int pas230_get_ps_int_pst ( struct i2c_client *client, unsigned char *int_pst )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, INT_PST, ( u8 *)int_pst );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, INT_PST );
    	
	return res;
}

static int pas230_set_ps_can_0 ( struct i2c_client *client, unsigned char ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_write_byte ( client, PS_CAN, ( u8 )ps_can );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_can_0 = (unsigned int)ps_can;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN );
    	}

	return res;
}

static int pas230_get_ps_can_0 ( struct i2c_client *client, unsigned char *ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, PS_CAN, ( u8 *)ps_can );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN );
    	
	return res;
}

static int pas230_set_ps_can_1 ( struct i2c_client *client, unsigned char ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_write_byte ( client, PS_CAN+1, ( u8 )ps_can );
    	if ( res == PAS230_SUCCESS )
    	{
    	    obj->ps_can_1 = (unsigned int)ps_can;
    	}
    	else{
  	      SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN+1 );
    	}

	return res;
}

static int pas230_get_ps_can_1 ( struct i2c_client *client, unsigned char *ps_can )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    int           res = 0;
    
   	res = pas230_read_byte ( client, PS_CAN+1, ( u8 *)ps_can );
    	if ( res != PAS230_SUCCESS )
  	      SENSOR_ERR ( "failed to read from PAS230 ( err=%d, reg=0x%02x )\n", res, PS_CAN+1 );
    	
	return res;
}


static int pas230_set_pilt ( struct i2c_client *client, unsigned short threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	int LSB,MSB;

    //SENSOR_FUN();

    LSB = threshold & 0x00FF ;
    MSB = (threshold & 0x0F00) >> 8;

	res = pas230_write_byte ( client, PS_THRES_LOW, ( u16 )LSB );
	res = pas230_write_byte ( client, PS_THRES_LOW+1, ( u16 )MSB );

	if ( res == PAS230_SUCCESS )
	{
		obj->pilt = (unsigned int)threshold;
	}

	return res;
}

static int pas230_set_piht ( struct i2c_client *client, unsigned short threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	int LSB,MSB;

    //SENSOR_FUN();

    LSB = threshold & 0x00FF ;
    MSB = (threshold & 0x0F00) >> 8;

	res = pas230_write_byte ( client, PS_THRES_UP, ( u16 )LSB );
	res = pas230_write_byte ( client, PS_THRES_UP+1, ( u16 )MSB );
	
	if ( res == PAS230_SUCCESS )
	{
		obj->piht = (unsigned int)threshold;
	}

	return res;
}
#ifdef PAS230_ALS_SENSOR_INT
static int pas230_set_aiht ( struct i2c_client *client, u32 threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int ret = 0;
	reg_defaults[ALS_THRES_UP] = (u8)threshold;
	reg_defaults[ALS_THRES_UP+1] = (u8)(threshold>>8);
	reg_defaults[ALS_THRES_UP+2] = (u8)((threshold>>16) & 0x07);

	ret = pas230_write_byte(client, ALS_THRES_UP, reg_defaults[ALS_THRES_UP]); 
	ret = pas230_write_byte(client, ALS_THRES_UP+1, reg_defaults[ALS_THRES_UP+1]); 
	ret = pas230_write_byte(client, ALS_THRES_UP+2, reg_defaults[ALS_THRES_UP+2]); 
	if ( ret != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", ret, ALS_THRES_UP );
		return ret;
	}
	return 0;
}

static int pas230_set_ailt ( struct i2c_client *client, u32 threshold )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int ret = 0;
	u8 als_thres_low[3] = {0, };
	reg_defaults[ALS_THRES_LOW] = (u8)threshold;
	reg_defaults[ALS_THRES_LOW+1] = (u8)(threshold>>8);
	reg_defaults[ALS_THRES_LOW+2] = (u8)((threshold>>16) & 0x07);

	ret = pas230_write_byte(client, ALS_THRES_LOW, reg_defaults[ALS_THRES_LOW]); 
	ret = pas230_write_byte(client, ALS_THRES_LOW+1, reg_defaults[ALS_THRES_LOW+1]); 
	ret = pas230_write_byte(client, ALS_THRES_LOW+2, reg_defaults[ALS_THRES_LOW+2]); 
	if ( ret != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", ret, ALS_THRES_LOW );
		return ret;
	}
	return 0;
}
#endif

static int pas230_get_status ( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = pas230_read_byte ( client, MAIN_STATUS, ( u8 * ) pData );
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_LOG ( "MAIN_STATUS=0x%02x\n", *pData );
	}

	return res;
}

static int pas230_get_pdata ( struct i2c_client *client, unsigned short *pData )
{
	int res = 0;

	res = pas230_read_word ( client, PS_DATA, ( u16 * ) pData, 2);
	*pData = *pData&0x3FF;
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_DBG ( "PDATA=0x%04x\n", *pData );
	}

	return res;
}

static int pas230_get_deivceid( struct i2c_client *client, unsigned char *pData )
{
	int res = 0;

	res = pas230_read_byte ( client, PART_ID, ( u8 * )pData );
	if ( res == PAS230_SUCCESS )
	{
		SENSOR_DBG ( "DEVICEID=0x%02x\n", *pData );
	}

	return res;
}

static int pas230_clear_interrupt ( struct i2c_client *client )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned char first_data, second_data;
	int           res = 0;

	res = pas230_get_status ( client, &first_data );
	if ( res == PAS230_SUCCESS ) {
        res = pas230_get_status ( client, &second_data );
        if ( res == PAS230_SUCCESS ) {
            if((second_data & PAS230_PINT) == 0) {
                SENSOR_DBG ( "PAS230 interrupt was cleared\n" );
            } else {
                res = PAS230_FAIL;
                SENSOR_DBG ( "PAS230 interrupt cleare failed\n" );
            }
        }
	}

	return res;
}
#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_cs_meas_rate ( struct i2c_client *client, unsigned char als_cs_meas_rate )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned char data;
    int           res = 0;
    
	res = pas230_write_byte ( client, ALS_CS_MEAS_RATE, ( u8 )als_cs_meas_rate );
    if ( res == PAS230_SUCCESS )
    {
        obj->als_cs_meas_rate = (unsigned int)als_cs_meas_rate;
    }
    else{
  	     SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_MEAS_RATE );
    }
	return res;
}
static int pas230_set_als_cs_gain ( struct i2c_client *client, unsigned char als_cs_gain )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned char data;
    int           res = 0;
    
	res = pas230_write_byte ( client, ALS_CS_GAIN, ( u8 )als_cs_gain );
    if ( res == PAS230_SUCCESS )
    {
        obj->als_cs_gain = (unsigned int)als_cs_gain;
    }
    else{
  	     SENSOR_ERR ( "failed to write from PAS230 ( err=%d, reg=0x%02x )\n", res, ALS_CS_GAIN );
    }
	return res;
}
#endif
//==========================================================
// pas230 Data Processign Funtions
//==========================================================
static int pas230_decide_ps_state ( struct i2c_client *client, int pdata )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int ps_status = obj->ps_status;
    unsigned short low_threshould;
    unsigned short high_threshould;

	if ( obj->ps_status == PS_FAR )
	{
	    if ( pdata >= obj->ps_th_far_high ) 
		{
			ps_status = PS_NEAR;
			SENSOR_LOG ( "PS = NEAR\n" );
			low_threshould	= obj->ps_th_near_low;
			high_threshould = (PS_TH_VAL_MAX&0x3ff); // MAX(10bit)
		}
		else
		{
			ps_status = PS_FAR;
			SENSOR_LOG ( "PS = FAR_2\n" );
			low_threshould	= PS_TH_VAL_MIN; // MIN(0)
			high_threshould = obj->ps_th_far_high;
		}
	}
	else
	{
		if ( pdata <= obj->ps_th_near_low )
		{
			ps_status = PS_FAR;
			SENSOR_LOG ( "PS = FAR\n" );
			low_threshould	= PS_TH_VAL_MIN; // MIN(0)
			high_threshould = obj->ps_th_far_high;
		}
		else
		{
			ps_status = PS_NEAR;
			SENSOR_LOG ( "PS = NEAR_2\n" );
			low_threshould	= obj->ps_th_near_low;
			high_threshould = (PS_TH_VAL_MAX&0x3ff); // MAX(10bit)
		}
	}

	/* to reduce the delay in first interrupt -reset original meas rate(200ms) */
	if (atomic_read(&first_int_flag) == 0) {
		pas230_set_ps_meas_rate ( client, (reg_defaults[PS_MEAS_RATE] & 0xf8) | ((u8)0x06));
		atomic_set(&first_int_flag, 1);
	}
	/* to reduce the delay in first interrupt */

	pas230_set_pilt ( client, low_threshould );
	pas230_set_piht ( client, high_threshould );
	//SENSOR_DBG ( "low_threshould=%d\n",low_threshould );
	//SENSOR_DBG ( "high_threshould=%d\n",high_threshould );

	return ps_status;
}


static long pas230_initialize ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int res = 0;
	unsigned char id = 0;
	u8 tmp;
	int pdata  = 0;

    res = pas230_get_deivceid(client, &id);
	if ( res != PAS230_SUCCESS )
	{
		SENSOR_ERR ( "failed to read Device ID and it means I2C error happened\n");
		return res;
	}
	
	if ( id == 0xb4) {
		atomic_set(&part_id, 0);
		SENSOR_LOG("PAS230 PART_ID = 0xb4");
	} else if ( id == 0xb1) {
		atomic_set(&part_id, 1);
		SENSOR_LOG("PAS230 PART_ID = 0xb1");
	}

	obj->enable_ps_sensor = 0;
	/* disable proximity */
	pas230_set_ps_enable(client,SET_IC_DISABLE);
#ifdef PAS230_ALS_SENSOR_ENABLE
	/* disable ALS */
	obj->enable_als_sensor = 0;
	pas230_set_als_enable(client, SET_IC_DISABLE);
#endif

	/* initialize registers of proximity */
	reg_defaults[PS_LED] = (reg_defaults[PS_LED] & 0xf8) | (u8)(obj->local_hw->ps_led_current);
	pas230_set_ps_led ( client, reg_defaults[PS_LED]);
	pas230_set_ps_pulse ( client, obj->local_hw->ppcount);//reg_defaults[PS_PULSES] );
	pas230_set_ps_meas_rate ( client, reg_defaults[PS_MEAS_RATE]);
#ifdef PAS230_ALS_SENSOR_ENABLE
	pas230_set_als_cs_meas_rate ( client, reg_defaults[ALS_CS_MEAS_RATE]);
	if(obj->local_hw->als_gain >= 0) {
		pas230_set_als_cs_gain ( client, obj->local_hw->als_gain);
		SENSOR_LOG("als_gain = %d", obj->local_hw->als_gain);
	} else {
		pas230_set_als_cs_gain ( client, reg_defaults[ALS_CS_GAIN]);
	}
#endif
	
	pas230_set_ps_int_cfg ( client, reg_defaults[INT_CFG]);
	pas230_set_ps_int_pst ( client, reg_defaults[INT_PST]);
	pas230_set_ps_can_0 ( client, reg_defaults[PS_CAN] );
	pas230_set_ps_can_1 ( client, reg_defaults[PS_CAN+1] );

	/* crosstalk value shall be set by LGP Server using I/O so init here to 150 */
	/* initialize threshold value of PS */
	if ( obj->ps_cross_talk > obj->local_hw->crosstalk_max || obj->ps_cross_talk == 0 )
	{
		obj->ps_cross_talk = PS_DEFAULT_CAL;//k430 default cal value
	}
	if ( obj->ps_cross_talk < 0 )
	{
		obj->ps_cross_talk = 0;
	}
	#ifdef FAKE_CROSS_OFFSET
	obj->ps_cross_talk += 50;
	#endif
	obj->ps_th_far_low = 0;
	obj->ps_th_far_high = obj->local_hw->near_offset + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - obj->local_hw->far_offset;
	obj->ps_th_near_high = (PS_TH_VAL_MAX&0x3ff);

	SENSOR_LOG("far high : %d/ near low : %d\n",obj->ps_th_far_high,obj->ps_th_near_low);
#ifdef PAS230_ALS_SENSOR_ENABLE	
	res = pas230_write_byte(client, ALS_THRES_UP, reg_defaults[ALS_THRES_UP]); 
	res = pas230_write_byte(client, ALS_THRES_UP+1, reg_defaults[ALS_THRES_UP+1]); 
	res = pas230_write_byte(client, ALS_THRES_UP+2, reg_defaults[ALS_THRES_UP+2]); 
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_UP );
		return res;
	}
	res = pas230_write_byte(client, ALS_THRES_LOW, reg_defaults[ALS_THRES_LOW]); 
	res = pas230_write_byte(client, ALS_THRES_LOW+1, reg_defaults[ALS_THRES_LOW+1]); 
	res = pas230_write_byte(client, ALS_THRES_LOW+2, reg_defaults[ALS_THRES_LOW+2]); 
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_LOW );
		return res;
	}
	res = pas230_write_byte(client, ALS_THRES_VAR, reg_defaults[ALS_THRES_VAR]); 
	if ( res != PAS230_SUCCESS ) {
		SENSOR_ERR ( "failed to write ( err=%d, reg=0x%02x )\n", res, ALS_THRES_VAR );
		return res;
	}
#endif
	
	res = pas230_setup_eint();
	if (res != 0) {
		SENSOR_ERR("setup eint: %d\n", res);
		return res;
	}

	return res;

}

static long pas230_ps_enable ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	struct hwm_sensor_data sensor_data;
	int res    = 0;
	int status = 0;
    unsigned short low_threshould;
    unsigned short high_threshould;
    unsigned char old_power_state;
	int pdata  = 0;
    old_power_state = (unsigned char)obj->enable_ps_sensor;

	//SENSOR_FUN();
    
	/* enable PAS230 */
	res = pas230_set_ps_enable ( client, SET_IC_ENABLE );
    if(res != PAS230_SUCCESS) {
	    goto enable_error;
    }

#if 1
	obj->ps_th_status = PS_FAR;
	obj->ps_status = PS_FAR;

	low_threshould	= (PS_TH_VAL_MAX&0x3ff); // MAX(10bit)
	high_threshould = PS_TH_VAL_MIN; // MIN(0)
	pas230_set_pilt ( client, low_threshould );
	pas230_set_piht ( client, high_threshould );
#else
	/* read sensor data */
	res = pas230_get_pdata ( client, (unsigned short*)&pdata );
	if(res != PAS230_SUCCESS) {
		goto enable_error;
	}

	/* decide current PS threshold state and set PS thershold to proper range */
	if ( pdata >= obj->ps_th_far_high )
	{
		obj->ps_th_status = PS_NEAR;
		obj->ps_status = PS_NEAR;
		pas230_set_pilt ( client, (unsigned short)obj->ps_th_near_low );
		pas230_set_piht ( client, (unsigned short)obj->ps_th_near_high );
		SENSOR_LOG ( "PS_TH=NEAR (near_low=%d / near_high=%d)\n",obj->ps_th_near_low,obj->ps_th_near_high );
	}
	else
	{
		obj->ps_th_status = PS_FAR;
		obj->ps_status = PS_FAR;
		pas230_set_pilt ( client, (unsigned short)obj->ps_th_far_low );
		pas230_set_piht ( client, (unsigned short)obj->ps_th_far_high );
		SENSOR_LOG ( "PS_TH=FAR (far_low=%d / far_high=%d)\n",obj->ps_th_far_low,obj->ps_th_far_high );
	}

#endif
	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->ps_status;
	if ( ( res = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	    goto enable_error;
	}
    
	//SENSOR_LOG ( "PAS230 was enabled\n" );

    return (PAS230_SUCCESS);
    
enable_error :
    pas230_set_ps_enable(client, old_power_state);
	SENSOR_ERR ( "failed to enable PAS230\n" );

    return (res);
}

static long pas230_ps_disable ( struct i2c_client *client  )
{
	int res = 0;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	/* disable PAS230 */
	res = pas230_set_ps_enable ( client, SET_IC_DISABLE);
	if ( res == PAS230_SUCCESS )
	{
		//SENSOR_LOG ( "PAS230 was disabled\n" );
	}
	else
	{
		SENSOR_ERR ( "failed to disable PAS230\n" );
	}

	return res;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static long pas230_als_enable ( struct i2c_client *client  )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	struct hwm_sensor_data sensor_data;
	int res    = 0;
	int alsdata  = 0;
    unsigned char old_power_state;
    u32 low_threshould;
    u32 high_threshould;

    old_power_state = (unsigned char)obj->enable_als_sensor;

	//SENSOR_FUN();
    
	/* enable PAS230 */
	res = pas230_set_als_enable ( client, SET_IC_ENABLE);
    if(res != PAS230_SUCCESS) {
	    goto enable_error;
    }

	low_threshould	= ALS_THRESHOLD_MAX;
	high_threshould = ALS_THRESHOLD_MIN;
	pas230_set_aiht(client, high_threshould);
	pas230_set_ailt(client, low_threshould);

	msleep(100);

	/* read sensor data */
	res = pas230_get_alsdata ( client, (unsigned short*)&alsdata );
    if(res != PAS230_SUCCESS) {
	    goto enable_error;
    }
	/* inform to upper layer ( hwmsen ) */
	sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	sensor_data.value_divide = 1;
	sensor_data.values[0] = obj->als_data;
	if ( ( res = hwmsen_get_interrupt_data ( ID_LIGHT, &sensor_data ) ) )
	{
		SENSOR_ERR ( "failed to send inform ( err = %d )\n", res );
	    goto enable_error;
	}
	//SENSOR_LOG ( "PAS230_ALS was enabled\n" );

    return (PAS230_SUCCESS);
    
enable_error :
    pas230_set_als_enable(client, old_power_state);
	SENSOR_ERR ( "failed to enable PAS230\n" );
    
    return (res);
}

static long pas230_als_disable ( struct i2c_client *client  )
{
	int res = 0;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	/* disable PAS230 */
	res = pas230_set_als_enable ( client, SET_IC_DISABLE);
	if ( res == PAS230_SUCCESS )
	{
		//SENSOR_LOG ( "PAS230_ALS was disabled\n" );
	}
	else
	{
		SENSOR_ERR ( "failed to disable PAS230_ALS\n" );
	}
	return res;
}

static int pas230_get_alsdata ( struct i2c_client *client, unsigned short *alsData)
{
	int res = 0;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	res = pas230_read_word ( client, GREEN_DATA, (u16*)alsData ,3 );
	if ( res == PAS230_SUCCESS )
	{
		//SENSOR_DBG ( "ALSDATA=0x%04x\n", *alsData );
	}

/* get ALS sensitivity(18bit, Gain 3) = 2.4/(2^(18-16)*3) = 0.2 */
	/* value /= 5; */
	/* glass conpensation = x5 */
	/* value *= 5; */

	if ( obj->local_hw->als_lux_coeff >= 1 ) {
		*alsData = ( (*alsData) * (obj->local_hw->als_lux_coeff) )/10;
		SENSOR_LOG("als value=%d", *alsData);
	}
	else {
		if( *alsData > 4 ) {
			*alsData -= 4;
			*alsData /= 3;
		} else {
			*alsData = 0;
		}
		SENSOR_LOG("als lux_value=%d", *alsData);
	}
	obj->als_data = *alsData;
	return res;
}
#endif

void pas230_swap(int *x, int *y)
{
     int temp = *x;
     *x = *y;
     *y = temp;
}

static int pas230_do_calibration ( struct i2c_client *client, int *value )
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
    unsigned int calib_data;
	unsigned int old_enable = 0;
    int           result,i;
    

	old_enable  = obj->enable_ps_sensor;
    
    calib_data = 0;
    for (i=0; i < CALB_TIMES; i++) {
        calib_data = pas230_calc_calibration(client);
        if( calib_data <= obj->local_hw->crosstalk_max /*200*/ /*25*/ /*870*/ ) {
			SENSOR_LOG("ps_cross_talk save : %d\n",calib_data);
            obj->ps_cross_talk = calib_data;
            break;
        }
    }

	result = pas230_set_ps_enable ( client, old_enable );
    if(result != PAS230_SUCCESS) {
        return (result);
    }
	*value = obj->ps_cross_talk;
    
    if(i >= CALB_TIMES){
		SENSOR_ERR ( "failed to calibrate cross talk/n" );
		return -1;
    } else {
		obj->ps_th_far_low = 0;
		obj->ps_th_far_high = obj->local_hw->near_offset + obj->ps_cross_talk;
		obj->ps_th_near_low = obj->ps_th_far_high - obj->local_hw->far_offset;
		obj->ps_th_near_high = (PS_TH_VAL_MAX&0x3ff);
    }

	/* we should store it to storage ( it should be free from factory reset ) but ATCI Demon will store it through LGP Demon */
	return 0;
}

static unsigned int pas230_calc_calibration ( struct i2c_client *client )
{
    unsigned int value;
	int temp_pdata[CALB_BOX_TIMES] = {0,};
	int temp_state[CALB_BOX_TIMES] = {0,};
	unsigned int i                 = 0;
	unsigned int j                 = 0;
    unsigned int sum_of_pdata      = 0;
    int result ;


	/* Enable PS and Mask interrupt */
	result = pas230_set_ps_enable ( client, SET_IC_DISABLE );
    if(result != PAS230_SUCCESS) {
        return (result);
    }

	result = pas230_set_ps_enable ( client, SET_IC_ENABLE );
    if(result != PAS230_SUCCESS) {
        return (result);
    }

	mdelay ( CALB_SYSTEM_WAIT );

	/* Read pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		pas230_get_pdata ( client, (unsigned short *)&( temp_pdata[i] ) );
		mdelay ( CALB_IC_WAIT );
	}

#if DEBUG
	SENSOR_LOG ( "Read Value = " );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		SENSOR_LOG ( "%d ", temp_pdata[i] );
	}
	SENSOR_LOG ( "\n" );
#endif

	/* sort pdata */
	for ( i = 0 ; i < CALB_BOX_TIMES - 1 ; i++ )
	{
		for ( j = i + 1 ; j < CALB_BOX_TIMES ; j++ )
		{
			if ( temp_pdata[i] > temp_pdata[j] )
			{
				pas230_swap ( temp_pdata+i, temp_pdata+j );
			}
		}
	}

#if DEBUG
	SENSOR_LOG (  );
	for ( i = 0 ; i < CALB_BOX_TIMES ; i++ )
	{
		SENSOR_LOG ( "count = %d Read Value = %d\n", i, temp_pdata[i] );
	}
#endif

	/* take ten middle data only */
	for ( i = CALB_REMOVAL_TIME ; i < (CALB_BOX_TIMES - CALB_REMOVAL_TIME) ; i++ )
	{
		sum_of_pdata = sum_of_pdata + temp_pdata[i];
	}

	/* calculate average */
    value = sum_of_pdata / (CALB_BOX_TIMES - (CALB_REMOVAL_TIME * 2));
	SENSOR_LOG ( "New calibrated cross talk = %d\n", value );

    return (value);
}

//==========================================================
// PAS230 General Control Funtions
//==========================================================
static long pas230_ps_activate ( struct i2c_client *client, int enable )
{
	SENSOR_FUN ();
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->enable_ps_sensor != enable )
	{
		if ( enable )
		{
			res = pas230_ps_enable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable PAS230\n" );
			}
		}
		else
		{
			res = pas230_ps_disable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was disabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to disable PAS230\n" );
			}
		}

		if ( res == PAS230_SUCCESS )
		{
			obj->enable_ps_sensor = enable;
		}
		
	}

	return res;
	
}
#ifdef PAS230_ALS_SENSOR_ENABLE
static long pas230_als_activate ( struct i2c_client *client, int enable )
{
	SENSOR_FUN ();
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long res = 0;

	if ( obj->enable_als_sensor != enable )
	{
		if ( enable )
		{
			res = pas230_als_enable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was enabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to enable PAS230\n" );
			}
		}
		else
		{
			res = pas230_als_disable ( client );
			if ( res == PAS230_SUCCESS )
			{
				SENSOR_LOG ( "PAS230 was disabled\n" );
			}
			else
			{
				SENSOR_ERR ( "failed to disable PAS230\n" );
			}
		}

		if ( res == PAS230_SUCCESS )
		{
			obj->enable_als_sensor = enable;
		}
	}

	return res;
	
}
#endif
//==========================================================
// PAS230 Interrupt Service Routines
//==========================================================
#if defined(CONFIG_OF)
static irqreturn_t pas230_eint_handler(int irq, void *desc)
{
	//SENSOR_FUN ();
	pas230_eint_func();
	disable_irq_nosync(g_pas230_ptr->irq);

	return IRQ_HANDLED;
}
#endif
void pas230_eint_func ( void )
{
	//SENSOR_FUN ();
	struct pas230_priv *obj = g_pas230_ptr;
	if ( !obj )
	{
		return;
	}
	schedule_work ( &obj->eint_work );
}

static void pas230_eint_work ( struct work_struct *work )
{
    struct pas230_priv *obj = ( struct pas230_priv * ) container_of ( work, struct pas230_priv, eint_work );
    struct i2c_client *client = obj->client;
    struct hwm_sensor_data sensor_data;

    int err;

    int int_status = 0;
    unsigned short pdata = 0;
    unsigned short adata = 0;
 
    int new_ps_status = 0;
    u32 low_als;
    u32 high_als;

    //SENSOR_DBG ( "External interrupt happened\n" );

    /* read main status register */
    err = pas230_get_status ( client, &int_status );
    if ( err != PAS230_SUCCESS)
    {
        SENSOR_ERR("ADC value is not valid so just skip this interrupt");
        goto CLEAR_INTERRUPT;
    }

    if((int_status & PAS230_PINT) == PAS230_PINT) {
        /* Check the system state */
        if(obj->enable_ps_sensor == (SET_IC_ENABLE)) {
            SENSOR_LOG ( "PS interrupt happened\n" );
          
            /* read sensor data */
            err = pas230_get_pdata ( client, &pdata );
            if ( err != PAS230_SUCCESS) {
                SENSOR_ERR("Can't access register of pdata.");
                goto CLEAR_INTERRUPT;
            }
            
			new_ps_status = pas230_decide_ps_state(client, pdata);
            /* inform to upper layer ( hwmsen ), if status was changed */
            if ( new_ps_status != obj->ps_status )
            {

                obj->ps_status    = new_ps_status;
                obj->ps_th_status = new_ps_status;

                sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
                sensor_data.value_divide = 1;
                sensor_data.values[0] = obj->ps_status;
                if ( ( err = hwmsen_get_interrupt_data ( ID_PROXIMITY, &sensor_data ) ) )
                {
                    SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
                }
            }
        }
    }
#ifdef PAS230_ALS_SENSOR_INT
	if(int_status & ALS_INT_MASK) {
		if(obj->enable_als_sensor == (SET_IC_ENABLE)){
			//SENSOR_LOG("ALS Interrupt happened\n");
			err = pas230_get_alsdata( client, &adata);
			if( err != PAS230_SUCCESS)
			{
	            SENSOR_ERR("Can't access register of pdata.");
	            goto CLEAR_INTERRUPT;
	        }
	          
		    sensor_data.status = SENSOR_STATUS_ACCURACY_MEDIUM;
	        sensor_data.value_divide = 1;
			sensor_data.values[0] =	obj->als_data;
	        if ( ( err = hwmsen_get_interrupt_data ( ID_LIGHT, &sensor_data ) ) )
	        {
	              SENSOR_ERR ( "failed to send inform ( err = %d )\n", err );
	        }

			if(atomic_read(&driver_suspend_flag) == 1) {		
				SENSOR_LOG("One threshold detect mode\n");
				if( adata <= ALS_LUX_THRESHOLD ) {
					SENSOR_DBG("als_data(%d)<=THRESOLD(%d), Change bright to dark\n", adata, ALS_LUX_THRESHOLD);
					high_als = obj->local_hw->als_up_thres;
					low_als = ALS_THRESHOLD_MIN;
				} else {
					SENSOR_DBG("als_data(%d)>THRESOLD(%d), Change dark to bright\n", adata, ALS_LUX_THRESHOLD);
					high_als = ALS_THRESHOLD_MAX;
					low_als = obj->local_hw->als_low_thres;
				}
			} else if(atomic_read(&driver_suspend_flag) == 0) {
				if( adata > ALS_THRESHOLD_MAX ) {
					high_als = ALS_THRESHOLD_MAX;
					low_als = (adata * (100-LUX_VAR_PERCENT)) / 100 ;
				} else {
					high_als = (adata * (100+LUX_VAR_PERCENT)) / 100 ;
					low_als = (adata * (100-LUX_VAR_PERCENT)) / 100 ;
				}
			}
			SENSOR_DBG("als_up %d / als_low %d\n", high_als, low_als);
			pas230_set_aiht(client, high_als);
			pas230_set_ailt(client, low_als);
		}
	}
#endif
CLEAR_INTERRUPT:	

	/* unmask external interrupt */
	enable_irq(g_pas230_ptr->irq);
	enable_irq_wake(g_pas230_ptr->irq);
}

//==========================================================
// PAS230 ADB Shell command function
//==========================================================
static ssize_t pas230_show_cali_value ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%u\n", data->ps_cross_talk );
}

static ssize_t pas230_store_cali_value ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	int ret;
	int data;

	ret = pas230_do_calibration ( client, &data );

	return count;
}

static ssize_t pas230_show_ps_led ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->ps_led  );
}

static ssize_t pas230_store_ps_led ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_led ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_led  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_ps_pulse ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->ps_pulse  );
}

static ssize_t pas230_store_ps_pulse ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_pulse ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_pulse  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_ps_meas_rate ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->ps_meas_rate  );
}

static ssize_t pas230_store_ps_meas_rate ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_ps_meas_rate ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_meas_rate  = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_pilt ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->pilt );
}

static ssize_t pas230_store_pilt ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_pilt ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_th_near_low = (unsigned int)val;

	return count;
}

static ssize_t pas230_show_piht ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", data->piht );
}

static ssize_t pas230_store_piht ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	ret = pas230_set_piht ( client, (unsigned short)val );

	if ( ret < 0 )
		return ret;

    obj->ps_th_far_high = (unsigned int)val;

	return count;
}


static ssize_t pas230_show_status ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    int status = 0;

    pas230_get_status ( client, &status );

    return sprintf ( buf, "0x%02x\n", status );
}

static ssize_t pas230_show_pdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned short data = 0;

    pas230_get_pdata ( client, &data );

    return sprintf ( buf, "%d\n", data );
}
#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_show_alsdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned short data = 0;
	int err = 0;
	err = pas230_get_alsdata( client, &data);
	if( err != PAS230_SUCCESS)
	{
		SENSOR_ERR("Can't access register of alsdata.");
	}
    return sprintf ( buf, "%d\n", data );
}
static ssize_t pas230_show_luxdata ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned short data = 0;
	int err = 0;
	err = pas230_get_alsdata( client, &data);
	if( err != PAS230_SUCCESS)
	{
		SENSOR_ERR("Can't access register of alsdata.");
	}
    return sprintf ( buf, "%d\n", data );
}
#endif
static ssize_t pas230_show_deviceid ( struct device_driver *dev, char *buf )
{
    struct i2c_client *client = pas230_i2c_client;
    unsigned char data = 0;

    pas230_get_deivceid ( client, &data );

    return sprintf ( buf, "%02x\n", data );
}

static ssize_t pas230_show_near_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", obj->local_hw->near_offset);
}

static ssize_t pas230_store_near_offset ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	obj->local_hw->near_offset = val;
	obj->ps_th_far_high = obj->local_hw->near_offset + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - obj->local_hw->far_offset;

	ret = pas230_set_piht ( client, (unsigned short)obj->ps_th_far_high );
	ret = pas230_set_pilt ( client, (unsigned short)obj->ps_th_near_low );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t pas230_show_far_offset ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", obj->local_hw->far_offset);
}

static ssize_t pas230_store_far_offset ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	obj->local_hw->far_offset = val;
	obj->ps_th_far_high = obj->local_hw->near_offset + obj->ps_cross_talk;
	obj->ps_th_near_low = obj->ps_th_far_high - obj->local_hw->far_offset;

	ret = pas230_set_piht ( client, (unsigned short)obj->ps_th_far_high );
	ret = pas230_set_pilt ( client, (unsigned short)obj->ps_th_near_low );

	if ( ret < 0 )
		return ret;

	return count;
}

static ssize_t pas230_show_crosstalk_max( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	return sprintf ( buf, "%d\n", obj->local_hw->crosstalk_max);
}

static ssize_t pas230_store_crosstalk_max ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret;

	obj->local_hw->crosstalk_max = val;
	SENSOR_LOG ( "pas230_store_crosstalk_max = %d\n", obj->local_hw->crosstalk_max);

	return count;
}

static ssize_t pas230_show_ps_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

    switch(data->enable_ps_sensor) {
          case 0:
         	   return sprintf ( buf, "%s\n", "Proximity Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Proximity Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Proximity Error" );
     	}
    
}

static ssize_t pas230_store_ps_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = pas230_ps_activate ( client, 0 );
            break;
          case 1:
            ret = pas230_ps_activate ( client, 1 );
            break;

           default:
           	break;
     	}

	if ( ret < 0 )
		return ret;

	return count;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static ssize_t pas230_show_als_enable ( struct device_driver *dev, char *buf )
{
	struct i2c_client *client = pas230_i2c_client;
	struct pas230_priv *data = i2c_get_clientdata ( client );

    switch(data->enable_als_sensor) {
          case 0:
         	   return sprintf ( buf, "%s\n", "Light Disabled");
          case 1:
               return sprintf ( buf, "%s\n", "Light Enabled" );

           default:
               return sprintf ( buf, "%s\n", "Light Error" );
     	}
    
}

static ssize_t pas230_store_als_enable ( struct device_driver *dev, char *buf, size_t count )
{
	struct i2c_client *client = pas230_i2c_client;
	unsigned long val = simple_strtoul ( buf, NULL, 10 );
	int ret=0;

    switch(val) {
          case 0:
            ret = pas230_als_activate ( client, 0 );
            break;
          case 1:
            ret = pas230_als_activate ( client, 1 );
            break;

           default:
           	break;
     	}

	if ( ret < 0 )
		return ret;

	return count;
}
#endif
static DRIVER_ATTR ( cali, S_IWUSR | S_IRUGO, pas230_show_cali_value, pas230_store_cali_value );
static DRIVER_ATTR ( ps_led, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_led, pas230_store_ps_led );
static DRIVER_ATTR ( ps_pulse, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_pulse, pas230_store_ps_pulse );
static DRIVER_ATTR ( ps_meas_rate, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_meas_rate, pas230_store_ps_meas_rate );
static DRIVER_ATTR ( pilt, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_pilt, pas230_store_pilt );
static DRIVER_ATTR ( piht, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_piht, pas230_store_piht );
static DRIVER_ATTR ( status, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_status, NULL );
static DRIVER_ATTR ( pdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_pdata, NULL );
#ifdef PAS230_ALS_SENSOR_ENABLE
static DRIVER_ATTR ( alsdata0, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_alsdata, NULL );
static DRIVER_ATTR ( luxdata, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_luxdata, NULL );
#endif
static DRIVER_ATTR ( deviceid, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_deviceid, NULL );  
static DRIVER_ATTR ( near_offset, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_near_offset, pas230_store_near_offset );
static DRIVER_ATTR ( far_offset, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_far_offset, pas230_store_far_offset );
static DRIVER_ATTR ( crosstalk_max, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_crosstalk_max, pas230_store_crosstalk_max );
static DRIVER_ATTR ( enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_ps_enable, pas230_store_ps_enable );
#ifdef PAS230_ALS_SENSOR_ENABLE
static DRIVER_ATTR ( als_enable, S_IWUSR | S_IRUGO | S_IWGRP | S_IRGRP | S_IROTH, pas230_show_als_enable, pas230_store_als_enable );
#endif
static struct driver_attribute *pas230_attr_list[] = {
	&driver_attr_cali,		   
	&driver_attr_ps_led,
	&driver_attr_ps_pulse,
	&driver_attr_ps_meas_rate,
	&driver_attr_pilt,
	&driver_attr_piht,
	&driver_attr_status,
	&driver_attr_pdata,
#ifdef PAS230_ALS_SENSOR_ENABLE
	&driver_attr_alsdata0,
	&driver_attr_luxdata,
#endif
	&driver_attr_deviceid,
	&driver_attr_near_offset,
	&driver_attr_far_offset,
	&driver_attr_crosstalk_max,
	&driver_attr_enable,
#ifdef PAS230_ALS_SENSOR_ENABLE
	&driver_attr_als_enable,
#endif
};

static int pas230_create_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( pas230_attr_list ) / sizeof ( pas230_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		if ( err = driver_create_file ( driver, pas230_attr_list[idx] ) )
		{
			SENSOR_ERR ( "driver_create_file (%s) = %d\n", pas230_attr_list[idx]->attr.name, err );
			break;
		}
	}

	return err;
}

static int pas230_delete_attr ( struct device_driver *driver )
{
	int idx;
	int err = 0;
	int num = ( int ) ( sizeof ( pas230_attr_list ) / sizeof ( pas230_attr_list[0] ) );

	if ( driver == NULL )
	{
		return -EINVAL;
	}

	for ( idx = 0 ; idx < num ; idx++ )
	{
		driver_remove_file ( driver, pas230_attr_list[idx] );
	}

	return err;
}

//==========================================================
// PAS230 Service APIs ( based on File I/O )
//==========================================================
static int pas230_open ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = pas230_i2c_client;

	if ( !file->private_data )
	{
		SENSOR_ERR ( "Invalid input paramerter\n" );
		return -EINVAL;
	}

	return nonseekable_open ( inode, file );
}

static int pas230_release ( struct inode *inode, struct file *file )
{
	SENSOR_FUN ();
	file->private_data = NULL;
	return 0;
}

static long pas230_unlocked_ioctl ( struct file *file, unsigned int cmd, unsigned long arg )
{
	struct i2c_client *client = ( struct i2c_client * ) file->private_data;
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	long err = 0;
	void __user *ptr = ( void __user * ) arg;
	int dat;
	unsigned short value=0;
	uint32_t enable;
	uint32_t crosstalk = 0;

	switch ( cmd )
	{
		case ALSPS_SET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_SET_PS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = pas230_ps_activate ( obj->client, 1 ) ) )
				{
					SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err = pas230_ps_activate ( obj->client, 0 ) ) )
				{
					SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;

		case ALSPS_GET_PS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_MODE\n" );
			enable = obj->enable_ps_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_DATA\n" );
			dat = obj->ps_status;
			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;

		case ALSPS_GET_PS_RAW_DATA:
			SENSOR_LOG ( "CMD = ALSPS_GET_PS_RAW_DATA\n" );
			if ( err = pas230_get_pdata ( obj->client, (unsigned short *)&dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_CALI:
			SENSOR_LOG ( "CMD = ALSPS_GET_CALI\n" );
			err = pas230_do_calibration ( obj->client, &dat );
			if ( err == 0 )
			{
				if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
				{
					err = -EFAULT;
					goto err_out;
				}
			}
			break;


		case ALSPS_SET_CALI:
            SENSOR_LOG ( "CMD = ALSPS_SET_CALI\n" );
            if ( copy_from_user ( &crosstalk, ptr, sizeof ( crosstalk ) ) )
            {
                err = -EFAULT;
                goto err_out;
            }

			if ( ( crosstalk >= (PS_TH_VAL_MAX&0x3ff) ) || ( crosstalk == 0 ) )
			{

				obj->ps_cross_talk = PS_DEFAULT_CAL;//k430 default calibration value

			}
			else
			{
				obj->ps_cross_talk = crosstalk;
			}
			#ifdef FAKE_CROSS_OFFSET
			obj->ps_cross_talk += 50;
			#endif
			obj->ps_th_far_high = obj->local_hw->near_offset + obj->ps_cross_talk;
		    obj->ps_th_near_low = obj->ps_th_far_high - obj->local_hw->far_offset;
            break;

		case ALSPS_GET_DEVICEID:
            SENSOR_LOG ( "CMD = ALSPS_GET_DEVICEID\n" );
			if ( err = pas230_get_deivceid ( obj->client, (unsigned char*) &dat ) )
			{
				goto err_out;
			}

			if ( copy_to_user ( ptr, &dat, sizeof ( dat ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
#ifdef PAS230_ALS_SENSOR_ENABLE
		case ALSPS_SET_ALS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_SET_ALS_MODE\n" );
			if ( copy_from_user ( &enable, ptr, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			if ( enable )
			{
				if ( ( err = pas230_als_activate ( obj->client, 1 ) ) )
				{
					SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			else
			{
				if ( ( err =  pas230_als_activate ( obj->client, 0 ) ) )
				{
					SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", (int)err );
					goto err_out;
				}
			}
			break;
		case ALSPS_GET_ALS_MODE:
			SENSOR_LOG ( "CMD = ALSPS_GET_ALS_MODE\n" );
			enable = obj->enable_als_sensor;
			if ( copy_to_user ( ptr, &enable, sizeof ( enable ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
		case ALSPS_GET_ALS_DATA:
			SENSOR_LOG( "CMD = ALSPS_GET_ALS_DATA\n" );
			pas230_get_alsdata ( obj->client, (unsigned short*)&value );
			if ( copy_to_user ( ptr, &value, sizeof ( value ) ) )
			{
				err = -EFAULT;
				goto err_out;
			}
			break;
#endif
		default:
			SENSOR_ERR ( "Invalid Command = 0x%04x\n", cmd );
			err = -ENOIOCTLCMD;
			break;
	}

	err_out : return err;
}

static struct file_operations pas230_fops = { .owner = THIS_MODULE, .open = pas230_open, .release = pas230_release,
												.unlocked_ioctl = pas230_unlocked_ioctl,  };

static struct miscdevice pas230_device = { .minor = MISC_DYNAMIC_MINOR, .name = "als_ps", .fops = &pas230_fops,  };

//==========================================================
// PAS230 Service APIs ( based on hwmsen Interface )
//==========================================================
#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_set_als_poll_delay(struct i2c_client *client,
		unsigned int val)
{
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int ret;
	int atime_index=0;

	//SENSOR_FUN ();
	/* minimum 5ms */
	if (val < 3000)
		val = 3000;

	/* convert us => ms */
	obj->als_poll_delay = val / 1000;

	/* pre-defined delays from SensorManager
	 * SENSOR_DELAY_NORMAL : 200ms
	 * SENSOR_DELAY_UI : 66.667ms
	 * SENSOR_DELAY_GAME : 20ms
	 * SENSOR_DELAY_FASTEST : 0ms
	 * hwmsen changes the delay to 10ms, less than 10ms
	 */

	return 0;
}
#endif
static int pas230_ps_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	//SENSOR_FUN ();
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;
	struct pas230_priv *obj = ( struct pas230_priv * ) self;

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			break;

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = pas230_ps_activate ( obj->client, 1 ) )
					{
						SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = pas230_ps_activate ( obj->client, 0 ) )
					{
						SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( struct hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				sensor_data = (struct hwm_sensor_data *)buff_out;
				sensor_data->values[0] = obj->ps_status;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}

#ifdef PAS230_ALS_SENSOR_ENABLE
static int pas230_als_operate ( void *self, uint32_t command, void *buff_in, int size_in, void *buff_out, int size_out, int *actualout )
{
	//SENSOR_FUN ();
	int err = 0;
	int value;
	struct hwm_sensor_data *sensor_data;
	struct pas230_priv *obj = ( struct pas230_priv * ) self;
    unsigned short adata = 0;

	switch ( command )
	{
		case SENSOR_DELAY:
			SENSOR_LOG ( "CMD = SENSOR_DELAY\n" );
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}

			else {
				/* ALS Integration Time setting as fast as polling rate */
				value = *(int *)buff_in;
				pas230_set_als_poll_delay(obj->client, value*1000);
			}
			break;
			

		case SENSOR_ENABLE:
			if ( ( buff_in == NULL ) || ( size_in < sizeof ( int ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
				value = *( int * ) buff_in;
				if ( value )
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Enable )\n" );
					if ( err = pas230_als_activate ( obj->client, 1 ) )
					{
						SENSOR_ERR ( "failed to activate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
				else
				{
					SENSOR_LOG ( "CMD = SENSOR_ENABLE ( Disable )\n" );
					if ( err = pas230_als_activate ( obj->client, 0 ) )
					{
						SENSOR_ERR ( "failed to deactivate PAS230 ( err = %d )\n", err );
						return -1;
					}
				}
			}
			break;

		case SENSOR_GET_DATA:
			SENSOR_LOG ( "CMD = SENSOR_GET_DATA\n" );
			if ( ( buff_out == NULL ) || ( size_out < sizeof ( struct hwm_sensor_data ) ) )
			{
				SENSOR_ERR ( "Invaild input parameter\n" );
				err = -EINVAL;
			}
			else
			{
	            err = pas230_get_alsdata ( obj->client, (unsigned short*)&adata );
				if( err != PAS230_SUCCESS)
				{
	            			SENSOR_ERR("Can't access register of als data.");
				}
				sensor_data = (struct hwm_sensor_data *)buff_out;
				sensor_data->values[0] = adata;
				sensor_data->value_divide = 1;
				sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
			}
			break;

		default:
			SENSOR_ERR ( "Invalid Command = %d\n", command );
			err = -1;
			break;
	}

	return err;
}
#endif

//==========================================================
// PAS230 Initialization related Routines
//==========================================================
static int pas230_init_client ( struct i2c_client *client )
{
	SENSOR_FUN ();
	struct pas230_priv *obj = i2c_get_clientdata ( client );
	int err = 0;

	err = pas230_initialize ( client );
	if ( err != PAS230_SUCCESS )
	{
		SENSOR_ERR ( "failed to init PAS230\n" );
	}

	return err;
}

/****************************************************************************
* I2C BUS Related Functions
****************************************************************************/

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void pas230_early_suspend ( struct early_suspend *h )
{
	SENSOR_FUN ();
}

static void pas230_late_resume ( struct early_suspend *h )
{
	SENSOR_FUN ();
}
#endif

static int pas230_i2c_probe ( struct i2c_client *client, const struct i2c_device_id *id )
{
	SENSOR_FUN ();
	struct pas230_priv *obj;
	struct hwmsen_object obj_ps;
#ifdef PAS230_ALS_SENSOR_ENABLE
	struct hwmsen_object obj_als;
#endif
	int err = 0;

	if ( !( obj = kzalloc ( sizeof ( *obj ), GFP_KERNEL ) ) )

	{
		err = -ENOMEM;
		goto exit;
	}
	memset ( obj, 0, sizeof ( *obj ) );

	obj->client = client;
	i2c_set_clientdata ( client, obj );

	g_pas230_ptr = obj;
	pas230_i2c_client = client;
	obj->hw = hw;
	obj->local_hw = proxi_hw;

	INIT_WORK ( &obj->eint_work, pas230_eint_work );

	/* Initialize PAS230 */ 
	if ( err = pas230_init_client ( client ) )
	{
		SENSOR_ERR ( "failed to init PAS230 ( err = %d )\n", err );
		goto exit_init_failed;
	}
	//SENSOR_LOG("init_client pass\n");

	/* Register PAS230 as a misc device for general I/O interface */
	if ( err = misc_register ( &pas230_device ) )
	{
		SENSOR_ERR ( "failed to register misc device ( err = %d )\n", err );
		goto exit_misc_device_register_failed;
	}

	if ( err = pas230_create_attr ( &(pas230_init_info.platform_diver_addr->driver) ) )
	{
		SENSOR_ERR ( "create attribute err = %d\n", err );
		goto exit_create_attr_failed;
	}

	/* Register PAS230 as a member device of hwmsen */
	obj_ps.self = obj;
	obj_ps.polling = hw->polling_mode_ps;
	obj_ps.sensor_operate = pas230_ps_operate;
	if ( err = hwmsen_attach ( ID_PROXIMITY, &obj_ps ) )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}
#ifdef PAS230_ALS_SENSOR_ENABLE
	/* light_timer settings. we poll for light values using a timer. */
	obj_als.self = obj;
	obj_als.polling = hw->polling_mode_als;
	obj_als.sensor_operate = pas230_als_operate;
	if ( err = hwmsen_attach ( ID_LIGHT, &obj_als ) )
	{
		SENSOR_ERR ( "failed to attach to hwmsen ( err = %d )\n", err );
		goto exit_hwsen_attach_failed;
	}
#endif

#if defined(CONFIG_FB)
	obj->fb_notif.notifier_call = light_fb_notifier_callback;
	fb_register_client(&obj->fb_notif);
#endif

	pas230_init_flag=PAS230_INIT_SUCC; 
	SENSOR_LOG("OK\n");

	return 0;

	exit_hwsen_attach_failed:
    //pas230_delete_attr(&pas230_alsps_driver.driver);
	exit_create_attr_failed:
	misc_deregister ( &pas230_device );
	exit_misc_device_register_failed:
	exit_init_failed:
	//unregister_early_suspend ( &obj->early_drv );
	kfree ( obj );
	exit:
	pas230_i2c_client = NULL;
	pas230_init_flag=PAS230_INIT_FAIL;
	SENSOR_ERR ( "Err = %d\n", err );

	return err;
}

#if defined(CONFIG_FB)
static int light_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct i2c_client *client = pas230_i2c_client;
	struct fb_event *evdata = (struct fb_event *)data;
	int *blank = NULL;
    u32 low_threshould;
    u32 high_threshould;
	
	if (evdata && evdata->data)
		blank = (int *)evdata->data;
	else
		return PAS230_SUCCESS;

	if (event == FB_EVENT_BLANK) {
		if (*blank == FB_BLANK_POWERDOWN) {
			atomic_set(&driver_suspend_flag, 1);
			SENSOR_DBG("[IN] LCD Sleep\n");
		} else if (*blank == FB_BLANK_UNBLANK) {
			atomic_set(&driver_suspend_flag, 0);
			SENSOR_DBG("[OUT] LCD Sleep\n");
			low_threshould	= ALS_THRESHOLD_MAX;
			high_threshould = ALS_THRESHOLD_MIN;
			pas230_set_aiht(client, high_threshould);
			pas230_set_ailt(client, low_threshould);
		}
	}
	return PAS230_SUCCESS;
}
#endif

static int pas230_suspend(struct device *dev)
{
	/* We disable power only if proximity is disabled.  If proximity
	   is enabled, we leave power on because proximity is allowed
	   to wake up device.  We remove power without changing
	   pas230->power_state because we use that state in resume.
	*/
	SENSOR_DBG();
	return 0;
}

static int pas230_resume(struct device *dev)
{
	/* Turn power back on if we were before suspend. */
	SENSOR_DBG();
	return 0;
}

static int pas230_i2c_remove ( struct i2c_client *client )
{
	SENSOR_FUN ();
	int err;
	struct pas230_priv *obj = i2c_get_clientdata ( client );

	free_irq(g_pas230_ptr->irq,NULL);
	if( obj->enable_ps_sensor == SET_IC_ENABLE || obj->enable_als_sensor == SET_IC_ENABLE ) {
		err = pas230_write_byte(client, MAIN_CTRL, ALL_OFF);
		if ( err == PAS230_SUCCESS )
		{
			obj->enable_ps_sensor = SET_IC_DISABLE;
			obj->enable_als_sensor = SET_IC_DISABLE;
			SENSOR_LOG ( "PAS230 was disabled\n" );
		}
	}

	#if defined(CONFIG_FB)
	fb_unregister_client(&obj->fb_notif);
	#endif
	if ( err = pas230_delete_attr ( &(pas230_init_info.platform_diver_addr->driver) ) )
	{
		SENSOR_ERR ( "pas230_delete_attr fail: %d\n", err );
	}

	if ( err = misc_deregister ( &pas230_device ) )
	{
		SENSOR_ERR ( "failed to deregister misc driver : %d\n", err );
	}

	pas230_i2c_client = NULL;
	i2c_unregister_device ( client );
	kfree ( i2c_get_clientdata ( client ) );

	return 0;
}

static int pas230_i2c_detect ( struct i2c_client *client, struct i2c_board_info *info )
{
	SENSOR_FUN ();
	strcpy ( info->type, PAS230_DEV_NAME );
	return 0;
}

static const struct i2c_device_id pas230_i2c_id[] = { { PAS230_DEV_NAME, 0 }, {} };
#ifdef CONFIG_OF
static const struct of_device_id alsps_of_match[] = {
	{ .compatible = "mediatek,alsps", },
	{},
};
#endif
#ifdef CONFIG_PM
static const struct dev_pm_ops pas230_pm_ops = {
	.suspend = pas230_suspend,
	.resume = pas230_resume
};
#endif

static struct i2c_driver pas230_i2c_driver = {
	.probe = pas230_i2c_probe,
	.remove = pas230_i2c_remove,
	.detect = pas230_i2c_detect,
	.id_table = pas230_i2c_id,
	.driver = {
		.name = PAS230_DEV_NAME,
#ifdef CONFIG_OF
		.of_match_table = alsps_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &pas230_pm_ops
#endif
	},
};

/****************************************************************************
* Linux Device Driver Related Functions
****************************************************************************/
static struct i2c_board_info __initdata i2c_PAS230 = { I2C_BOARD_INFO ( PAS230_DEV_NAME, PAS230_I2CADDR ) };

static int  pas230_local_init(void)
{ 
	SENSOR_FUN();
	/* Turn on the power */
	//pas230_main_power ( hw, 1 );
	//msleep ( 9 );
	i2c_register_board_info(hw->i2c_num, &i2c_PAS230, 1);
	/* Add PAS230 as I2C driver */
	if ( i2c_add_driver ( &pas230_i2c_driver ) )
	{
		SENSOR_ERR ( "failed to add i2c driver\n" );
		return -1;
	}

	if(-1 == pas230_init_flag)
	{
	   return -1;
	}

	return 0;
}
static int pas230_remove ( void )
{
	SENSOR_FUN ();
	/* not support Turn on the power - always power on */
	i2c_del_driver ( &pas230_i2c_driver );

	return 0;
}

static int __init pas230_init ( void )
{
	SENSOR_FUN ();
	const char *name = "mediatek,alsps";

	hw =   get_alsps_dts_func(name, hw);
	if (!hw)
		SENSOR_ERR("get dts info fail\n");
	
	proxi_hw = get_pas230_dts_func(name, proxi_hw);
	if (!proxi_hw)
		SENSOR_ERR("get dts info fail\n");
	
	alsps_driver_add(&pas230_init_info);
	return 0;
}

static void __exit pas230_exit ( void )
{
	SENSOR_FUN ();
	//platform_driver_unregister ( &pas230_alsps_driver );
}


module_init ( pas230_init );
module_exit ( pas230_exit );

MODULE_AUTHOR ( "Seo Ji Won" );
MODULE_DESCRIPTION ( "pas230 driver" );
MODULE_LICENSE ( "GPL" );

/* End Of File */
