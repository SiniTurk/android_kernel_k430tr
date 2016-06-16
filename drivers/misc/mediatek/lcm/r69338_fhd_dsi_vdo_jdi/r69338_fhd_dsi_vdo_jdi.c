/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#include <string.h>
#include <mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <linux/string.h>
#include "mt_gpio.h"
#include <mach/gpio_const.h>
#include <mach/board_lge.h>
#endif

#include "lcm_drv.h"
#if defined(BUILD_LK)
#define LCM_PRINT printf
#elif defined(BUILD_UBOOT)
#define LCM_PRINT printf
#else
#define LCM_PRINT printk
#endif
#if defined(BUILD_LK)
#include <boot_mode.h>
#else
#include <upmu_hw.h>
#include <mt_typedefs.h>
#endif
#if defined(CONFIG_TOUCHSCREEN_MTK_S3320)
#include <linux/input/unified_driver_3/lgtp_common_notify.h>
#endif
#if defined(CONFIG_SENSOR_TOUCH_KEY_ATMF04)
#include <linux/input/atmf04/atmf04.h>
#endif
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// pixel
#define FRAME_WIDTH  			(1080)
#define FRAME_HEIGHT 			(1920)

// physical dimension
#define PHYSICAL_WIDTH        (65)
#define PHYSICAL_HEIGHT         (115)

#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE		0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl, size, force_update)   	lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)											lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

static unsigned int need_set_lcm_addr = 1;

#ifndef GPIO_LCM_RST
#define GPIO_LCM_RST         (GPIO146 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO   GPIO_MODE_00
#define GPIO_LCM_RST_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_TOUCH_RESET
#define GPIO_TOUCH_RESET         (GPIO62 | 0x80000000)
#define GPIO_TOUCH_RESET_M_GPIO   GPIO_MODE_00
#define GPIO_TOUCH_RESET_M_LCM_RST   GPIO_MODE_01
#endif

#ifndef GPIO_DSV_ENP_EN
#define GPIO_DSV_ENP_EN (GPIO59 | 0x80000000)
#define GPIO_DSV_ENP_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENP_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENP_EN_M_PWM GPIO_MODE_05
#endif

#ifndef GPIO_DSV_ENN_EN
#define GPIO_DSV_ENN_EN (GPIO60 | 0x80000000)
#define GPIO_DSV_ENN_EN_M_GPIO GPIO_MODE_00
#define GPIO_DSV_ENN_EN_M_KROW GPIO_MODE_06
#define GPIO_DSV_ENN_EN_M_PWM GPIO_MODE_05
#endif

extern kal_uint16 pmic_set_register_value(PMU_FLAGS_LIST_ENUM flagname,kal_uint32 val);
extern void rt4832_dsv_ctrl(int enable);
extern void rt4832_dsv_toggle_ctrl(void);

#define EVB_PCB_REVISION 2 /* EVB:2  RevA:3 To Do:  HW_REV_A */

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

extern unsigned short mt6328_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);
static struct LCM_setting_table lcm_initialization_setting[] = {

	{0x51,  1,  {0xFF}},     //[1] Power On #1
	{0x53,  1,  {0x0C}},     //[1] Power On #2
	{0x55,  1,  {0x00}},     //[1] Power On #3
#if 0
	{0xB0, 1, {0x04}},
	{0XC8, 19, {0x01,0x00,0x00,0xFA,0x00,
			0xFC,0x00,0x00,0x00,0xFA,
			0x00,0xFC,0x00,0x00,0x00,
			0x00,0x00,0xFC,0x00}},
	{0XB8, 6, {0x07,0x90,0x1E,0x10,0x1E,0x32}}, //Back Light Control 1
	{0XB9, 6, {0x07,0x82,0x3C,0x10,0x3C,0x87}},
	{0XBA, 6, {0x07,0x78,0x64,0x10,0x64,0xB4}},
	{0XCE, 24, {0x75,0x40,0x43,0x49,0x55,
			0x62,0x71,0x82,0x94,0xA8,
			0xB9,0xCB,0xDB,0xE9,0xF5,
			0xFC,0xFF,0x07,0x25,0x04,
			0x04,0x44,0x00,0x00}},
	{0XBB, 3, {0x01,0x1E,0x14}},
	{0XBC, 3, {0x01,0x50,0x32}},
	{0XBD, 3, {0x00,0xB4,0xA0}},

	{0xCA, 32, {0x00,0x80,0x80,0x80,0x80,
			0x80,0x80,0x80,0x08,0x20,
			0x80,0x80,0x0a,0x4a,0x37,
			0xa0,0x55,0xf8,0x0c,0x0c,
			0x20,0x10,0x3f,0x3f,0x00,
			0x00,0x10,0x10,0x3f,0x3f,
			0x3f,0x3f}},

	{0xB0, 1, {0x04}},
	{0xB1, 4, {0x84,0x60,0x00,0x10}},
	{0XD6, 1, {0x01}},
#endif
	/*display on*/
	{0x29,	0,  {}},            //[5] set display on
	//{REGFLAG_DELAY, 120, {}},    //MDELAY(120)
	/* exit sleep*/
	{0x11,	0,  {}},            //[3] exit sleep mode
	{REGFLAG_END_OF_TABLE, 0x00, {}},
};


static LCM_setting_table_V3 lcm_initialization_setting_rotation_V3[] = {

	{0X15, 0X36, 1, {0XC0}},	/* 180 rotation */
	{REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3[] = {

	{0X15, 0X51, 1, {0XFF}},	//Write_Display_Brightness
	{0X15, 0X53, 1, {0X0C}},		//Write_CTRL_Display
	{0X15, 0X55, 1, {0X00}},		//Write_CABC
	{0X05, 0X29, 1, {0x00}},
	{0x05, 0x11, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, 1,{}},

};

#if 0
static LCM_setting_table_V3 lcm_initialization_setting_V3_Disp_On[] = {
	{0x05, 0x11,	1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};
#endif

static LCM_setting_table_V3 lcm_initialization_for_sleep_in_pre[] =
{
	{0x05, 0x28,	1, {0x00}},
	{REGFLAG_ESCAPE_ID, REGFLAG_DELAY_MS_V3, 60,{}},
	{0x05, 0x10, 1, {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

#if 0
static LCM_setting_table_V3 lcm_initialization_for_sleep_in_post[] =
{
	{REGFLAG_END_OF_TABLE, 0x00,1, {}},
};
#endif 


static struct LCM_setting_table __attribute__ ((unused)) lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 1
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {
		case REGFLAG_DELAY:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
	LCM_PRINT("[LCD] push_table \n");
}
#endif
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy((void*)&lcm_util, (void*)util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS * params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// physical size
	params->physical_width = PHYSICAL_WIDTH;
	params->physical_height = PHYSICAL_HEIGHT;

       params->dsi.mode   = SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;
	 // enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM				    = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 	= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   	= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     	= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      	= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	params->dsi.packet_size=256;
	//params->dsi.intermediat_buffer_num = 0;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//params->dsi.cont_clock = TRUE;
	
	params->dsi.vertical_sync_active	= 1; //20150824 , 1->2
	params->dsi.vertical_backporch		= 3;
	params->dsi.vertical_frontporch		= 7;
	params->dsi.vertical_active_line	= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active	= 12;
	params->dsi.horizontal_backporch	= 36; //45;
	params->dsi.horizontal_frontporch	= 96; //50; //96;
	params->dsi.horizontal_active_pixel 	= FRAME_WIDTH;

	//params->dsi.HS_TRAIL = 10;
	//params->dsi.HS_ZERO= 20;
	//params->dsi.LPX= 20;
	//params->dsi.HS_PRPR= 8;

	params->dsi.PLL_CLOCK = 464; //426(55fps); //338; //312; // 234;

}

static void init_lcm_registers(void)
{

#if 1
	//DSI_clk_HS_mode(DISP_MODULE_DSI0, NULL, 1);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
	//if ( lge_get_board_revno() > EVB_PCB_REVISION )
		//dsi_set_cmdq_V3(lcm_initialization_setting_rotation_V3, sizeof(lcm_initialization_setting_rotation_V3) / sizeof(LCM_setting_table_V3), 1);
	dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(120);
	//dsi_set_cmdq_V3(lcm_initialization_setting_V3_Disp_On, sizeof(lcm_initialization_setting_V3_Disp_On) / sizeof(LCM_setting_table_V3), 1);
#else
	unsigned int data_array[32];
	dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
	data_array[0] = 0x00290500;	//Display On
	dsi_set_cmdq(data_array, 1, 1);

	data_array[0] = 0x00110500;	//Sleep Out
	dsi_set_cmdq(data_array, 1, 1);

	MDELAY(120);
#endif
	LCM_PRINT("[LCD] init_lcm_registers \n");
}

static void init_lcm_registers_sleep(void)
{
	dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_pre, sizeof(lcm_initialization_for_sleep_in_pre) / sizeof(LCM_setting_table_V3), 1);
	MDELAY(60);
	LCM_PRINT("[LCD] init_lcm_registers_sleep \n");
}

#if 0
void init_lcm_registers_sleep_2nd(void)
{
	unsigned int data_array[1];
	//dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_pre, sizeof(lcm_initialization_for_sleep_in_pre) / sizeof(LCM_setting_table_V3), 1);
	//MDELAY(150);
	dsi_set_cmdq_V3(lcm_initialization_for_sleep_in_post, sizeof(lcm_initialization_for_sleep_in_post) / sizeof(LCM_setting_table_V3), 1);

	LCM_PRINT("[LCD] init_lcm_registers_sleep 2nd \n");
}


static void init_lcm_registers_sleep_out(void)
{
	unsigned int data_array[1];

	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(50);
	LCM_PRINT("[LCD] init_lcm_registers_sleep \n");
}
#endif

/* 1.8v LDO is always on */
static void ldo_1v8io_on(void)
{
	mt6328_set_register_value(PMIC_RG_VRF18_1_VOSEL, 3);
	mt6328_set_register_value(PMIC_RG_VRF18_1_EN, 1);
}

/* 1.8v LDO is always on */
static void ldo_1v8io_off(void)
{
	mt6328_set_register_value(PMIC_RG_VRF18_1_EN, 0);
}

/* VGP1 3.0v LDO enable */
static void ldo_3v0_on(void)
{
	mt6328_set_register_value(PMIC_RG_VGP1_VOSEL, 6);
	mt6328_set_register_value(PMIC_RG_VGP1_EN, 1);
}

/* VGP1 3.0v LDO disable */
static void ldo_3v0_off(void)
{
	mt6328_set_register_value(PMIC_RG_VGP1_EN, 0);
}

/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_on(void)
{
#if defined(BUILD_LK)
	chargepump_DSV_on();
#else
	rt4832_dsv_ctrl(1);
#endif
}

/* DSV power +5.8V,-5.8v */
static void ldo_p5m5_dsv_5v5_off(void)
{
#if defined(BUILD_LK)
	chargepump_DSV_off();
#else
	rt4832_dsv_ctrl(0);
#endif

}


static void reset_lcd_module(unsigned char reset)
{
	mt_set_gpio_mode(GPIO_LCM_RST, GPIO_LCM_RST_M_GPIO);
	//mt_set_gpio_pull_enable(GPIO_LCM_RST, GPIO_PULL_ENABLE);
	mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);

   if(reset){
   	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
	LCM_PRINT("[LCD] Reset High \n");
   }
   else
   {
   mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
   LCM_PRINT("[LCD] Reset Low \n");
   }
}

static void touch_reset_pin (int mode)
{
	mt_set_gpio_mode(GPIO_TOUCH_RESET, GPIO_TOUCH_RESET_M_GPIO);
	mt_set_gpio_dir(GPIO_TOUCH_RESET, GPIO_DIR_OUT);

	if(mode == 1)
	{
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ONE);
	}
	else if(mode == 0)
	{
		mt_set_gpio_out(GPIO_TOUCH_RESET, GPIO_OUT_ZERO);
	}
}

static void lcm_init(void)
{
	ldo_3v0_on();
	MDELAY(10);

	ldo_1v8io_on();
	MDELAY(10);

	touch_reset_pin(1);
	MDELAY(10);

	ldo_p5m5_dsv_5v5_on();
	MDELAY(10);

	reset_lcd_module(1);
	MDELAY(10);
	
	init_lcm_registers();

	MDELAY(120);

	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_init \n");
}

static void lcm_suspend(void)
{
	init_lcm_registers_sleep();
	//rt4832_dsv_toggle_ctrl();
#if defined(CONFIG_TOUCHSCREEN_MTK_S3320)
	printk(KERN_INFO"[Touch] lcm notify_call_chain_suspend \n");
	touch_notifier_call_chain(LCD_EVENT_TOUCH_LPWG_ON, NULL);
#endif
#if defined(CONFIG_SENSOR_TOUCH_KEY_ATMF04)
	printk(KERN_INFO"[Touch] lcm notify_call_chain_suspend for ATMF04 \n");
	touch_key_notifier_call_chain(ATMF04_LCD_EVENT_OFF, NULL);
#endif
	LCM_PRINT("[LCD] lcm_suspend \n");
}


static void lcm_resume(void)
{
#if defined(CONFIG_TOUCHSCREEN_MTK_S3320)
    printk(KERN_INFO"[Touch] lcm notify_call_chain_resume \n");
    touch_notifier_call_chain(LCD_EVENT_TOUCH_LPWG_OFF, NULL);
#endif
#if defined(CONFIG_SENSOR_TOUCH_KEY_ATMF04)
	printk(KERN_INFO"[Touch] lcm notify_call_chain_resume for ATMF04\n");
	touch_key_notifier_call_chain(ATMF04_LCD_EVENT_ON, NULL);
#endif
#if 1

#if 0
	touch_reset_pin(0);
	MDELAY(10);
	reset_lcd_module(0);
	MDELAY(10);
	ldo_p5m5_dsv_5v5_off();
	MDELAY(10);
	ldo_1v8io_off();
	MDELAY(1);
	ldo_3v0_off();
	MDELAY(20);
	lcm_init();
#endif
	reset_lcd_module(0);
	MDELAY(20);

	reset_lcd_module(1);
	MDELAY(15);

	reset_lcd_module(0);
	MDELAY(5);

	reset_lcd_module(1);
	MDELAY(15);

	init_lcm_registers();

	MDELAY(120);
#else
	init_lcm_registers_sleep_out();
#endif
	need_set_lcm_addr = 1;
	LCM_PRINT("[LCD] lcm_resume JDI\n");
}

#if 0
static void lcm_esd_recover(void)
{
	lcm_suspend();
	lcm_resume();

	LCM_PRINT("[LCD] lcm_esd_recover \n");
}

static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	// need update at the first time
	if(need_set_lcm_addr)
	{
		data_array[0]= 0x00053902;
		data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
		data_array[2]= (x1_LSB);
		dsi_set_cmdq(data_array, 3, 1);

		data_array[0]= 0x00053902;
		data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
		data_array[2]= (y1_LSB);
		dsi_set_cmdq(data_array, 3, 1);
		need_set_lcm_addr = 0;
	}

	data_array[0]= 0x002c3909;
   dsi_set_cmdq(data_array, 1, 0);
	LCM_PRINT("[LCD] lcm_update \n");
}

static unsigned int lcm_compare_id(void)
{
#if 0
	unsigned int id=0;
	unsigned char buffer[2];
	unsigned int array[16];
    SET_RESET_PIN(1);
    SET_RESET_PIN(0);
    MDELAY(1);
    SET_RESET_PIN(1);
    MDELAY(10);//Must over 6 ms
	array[0]=0x00043902;
	array[1]=0x8983FFB9;// page enable
	dsi_set_cmdq(array, 2, 1);
	MDELAY(10);
	array[0] = 0x00023700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(10);
	read_reg_v2(0xF4, buffer, 2);
	id = buffer[0];
	LCM_PRINT("%s, id = 0x%08x\n", __func__, id);
	return (LCM_ID == id)?1:0;
#else
	LCM_PRINT("[SEOSCTEST] lcm_compare_id \n");
	return 1;
#endif
}

#endif

// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER r69338_fhd_dsi_vdo_jdi_drv = {
	.name = "r69338_fhd_dsi_vdo_jdi",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
//	.suspend_2nd_cmd = init_lcm_registers_sleep_2nd,
//	.set_pwm_for_mix = lcm_set_pwm_for_mix,
//	.compare_id = lcm_compare_id,
//	.update = lcm_update,
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
//	.esd_recover = lcm_esd_recover,
#endif
};
