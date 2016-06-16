#define MODULE_TAG	"<atmf04>"

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/wakelock.h>
#include <linux/unistd.h>
#include <linux/async.h>
#include <linux/in.h>
#include <linux/notifier.h>

#include <linux/syscalls.h>
#include <linux/uaccess.h>
#include <linux/sort.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>
#include <linux/of_irq.h>
#include <linux/firmware.h>
#ifdef CONFIG_OF
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#endif

#include <linux/pinctrl/consumer.h>
#include <linux/dma-mapping.h>
#include <mach/board_lge.h>
#include <linux/input/atmf04/atmf04.h>
#include <linux/irqchip/mt-eic.h>
#include "bs_log.h"



#define ATMF04_DRV_NAME     "atmf04"

/* I2C Suspend Check */
#define ATMF04_STATUS_RESUME        0
#define ATMF04_STATUS_SUSPEND       1
#define ATMF04_STATUS_QUEUE_WORK    2

/* LCD STATUS */
#define ATMF04_LCD_OFF	0
#define ATMF04_LCD_ON	1

/* Calibration Check */

#define ATMF04_CRCS_DUTY_LOW        770
#define ATMF04_CRCS_DUTY_HIGH       1490
#define ATMF04_CRCS_COUNT           100
#define ATMF04_CSCR_RESULT          -1
#define ATMF04_HB_CRCS_DUTY_LOW        635
#define ATMF04_HB_CRCS_DUTY_HIGH       1290
#define ATMF04_HB_CRCS_COUNT           140
#define ATMF04_HB_CSCR_RESULT          -1

/* I2C Register */

#define I2C_ADDR_SSTVT_H            0x07
#define I2C_ADDR_SSTVT_L            0x08
#define I2C_ADDR_TCH_ONOFF_CNT      0x02
#define I2C_ADDR_DIG_FILTER         0x15
#define I2C_ADDR_TEMP_COEF_UP       0x06
#define I2C_ADDR_TEMP_COEF_DN       0x07
#define I2C_ADDR_SAFE_DUTY_CHK      0x0E
#define I2C_ADDR_SYS_CTRL           0x12
#define I2C_ADDR_SYS_STAT           0x13
#define I2C_ADDR_CR_DUTY_H          0x28
#define I2C_ADDR_CR_DUTY_L          0x29
#define I2C_ADDR_CS_DUTY_H          0x2A
#define I2C_ADDR_CS_DUTY_L          0x2B
#define I2C_ADDR_PER_H              0x15
#define I2C_ADDR_PER_L              0x16
#define I2C_ADDR_TCH_OUTPUT         0x14
#define I2C_ADDR_PGM_VER_MAIN       0x18
#define I2C_ADDR_PGM_VER_SUB        0x19

/* Touch Key DUTY Data */
#define DM_DUTY_H   0x26
#define DM_DUTY_L   0x27
#define CH1_DUTY_H   0x28
#define CH1_DUTY_L   0x29
#define CH2_DUTY_H   0x2A
#define CH2_DUTY_L   0x2B


//Calibration Data Backup/Restore
#define I2C_ADDR_CMD_OPT 					0x7E
#define I2C_ADDR_COMMAND 					0x7F
#define I2C_ADDR_REQ_DATA					0x80
#define CMD_R_CD_DUTY						0x04		//Cal Data Duty Read
#define CMD_R_CD_REF						0x05		//Cal Data Ref Read
#define CMD_W_CD_DUTY						0x84		//Cal Data Duty Read
#define CMD_W_CD_REF						0x85		//Cal Data Ref Read
#define SZ_CALDATA_UNIT 					16
int CalData[4][SZ_CALDATA_UNIT];

#define ATMF_GPIO_EN	(125 | 0x80000000)
#define ATMF_GPIO_MENU	(126 | 0x80000000)
#define ATMF_GPIO_BACK	(127 | 0x80000000)

#define PRESS	0
#define RELEASE	1

#define MAX_I2C_TRANSFER_SIZE	255
#define INIT_I2C_CNT	10
#define TOUCH_PLATFORM_MTK


#define BIT_PERCENT_UNIT            8.192
#define MK_INT(X, Y)                (((int)X << 8)+(int)Y)

#define ENABLE_TOUCHKEY_PINS          0
#define DISABLE_TOUCHKEY_PINS         1

#define ON_TOUCHKEY                   0
#define OFF_TOUCHKEY                  1
#define PATH_CAPSENSOR_CAL  "/sns/capsensor_cal.dat"

#define CNT_INITCODE               13
const unsigned char InitCodeAddr[CNT_INITCODE] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x0C, 0x0D, 0x0E, 0x20, 0x21 };
const unsigned char InitCodeVal[CNT_INITCODE]   = { 0x00, 0x19, 0x33, 0x0B, 0x08, 0x6F, 0x6D, 0x07, 0x00, 0x0C, 0x14, 0x81, 0x20 }; // High Band ANT
const unsigned char InitCodeVal2[CNT_INITCODE] = { 0x00, 0x17, 0x33, 0x23, 0x08, 0x6E, 0x6D, 0x06, 0x00, 0x0C, 0x10, 0x01, 0x20 }; // Low Band  ANT

#if defined(TOUCH_PLATFORM_MTK)
static u8 *I2CDMABuf_va;
static u32 I2CDMABuf_pa;
#endif

/* initial Code Set */
/***********************
Key Table
ch1 = back
ch2 = menu

addr : 0x07 hi ch1
addr : 0x08 lo ch1
addr : 0x09 hi ch2
addr : 0x0A lo ch2
************************/
unsigned char i2c_addr_set[INIT_I2C_CNT] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
unsigned char i2c_data_set[INIT_I2C_CNT] = {0x03, 0x33, 0x00, 0x3C, 0x00, 0x1E, 0x00, 0x45, 0x00, 0x3D};

static int atmf_i2c_write(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen);
static int atmf_i2c_read(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen);

static struct i2c_driver atmf04_driver;
static struct workqueue_struct *atmf04_workqueue;

static BLOCKING_NOTIFIER_HEAD(touch_key_notifier_list);
unsigned char fuse_data[SZ_PAGE_DATA];

#ifdef CONFIG_OF
enum touch_key_dt_entry_status {
	DT_REQUIRED,
	DT_SUGGESTED,
	DT_OPTIONAL,
};

enum touch_key_dt_entry_type {
	DT_U32,
	DT_GPIO,
	DT_BOOL,
	DT_STRING,
};

struct touch_key_dt_to_pdata_map {
	const char			*dt_name;
	void				*ptr_data;
	enum touch_key_dt_entry_status status;
	enum touch_key_dt_entry_type	type;
	int				default_val;
};
#endif

static struct i2c_client *atmf04_i2c_client; /* global i2c_client to support ioctl */

struct atmf04_platform_data {
	int (*init)(struct i2c_client *client);
	void (*exit)(struct i2c_client *client);
	int (*power_on)(struct i2c_client *client, bool on);

	unsigned int irq_gpio_menu;
	unsigned int irq_gpio_back;
	unsigned long chip_enable;

	u32 irq_gpio_flags;

	bool i2c_pull_up;
	const char *fw_name;
};

struct atmf04_data {
	int (*get_nirq_low)(void);
	struct i2c_client *client;
	struct mutex update_lock;
	struct delayed_work	dmwork;		/* for Menu interrupt */
	struct delayed_work	dbwork;		/* for Back interrupt */
	struct wake_lock ps_wlock;
	struct input_dev *input_dev_cap;
#ifdef CONFIG_OF
	struct atmf04_platform_data *platform_data;
#endif
	unsigned int enable;
	unsigned int sw_mode;
	atomic_t i2c_status;
	atomic_t lcd_status;
	struct pinctrl *atmf04_pinctrl;
	struct pinctrl_state *atmf04_active;
	struct pinctrl_state *atmf04_suspend;

	unsigned int touch_out;
	struct notifier_block	common_notif;
};

static bool on_touchkey = false;
static bool probe_end_flag = false;

int get_bit(unsigned short x, int n);

void chg_mode(unsigned char flag, struct i2c_client *client)
{
	if(flag == ON) {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x80);
		PINFO("change_mode : %d",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	else {
		i2c_smbus_write_byte_data(client, ADDR_EFLA_STS, 0x00);
		PINFO("change_mode : %d",i2c_smbus_read_byte_data(client, ADDR_EFLA_STS));
	}
	mdelay(1);
}

unsigned char chk_done(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) {
			PINFO("RTN_TIMEOUT");
			return RTN_TIMEOUT;
		}
		mdelay(1);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE) != FLAG_DONE);

	return RTN_SUCC;
}

unsigned char chk_done_erase(unsigned int wait_cnt, struct i2c_client *client)
{
	unsigned int trycnt = 0;
	unsigned char rtn;

	do
	{
		if(++trycnt > wait_cnt) return RTN_TIMEOUT;

		mdelay(1);
		rtn = i2c_smbus_read_byte_data(client, ADDR_EFLA_STS);
	}while((rtn & FLAG_DONE_ERASE) != FLAG_DONE_ERASE);

	return RTN_SUCC;
}

unsigned char erase_eflash(struct i2c_client *client)
{
	i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_ERASE_ALL);

	if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT)
		return RTN_TIMEOUT; //timeout

	return RTN_SUCC;
}

unsigned char write_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * wdata, struct i2c_client *client)
{
	unsigned char paddr[2];
	int ret = 0;
	u8 addr = 0;
	int i=0;

	addr = 0x00;
	ret = atmf_i2c_write(client, &addr, 1, wdata, SZ_PAGE_DATA);
	if (ret < 0) {
	    PINFO("[%s] efylash page Read Fail, ret = %d", __func__, ret);
	    PINFO("================================================");
	    for(i = 0; i < SZ_PAGE_DATA; i++) {
		PINFO("i = %d: wdata = %d \n",i, wdata[i]);
	    }
	    PINFO("================================================");
	    return RTN_FAIL;
	}

	if(flag != FLAG_FUSE)
	{
		paddr[0] = page_addr[1];
		paddr[1] = page_addr[0];
	}
	else	//Extra User Memory
	{
		paddr[0] = 0x00;
		paddr[1] = 0x00;
	}

	addr = ADDR_EFLA_PAGE_L;
	ret = atmf_i2c_write(client, &addr, 1, paddr, 2);
	if (ret < 0) {
	    PINFO("[%s] can't write ADDR_EFLA_PAGE_L", __func__);
	    return RTN_FAIL;
	}

	if(flag != FLAG_FUSE)
	{
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);
	}
	else
	{
		//Erase
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_ERASE);
		if(chk_done_erase(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;
		//Write
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_WR);
	}

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	return RTN_SUCC;
}

#if defined(TOUCH_PLATFORM_MTK)
static int dma_allocation(void)
{
	I2CDMABuf_va = (u8 *)dma_alloc_coherent(NULL, 4096, &I2CDMABuf_pa, GFP_KERNEL);
	if (I2CDMABuf_va == NULL) {
		PINFO("[%s] atmf fail to allocate DMA", __func__);
		return -1;
	}

	return 0;
}

static int atmf04_dma_free(void)
{
	dma_free_coherent(NULL, 4096, I2CDMABuf_va, I2CDMABuf_pa);
	PINFO("atmf04 dma_allocation free.");

	return 0;
}

static int i2c_dma_write(struct i2c_client *client, const uint8_t *buf, int len)
{
	int i = 0;

	for (i = 0; i < len; i++)
		I2CDMABuf_va[i] = buf[i];

	if (len < 8) {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG;
		client->timing = 100;
		return i2c_master_send(client, buf, len);
	} else {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 100;
		return i2c_master_send(client, I2CDMABuf_pa, len);
	}
}

static int i2c_dma_read(struct i2c_client *client, uint8_t *buf, int len)
{
	int i = 0;
	int ret = 0;

	if (len < 8) {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG;
		client->timing = 100;
		return i2c_master_recv(client, buf, len);
	} else {
		client->addr = (client->addr & I2C_MASK_FLAG) | I2C_DMA_FLAG | I2C_ENEXT_FLAG;
		client->timing = 100;
		ret = i2c_master_recv(client, I2CDMABuf_pa, len);
		if (ret < 0)
			return ret;

		for (i = 0; i < len; i++)
			buf[i] = I2CDMABuf_va[i];
	}

	return ret;
}

static int i2c_msg_transfer(struct i2c_client *client, struct i2c_msg *msgs, int count)
{
	int i = 0;
	int ret = 0;

	for (i = 0; i < count; i++) {
		if (msgs[i].flags & I2C_M_RD) {
			ret = i2c_dma_read(client, msgs[i].buf, msgs[i].len);
			client->addr = (client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG; /* client mode restore */
		} else {
			ret = i2c_dma_write(client, msgs[i].buf, msgs[i].len);
			client->addr = (client->addr & I2C_MASK_FLAG) | I2C_ENEXT_FLAG; /* client mode restore */
		}

		if (ret < 0)
			return ret;
	}

	return 0;
}
#endif

static int atmf_i2c_read(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen)
{
#if defined(TOUCH_PLATFORM_QCT)

	int result = 0;
	int ret = 0;

	struct i2c_msg msgs[2] = {
		{
		    .addr = client->addr,
		    .flags = 0,
		    .len = regLen,
		    .buf = reg,
		},
		{
		    .addr = client->addr,
		    .flags = I2C_M_RD,
		    .len = dataLen,
		    .buf = buf,
		},
	};

	ret = i2c_transfer(client->adapter, msgs, 2);
	if (ret < 0)
		result = -1;

	return result;

#elif defined(TOUCH_PLATFORM_MTK)

	if (dataLen <= MAX_I2C_TRANSFER_SIZE) {
		int result = 0;
		int ret = 0;

		struct i2c_msg msgs[2] = {
		    {
			.addr = client->addr,
			.flags = 0,
			.len = regLen,
			.buf = reg,
		    },
		    {
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = dataLen,
			.buf = buf,
		    },
		};

		ret = i2c_msg_transfer(client, msgs, 2);
		if (ret < 0)
			result = -1;

		return result;
	} else {
		int result = 0;
		int ret = 0;
		int i = 0;

		int msgCount = 0;
		int remainedDataLen = 0;

		struct i2c_msg *msgs = NULL;

		remainedDataLen = dataLen%MAX_I2C_TRANSFER_SIZE;

		msgCount = 1; /* msg for register */
		msgCount += (int)(dataLen/MAX_I2C_TRANSFER_SIZE); /* add msgs for data read */
		if (remainedDataLen > 0)
			msgCount += 1; /* add msg for remained data */

		msgs = (struct i2c_msg *)kcalloc(msgCount, sizeof(struct i2c_msg), GFP_KERNEL);
		if (msgs != NULL)
			memset(msgs, 0x00, sizeof(struct i2c_msg));
		else
			return -1;

		msgs[0].addr = client->addr;
		msgs[0].flags = 0;
		msgs[0].len = regLen;
		msgs[0].buf = reg;

		for (i = 1; i < msgCount; i++) {
			msgs[i].addr = client->addr;
			msgs[i].flags = I2C_M_RD;
			msgs[i].len = MAX_I2C_TRANSFER_SIZE;
			msgs[i].buf = buf + MAX_I2C_TRANSFER_SIZE * (i-1);
		}

		if (remainedDataLen > 0)
			msgs[msgCount-1].len = remainedDataLen;

		ret = i2c_msg_transfer(client, msgs, msgCount);
		if (ret < 0)
			result = -1;

		kfree(msgs);

		return result;
	}

#else
	#error "Platform should be defined"
#endif

}
static int atmf_i2c_write(struct i2c_client *client, u8 *reg, int regLen, u8 *buf, int dataLen)
{
	int result = 0;
	int ret = 0;

	u8 *pTmpBuf = NULL;

	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = 0,
		.len = (regLen+dataLen),
		.buf = NULL,
	};

	#if defined(TOUCH_PLATFORM_MTK)
	if (dataLen > MAX_I2C_TRANSFER_SIZE) {
		PINFO("data length to write is exceed the limit ( length = %d, limit = %d )", dataLen,
				MAX_I2C_TRANSFER_SIZE);
		PINFO("You should implement to overcome this problem like read");
		return -1;
	}
	#endif


	pTmpBuf = (u8 *)kcalloc(1, regLen+dataLen, GFP_KERNEL);
	if (pTmpBuf != NULL)
		memset(pTmpBuf, 0x00, regLen+dataLen);
	else
		return -1;

	memcpy(pTmpBuf, reg, regLen);
	memcpy((pTmpBuf+regLen), buf, dataLen);

	msg.buf = pTmpBuf;

	#if defined(TOUCH_PLATFORM_QCT)
	ret = i2c_transfer(client->adapter, &msg, 1);
	#elif defined(TOUCH_PLATFORM_MTK)
	ret = i2c_msg_transfer(client, &msg, 1);
	#else
	#error "Platform should be defined"
	#endif

	if (ret < 0)
		result = -1;

	kfree(pTmpBuf);

	return result;
}

unsigned char read_eflash_page(unsigned char flag, unsigned char * page_addr, unsigned char * rdata, struct i2c_client *client)
{
	unsigned char paddr[2];
	int ret = 0;
	u8 addr = 0;
	int i = 0;

	if(flag != FLAG_FUSE)
	{
		paddr[0] = page_addr[1];
		paddr[1] = page_addr[0];
	}
	else	//Extra User Memory
	{
		paddr[0] = 0x00;
		paddr[1] = 0x00;
	}

	addr = ADDR_EFLA_PAGE_L;
	ret = atmf_i2c_write(client, &addr, 1, paddr, 2);
	if( ret < 0){
	    PINFO("[%s] atmf fail to write", __func__);
	    return RTN_FAIL;
	}

	if(flag != FLAG_FUSE)
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_RD);
	else
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EUM_RD);

	if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) return RTN_TIMEOUT;

	addr = 0x00;
	ret = atmf_i2c_read(client, &addr, 1, rdata, SZ_PAGE_DATA);
	if (ret < 0) {
	    PINFO("[%s] efylash page Read Fail, ret = %d", __func__, ret);
	    PINFO("================================================");
	    for(i = 0; i < SZ_PAGE_DATA; i++) {
		PINFO("i = %d: rdata = 0x%02x", i, rdata[i]);
	    }
	    PINFO("================================================");
	    return RTN_FAIL;
	}
	return RTN_SUCC;
}

static void atmf_disable_irq(void)
{
    mt_eint_mask(126);
    mt_eint_mask(127);
    PINFO("ATMF IRQ Disabled ");
}

static void atmf_enable_irq(void)
{
    mt_eint_unmask(126);
    mt_eint_unmask(127);
    PINFO("ATMF IRQ Enabled ");
}

static void onoff_touchkey(struct atmf04_data *data, int onoff_mode)
{
    int nparse_mode;

    nparse_mode = onoff_mode;
    PINFO("onoff_touch_key: nparse_mode [%d]",nparse_mode);

    if (nparse_mode == ENABLE_TOUCHKEY_PINS) {
		mt_set_gpio_out(ATMF_GPIO_EN, GPIO_OUT_ZERO);
		mdelay(250);
		if (!on_touchkey){
			atmf_enable_irq();
		}
		on_touchkey = true;
    }
    if (nparse_mode == DISABLE_TOUCHKEY_PINS) {
		if (on_touchkey){
			atmf_disable_irq();
		}
		mt_set_gpio_out(ATMF_GPIO_EN, GPIO_OUT_ONE);
		on_touchkey = false;
    }
}

int initial_reg_write(struct i2c_client *client)
{
    unsigned char i;
    u8 retry = 0;
    u8 max_retry = 10;

    for(i=0;i<INIT_I2C_CNT;i++)
	i2c_smbus_write_byte_data(client, i2c_addr_set[i],i2c_data_set[i]);

    //Save Initial code value to eflash
    i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);	//EFL Write
    //Wait until write end
    while((i2c_smbus_read_byte_data(client, I2C_ADDR_SYS_STAT) &0x01) ==0x01)
    {
	PINFO("[%s] wait until ATMF init code flashing..",__func__);
	mdelay(10);
	if(retry > max_retry) {
	    PINFO("[%s][%d] atmf init code flashing time out...cnt = %u ",__func__, __LINE__, retry);
	    return -1;
	}
	retry++;
    }   //Wait until done
    //SW RST
    i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL,0x01);
    mdelay(250);

    return 0;
}
unsigned char load_firmware(struct atmf04_data *data, struct i2c_client *client, const char *name)
{
	const struct firmware *fw = NULL;
	unsigned char rtn = 0;
	int ret, i, count = 0;
	int max_page;
	unsigned short main_version, sub_version;
	unsigned char page_addr[2];
	unsigned char fw_version, ic_fw_version, page_num;
	int version_addr;
	u8 i2c_init_check[INIT_I2C_CNT] = {0,};

	PINFO("ATMF04 Firmware Update Start ");

	atmf_disable_irq();
	mt_set_gpio_out(ATMF_GPIO_EN, GPIO_OUT_ZERO);

	ret = request_firmware(&fw, name, &data->client->dev);
	if (ret) {
		PINFO("Unable to open bin [%s]  ret %d", name, ret);
		return 1;
	} else {
		PINFO("Open bin [%s] ret : %d ", name, ret);
	}

	/* get F/W Info */
	max_page = (fw->size)/SZ_PAGE_DATA;
	version_addr = (fw->size)-SZ_PAGE_DATA;
	fw_version = MK_INT(fw->data[version_addr], fw->data[version_addr+1]);
	page_num = fw->data[version_addr+3];
	PINFO("#fw version : %d.%d, fw_version : %d, page_num : %d#", fw->data[version_addr], fw->data[version_addr+1], fw_version, page_num);

	/* get IC F/W Info */
	main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
	sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
	ic_fw_version = MK_INT(main_version, sub_version);
	PINFO("#ic version : %d.%d, ic_fw_version : %d#", main_version, sub_version, ic_fw_version);

	if (fw_version > ic_fw_version || fw->data[version_addr] > main_version) {
	    /* IC Download Mode Change */
	    chg_mode(ON, client);

	    /* fuse data process */
	    page_addr[0] = 0x00;
	    page_addr[1] = 0x00;

	    rtn = read_eflash_page(FLAG_FUSE, page_addr, fuse_data, client);
	    if (rtn != RTN_SUCC) {
		PERR("read eflash page fail!");
		chg_mode(OFF, client);
		goto err_exit;		/* fuse read fail */
	    }

	    fuse_data[51] |= 0x80;

	    rtn = write_eflash_page(FLAG_FUSE, page_addr, fuse_data, client);
	    if (rtn != RTN_SUCC) {
		PERR("write eflash page fail!");
		chg_mode(OFF, client);
		goto err_exit;		/* fuse write fail */
	    }

	    /* firmware write process */
	    rtn = erase_eflash(client);
	    if(rtn != RTN_SUCC) {
		PINFO("earse fail");
		chg_mode(OFF, client);
		goto err_exit;		//earse fail
	    }

	    while(count < page_num) {
		//PINFO("%d\n",count);
		for(i=0; i < SZ_PAGE_DATA; i++) {
		    i2c_smbus_write_byte_data(client, i, fw->data[i + (count*SZ_PAGE_DATA)]);
		    //PINFO("%d : %x ",i + (count*SZ_PAGE_DATA),fw->data[i + (count*SZ_PAGE_DATA)]);
		}
		//PINFO("\n");
		page_addr[1] = (unsigned char)((count & 0xFF00) >> 8);
		page_addr[0] = (unsigned char)(count & 0x00FF);

		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_L, page_addr[0]);
		i2c_smbus_write_byte_data(client, ADDR_EFLA_PAGE_H, page_addr[1]);

		/*Eflash write command 0xFC -> 0x01 Write*/
		i2c_smbus_write_byte_data(client, ADDR_EFLA_CTRL, CMD_EFL_L_WR);

		if((count % 10) == 0 || (count == (page_num-1)))
		    PINFO("[%s] ATMF F/W Download <%d / %d > ", __func__, count, page_num-1);

		if(chk_done(FL_EFLA_TIMEOUT_CNT, client) == RTN_TIMEOUT) {
		    PINFO("ATMF Check Done Fail ");
		    chg_mode(OFF, client);
		    rtn =  RTN_TIMEOUT;
		    goto err_exit;
		}
		count++;
	    }

	    /* IC Download Mode Change */
	    chg_mode(OFF, client);

	    /* Power Off/On / Delay 250 */
	    PINFO("[%s] Flashing Initial Code...after f/w download ", __func__);
	    mt_set_gpio_out(ATMF_GPIO_EN, GPIO_OUT_ONE);
	    mdelay(20);
	    mt_set_gpio_out(ATMF_GPIO_EN, GPIO_OUT_ZERO);
	    mdelay(250);
	} else {
	    PINFO("No Need to Update. Firmware version is lower than ic.(Or same version)");
	}
	/* End of Firmware Download */

	/* Check Init Code */
	for(i=0; i<INIT_I2C_CNT; i++) {
	    i2c_init_check[i] = i2c_smbus_read_byte_data(client, i2c_addr_set[i]);
	    PINFO("[%s] checking init code..0x%02x, read : 0x%02x, set : 0x%02x ", __func__, i2c_addr_set[i],  i2c_init_check[i], i2c_data_set[i]);

	    if(i2c_init_check[i] != i2c_data_set[i]){
		PINFO("[%s] ATMF Init Code Difference Reflash Init Code", __func__);
		rtn = initial_reg_write(client);
		if (rtn < 0) {
		    PINFO(" ATMF04 Fail to rewrite initial_reg ");
		    goto err_exit;
		}
		for(i=0; i<INIT_I2C_CNT; i++) {
		    i2c_init_check[i] = i2c_smbus_read_byte_data(client, i2c_addr_set[i]);
		    PINFO("[%s] after checking init code..0x%02x, read : 0x%02x, set : 0x%02x ", __func__, i2c_addr_set[i],  i2c_init_check[i], i2c_data_set[i]);
		}
		break;
	    }
	}

err_exit:
	onoff_touchkey(data,DISABLE_TOUCHKEY_PINS);
	mdelay(10);
	onoff_touchkey(data,ENABLE_TOUCHKEY_PINS);
	release_firmware(fw);
	PINFO("[%s] end", __func__);

	return rtn;
}

static ssize_t atmf04_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int loop;

	int ret = 0;

    for (loop = 0; loop < CNT_INITCODE; loop++) {
        PINFO("###### [0x%x][0x%x]###", InitCodeAddr[loop], i2c_smbus_read_byte_data(client, InitCodeAddr[loop]));
    }
    return ret;
}

static ssize_t atmf04_store_reg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned char loop;

	for (loop = 0; loop < CNT_INITCODE; loop++) {
		i2c_smbus_write_byte_data(client, InitCodeAddr[loop], InitCodeVal[loop]);
		PINFO("##[0x%x][0x%x]##", InitCodeAddr[loop], InitCodeVal[loop]);
	}
		i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x01);
	return count;
}


static ssize_t atmf04_store_onoff_touch_key(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
    struct atmf04_data *data = dev_get_drvdata(dev);
	unsigned long val;
	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
		if(val == ON_TOUCHKEY) {
			onoff_touchkey(data,ENABLE_TOUCHKEY_PINS);
			PINFO("Store ON_TOUCHKEY menu[%d], back[%d]",data->platform_data->irq_gpio_menu, data->platform_data->irq_gpio_back);
		}
		else if (val == OFF_TOUCHKEY) {
			onoff_touchkey(data,DISABLE_TOUCHKEY_PINS);
			PINFO("Store OFF_TOUCHKEY menu[%d], back[%d]",data->platform_data->irq_gpio_menu, data->platform_data->irq_gpio_back);
		}
	return count;
}

static ssize_t atmf04_store_regreset(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct atmf04_data *data = i2c_get_clientdata(client);
	short tmp;
	short cs_per[2], cs_per_result;
	short cr_duty[2], cs_duty[2], cr_duty_val, cs_duty_val;

	int ret;

	ret = i2c_smbus_write_byte_data(client, I2C_ADDR_SYS_CTRL, 0x02);
	if(ret)
		PINFO("[%d]i2c_write_fail",data-> platform_data->irq_gpio_menu);

	// Debug Log Print
	cs_per[0] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_H);
	cs_per[1] = i2c_smbus_read_byte_data(client,I2C_ADDR_PER_L);
	tmp = MK_INT(cs_per[0], cs_per[1]);
	cs_per_result = tmp / 8;    // BIT_PERCENT_UNIT;

	cr_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_H);
	cr_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CR_DUTY_L);
	cr_duty_val = MK_INT(cr_duty[1], cr_duty[0]);

	cs_duty[1] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_H);
	cs_duty[0] = i2c_smbus_read_byte_data(client, I2C_ADDR_CS_DUTY_L);
	cs_duty_val = MK_INT(cs_duty[1], cs_duty[0]);

	PINFO("[%d] Result : %2d %6d %6d", data-> platform_data->irq_gpio_menu, cs_per_result, cr_duty_val, cs_duty_val);

	return count;
}

int get_bit(unsigned short x, int n) {
	return (x & (1 << n)) >> n;
}

static ssize_t atmf04_store_firmware(struct device *dev,
                     struct device_attribute *attr,
                     const char *buf, size_t count)
{
	const char *fw_name = NULL;
	struct atmf04_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = to_i2c_client(dev);

	fw_name = data->platform_data->fw_name;
	load_firmware(data, client, fw_name);
	return count;
}

static ssize_t atmf04_show_version(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
    unsigned short main_version, sub_version;
    unsigned char ic_fw_version;
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = dev_get_drvdata(dev);
    int lcd_status = 0;
    int ret = 0;

    lcd_status = atomic_read(&data->lcd_status);
    if(lcd_status == ATMF04_LCD_OFF) {
	PINFO("LCD Off, can not read version : %d", lcd_status);
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "[LCD OFF] Can not read version \n");
	return ret;
    }

    main_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_MAIN);
    sub_version = i2c_smbus_read_byte_data(client, I2C_ADDR_PGM_VER_SUB);
    ic_fw_version = MK_INT(main_version, sub_version);
    PINFO("###########ic version : %d.%d, ic_fw_version : %d###########", main_version, sub_version, ic_fw_version);
    return snprintf(buf+ret, PAGE_SIZE-ret, "IC version : %d.%d \n",main_version, sub_version);
}

static ssize_t atmf04_show_check_init(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    u8 i2c_init_check[INIT_I2C_CNT] = {0,};
    int ret = 0;
    int i=0;
    int lcd_status = 0;

    lcd_status = atomic_read(&data->lcd_status);
    if(lcd_status == ATMF04_LCD_OFF) {
	PINFO("LCD Off, can not read init code. %d", lcd_status);
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "[LCD OFF] Can not read init code \n");
	return ret;
    }

    for(i=0; i<INIT_I2C_CNT; i++) {
	i2c_init_check[i] = i2c_smbus_read_byte_data(client, i2c_addr_set[i]);
	PINFO("[%s] checking init code..0x%02x, 0x%02x : 0x%02x \n", __func__, i2c_addr_set[i],  i2c_init_check[i], i2c_data_set[i]);
	ret += snprintf(buf+ret, PAGE_SIZE - ret, "0x%02x, 0x%02x : 0x%02x \n", i2c_addr_set[i],  i2c_init_check[i], i2c_data_set[i]);
    }

    return ret;
}
/*
static ssize_t show_atmf04_125(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);

    int gpio_state = 0;

    gpio_state = mt_get_gpio_in(125 | 0x80000000);

    return sprintf(buf, "%d\n", gpio_state);
}

static ssize_t show_atmf04_126(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);

    int gpio_state = 0;

    gpio_state = mt_get_gpio_in(126 | 0x80000000);

    return sprintf(buf, "%d\n", gpio_state);
}

static ssize_t show_atmf04_127(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);

    int gpio_state = 0;

    gpio_state = mt_get_gpio_in(127 | 0x80000000);

    return sprintf(buf, "%d\n", gpio_state);
}

static ssize_t show_check_yym(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct atmf04_data *data = i2c_get_clientdata(client);
    int ret = 0;
    int gpio_state = 0;
    u8 enable = 0;

    //gpio_state = mt_get_gpio_in(127 | 0x80000000);
    enable = i2c_smbus_read_byte_data(client, 0x01);
    ret += sprintf(buf,"before %d \n", enable);

    return ret;
}

*/

static DEVICE_ATTR(onoff,        0664, NULL, atmf04_store_onoff_touch_key);
static DEVICE_ATTR(reg_ctrl,     0664, atmf04_show_reg, atmf04_store_reg);
static DEVICE_ATTR(regreset,     0664, NULL, atmf04_store_regreset);
static DEVICE_ATTR(download,     0664, NULL, atmf04_store_firmware);
static DEVICE_ATTR(version,      0664, atmf04_show_version, NULL);
static DEVICE_ATTR(check_init,    0664, atmf04_show_check_init, NULL);
/*
static DEVICE_ATTR(check_125,    0664, show_atmf04_125, NULL);
static DEVICE_ATTR(check_126,    0664, show_atmf04_126, NULL);
static DEVICE_ATTR(check_127,    0664, show_atmf04_127, NULL);
static DEVICE_ATTR(check_yym,    0664, show_check_yym, NULL);
*/

static struct attribute *atmf04_attributes[] = {
    &dev_attr_onoff.attr,
    &dev_attr_reg_ctrl.attr,
    &dev_attr_regreset.attr,
    &dev_attr_download.attr,
    &dev_attr_version.attr,
    &dev_attr_check_init.attr,
    /*
       &dev_attr_check_125,
       &dev_attr_check_126,
       &dev_attr_check_127,
       &dev_attr_check_yym,
       */
    NULL,
};

static struct attribute_group atmf04_attr_group = {
    .attrs = atmf04_attributes,
};

static void atmf04_reschedule_menu_key_work(struct atmf04_data *data,
				     unsigned long delay)
{
	int ret;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//dev_err(&client->dev, "atmf04_reschedule_menu_key_work : set wake lock timeout!\n");
	cancel_delayed_work(&data->dmwork);
	ret = queue_delayed_work(atmf04_workqueue, &data->dmwork, delay);
	if (ret < 0)
		PINFO("queue_work fail, ret = %d", ret);
}

static void atmf04_reschedule_back_key_work(struct atmf04_data *data,
				     unsigned long delay)
{
	int ret;
	/*
	 * If work is already scheduled then subsequent schedules will not
	 * change the scheduled time that's why we have to cancel it first.
	 */
	//dev_err(&client->dev, "atmf04_reschedule_back_key_work : set wake lock timeout!\n");
	cancel_delayed_work(&data->dbwork);
	ret = queue_delayed_work(atmf04_workqueue, &data->dbwork, delay);
	if (ret < 0)
		PINFO("queue_work fail, ret = %d", ret);
}

static irqreturn_t atmf04_interrupt_menu(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct atmf04_data *data = i2c_get_clientdata(client);
	int lcd_status = 0;

	lcd_status = atomic_read(&data->lcd_status);
	if(lcd_status == ATMF04_LCD_OFF) {
	    PINFO("LCD Off, Ignored IRQ menu %d", lcd_status);
	    return IRQ_HANDLED;
	}

	//dev_err(&client->dev,"atmf04_interrupt_menu\n");
	atmf04_reschedule_menu_key_work(data, 0);
	return IRQ_HANDLED;
}

static irqreturn_t atmf04_interrupt_back(int vec, void *info)
{
	struct i2c_client *client = (struct i2c_client *)info;
	struct atmf04_data *data = i2c_get_clientdata(client);
	int lcd_status = 0;

	lcd_status = atomic_read(&data->lcd_status);
	if(lcd_status == ATMF04_LCD_OFF) {
	    PINFO("LCD Off, Ignored IRQ back %d", lcd_status);
	    return IRQ_HANDLED;
	}

	//dev_err(&client->dev,"atmf04_interrupt_back\n");
	atmf04_reschedule_back_key_work(data, 0);

	return IRQ_HANDLED;
}

static void atmf04_work_menu_handler(struct work_struct *work)
{
	struct atmf04_data *data = container_of(work, struct atmf04_data, dmwork.work);

	int menu_gpio = 0;
	if(probe_end_flag == true) {
		menu_gpio = mt_get_gpio_in(ATMF_GPIO_MENU);

		if(menu_gpio) {
		    input_report_key(data->input_dev_cap, KEY_MENU, PRESS);
		    input_sync(data->input_dev_cap);
		    PINFO("[Touch] Menu Key Release. ");
		} else {
		    input_report_key(data->input_dev_cap, KEY_MENU, RELEASE);
		    input_sync(data->input_dev_cap);
		    PINFO("[Touch] Menu Key Press. ");
		}
	} else{
		PINFO("probe_end_flag = False[%d]",probe_end_flag);
	}
}

static void atmf04_work_back_handler(struct work_struct *work)
{
	struct atmf04_data *data = container_of(work, struct atmf04_data, dbwork.work);

	int back_gpio = 0;
	if(probe_end_flag == true) {
		back_gpio = mt_get_gpio_in(ATMF_GPIO_BACK);

		if(back_gpio) {
		    input_report_key(data->input_dev_cap, KEY_BACK, PRESS);
		    input_sync(data->input_dev_cap);
		    PINFO("[Touch] Back Key Release.");
		} else {
		    input_report_key(data->input_dev_cap, KEY_BACK, RELEASE);
		    input_sync(data->input_dev_cap);
		    PINFO("[Touch] Back Key Press.");
		}
	} else{
		PINFO("probe_end_flag = False[%d]",probe_end_flag);
	}
}


static int touch_key_platform_hw_init(struct i2c_client *client)
{
	int error = 0;

	//error = touch_key_regulator_configure(data, true);

	error = mt_set_gpio_mode(ATMF_GPIO_EN, GPIO_MODE_00);
	if(error) {
		PINFO("chip_enable request fail = %d",error);
		return error;
	}

	mt_set_gpio_dir(ATMF_GPIO_EN, GPIO_DIR_OUT);
	PINFO("gpio CHIP_EN direction output ok");

	error = mt_set_gpio_mode(ATMF_GPIO_MENU, GPIO_MODE_00);
	if(error) {
		PINFO("chip_enable request fail = %d",error);
		return error;
	}

	mt_set_gpio_dir(ATMF_GPIO_MENU, GPIO_DIR_IN);
	PINFO("gpio MENU direction input ok");

	error = mt_set_gpio_mode(ATMF_GPIO_BACK, GPIO_MODE_00);
	if(error) {
		PINFO("chip_enable request fail = %d",error);
		return error;
	}

	mt_set_gpio_dir(ATMF_GPIO_BACK, GPIO_DIR_IN);
	PINFO("gpio BACK direction input ok");

	PINFO("touch_key_platform_hw_init entered");
	return 0;
}

static void touch_key_platform_hw_exit(struct i2c_client *client)
{
	//struct atmf04_data *data = i2c_get_clientdata(client);;

	//touch_key_regulator_configure(data, false);

	//if (gpio_is_valid(data->platform_data->irq_gpio))
	//	gpio_free(data->platform_data->irq_gpio);
	PINFO("touch_key_platform_hw_exit entered");
}

static int touch_key_parse_dt(struct device *dev,
			   struct atmf04_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	int ret, err = 0;
	struct touch_key_dt_to_pdata_map *itr;
	struct touch_key_dt_to_pdata_map map[] = {
		{"Adsemicon,fw_name",              &pdata->fw_name,             DT_SUGGESTED,   DT_STRING,  0},
		/* add */
		{NULL,				NULL,				0,		0,		0},
	};

	for (itr = map; itr->dt_name ; ++itr) {
		switch (itr->type) {
		case DT_GPIO:
			ret = of_get_named_gpio(np, itr->dt_name, 0);
			if (ret >= 0) {
				*((int *) itr->ptr_data) = ret;
				ret = 0;
			}
			break;
		case DT_U32:
			ret = of_property_read_u32(np, itr->dt_name,
						   (u32 *) itr->ptr_data);
			break;
		case DT_BOOL:
			*((bool *) itr->ptr_data) =
				of_property_read_bool(np, itr->dt_name);
			ret = 0;
			break;
		case DT_STRING:
			ret = of_property_read_string(np, itr->dt_name, itr->ptr_data);
			break;
		default:
			PINFO("%d is an unknown DT entry type",
			      itr->type);
			ret = -EBADE;
		}

		PINFO("DT entry ret:%d name:%s val:%d",
		      ret, itr->dt_name, *((int *)itr->ptr_data));

		if (ret) {
			*((int *)itr->ptr_data) = itr->default_val;

			if (itr->status < DT_OPTIONAL) {
				PINFO("Missing '%s' DT entry",
				      itr->dt_name);

				/* cont on err to dump all missing entries */
				if (itr->status == DT_REQUIRED && !err)
					err = ret;
			}
		}
	}

	/* set functions of platform data */
	pdata->init = touch_key_platform_hw_init;
	pdata->exit = touch_key_platform_hw_exit;

	return err;
	return 0;
}

int touch_key_notifier_call_chain(unsigned long val, void *v)
{
	PINFO("touch_key_notifier_call_chain in");
	return blocking_notifier_call_chain(&touch_key_notifier_list, val, v);
}
EXPORT_SYMBOL(touch_key_notifier_call_chain);

int touch_key_register_client(struct notifier_block *nb)
{
	PINFO("touch_key_register_client in");
	return blocking_notifier_chain_register(&touch_key_notifier_list, nb);
}
EXPORT_SYMBOL(touch_key_register_client);

int touch_key_unregister_client(struct notifier_block *nb)
{
	PINFO("touch_key_unregister_client in");
	return blocking_notifier_chain_unregister(&touch_key_notifier_list, nb);
}
EXPORT_SYMBOL(touch_key_unregister_client);

static int touch_key_common_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct atmf04_data *data1 = container_of(self, struct atmf04_data, common_notif);
	PINFO("touch_key_common_notifier_callback start");
	mutex_lock(&data1->update_lock);

	switch (event) {
	    case ATMF04_LCD_EVENT_ON:
		PINFO("ATMF04_LCD_EVENT_ON");
		atomic_set(&data1->lcd_status, ATMF04_LCD_ON);
		onoff_touchkey(data1,ENABLE_TOUCHKEY_PINS);
		break;
	    case ATMF04_LCD_EVENT_OFF:
		PINFO("ATMF04_LCD_EVENT_OFF");
		atomic_set(&data1->lcd_status, ATMF04_LCD_OFF);
		onoff_touchkey(data1,DISABLE_TOUCHKEY_PINS);
		break;
	    default:
		break;
	}

	mutex_unlock(&data1->update_lock);
	PINFO("touch_key_common_notifier_callback end");
	return 0;
}


static int atmf04_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct atmf04_data *data;
#ifdef CONFIG_OF
	struct atmf04_platform_data *platform_data;
#endif
	int err = 0;
	struct device_node *node = NULL;
	unsigned int atmf_irq_menu = 0;
	unsigned int atmf_irq_back = 0;

	PINFO("HW_REV : [%d]", lge_get_board_revno());
	/*check lge board revision*/
#if 0
	if(lge_get_board_revno() < HW_REV_EVB2){
		PINFO("HW_REV : %d, does not support this sensor\n", lge_get_board_revno());
		return -ENODEV;
	}
#endif
	/*check i2c_functionality*/
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		PINFO("i2c_check_functionality fail");
		return -EIO;
	}

	/*mem alloc*/
	data = devm_kzalloc(&client->dev, sizeof(struct atmf04_data), GFP_KERNEL);
	if (!data) {
		PINFO("devm_kzalloc fail");
		return -ENOMEM;
	}
	PINFO("ATMF04 probe, kzalloc complete");

#ifdef CONFIG_OF
	if (client->dev.of_node) {
		platform_data = devm_kzalloc(&client->dev,
					     sizeof(struct atmf04_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory");
			return -ENOMEM;
		}
		data->platform_data = platform_data;
		client->dev.platform_data = platform_data;
		err = touch_key_parse_dt(&client->dev, platform_data);
		if (err)
			return err;

	} else {
		platform_data = client->dev.platform_data;
	}
#endif
	data->client = client;
	atmf04_i2c_client = client;
	i2c_set_clientdata(client, data);

	err = dma_allocation();
	if ( err < 0 ) {
	    PINFO(" ATMF dma allocation fail");
	    return -ENOMEM;
	}

#ifdef CONFIG_OF
	/* h/w initialization */
	if (platform_data->init)
		err = platform_data->init(client);

	if (platform_data->power_on)
		err = platform_data->power_on(client, true);
#endif

	client->adapter->retries = 15;

	if (client->adapter->retries == 0)
		goto exit;

	atomic_set(&data->i2c_status, ATMF04_STATUS_RESUME);
	atomic_set(&data->lcd_status, ATMF04_LCD_ON);

	mutex_init(&data->update_lock);

	wake_lock_init(&data->ps_wlock, WAKE_LOCK_SUSPEND, "touch_key_wakelock");
	INIT_DELAYED_WORK(&data->dmwork, atmf04_work_menu_handler);
	INIT_DELAYED_WORK(&data->dbwork, atmf04_work_back_handler);

	/* Register IRQ handler */
	node = of_find_compatible_node(NULL, NULL, "adsemicon,atmf04");

	if (node) {
		atmf_irq_menu = irq_of_parse_and_map(node, 0);
		atmf_irq_back = irq_of_parse_and_map(node, 1);
		platform_data->irq_gpio_menu = atmf_irq_menu;
		platform_data->irq_gpio_back = atmf_irq_back;

		PINFO("**Touch_Key irq menu = %d", platform_data->irq_gpio_menu);
		PINFO("**Touch_Key irq back = %d", platform_data->irq_gpio_back);

		err = request_irq(atmf_irq_menu, atmf04_interrupt_menu, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"ATMF04-menu-eint", (void *)client);
		if (err > 0) {
			goto exit_free_dev_cap;
		}

		err = request_irq(atmf_irq_back, atmf04_interrupt_back, IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				"ATMF04-back-eint", (void *)client);
		if (err > 0) {
			goto exit_free_dev_cap;
		}
	} else {
		PINFO("Can't find node");
		goto exit_irq_init_failed;
	}

	/* pin ctrl*/
	PINFO("pin ctrl start");
	data->atmf04_pinctrl = devm_pinctrl_get(&(client->dev));
	if (IS_ERR(data->atmf04_pinctrl)) {
	    if (PTR_ERR(data->atmf04_pinctrl) == -EPROBE_DEFER) {
		PINFO("atmf04 pin ctrl = -EPROBE_DEFER");
	    }
	    PINFO("atmf target does not use pin ctlr");
	    data->atmf04_pinctrl = NULL;
	    goto exit_free_irq;
	}

	if(data->atmf04_pinctrl) {
	    data->atmf04_active = pinctrl_lookup_state(data->atmf04_pinctrl, "touch_key_active");
	    if (IS_ERR(data->atmf04_active)) {
		PINFO("pin ctrl active error");
		goto exit_pin_ctrl;
	    }

	    data->atmf04_suspend = pinctrl_lookup_state(data->atmf04_pinctrl, "touch_key_suspend");
	    if (IS_ERR(data->atmf04_suspend)) {
		PINFO("pin ctrl suspend error");
		goto exit_pin_ctrl;
	    }

	    if(data->atmf04_active){
		err = pinctrl_select_state(data->atmf04_pinctrl, data->atmf04_active);
		if(err < 0)
		    PINFO("can not set atmf04 pin active");

	    } else {
		PINFO("pin ctrl active null");
	    }
	} else {
	    PINFO("data->atmf04_pinctrl is NULL");
	}

	PINFO("pin ctrl done");

	/* Allocate Input Device */
	data->input_dev_cap = input_allocate_device();
	if (!data->input_dev_cap) {
	    err = -ENOMEM;
	    PINFO("Failed to allocate input device touch_key !\n");
	    goto exit_pin_ctrl;
	}

	/* Set Event bit & Key bit */
	set_bit(EV_ABS, data->input_dev_cap->evbit);
	set_bit(EV_SYN, data->input_dev_cap->evbit);
	set_bit(EV_KEY, data->input_dev_cap->evbit);
	set_bit(KEY_MENU, data->input_dev_cap->keybit);
	set_bit(KEY_BACK, data->input_dev_cap->keybit);

	input_set_abs_params(data->input_dev_cap, ABS_DISTANCE, 0, 1, 0, 0);

	data->input_dev_cap->name = ATMF04_DRV_NAME;
	data->input_dev_cap->id.bustype = BUS_I2C;

	/* Input Device Register */
	err = input_register_device(data->input_dev_cap);
	if (err) {
		err = -ENOMEM;
		PINFO("Unable to register input device touch_key(%s)",
				data->input_dev_cap->name);
		goto exit_pin_ctrl;
	}

	/* Create Sysfs Group */
	err = sysfs_create_group(&client->dev.kobj, &atmf04_attr_group);
	if (err) {
		PINFO("touch_key sysfs create fail!");
		goto exit_input_register;
	}

	/* firmware update */
	if (data->platform_data->fw_name) {
		err = load_firmware(data, client, data->platform_data->fw_name);
		if (err) {
			PINFO("Failed to request touch_key firmware");
			//goto exit_free_irq;
			goto exit_sysfs_create;
		}
	}

#if defined( CONFIG_FB )
	data->common_notif.notifier_call = touch_key_common_notifier_callback;
	touch_key_register_client(&data->common_notif);
	PINFO("notifier call back function registered");
#endif

	/* default touch key off */
	probe_end_flag = true;

	/* Power On Seq */
	onoff_touchkey(data, ENABLE_TOUCHKEY_PINS);

	PINFO("touch_key driver probe done.");

	return 0;

exit_sysfs_create:
	sysfs_remove_group(&client->dev.kobj, &atmf04_attr_group);
exit_input_register:
	input_unregister_device(data->input_dev_cap);
exit_pin_ctrl:
	devm_pinctrl_put(data->atmf04_pinctrl);
exit_free_irq:
	free_irq(data->platform_data->irq_gpio_menu, client);
	free_irq(data->platform_data->irq_gpio_back, client);
exit_irq_init_failed:
	wake_lock_destroy(&data->ps_wlock);
	mutex_destroy(&data->update_lock);
exit_free_dev_cap:
	atmf04_dma_free();
	devm_kfree(&client->dev, platform_data);
	devm_kfree(&client->dev, data);
exit:
	PINFO("touch_key driver probe fail");
	return err;
}

static int atmf04_remove(struct i2c_client *client)
{
	struct atmf04_data *data = i2c_get_clientdata(client);
	struct atmf04_platform_data *pdata = data->platform_data;

	if (pdata->power_on)
		pdata->power_on(client, false);

	if (pdata->exit)
		pdata->exit(client);

	onoff_touchkey(data, DISABLE_TOUCHKEY_PINS);

	touch_key_unregister_client(&data->common_notif);
	sysfs_remove_group(&client->dev.kobj, &atmf04_attr_group);

	input_unregister_device(data->input_dev_cap);
	devm_pinctrl_put(data->atmf04_pinctrl);

	free_irq(pdata->irq_gpio_menu, client);
	free_irq(pdata->irq_gpio_back, client);

	wake_lock_destroy(&data->ps_wlock);
	mutex_destroy(&data->update_lock);

	atmf04_dma_free();
	devm_kfree(&client->dev, pdata);
	devm_kfree(&client->dev, data);

	PINFO("ATMF04 remove");
	return 0;
}

static int atmf04_pm_suspend(struct device *dev)
{
    struct atmf04_data * data = dev_get_drvdata(dev);
    int err = 0;
    if(data->atmf04_suspend){
	err = pinctrl_select_state(data->atmf04_pinctrl, data->atmf04_suspend);
	if(err < 0)
	    PINFO("can not set atmf04 pin suspend");
    } else {
	PINFO("pin ctrl suspend null");
    }

    PINFO("atmf04 pm suspend");
    return 0;
}

static int atmf04_pm_resume(struct device *dev)
{
    struct atmf04_data * data = dev_get_drvdata(dev);
    int err = 0;

    if(data->atmf04_active) {
	err = pinctrl_select_state(data->atmf04_pinctrl, data->atmf04_active);
	if(err < 0)
	    PINFO("can not set atmf04 pin active");
    } else {
	PINFO("pin ctrl active null");
    }

    PINFO("atmf04 pm resume");
    return 0;
}

static const struct i2c_device_id atmf04_id[] = {
	{ "atmf04", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, atmf04_id);

#ifdef CONFIG_OF
static struct of_device_id atmf04_match_table[] = {
	{ .compatible = "adsemicon,atmf04",},
	{ },
};
#else
#define atmf04_match_table NULL
#endif

static const struct dev_pm_ops atmf04_pm_ops = {
	.suspend = atmf04_pm_suspend,
	.resume = atmf04_pm_resume,
};

static struct i2c_driver atmf04_driver = {
	.driver = {
		.name   = ATMF04_DRV_NAME,
		.owner  = THIS_MODULE,
		.pm	= &atmf04_pm_ops,
		.of_match_table = atmf04_match_table,
	},
	.probe  = atmf04_probe,
	.remove = atmf04_remove,
	.id_table = atmf04_id,
};

static int __init atmf04_init(void)
{
	PINFO("ATMF04 module init");
	atmf04_workqueue = create_workqueue("key_touch");
	if (i2c_add_driver(&atmf04_driver)) {
		PINFO("failed at i2c_add_driver()");
		destroy_workqueue(atmf04_workqueue);
		return -ENODEV;
	}
	return 0;
}

static void __exit atmf04_exit(void)
{
	PINFO("ATMF04 module: release.");
	if (atmf04_workqueue)
			destroy_workqueue(atmf04_workqueue);

	atmf04_workqueue = NULL;

	i2c_del_driver(&atmf04_driver);
}

MODULE_DESCRIPTION("ATMF04 touch_key driver");
MODULE_LICENSE("GPL");

module_init(atmf04_init);
module_exit(atmf04_exit);
