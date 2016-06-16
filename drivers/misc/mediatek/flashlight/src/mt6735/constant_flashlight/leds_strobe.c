#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>



/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
//	#define PK_DBG PK_DBG_FUNC
    #define PK_DBG printk
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_duty=-1;
static int g_timeOutTimeMs=0;

   //[LGE_UPDATE_S] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
   static u32 strobe_width = 0; /* 0 is disable */

   static const MUINT32 strobeLevelLUT[32] = {1,1,1,1,3,3,3,3,3,3,3,2,4,4,4,4,5,5,5,2,5,5,5,5,5,5,5,5,5,5,5,5};
  //[LGE_UPDATE_E] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


//#define STROBE_DEVICE_ID 0xC6
//#define STROBE_DEVICE_ID 0x39
#define STROBE_DEVICE_ID 0x11


static struct work_struct workTimeOut;

//#define FLASH_GPIO_ENF (GPIO42 | 0x80000000)
//#define FLASH_GPIO_ENT (GPIO43 | 0x80000000)

/*****************************************************************************
Device Tree GPIO Control Function
*****************************************************************************/
/* GPIO Pin control*/
static struct pinctrl *cam_flash_ctrl = NULL;
static struct pinctrl_state *cam_flash_en_h = NULL;
static struct pinctrl_state *cam_flash_en_l = NULL;
static struct pinctrl_state *cam_flash_tx_h = NULL;
static struct pinctrl_state *cam_flash_tx_l = NULL;

typedef enum {
	CAM_FLASH_EN,
	CAM_FLASH_TX,
} CAMFlashType;

int mtkcam_flash_gpio_init(struct platform_device *pdev)
{
	int ret = 0;
	cam_flash_ctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(cam_flash_ctrl)) {
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(cam_flash_ctrl);
	}

	cam_flash_en_h = pinctrl_lookup_state(cam_flash_ctrl, "cam_flash_en1");
	if (IS_ERR(cam_flash_en_h)) {
		ret = PTR_ERR(cam_flash_en_h);
		PK_DBG("%s : pinctrl err, cam_flash_en_h\n", __func__);
	}

	cam_flash_en_l = pinctrl_lookup_state(cam_flash_ctrl, "cam_flash_en0");
	if (IS_ERR(cam_flash_en_l)) {
		ret = PTR_ERR(cam_flash_en_l);
		PK_DBG("%s : pinctrl err, cam_flash_en_l\n", __func__);
	}

	cam_flash_tx_h = pinctrl_lookup_state(cam_flash_ctrl, "cam_flash_tx1");
	if (IS_ERR(cam_flash_tx_h)) {
		ret = PTR_ERR(cam_flash_tx_h);
		PK_DBG("%s : pinctrl err, cam_flash_tx_h\n", __func__);
	}

	cam_flash_tx_l = pinctrl_lookup_state(cam_flash_ctrl, "cam_flash_tx0");
	if (IS_ERR(cam_flash_tx_l)) {
		ret = PTR_ERR(cam_flash_tx_l);
		PK_DBG("%s : pinctrl err, cam_flash_tx_l\n", __func__);
	}
    printk("exit function mtkcam_flash_gpio_init\n");
	return ret;
}

int mtkcam_flash_gpio_set(int FlashType, int Val)
{
	int ret = 0;
	switch (FlashType) {
	case CAM_FLASH_EN:
		if (Val == 0)
			pinctrl_select_state(cam_flash_ctrl, cam_flash_en_l);
		else
			pinctrl_select_state(cam_flash_ctrl, cam_flash_en_h);
		break;
	case CAM_FLASH_TX:
		if (Val == 0)
			pinctrl_select_state(cam_flash_ctrl, cam_flash_tx_l);
		else
			pinctrl_select_state(cam_flash_ctrl, cam_flash_tx_h);
		break;
	default:
		PK_DBG("FlashType(%d) is invalid !!\n", FlashType);
		break;
	};

	PK_DBG("FlashType(%d) val(%d)\n", FlashType, Val);

	return ret;
}


/*****************************************************************************
Functions
*****************************************************************************/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#if defined(CONFIG_LEDS_RT4832)
extern void rt4832_flash_strobe_en(void);
extern void rt4832_flash_strobe_prepare(char OnOff,char ActiveHigh);
extern void rt4832_flash_strobe_level(char level);
#endif

#if defined(CONFIG_LEDS_RT8542)
extern void rt8542_flash_strobe_en(void);
extern void rt8542_flash_strobe_prepare(char OnOff,char ActiveHigh);
extern void rt8542_flash_strobe_level(char level);
#endif
static void work_timeOutFunc(struct work_struct *data);



int FL_Enable(void)
{
     if(strobe_width == 0)  //torch mode
     {
	 #if defined(CONFIG_LEDS_RT4832)
             rt4832_flash_strobe_prepare(2,1);
	 #endif
	 #if defined(CONFIG_LEDS_RT8542)
             rt8542_flash_strobe_prepare(2,1);
	 #endif	 
     }
     else  //strobe mode
     {
	 #if defined(CONFIG_LEDS_RT4832)
             rt4832_flash_strobe_prepare(1,1);
	 #endif
	 #if defined(CONFIG_LEDS_RT8542)
             rt8542_flash_strobe_prepare(1,1);
	 #endif	 
     }
/*
    char buf[2];
    char bufR[2];
    buf[0]=1;
    buf[1]=0x00;
    iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

    buf[0]=10;
    buf[1]=0x02;
    iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);

    buf[0]=9;
    buf[1]=0x73;
    iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d\n",__LINE__);
*/
    mtkcam_flash_gpio_set(CAM_FLASH_EN,1);
    mtkcam_flash_gpio_set(CAM_FLASH_TX,1);
/*
    mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ONE);
    mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ONE);
*/
    //////////////////////////////////////////////
    //////////////////////////////////////////////
/*
    buf[0]=0;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d reg0 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=1;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d reg1 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=6;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d reg6 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=8;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d reg8 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=9;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d reg9 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=10;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d regA %d\n",__LINE__, (int)bufR[0]);

    buf[0]=11;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable line=%d regB %d\n",__LINE__, (int)bufR[0]);
*/
#if defined(CONFIG_LEDS_RT4832)
        rt4832_flash_strobe_en();
#endif
#if defined(CONFIG_LEDS_RT8542)
        rt8542_flash_strobe_en();
#endif
    return 0;
}



int FL_Disable(void)
{
#if defined(CONFIG_LEDS_RT4832)
       rt4832_flash_strobe_prepare(0,0);
#endif
#if defined(CONFIG_LEDS_RT8542)
       rt8542_flash_strobe_prepare(0,0);
#endif
/*
        char buf[2];
        char bufR[2];


        buf[0]=0;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d reg0 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=1;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d reg1 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=6;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d reg6 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=8;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d reg8 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=9;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d reg9 %d\n",__LINE__, (int)bufR[0]);

    buf[0]=10;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d regA %d\n",__LINE__, (int)bufR[0]);

    buf[0]=11;
    iReadRegI2C(buf, 1, bufR, 1, STROBE_DEVICE_ID);
    PK_DBG(" FL_Enable0 line=%d regB %d\n",__LINE__, (int)bufR[0]);

///////////////////////
    buf[0]=10;
    buf[1]=0x00;
    iWriteRegI2C(buf , 2, STROBE_DEVICE_ID);
    */
    mtkcam_flash_gpio_set(CAM_FLASH_EN,0);
    mtkcam_flash_gpio_set(CAM_FLASH_TX,0);
/*
    mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO);
    mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO);
*/
    PK_DBG(" FL_Disable line=%d\n",__LINE__);
#if defined(CONFIG_LEDS_RT4832)
    rt4832_flash_strobe_en();
#endif
#if defined(CONFIG_LEDS_RT8542)
    rt8542_flash_strobe_en();
#endif
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    g_duty =  duty;
#if defined(CONFIG_LEDS_RT4832)
    rt4832_flash_strobe_level((char)duty);
#endif
#if defined(CONFIG_LEDS_RT8542)
    rt8542_flash_strobe_level((char)duty);
#endif
    return 0;
}


int FL_Init(void)
{

    mtkcam_flash_gpio_set(CAM_FLASH_EN,0);
    mtkcam_flash_gpio_set(CAM_FLASH_TX,0);
/*
    if(mt_set_gpio_mode(FLASH_GPIO_ENT,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENT,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENT,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}

    if(mt_set_gpio_mode(FLASH_GPIO_ENF,GPIO_MODE_00)){PK_DBG("[constant_flashlight] set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(FLASH_GPIO_ENF,GPIO_DIR_OUT)){PK_DBG("[constant_flashlight] set gpio dir failed!! \n");}
    if(mt_set_gpio_out(FLASH_GPIO_ENF,GPIO_OUT_ZERO)){PK_DBG("[constant_flashlight] set gpio failed!! \n");}
*/



    INIT_WORK(&workTimeOut, work_timeOutFunc);
#if defined(CONFIG_LEDS_RT4832)
     rt4832_flash_strobe_prepare(1,1);
     rt4832_flash_strobe_level((char)32);
#endif
#if defined(CONFIG_LEDS_RT8542)
     rt8542_flash_strobe_prepare(1,1);
     rt8542_flash_strobe_level((char)32);
#endif
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
    FL_Disable();
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}



enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=ledTimeOutCallback;

}



static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
   int iFlashType = (int)FLASHLIGHT_NONE;
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
    PK_DBG("constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
    switch(cmd)
    {

        case FLASH_IOC_SET_TIME_OUT_TIME_MS:
            PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
            g_timeOutTimeMs=arg;
        break;


        case FLASH_IOC_SET_DUTY :
            PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
            //[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-06-10] Add Main Flash current(4 Step)
            #if 0
            //[LGE_UPDATE_S][yonghwan.lym@lge.com][2014-05-29] Flash High Current mode
            if(KD_STROBE_HIGH_CURRENT_WIDTH==arg)
            {
                strobe_width = KD_STROBE_HIGH_CURRENT_WIDTH;
            }
            else
            {
                //[LGE_UPDATE_S] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
                if(arg>32)
                {
                    arg = 32;
                }
                else if(arg<1)
                {
                    arg = 1;
                }
                arg = strobeLevelLUT[arg-1];
                PK_DBG("mapping level : %u \n",(int)arg);

                strobe_width = arg;
            }
            FL_Init();
            //[LGE_UPDATE_E] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
            FL_dim_duty(arg);
            break;
            //[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-05-29] Flash High Current mode
            #else
            strobe_width = arg;
            //FL_Init(); //[LGE_UPDATE][yonghwan.lym@lge.com][2014-06-20] Flash is fired more than 4 times.
            FL_dim_duty(arg);
            break;
            #endif
            //[LGE_UPDATE_E][yonghwan.lym@lge.com][2014-06-10] Add Main Flash current(4 Step)

        case FLASH_IOC_SET_STEP:
            PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

            break;

        case FLASH_IOC_SET_ONOFF :
            PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
            if(arg==1)
            {
                if(g_timeOutTimeMs!=0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                    hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
                }
                FL_Enable();
            }
            else
            {
                FL_Disable();
                hrtimer_cancel( &g_timeOutTimer );
            }
            break;
             // [LGE_UPDATE_S] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
         case FLASHLIGHTIOC_G_FLASHTYPE:
                 //spin_lock(&g_strobeSMPLock);    /* cotta-- SMP proection */

                 iFlashType = FLASHLIGHT_LED_CONSTANT;
                 //spin_unlock(&g_strobeSMPLock);
                 if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
                 {
                         PK_DBG(" ioctl copy to user failed\n");
                         return -EFAULT;
                 }
                 break;
         case FLASHLIGHTIOC_T_STATE:
                 break;
              // [LGE_UPDATE_E] [kipo.yun@lge.com] [2014-05-10] Enable Flash Driver
        default :
            PK_DBG(" No such command \n");
            i4RetValue = -EPERM;
            break;
    }
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    if (0 == strobe_Res)
    {
        FL_Init();
        timerInit();
    }
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
    spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_DBG(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        FL_Uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT    constantFlashlightFunc=
{
    constant_flashlight_open,
    constant_flashlight_release,
    constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}


/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}


EXPORT_SYMBOL(strobe_VDIrq);
EXPORT_SYMBOL(mtkcam_flash_gpio_init);
