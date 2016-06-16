#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
//#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#include <linux/interrupt.h>

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>         /* wake_lock, unlock */

#include "../../broadcast_tdmb_drv_ifdef.h"
#include "../inc/broadcast_fc8080.h"
#include "../inc/fci_types.h"
#include "../inc/bbm.h"

//#include <linux/pm_qos.h> // FEATURE_DMB_USE_PM_QOS

#include <linux/err.h>
#include <linux/of_gpio.h>

#include <linux/clk.h>

#if defined(CONFIG_ARCH_MT6753)
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irqchip/mt-eic.h>
#include <mt-plat/mt_gpio.h>
#include <mt-plat/mt6735/include/mach/gpio_const.h>
#include"../../../../spi/mediatek/mt6735/mt_spi.h"

#define GPIO_DMB_EN_PIN         (GPIO108 | 0x80000000)
#define GPIO_DMB_SPI_CS_PIN         (GPIO65 | 0x80000000)
#define GPIO_DMB_SPI_SCK_PIN         (GPIO66 | 0x80000000)
#define GPIO_DMB_SPI_MISO_PIN         (GPIO67 | 0x80000000)
#define GPIO_DMB_SPI_MOSI_PIN         (GPIO68 | 0x80000000)
#define GPIO_DMB_INT_PIN         (GPIO109 | 0x80000000)

static struct mt_chip_conf mtk_spi_config = {
    .setuptime = 3,
    .holdtime = 3,
    .high_time = 3,
    .low_time = 3,
    .cs_idletime = 2,
    .ulthgh_thrsh = 0,
    .cpol = 0,
    .cpha = 0,
    .rx_mlsb = 1,
    .tx_mlsb = 1,
    .tx_endian = 0,
    .rx_endian = 0,
    .com_mod = DMA_TRANSFER,
    .pause = 0,
    .finish_intr = 1,
    .deassert = 0,
    .ulthigh = 0,
    .tckdly = 2,
};
#endif

//#include <linux/msm-bus.h> // FEATURE_DMB_USE_BUS_SCALE
//#define FEATURE_DMB_USE_PINCTRL
#ifdef FEATURE_DMB_USE_PINCTRL
#include <linux/pinctrl/consumer.h>
#endif
/* external function */
extern int broadcast_fc8080_drv_if_isr(void);
extern void tunerbb_drv_fc8080_isr_control(fci_u8 onoff);

/* proto type declare */
static int broadcast_tdmb_fc8080_probe(struct spi_device *spi);
static int broadcast_tdmb_fc8080_remove(struct spi_device *spi);
static int broadcast_tdmb_fc8080_suspend(struct spi_device *spi, pm_message_t mesg);
static int broadcast_tdmb_fc8080_resume(struct spi_device *spi);

/* SPI Data read using workqueue */
//#define FEATURE_DMB_USE_WORKQUEUE
#if defined (CONFIG_MACH_MSM8916_YG_SKT_KR) || defined (CONFIG_MACH_MSM8939_P1BSSN_SKT_KR)
#define FEATURE_DMB_USE_XO
#endif
//#define FEATURE_DMB_USE_BUS_SCALE
//#define FEATURE_DMB_USE_PM_QOS

/************************************************************************/
/* LINUX Driver Setting                                                 */
/************************************************************************/
static uint32 user_stop_flg = 0;
struct tdmb_fc8080_ctrl_blk
{
    boolean                                    TdmbPowerOnState;
    struct spi_device*                        spi_ptr;
#ifdef FEATURE_DMB_USE_WORKQUEUE
    struct work_struct                        spi_work;
    struct workqueue_struct*                spi_wq;
#endif
    struct mutex                            mutex;
    struct wake_lock                        wake_lock;    /* wake_lock,wake_unlock */
    boolean                                    spi_irq_status;
    spinlock_t                                spin_lock;
#ifdef FEATURE_DMB_USE_XO
    struct clk                                *clk;
#endif
    struct platform_device                    *pdev;
#ifdef FEATURE_DMB_USE_BUS_SCALE
    struct msm_bus_scale_pdata                *bus_scale_pdata;
    fci_u32                                        bus_scale_client_id;
#endif
#ifdef FEATURE_DMB_USE_PM_QOS
    struct pm_qos_request    pm_req_list;
#endif
#if defined(CONFIG_ARCH_MT6753)
    struct device_node *dmb_irq_node;
#endif
    uint32                            dmb_en;
    uint32                            dmb_irq;
#if defined (CONFIG_MACH_MSM8926_VFP_KR) || defined(CONFIG_MACH_MSM8916_YG_SKT_KR) || defined(CONFIG_MACH_MSM8916_C100N_KR) || defined(CONFIG_MACH_MSM8916_C100N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8916_C100_GLOBAL_COM)
    uint32                            dmb_ant;
#endif
};

static struct tdmb_fc8080_ctrl_blk fc8080_ctrl_info;

static Device_drv device_fc8080 = {
    &broadcast_fc8080_drv_if_power_on,
    &broadcast_fc8080_drv_if_power_off,
    &broadcast_fc8080_drv_if_init,
    &broadcast_fc8080_drv_if_stop,
    &broadcast_fc8080_drv_if_set_channel,
    &broadcast_fc8080_drv_if_detect_sync,
    &broadcast_fc8080_drv_if_get_sig_info,
    &broadcast_fc8080_drv_if_get_fic,
    &broadcast_fc8080_drv_if_get_msc,
    &broadcast_fc8080_drv_if_reset_ch,
    &broadcast_fc8080_drv_if_user_stop,
    &broadcast_fc8080_drv_if_select_antenna,
    &broadcast_fc8080_drv_if_set_nation,
    &broadcast_fc8080_drv_if_is_on
};

#if 0
void tdmb_fc8080_spi_write_read_test(void)
{
    uint16 i;
    uint32 wdata = 0;
    uint32 ldata = 0;
    uint32 data = 0;
    uint32 temp = 0;

#define TEST_CNT    5
    tdmb_fc8080_power_on();

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_write(NULL, 0xa4, i & 0xff);
        bbm_com_read(NULL, 0xa4, (fci_u8*)&data);
        printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 byte test (0x%x,0x%x)\n", i & 0xff, data);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_word_write(NULL, 0xa4, i & 0xffff);
        bbm_com_word_read(NULL, 0xa4, (fci_u16*)&wdata);
        printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
        if((i & 0xffff) != wdata)
            printk("FC8080 word test (0x%x,0x%x)\n", i & 0xffff, wdata);
    }

    for(i=0;i<TEST_CNT;i++)
    {
        bbm_com_long_write(NULL, 0xa4, i & 0xffffffff);
        bbm_com_long_read(NULL, 0xa4, (fci_u32*)&ldata);
        printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
        if((i & 0xffffffff) != ldata)
            printk("FC8080 long test (0x%x,0x%x)\n", i & 0xffffffff, ldata);
    }

    data = 0;

    for(i=0;i<TEST_CNT;i++)
    {
        temp = i&0xff;
        bbm_com_tuner_write(NULL, 0x58, 0x01, (fci_u8*)&temp, 0x01);
        bbm_com_tuner_read(NULL, 0x58, 0x01, (fci_u8*)&data, 0x01);
        printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
        if((i & 0xff) != data)
            printk("FC8080 tuner test (0x%x,0x%x)\n", i & 0xff, data);
    }
    tdmb_fc8080_power_off();
}
#endif

void tunerbb_drv_hw_setting(void)
{
    printk("[tdmb][MTK] GPIO_ISDBT_PWR_EN Port request!!!\n");
    mt_set_gpio_mode(GPIO_DMB_EN_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DMB_EN_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_EN_PIN, GPIO_PULL_DOWN);
    mdelay(50);
    mt_set_gpio_dir(GPIO_DMB_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DMB_EN_PIN, GPIO_OUT_ZERO);

    printk("[tdmb][MTK] ISDBT_IRQ_INT Port request!!!\n");
    mt_set_gpio_mode(GPIO_DMB_INT_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DMB_INT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_INT_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DMB_INT_PIN, GPIO_DIR_IN);
    mdelay(50);

    //SPI MISO
    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_MISO_PIN, GPIO_PULL_DOWN);

    //SPI MOSI
    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_MOSI_PIN, GPIO_PULL_DOWN);

    //SPI CLK
    mt_set_gpio_pull_enable(GPIO_DMB_SPI_SCK_PIN , GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_SCK_PIN , GPIO_PULL_DOWN);

    //SPI CS
    mt_set_gpio_pull_enable(GPIO_DMB_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_CS_PIN, GPIO_PULL_DOWN);

    return;
}

void tunerbb_drv_hw_init(void)
{
    printk("[tdmb][MTK] tunerbb_drv_hw_init\n");

    mt_set_gpio_out(GPIO_DMB_EN_PIN, GPIO_OUT_ONE);
    mdelay(30);

    mt_set_gpio_pull_enable(GPIO_DMB_INT_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MISO_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MOSI_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_SCK_PIN, GPIO_PULL_DISABLE);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_CS_PIN, GPIO_PULL_DISABLE);
    mt_set_gpio_mode(GPIO_DMB_SPI_CS_PIN, GPIO_MODE_01);
    mt_set_gpio_dir(GPIO_DMB_SPI_CS_PIN, GPIO_DIR_IN);

    return;
}

void tunerbb_drv_hw_deinit(void)
{
    printk("[tdmb][MTK] tunerbb_drv_hw_deinit\n");

    //tdmb_fc8080_interrupt_lock();

    mt_set_gpio_pull_enable(GPIO_DMB_EN_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_EN_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DMB_EN_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DMB_EN_PIN, GPIO_OUT_ZERO);

    mt_set_gpio_pull_enable(GPIO_DMB_INT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_INT_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MISO_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_MISO_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_MOSI_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_MOSI_PIN, GPIO_PULL_DOWN);

    mt_set_gpio_pull_enable(GPIO_DMB_SPI_SCK_PIN , GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_SCK_PIN , GPIO_PULL_DOWN);

    mt_set_gpio_mode(GPIO_DMB_SPI_CS_PIN, GPIO_MODE_00);
    mt_set_gpio_pull_enable(GPIO_DMB_SPI_CS_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_DMB_SPI_CS_PIN, GPIO_PULL_DOWN);
    mt_set_gpio_dir(GPIO_DMB_SPI_CS_PIN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_DMB_SPI_CS_PIN, GPIO_OUT_ZERO);

    return;
}

struct spi_device *tdmb_fc8080_get_spi_device(void)
{
    return fc8080_ctrl_info.spi_ptr;
}


void tdmb_fc8080_set_userstop(int mode)
{
    user_stop_flg = mode;
    printk("tdmb_fc8080_set_userstop, user_stop_flg = %d \n", user_stop_flg);
}

int tdmb_fc8080_mdelay(int32 ms)
{
    int32    wait_loop =0;
    int32    wait_ms = ms;
    int        rc = 1;  /* 0 : false, 1 : true */

    if(ms > 100)
    {
        wait_loop = (ms /100);   /* 100, 200, 300 more only , Otherwise this must be modified e.g (ms + 40)/50 */
        wait_ms = 100;
    }

    do
    {
        mdelay(wait_ms);

        if(user_stop_flg == 1)
        {
            printk("~~~~~~~~ Ustop flag is set so return false ms =(%d)~~~~~~~\n", ms);
            rc = 0;
            break;
        }
    }while((--wait_loop) > 0);

    return rc;
}

void tdmb_fc8080_Must_mdelay(int32 ms)
{
    mdelay(ms);
}

int tdmb_fc8080_tdmb_is_on(void)
{
    return (int)fc8080_ctrl_info.TdmbPowerOnState;
}

/* EXPORT_SYMBOL() : when we use external symbol
which is not included in current module - over kernel 2.6 */
//EXPORT_SYMBOL(tdmb_fc8080_tdmb_is_on);
/*[BCAST002][S] 20140804 seongeun.jin - modify chip init check timing issue on BLT*/
#ifdef FEATURE_POWER_ON_RETRY
int tdmb_fc8080_power_on_retry(void)
{
    int res;
    int i;

    tdmb_fc8080_interrupt_lock();

    for(i = 0 ; i < 10; i++)
    {
        printk("[FC8080] tdmb_fc8080_power_on_retry :  %d\n", i);
#ifdef FEATURE_DMB_USE_XO
        if(fc8080_ctrl_info.clk != NULL) {
            clk_disable_unprepare(fc8080_ctrl_info.clk);
            printk("[FC8080] retry clk_disable %d\n", i);
        }
        else
            printk("[FC8080] ERR fc8080_ctrl_info.clkdis is NULL\n");
#endif

#if defined(CONFIG_ARCH_MT6753)
    tunerbb_drv_hw_init();
#else
    gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
    mdelay(150);
    gpio_set_value(fc8080_ctrl_info.dmb_en, 1);
    mdelay(5);
#endif

#ifdef FEATURE_DMB_USE_XO
        if(fc8080_ctrl_info.clk != NULL) {
            res = clk_prepare_enable(fc8080_ctrl_info.clk);
            if (res) {
                printk("[FC8080] retry clk_prepare_enable fail %d\n", i);
            }
        }
        else
            printk("[FC8080] ERR fc8080_ctrl_info.clken is NULL\n");
#endif
        mdelay(30);

        res = bbm_com_probe(NULL);

        if (!res)
            break;

    }

    tdmb_fc8080_interrupt_free();

    return res;
}
#endif
/*[BCAST002][E]*/
int tdmb_fc8080_power_on(void)
{
    int rc = FALSE;

    printk("tdmb_fc8080_power_on \n");
    if ( fc8080_ctrl_info.TdmbPowerOnState == FALSE )
    {
        wake_lock(&fc8080_ctrl_info.wake_lock);

#if defined(CONFIG_ARCH_MT6753)
    tunerbb_drv_hw_init();
#else
    gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
    mdelay(5);
    gpio_set_value(fc8080_ctrl_info.dmb_en, 1);
    mdelay(5);
#endif

#ifdef FEATURE_DMB_USE_BUS_SCALE
        msm_bus_scale_client_update_request(fc8080_ctrl_info.bus_scale_client_id, 1); /* expensive call, index:1 is the <84 512 3000 152000> entry */
#endif
#ifdef FEATURE_DMB_USE_XO
        if(fc8080_ctrl_info.clk != NULL) {
            rc = clk_prepare_enable(fc8080_ctrl_info.clk);
            if (rc) {
                gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
                mdelay(5);
                dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
                return rc;
            }
        }
#endif
#ifdef FEATURE_DMB_USE_PM_QOS
        if(pm_qos_request_active(&fc8080_ctrl_info.pm_req_list)) {
            pm_qos_update_request(&fc8080_ctrl_info.pm_req_list, 20);
        }
#endif
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 0);
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 1);
#if defined (CONFIG_MACH_MSM8926_VFP_KR) || defined(CONFIG_MACH_MSM8916_YG_SKT_KR) || defined(CONFIG_MACH_MSM8916_C100N_KR) || defined(CONFIG_MACH_MSM8916_C100N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8916_C100_GLOBAL_COM)
        gpio_set_value(fc8080_ctrl_info.dmb_ant, 0);
#endif

        mdelay(30); /* Due to X-tal stablization Time */

        tdmb_fc8080_interrupt_free();

        fc8080_ctrl_info.TdmbPowerOnState = TRUE;

        printk("tdmb_fc8080_power_on OK\n");
    }
    else
    {
        printk("tdmb_fc8080_power_on the power already turn on \n");
    }

    printk("tdmb_fc8080_power_on completed \n");
    rc = TRUE;

    return rc;
}

int tdmb_fc8080_power_off(void)
{
    if ( fc8080_ctrl_info.TdmbPowerOnState == TRUE )
    {
        tdmb_fc8080_interrupt_lock();

#ifdef FEATURE_DMB_USE_XO
        if(fc8080_ctrl_info.clk != NULL) {
            clk_disable_unprepare(fc8080_ctrl_info.clk);
        }
#endif

        fc8080_ctrl_info.TdmbPowerOnState = FALSE;

#if defined(CONFIG_ARCH_MT6753)
        tunerbb_drv_hw_deinit();
#else
        gpio_set_value(fc8080_ctrl_info.dmb_en, 0);
#endif

        wake_unlock(&fc8080_ctrl_info.wake_lock);
        mdelay(20);

//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_P-1), 1);    // for ESD TEST
//        gpio_set_value(PM8058_GPIO_PM_TO_SYS(DMB_ANT_SEL_N-1), 0);
#if defined (CONFIG_MACH_MSM8926_VFP_KR) || defined(CONFIG_MACH_MSM8916_YG_SKT_KR) || defined(CONFIG_MACH_MSM8916_C100N_KR) || defined(CONFIG_MACH_MSM8916_C100N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8916_C100_GLOBAL_COM)  
        gpio_set_value(fc8080_ctrl_info.dmb_ant, 1);
#endif

#ifdef FEATURE_DMB_USE_BUS_SCALE
        msm_bus_scale_client_update_request(fc8080_ctrl_info.bus_scale_client_id, 0); /* expensive call, index:0 is the <84 512 0 0> entry */
#endif
#ifdef FEATURE_DMB_USE_PM_QOS
        if(pm_qos_request_active(&fc8080_ctrl_info.pm_req_list)) {
            pm_qos_update_request(&fc8080_ctrl_info.pm_req_list, PM_QOS_DEFAULT_VALUE);
        }
#endif
    }
    else
    {
        printk("tdmb_fc8080_power_on the power already turn off \n");
    }

    printk("tdmb_fc8080_power_off completed \n");
    return TRUE;
}

int tdmb_fc8080_select_antenna(unsigned int sel)
{
    return FALSE;
}

void tdmb_fc8080_interrupt_lock(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_interrupt_lock fail\n");
    }
    else
    {
#if defined(CONFIG_ARCH_MT6753)
        mt_eint_mask(fc8080_ctrl_info.dmb_irq);
#else
        disable_irq(fc8080_ctrl_info.spi_ptr->irq);
#endif
    }
}

void tdmb_fc8080_interrupt_free(void)
{
    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_interrupt_free fail\n");
    }
    else
    {
#if defined(CONFIG_ARCH_MT6753)
        mt_eint_unmask(fc8080_ctrl_info.dmb_irq);
#else
        enable_irq(fc8080_ctrl_info.spi_ptr->irq);
#endif
    }
}

int tdmb_fc8080_spi_write_read(uint8* tx_data, int tx_length, uint8 *rx_data, int rx_length)
{
    int rc;

    struct spi_transfer    t = {
            .tx_buf        = tx_data,
            .rx_buf        = rx_data,
            .len        = tx_length+rx_length,
        };

    struct spi_message    m;

    if (fc8080_ctrl_info.spi_ptr == NULL)
    {
        printk("tdmb_fc8080_spi_write_read error txdata=0x%x, length=%d\n", (unsigned int)tx_data, tx_length+rx_length);
        return FALSE;
    }

    mutex_lock(&fc8080_ctrl_info.mutex);

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    rc = spi_sync(fc8080_ctrl_info.spi_ptr, &m);

    if ( rc < 0 )
    {
        printk("tdmb_fc8080_spi_read_burst result(%d), actual_len=%d\n",rc, m.actual_length);
    }

    mutex_unlock(&fc8080_ctrl_info.mutex);

    return TRUE;
}

#ifdef FEATURE_DMB_USE_WORKQUEUE
static irqreturn_t broadcast_tdmb_spi_isr(int irq, void *handle)
{
    struct tdmb_fc8080_ctrl_blk* fc8080_info_p;

    fc8080_info_p = (struct tdmb_fc8080_ctrl_blk *)handle;
    if ( fc8080_info_p && fc8080_info_p->TdmbPowerOnState )
    {
        unsigned long flag;
        if (fc8080_info_p->spi_irq_status)
        {
            printk("######### spi read function is so late skip #########\n");
            return IRQ_HANDLED;
        }
//        printk("***** broadcast_tdmb_spi_isr coming *******\n");
        spin_lock_irqsave(&fc8080_info_p->spin_lock, flag);
        queue_work(fc8080_info_p->spi_wq, &fc8080_info_p->spi_work);
        spin_unlock_irqrestore(&fc8080_info_p->spin_lock, flag);
    }
    else
    {
        printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
    }

    return IRQ_HANDLED;
}

static void broacast_tdmb_spi_work(struct work_struct *tdmb_work)
{
    struct tdmb_fc8080_ctrl_blk *pTdmbWorkData;

    pTdmbWorkData = container_of(tdmb_work, struct tdmb_fc8080_ctrl_blk, spi_work);
    if ( pTdmbWorkData )
    {
        tunerbb_drv_fc8080_isr_control(0);
        pTdmbWorkData->spi_irq_status = TRUE;
        broadcast_fc8080_drv_if_isr();
        pTdmbWorkData->spi_irq_status = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    }
    else
    {
        printk("~~~~~~~broadcast_tdmb_spi_work call but pTdmbworkData is NULL ~~~~~~~\n");
    }
}
#else

static irqreturn_t broadcast_tdmb_spi_event_handler(int irq, void *handle)
{
    struct tdmb_fc8080_ctrl_blk* fc8080_info_p;
    fc8080_info_p = (struct tdmb_fc8080_ctrl_blk *)handle;

    if ( fc8080_info_p && fc8080_info_p->TdmbPowerOnState )
    {
        if (fc8080_info_p->spi_irq_status)
        {
            printk("######### spi read function is so late skip ignore #########\n");
            return IRQ_HANDLED;
        }
        tunerbb_drv_fc8080_isr_control(0);
        fc8080_info_p->spi_irq_status = TRUE;
        broadcast_fc8080_drv_if_isr();
        fc8080_info_p->spi_irq_status = FALSE;
        tunerbb_drv_fc8080_isr_control(1);
    }
    else
    {
        printk("broadcast_tdmb_spi_isr is called, but device is off state\n");
    }
    return IRQ_HANDLED;
}
#endif

#ifdef FEATURE_DMB_USE_PINCTRL
static int tdmb_pinctrl_init(void)
{
    struct pinctrl *tdmb_pinctrl;
    struct pinctrl_state *gpio_state_suspend;

    tdmb_pinctrl = devm_pinctrl_get(&(fc8080_ctrl_info.pdev->dev));


    if(IS_ERR_OR_NULL(tdmb_pinctrl)) {
        pr_err("%s: Getting pinctrl handle failed\n", __func__);
        return -EINVAL;
    }
    gpio_state_suspend
     = pinctrl_lookup_state(tdmb_pinctrl, "gpio_tdmb_suspend");

     if(IS_ERR_OR_NULL(gpio_state_suspend)) {
         pr_err("%s: Failed to get the suspend state pinctrl handle\n", __func__);
         return -EINVAL;
    }

    if(pinctrl_select_state(tdmb_pinctrl, gpio_state_suspend)) {
        pr_err("%s: error on pinctrl_select_state for tdmb enable and irq pin\n", __func__);
        return -EINVAL;
    }
    else {
        printk("%s: success to set pinctrl_select_state for tdmb enable and irq pin\n", __func__);
    }

    return 0;
}
#endif
static int tdmb_configure_gpios(void)
{
    int rc = OK;
    int err_count = 0;

    fc8080_ctrl_info.dmb_en = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node,"tdmb-fc8080,en-gpio",0);

    rc = gpio_request(fc8080_ctrl_info.dmb_en, "DMB_EN");
    if (rc < 0) {
        err_count++;
        printk("%s:Failed GPIO DMB_EN request!!!\n",__func__);
    }

    fc8080_ctrl_info.dmb_irq = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node,"tdmb-fc8080,irq-gpio",0);

    rc = gpio_request(fc8080_ctrl_info.dmb_irq, "DMB_INT_N");
    if (rc < 0) {
        err_count++;
        printk("%s:Failed GPIO DMB_INT_N request!!!\n",__func__);
    }

#if defined (CONFIG_MACH_MSM8926_VFP_KR ) || defined(CONFIG_MACH_MSM8916_YG_SKT_KR) || defined(CONFIG_MACH_MSM8916_C100N_KR) || defined(CONFIG_MACH_MSM8916_C100N_GLOBAL_COM) || defined(CONFIG_MACH_MSM8916_C100_GLOBAL_COM)
    fc8080_ctrl_info.dmb_ant = of_get_named_gpio(fc8080_ctrl_info.pdev->dev.of_node,"tdmb-fc8080,ant-gpio",0);

    rc = gpio_request(fc8080_ctrl_info.dmb_ant, "DMB_ANT");
    if (rc < 0) {
        err_count++;
        printk("%s:Failed GPIO DMB_ANT request!!!\n",__func__);
    }
    gpio_direction_output(fc8080_ctrl_info.dmb_ant,0);
#endif

    gpio_direction_output(fc8080_ctrl_info.dmb_en, 0);
    gpio_direction_input(fc8080_ctrl_info.dmb_irq);

    if(err_count > 0) rc = -EINVAL;

    return rc;
}

static int broadcast_tdmb_fc8080_probe(struct spi_device *spi)
{
    int rc;

    if(spi == NULL)
    {
        printk("broadcast_fc8080_probe spi is NULL, so spi can not be set\n");
        return -1;
    }

    fc8080_ctrl_info.TdmbPowerOnState = FALSE;
    fc8080_ctrl_info.spi_ptr                 = spi;
    fc8080_ctrl_info.spi_ptr->mode             = SPI_MODE_0;
    fc8080_ctrl_info.spi_ptr->bits_per_word     = 8;
    fc8080_ctrl_info.spi_ptr->max_speed_hz     = (16000*1000);
    fc8080_ctrl_info.pdev = to_platform_device(&spi->dev);

#ifdef FEATURE_DMB_USE_BUS_SCALE
    fc8080_ctrl_info.bus_scale_pdata = msm_bus_cl_get_pdata(fc8080_ctrl_info.pdev);
    fc8080_ctrl_info.bus_scale_client_id = msm_bus_scale_register_client(fc8080_ctrl_info.bus_scale_pdata);
#endif

    // Once I have a spi_device structure I can do a transfer anytime

    rc = spi_setup(spi);
    printk("broadcast_tdmb_fc8080_probe spi_setup=%d\n", rc);
       if (rc){
            printk( "[fc8080] Spi setup error(%d)\n", rc);
            return rc;
        }
    bbm_com_hostif_select(NULL, 1);

#ifdef FEATURE_DMB_USE_XO
    fc8080_ctrl_info.clk = clk_get(&fc8080_ctrl_info.spi_ptr->dev, "tdmb_xo");
    if (IS_ERR(fc8080_ctrl_info.clk)) {
        rc = PTR_ERR(fc8080_ctrl_info.clk);
        dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not get clock\n");
        return rc;
    }

    /* We enable/disable the clock only to assure it works */
    rc = clk_prepare_enable(fc8080_ctrl_info.clk);
    if (rc) {
        dev_err(&fc8080_ctrl_info.spi_ptr->dev, "could not enable clock\n");
        return rc;
    }
    clk_disable_unprepare(fc8080_ctrl_info.clk);
#endif

#ifdef FEATURE_DMB_USE_WORKQUEUE
    INIT_WORK(&fc8080_ctrl_info.spi_work, broacast_tdmb_spi_work);
    fc8080_ctrl_info.spi_wq = create_singlethread_workqueue("tdmb_spi_wq");
    if(fc8080_ctrl_info.spi_wq == NULL){
        printk("Failed to setup tdmb spi workqueue \n");
        return -ENOMEM;
    }
#endif
    
#if defined(CONFIG_ARCH_MT6753)
    tunerbb_drv_hw_setting();

    fc8080_ctrl_info.dmb_irq_node = of_find_compatible_node(NULL, NULL, "mediatek,dtv");
        if (fc8080_ctrl_info.dmb_irq_node) {
            fc8080_ctrl_info.dmb_irq = irq_of_parse_and_map(fc8080_ctrl_info.dmb_irq_node, 0);

            if(fc8080_ctrl_info.dmb_irq < 0) {
                printk("[tdmb] irq_of_parse_and_map IRQ can't parsing!.\n");
            }

            rc = request_threaded_irq(fc8080_ctrl_info.dmb_irq, NULL, broadcast_tdmb_spi_event_handler, 
                IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING, spi->dev.driver->name, &fc8080_ctrl_info);

            if (rc){
                printk("[tdmb] dmb request irq fail : %d\n", rc);
            }
        }
#else
    tdmb_configure_gpios();

#ifdef FEATURE_DMB_USE_PINCTRL
    tdmb_pinctrl_init();
#endif

#ifdef FEATURE_DMB_USE_WORKQUEUE
    rc = request_irq(spi->irq, broadcast_tdmb_spi_isr, IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                       spi->dev.driver->name, &fc8080_ctrl_info);
#else

    rc = request_threaded_irq(spi->irq, NULL, broadcast_tdmb_spi_event_handler, IRQF_ONESHOT | IRQF_DISABLED | IRQF_TRIGGER_FALLING,
                       spi->dev.driver->name, &fc8080_ctrl_info);
#endif
#endif /* defined(CONFIG_ARCH_MT6753) */

    tdmb_fc8080_interrupt_lock();

    mutex_init(&fc8080_ctrl_info.mutex);

    wake_lock_init(&fc8080_ctrl_info.wake_lock,  WAKE_LOCK_SUSPEND, dev_name(&spi->dev));

    spin_lock_init(&fc8080_ctrl_info.spin_lock);

#ifdef FEATURE_DMB_USE_PM_QOS
    pm_qos_add_request(&fc8080_ctrl_info.pm_req_list, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
#endif
    printk("broadcast_fc8080_probe End\n");

    return rc;
}

static int broadcast_tdmb_fc8080_remove(struct spi_device *spi)
{
    printk("broadcast_tdmb_fc8080_remove \n");

#ifdef FEATURE_DMB_USE_WORKQUEUE
    if (fc8080_ctrl_info.spi_wq)
    {
        flush_workqueue(fc8080_ctrl_info.spi_wq);
        destroy_workqueue(fc8080_ctrl_info.spi_wq);
    }
#endif

#ifdef FEATURE_DMB_USE_BUS_SCALE
    msm_bus_scale_unregister_client(fc8080_ctrl_info.bus_scale_client_id);
#endif

#if defined(CONFIG_ARCH_MT6753)
    mt_set_gpio_mode(GPIO_DMB_INT_PIN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_DMB_INT_PIN, GPIO_DIR_IN);
#else
    free_irq(spi->irq, &fc8080_ctrl_info);
#endif

    mutex_destroy(&fc8080_ctrl_info.mutex);

    wake_lock_destroy(&fc8080_ctrl_info.wake_lock);

#ifdef FEATURE_DMB_USE_PM_QOS
    pm_qos_remove_request(&fc8080_ctrl_info.pm_req_list);
#endif
    memset((unsigned char*)&fc8080_ctrl_info, 0x0, sizeof(struct tdmb_fc8080_ctrl_blk));
    return 0;
}

static int broadcast_tdmb_fc8080_suspend(struct spi_device *spi, pm_message_t mesg)
{
    printk("broadcast_tdmb_fc8080_suspend \n");
    return 0;
}

static int broadcast_tdmb_fc8080_resume(struct spi_device *spi)
{
    printk("broadcast_tdmb_fc8080_resume \n");
    return 0;
}
#if 0
static int broadcast_tdmb_fc8080_check_chip_id(void)
{
    int rc = ERROR;

    rc = tdmb_fc8080_power_on();

    if(rc == TRUE && !bbm_com_probe(NULL)) //rc == TRUE : power on success,  OK : 0
        rc = OK;
    else
        rc = ERROR;

    tdmb_fc8080_power_off();

    return rc;
}
#endif

#if defined(CONFIG_ARCH_MT6753)
struct spi_device_id tdmb_spi_table = {"dmb_spi", 0};

static struct spi_driver broadcast_tdmb_driver = {
	.driver = {
		.name = "dmb_spi",
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = broadcast_tdmb_fc8080_probe,
	.remove= __exit_p(broadcast_tdmb_fc8080_remove),
	.id_table = &tdmb_spi_table,
};

static struct spi_board_info spi_board_devs[] __initdata = {
    [0] = {
        .modalias="dmb_spi",
        .bus_num = 0,
        .chip_select = 0,
        .mode = SPI_MODE_0,
       //.max_speed_hz = 24*1000*1000,//24MHz
        .controller_data  = &mtk_spi_config,
    },
};
#else
static struct of_device_id tdmb_spi_table[] = {
    {
        .compatible = "mediatek,mt6753-spi",
    },
    {}
};

static struct spi_driver broadcast_tdmb_driver = {
    .probe = broadcast_tdmb_fc8080_probe,
    .remove    = broadcast_tdmb_fc8080_remove,
    .suspend = broadcast_tdmb_fc8080_suspend,
    .resume  = broadcast_tdmb_fc8080_resume,
    .driver = {
        .name = "dmb_spi",
        .of_match_table = tdmb_spi_table,
        .bus    = &spi_bus_type,
        .owner = THIS_MODULE,
    },
};
#endif

static int __init broadcast_tdmb_fc8080_drv_init(void)
{
    int rc;

    if(broadcast_tdmb_drv_check_module_init() != OK) {
        rc = ERROR;
        return rc;
    }

#if defined(CONFIG_ARCH_MT6753)
    spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif

    rc = spi_register_driver(&broadcast_tdmb_driver);
    printk("spi_register_driver %d \n", rc);
#if 0
    if(broadcast_tdmb_fc8080_check_chip_id() != OK) {
        spi_unregister_driver(&broadcast_tdmb_driver);
        rc = ERROR;
        return rc;
    }
#endif

    rc = broadcast_tdmb_drv_start(&device_fc8080);
        if (rc < 0)
            printk("[fc8080] SPI driver register failed\n");

    return rc;
}

static void __exit broadcast_tdmb_fc8080_drv_exit(void)
{
    spi_unregister_driver(&broadcast_tdmb_driver);
}

module_init(broadcast_tdmb_fc8080_drv_init);
module_exit(broadcast_tdmb_fc8080_drv_exit);

/* optional part when we include driver code to build-on
it's just used when we make device driver to module(.ko)
so it doesn't work in build-on */
MODULE_DESCRIPTION("FC8080 tdmb device driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("FCI");
