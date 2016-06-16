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

#endif

#include "lcm_drv.h"
#if defined(BUILD_LK)
#define LCM_PRINT(string, args...) printf("[LCD][LK] "string, ##args)
#elif defined(BUILD_UBOOT)
#define LCM_PRINT(string, args...) printf("[LCD][UBOOT] "string, ##args)
#else
#define LCM_PRINT(string, args...) printk("[LCD][KERNEL] "string, ##args)
#endif
#if defined(BUILD_LK)
#include <boot_mode.h>
#else
#include <upmu_hw.h>
#include <mt_typedefs.h>
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
// pixel
#define FRAME_WIDTH              (720)
#define FRAME_HEIGHT             (1280)

// physical dimension
#define PHYSICAL_WIDTH        (62)
#define PHYSICAL_HEIGHT         (110)

#define LCM_ID       (0xb9)
#define LCM_DSI_CMD_MODE        0

#define REGFLAG_DELAY 0xAB
#define REGFLAG_END_OF_TABLE 0xAA // END OF REGISTERS MARKER

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)                                    (lcm_util.set_reset_pin((v)))
#define UDELAY(n)                                             (lcm_util.udelay(n))
#define MDELAY(n)                                             (lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V3(para_tbl, size, force_update)       lcm_util.dsi_set_cmdq_V3(para_tbl, size, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)            lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)                                        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)                    lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)                                            lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)                   lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

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

#define LGE_LPWG_SUPPORT

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

extern unsigned short mt6328_set_register_value(PMU_FLAGS_LIST_ENUM flagname, unsigned int val);

static LCM_setting_table_V3 lcm_initialization_setting_V3[] = {
// K6P_EVB Bring up, PH1 5.7" panel, 151119 ver2.0
    {0x15, 0xB0,  1,  {0xAC}},
    {0x39, 0xB1,  4,  {0x10, 0x30, 0x16, 0x00}},
    {0x39, 0xB2,  12, {0x01, 0x00, 0x11, 0x20, 0x18, 0x01, 0x7E, 0x10, 0x14, 0x15, 0x16, 0x17}},
    {0x39, 0xB4,  3,  {0x00, 0x9F, 0x00}},
    {0x39, 0xB5,  5,  {0x42, 0xC0, 0x80, 0x00, 0x00}},
    {0x39, 0xB6,  3,  {0x77, 0x34, 0x48}},
    {0x39, 0xBD,  9,  {0xD0, 0x02, 0xE5, 0x01, 0x0A, 0x0A, 0x12, 0x20, 0x22}},
    {0x39, 0xBE,  7,  {0xE0, 0xE0, 0xC9, 0xC9, 0xC8, 0xF8, 0x00}},
    {0x39, 0xC1,  6,  {0x01, 0xE8, 0xD0, 0xC2, 0xC1, 0x00}},
    {0x39, 0xC2,  3,  {0x66, 0x10, 0x0F}},
    {0x39, 0xC3,  6,  {0x15, 0x2F, 0x2F, 0x00, 0x66, 0x61}},
    {0x39, 0xC4,  3,  {0x01, 0x00, 0x38}},
    {0x39, 0xC5,  5,  {0x25, 0x40, 0x20, 0x0B, 0x0B}},
    {0x39, 0xC7,  12, {0x10, 0x22, 0x00, 0x28, 0x00, 0x24, 0x48, 0x24, 0x3F, 0xFF, 0xFC, 0x04}},
    {0x39, 0xCC,  14, {0x22, 0x2F, 0x85, 0x26, 0x21, 0xB8, 0x02, 0x28, 0x48, 0x28, 0x3F, 0x7F, 0xFE, 0x04}},
    {0x39, 0xD0,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xD1,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xD2,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xD3,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xD4,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xD5,  12, {0x02, 0x21, 0x36, 0x44, 0x43, 0x35, 0x14, 0x10, 0x03, 0x54, 0x03, 0x02}},
    {0x39, 0xE4,  21, {0x4F, 0xC3, 0x4F, 0xC0, 0xC1, 0x45, 0xC6, 0xCC, 0xCC, 0xCB, 0x4F, 0xCB, 0xCA, 0xCA, 0xC9, 0xC9, 0xC7, 0xC8, 0x4F, 0x4F, 0x4F}},
    {0x39, 0xE5,  12, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x03}},
    {0x05, 0x11,  1,  {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};

static LCM_setting_table_V3 lcm_initialization_setting_V3_Disp_On[] = {
    {0x05, 0x29,    1, {0x00}},
    {REGFLAG_END_OF_TABLE, 0x00, 1,{}},
};


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
    LCM_PRINT("push_table \n");
}
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

       params->dsi.mode   = SYNC_PULSE_VDO_MODE; //SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE;//SYNC_PULSE_VDO_MODE;
     // enable tearing-free
    params->dbi.te_mode                 = LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity        = LCM_POLARITY_RISING;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM                    = LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order     = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq       = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding         = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format          = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    params->dsi.packet_size=256;
    //params->dsi.intermediat_buffer_num = 0;

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.cont_clock = TRUE;

// K6P EVB
    params->dsi.vertical_sync_active = 2;
    params->dsi.vertical_backporch = 13;
    params->dsi.vertical_frontporch = 500;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active              = 4;
    params->dsi.horizontal_backporch                = 34;
    params->dsi.horizontal_frontporch               = 14;
    params->dsi.horizontal_active_pixel             = FRAME_WIDTH;

    /*params->dsi.HS_TRAIL = 10;
    params->dsi.HS_ZERO= 20;
    params->dsi.LPX= 20;
    params->dsi.HS_PRPR= 8;*/

    params->dsi.PLL_CLOCK = 260;
}

static void init_lcm_registers(void)
{
    unsigned int data_array[32];

    dsi_set_cmdq_V3(lcm_initialization_setting_V3, sizeof(lcm_initialization_setting_V3) / sizeof(LCM_setting_table_V3), 1);
    MDELAY(120);
    dsi_set_cmdq_V3(lcm_initialization_setting_V3_Disp_On, sizeof(lcm_initialization_setting_V3_Disp_On) / sizeof(LCM_setting_table_V3), 1);
    LCM_PRINT("init_lcm_registers \n");
}

static void init_lcm_registers_sleep(void)
{
    unsigned int data_array[1];

    MDELAY(10);
    data_array[0] = 0x00280500; //Display Off
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(30);
    data_array[0] = 0x00100500; //enter sleep
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    LCM_PRINT("init_lcm_registers_sleep \n");
}

static void init_lcm_registers_sleep_out(void)
{
    unsigned int data_array[1];

    data_array[0] = 0x00110500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(120);
    data_array[0] = 0x00290500;
    dsi_set_cmdq(data_array, 1, 1);
    MDELAY(50);
    LCM_PRINT("init_lcm_registers_sleep \n");
}

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
    LCM_PRINT("Reset High \n");
   }
   else
   {
   mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
   LCM_PRINT("Reset Low \n");
   }
}

static void lcm_init(void)
{
    reset_lcd_module(1);
    MDELAY(5);

    ldo_p5m5_dsv_5v5_on();
    MDELAY(5);

    init_lcm_registers();
    MDELAY(50);
    need_set_lcm_addr = 1;
    LCM_PRINT("lcm_init \n");
}

static void lcm_suspend(void)
{
    init_lcm_registers_sleep();

    LCM_PRINT("lcm_suspend \n");
}

static void lcm_suspend_mfts(void)
{
    reset_lcd_module(0);
    MDELAY(10);
    ldo_p5m5_dsv_5v5_off();
    ldo_3v0_off();
    ldo_1v8io_off();
    MDELAY(20);

    LCM_PRINT("lcm_suspend_for_mfts \n");
}

static void lcm_resume_power(void)
{
    reset_lcd_module(0);
    MDELAY(10);
    ldo_p5m5_dsv_5v5_off();
    ldo_3v0_off();
    ldo_1v8io_off();
    MDELAY(20);

    ldo_1v8io_on();
    ldo_3v0_on();
    MDELAY(5);

    LCM_PRINT("lcm_resume \n");
}

static void lcm_resume(void)
{
    lcm_init();
    need_set_lcm_addr = 1;
    LCM_PRINT("lcm_resume \n");
}

static void lcm_esd_recover(void)
{
    lcm_suspend();
    lcm_resume();

    LCM_PRINT("lcm_esd_recover \n");
}

static void lcm_resume_mfts(void)
{

    ldo_1v8io_on();
    ldo_3v0_on();
    MDELAY(5);

    LCM_PRINT("lcm_resume_mfts \n");
}

static void lcm_shutdown(void)
{
    reset_lcd_module(0);
    MDELAY(10);
    ldo_p5m5_dsv_5v5_off();
    ldo_3v0_off();
    ldo_1v8io_off();
    MDELAY(10);

    LCM_PRINT("lcm_shutdown \n");
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
    LCM_PRINT("lcm_update \n");
}

static unsigned int lcm_compare_id(void)
{
    LCM_PRINT("lcm_compare_id \n");
    return 1;
}
// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER td4100_hd720_dsi_vdo_synaptics_drv = {
    .name = "td4100_hd720_dsi_vdo_synaptics",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params = lcm_get_params,
    .init = lcm_init,
    .suspend = lcm_suspend,
    .resume = lcm_resume,
    .resume_power = lcm_resume_power,
    .shutdown = lcm_shutdown,
    .suspend_mfts = lcm_suspend_mfts,
    .resume_mfts = lcm_resume_mfts,
//    .set_pwm_for_mix = lcm_set_pwm_for_mix,
//    .compare_id = lcm_compare_id,
//    .update = lcm_update,
#if (!defined(BUILD_UBOOT) && !defined(BUILD_LK))
//    .esd_recover = lcm_esd_recover,
#endif
};
