/*****************************************************************************
 *
 * Filename:
 * ---------
 *    external_charging.c
 *
 * Project:
 * --------
 *   ALPS_Software
 *
 * Description:
 * ------------
 *   This file implements the interface between BMT and ADC scheduler.
 *
 * Author:
 * -------
 *  Oscar Liu
 *
 *============================================================================
  * $Revision:   1.0  $
 * $Modtime:   08 Apr 2014 07:47:16  $
 * $Log:   //mtkvs01/vmdata/Maui_sw/archives/mcu/hal/peripheral/inc/bmt_chr_setting.h-arc  $
 *             HISTORY
 * Below this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *------------------------------------------------------------------------------
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by PVCS VM. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <mach/mt_battery_meter.h>
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/charging.h>
#include <mt-plat/mt_boot.h>

#ifdef CONFIG_LGE_PM_BATTERY_ID
#include <mach/lge_battery_id.h>
#endif
#ifdef CONFIG_MACH_LGE
#include <mach/board_lge.h>
#endif

// ============================================================ //
//define
// ============================================================ //
//cut off to full
#define POST_CHARGING_TIME	30 * 60 // 30mins

// ============================================================ //
//global variable
// ============================================================ //
unsigned int g_bcct_flag=0;
unsigned int g_bcct_value=0;
CHR_CURRENT_ENUM g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
CHR_CURRENT_ENUM g_temp_thermal_CC_value = CHARGE_CURRENT_0_00_MA;
unsigned int g_usb_state = USB_UNCONFIGURED;
static bool usb_unlimited=false;
BATTERY_VOLTAGE_ENUM g_cv_voltage = BATTERY_VOLT_04_350000_V;
#if defined(CONFIG_MTK_HAFG_20)
unsigned int get_cv_voltage(void)
{
	return g_cv_voltage;
}
#endif

// ============================================================ //
// function prototype
// ============================================================ //

// ============================================================ //
//extern variable
// ============================================================ //
extern int g_platform_boot_mode;
#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
extern int get_AtCmdChargingModeOff(void);
#endif


// ============================================================ //
//extern function
// ============================================================ //
#ifdef CONFIG_LGE_PM_BACKLIGHT_CHG_CONTROL
extern int get_cur_main_lcd_level(void);
bool current_limit_unlock = true;
int check_sec = 1;
#endif

// ============================================================ //
void BATTERY_SetUSBState(int usb_state_value)
{
#if defined(CONFIG_POWER_EXT)
	battery_log(BAT_LOG_CRTI, "[BATTERY_SetUSBState] in FPGA/EVB, no service\n");
#else
	if ((usb_state_value < USB_SUSPEND) || (usb_state_value > USB_CONFIGURED)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] BAT_SetUSBState Fail! Restore to default value\n");
		usb_state_value = USB_UNCONFIGURED;
	} else {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] BAT_SetUSBState Success! Set %d\n", usb_state_value);
		g_usb_state = usb_state_value;
	}
#endif
}

unsigned int get_charging_setting_current(void)
{
	return g_temp_CC_value;
}

bool get_usb_current_unlimited(void)
{
	if (BMT_status.charger_type == STANDARD_HOST || BMT_status.charger_type == CHARGING_HOST)
		return usb_unlimited;
	else
		return false;
}

void set_usb_current_unlimited(bool enable)
{
	usb_unlimited = enable;
}

void select_charging_current_bcct(void)
{
	if (g_temp_thermal_CC_value < g_temp_input_CC_value) {
		g_temp_input_CC_value = g_temp_thermal_CC_value;
		battery_log(BAT_LOG_CRTI, "[BATTERY] bcct input current limit = %dmA\n",
			g_temp_input_CC_value / 100);

	}
}

static void pchr_turn_on_charging (void);
unsigned int set_bat_charging_current_limit(int current_limit)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] set_bat_charging_current_limit (%d)\n", current_limit);

	if (current_limit != -1) {
		g_bcct_flag=1;
		g_bcct_value = current_limit;
		g_temp_thermal_CC_value = current_limit * 100;
	} else {
		//change to default current setting
		g_bcct_flag=0;
		g_temp_thermal_CC_value = CHARGE_CURRENT_2000_00_MA;
	}

	pchr_turn_on_charging();

	return g_bcct_flag;
}

void select_charging_current(void)
{
#ifdef CONFIG_MACH_LGE
	/* Set MAX Charging current when factory cable connected */
	if (lge_get_board_cable() == LT_CABLE_56K ||
		lge_get_board_cable() == LT_CABLE_130K ||
		lge_get_board_cable() == LT_CABLE_910K) {
		g_temp_input_CC_value = CHARGE_CURRENT_2000_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		return;
	}
#endif

	switch (BMT_status.charger_type) {
	case CHARGER_UNKNOWN:
		g_temp_input_CC_value = CHARGE_CURRENT_0_00_MA;
		g_temp_CC_value = CHARGE_CURRENT_0_00_MA;
		break;
	case NONSTANDARD_CHARGER:
		g_temp_input_CC_value = NON_STD_AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	case STANDARD_CHARGER:
		g_temp_input_CC_value = AC_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	default:
		g_temp_input_CC_value = USB_CHARGER_CURRENT;
		g_temp_CC_value = CHARGE_CURRENT_2000_00_MA;
		break;
	}

#ifdef CONFIG_LGE_PM_CHARGING_SCENARIO
	/* Decrease charging current when battery is too hot */
	if (BMT_status.bat_charging_state == CHR_HOLD) {
		g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
		battery_log(BAT_LOG_CRTI, "[BATTERY] lcs current limit = %dmA\n",
			g_temp_CC_value / 100);
	}

#ifdef CONFIG_LGE_PM_BACKLIGHT_CHG_CONTROL
	if (BMT_status.bat_charging_state == CHR_CC && BMT_status.charger_type == STANDARD_CHARGER) {
		if (217 < get_cur_main_lcd_level()) {
			if (current_limit_unlock) {
				g_temp_CC_value = AC_CHARGER_CURRENT;
				pr_info("[SPIN] LCD On status current limit unlock = %dmA\n",
				g_temp_CC_value / 100);
			} else {
				g_temp_CC_value = CHARGE_CURRENT_400_00_MA;
				pr_info("[SPIN] LCD On status current limit lock = %dmA\n",
				g_temp_CC_value / 100);
			}
		} else {
			g_temp_CC_value = AC_CHARGER_CURRENT;
			pr_info("[SPIN] LCD Off status current limit = %dmA\n",
				g_temp_CC_value / 100);
		}
	}
#endif

#endif


#ifdef CONFIG_LGE_PM_USB_CURRENT_MAX
	/* To avoid power-off in ATS test, increase charging current */
	if (BMT_status.usb_current_max_enabled) {
		if (g_temp_input_CC_value < CHARGE_CURRENT_700_00_MA) {
			g_temp_input_CC_value = CHARGE_CURRENT_700_00_MA;
			battery_log(BAT_LOG_CRTI, "[BATTERY] USB current MAX Mode enabled.\n");
		}
	}
#endif
#if 0	/* enable this after thermal config is done */
	if (g_bcct_flag) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] select_charging_current_bcct\n");
		select_charging_current_bcct();
	}
#endif
}

static int recharging_check(void)
{
	if (BMT_status.bat_charging_state != CHR_BATFULL)
		return false;

	if (BMT_status.SOC < 100)
		return true;

	if (BMT_status.bat_vol <= RECHARGING_VOLTAGE)
		return true;

	return false;
}

static int eoc_check(void)
{
	int data = BMT_status.bat_vol;
	int eoc = false;

	if (!BMT_status.bat_exist)
		return false;

	if (BMT_status.bat_vol < RECHARGING_VOLTAGE)
		return false;

	battery_charging_control(CHARGING_CMD_GET_CHARGING_STATUS, &data);
	eoc = data;

	battery_log(BAT_LOG_CRTI, "[BATTERY] EOC = %d\n", eoc);
	if (eoc == true)
		return true;

	return false;
}

static void pchr_turn_on_charging (void)
{
	unsigned int charging_enable = true;

	if (BMT_status.bat_charging_state == CHR_ERROR) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Charger Error, turn OFF charging\n");
		charging_enable = false;
	} else if ((g_platform_boot_mode==META_BOOT) || (g_platform_boot_mode==ADVMETA_BOOT)) {
		battery_log(BAT_LOG_CRTI,
			"[BATTERY] In meta or advanced meta mode, disable charging.\n");
		charging_enable = false;
	} else if (BMT_status.bat_charging_state == CHR_BATFULL) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Full, turn OFF charging\n");
		charging_enable = false;
	} else {
		/*HW initialization*/
		battery_charging_control(CHARGING_CMD_INIT, NULL);

		battery_log(BAT_LOG_FULL, "charging_hw_init\n" );

		/* Set Charging Current */
		select_charging_current();

		if (g_temp_CC_value == CHARGE_CURRENT_0_00_MA
			|| g_temp_input_CC_value == CHARGE_CURRENT_0_00_MA) {
			charging_enable = false;

			battery_log(BAT_LOG_CRTI,
				"[BATTERY] charging current is set 0mA, turn off charging\n");
		} else {
			battery_charging_control(CHARGING_CMD_SET_INPUT_CURRENT,
				&g_temp_input_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CURRENT, &g_temp_CC_value);
			battery_charging_control(CHARGING_CMD_SET_CV_VOLTAGE, &g_cv_voltage);
		}
	}

	/* enable/disable charging */
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	battery_log(BAT_LOG_FULL, "[BATTERY] pchr_turn_on_charging(), enable=%d\n", charging_enable);
}

PMU_STATUS BAT_PreChargeModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Pre-CC mode charge, timer=%u on %u\n", BMT_status.PRE_charging_time, BMT_status.total_charging_time);

#ifdef CONFIG_LGE_PM_BATTERY_THREAD_TIME
	BMT_status.PRE_charging_time += BAT_TASK_PERIOD_LOW_BAT;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD_LOW_BAT;
#else
	BMT_status.PRE_charging_time += BAT_TASK_PERIOD;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;
#endif

	if ( BMT_status.bat_vol > V_PRE2CC_THRES ) {
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_ConstantCurrentModeAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] CC mode charge, timer=%u on %u !!\n", BMT_status.CC_charging_time, BMT_status.total_charging_time);

#ifdef CONFIG_LGE_PM_BATTERY_THREAD_TIME
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD_LOW_BAT;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD_LOW_BAT;
#else
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time += BAT_TASK_PERIOD;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.total_charging_time += BAT_TASK_PERIOD;
#endif

	if (eoc_check() == true) {
		if (BMT_status.SOC >= 100) {
			BMT_status.bat_charging_state = CHR_BATFULL;
		}
		BMT_status.bat_full = true;
		g_charging_full_reset_bat_meter = true;
	}

#ifdef CONFIG_LGE_PM_BACKLIGHT_CHG_CONTROL
	if (BMT_status.total_charging_time % 60 == 0) {
		if(check_sec & 0x2){
			check_sec = 0;
			current_limit_unlock = false;
		}else{
			check_sec++;
			current_limit_unlock = true;
		}
	}
#endif

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryFullAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Battery full !!\n");

	BMT_status.bat_full = true;
	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;
	BMT_status.bat_in_recharging_state = false;

	if (recharging_check() == true) {
		battery_log(BAT_LOG_CRTI, "[BATTERY] Battery Re-charging !!\n");

		BMT_status.bat_in_recharging_state = true;
		BMT_status.bat_full = false;
		BMT_status.bat_charging_state = CHR_CC;
	}

	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryHoldAction(void)
{
	battery_log(BAT_LOG_CRTI, "[BATTERY] Hold mode !!\n");

	/* Enable charger */
	pchr_turn_on_charging();

	return PMU_STATUS_OK;
}

PMU_STATUS BAT_BatteryStatusFailAction(void)
{
	unsigned int charging_enable;

	battery_log(BAT_LOG_CRTI, "[BATTERY] BAD Battery status... Charging Stop !!\n");

	BMT_status.total_charging_time = 0;
	BMT_status.PRE_charging_time = 0;
	BMT_status.CC_charging_time = 0;
	BMT_status.TOPOFF_charging_time = 0;
	BMT_status.POSTFULL_charging_time = 0;

	/*  Disable charger */
	charging_enable = false;
	battery_charging_control(CHARGING_CMD_ENABLE, &charging_enable);

	return PMU_STATUS_OK;
}

void mt_battery_charging_algorithm()
{
	battery_charging_control(CHARGING_CMD_RESET_WATCH_DOG_TIMER, NULL);

#ifdef CONFIG_LGE_PM_AT_CMD_SUPPORT
	if (get_AtCmdChargingModeOff()) {
		if (BMT_status.bat_charging_state != CHR_ERROR) {
			BMT_status.bat_charging_state = CHR_ERROR;
			pchr_turn_on_charging();
		}

		battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
		return;
	}
#endif

#ifdef CONFIG_LGE_PM_BATTERY_ID
	if (BMT_status.bat_exist && lge_get_battery_id() == BATT_ID_UNKNOWN) {
		/* Invalid battery inserted. Stop charging */
		BMT_status.bat_charging_state = CHR_ERROR;
	}
#endif

	battery_log(BAT_LOG_CRTI, "[BATTERY] Charging State = 0x%x\n", BMT_status.bat_charging_state);
	switch (BMT_status.bat_charging_state) {
	case CHR_PRE :
		/* Default State */
		BAT_PreChargeModeAction();
		break;
	case CHR_CC :
		/* Normal Charging */
		BAT_ConstantCurrentModeAction();
		break;
	case CHR_BATFULL:
		/* End of Charging */
		BAT_BatteryFullAction();
		break;
	case CHR_HOLD:
		/* Current decreased by OTP */
		BAT_BatteryHoldAction();
		break;
	case CHR_ERROR:
		/* Charging Stop by OTP */
		BAT_BatteryStatusFailAction();
		break;
	default:
		battery_log(BAT_LOG_CRTI, "[BATTERY] Should not be in here. Check the code.\n");
		BMT_status.bat_charging_state = CHR_PRE;
		break;
	}

	battery_charging_control(CHARGING_CMD_DUMP_REGISTER, NULL);
}

