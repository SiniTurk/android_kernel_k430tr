#include <linux/types.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>

#include <linux/delay.h>
#include <linux/reboot.h>
#include <mt-plat/mt_boot.h>
#include <mt-plat/battery_common.h>
#include <mach/mt_charging.h>
#include <mach/mt_pmic.h>
#include <linux/power_supply.h>

#include <rt9536.h>

/* ============================================================ // */
/* define */
/* ============================================================ // */
#define STATUS_OK	0
#define STATUS_UNSUPPORTED	-1

/* ============================================================ // */
/* global variable */
/* ============================================================ // */

/* ============================================================ // */
/* internal variable */
/* ============================================================ // */
#if 0
static char *support_charger[] = {
	"rt9536",
};
#endif				/* 0 */
/* static struct power_supply *external_charger = NULL; */

/* static kal_bool charging_type_det_done = KAL_TRUE; */
/* static CHARGER_TYPE g_charger_type = CHARGER_UNKNOWN; */

/* static unsigned int g_charging_enabled = 0; */
static unsigned int g_charging_current;
/* static unsigned int g_charging_current_limit = 0; */
/* static unsigned int g_charging_voltage = 0; */
/* static unsigned int g_charging_setting_chagned = 0; */

/* ============================================================ // */
/* extern variable */
/* ============================================================ // */

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
/* extern unsigned int upmu_get_reg_value(unsigned int reg); */
/* extern void Charger_Detect_Init(void); */
/* extern void Charger_Detect_Release(void); */
/* extern void mt_power_off(void); */

#define GETARRAYNUM(array) (sizeof(array)/sizeof(array[0]))

/* Internal APIs for PowerSupply */
#if 0
static int is_property_support(enum power_supply_property prop)
{
	int support = 0;
	int i;

	if (!external_charger)
		return 0;

	if (!external_charger->get_property)
		return 0;

	for (i = 0; i < external_charger->num_properties; i++) {
		if (external_charger->properties[i] == prop) {
			support = 1;
			break;
		}
	}

	return support;
}

static int is_property_writeable(enum power_supply_property prop)
{
	if (!external_charger->set_property)
		return 0;

	if (!external_charger->property_is_writeable)
		return 0;

	return external_charger->property_is_writeable(external_charger, prop);
}

static int get_property(enum power_supply_property prop, int *data)
{
	union power_supply_propval val;
	int rc = 0;

	*(int *)data = STATUS_UNSUPPORTED;

	if (!is_property_support(prop))
		return 0;

	rc = external_charger->get_property(external_charger, prop, &val);
	if (rc) {
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to get property %d\n", prop);
		*(int *)data = 0;
		return rc;
	}

	*(int *)data = val.intval;

	battery_log(BAT_LOG_CRTI, "[CHARGER] set property %d to %d\n", prop, val.intval);
	return rc;
}

static int set_property(enum power_supply_property prop, int data)
{
	union power_supply_propval val;
	int rc = 0;

	if (!is_property_writeable(prop))
		return 0;

	val.intval = data;
	rc = external_charger->set_property(external_charger, prop, &val);
	if (rc)
		battery_log(BAT_LOG_CRTI, "[CHARGER] failed to set property %d\n", prop);

	battery_log(BAT_LOG_CRTI, "[CHARGER] set property %d to %d\n", prop, data);
	return rc;
}
#endif				/* 0 */

/* Charger Control Interface Handler */
static unsigned int charging_hw_init(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(MTK_WIRELESS_CHARGER_SUPPORT)
	if (wireless_charger_gpio_number != 0) {
		mt_set_gpio_mode(wireless_charger_gpio_number, 0);	/* 0:GPIO mode */
		mt_set_gpio_dir(wireless_charger_gpio_number, 0);	/* 0: input, 1: output */
	}
#endif

	return status;
}

static unsigned int charging_dump_register(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static unsigned int charging_enable(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int enable = *(unsigned int *)(data);

	if (KAL_TRUE == enable)
		rt9536_charging_enable(g_charging_current, enable);
	else {
		g_charging_current = 0;
		rt9536_charging_enable(g_charging_current, enable);
	}

	battery_log(BAT_LOG_CRTI,
		    "[%s][charger_rt9536] charger enable = %d, g_charging_current = %d \r\n",
		    __func__, enable, g_charging_current);

	return status;
}

static unsigned int charging_set_cv_voltage(void *data)
{
	unsigned int status = STATUS_OK;
	/* unsigned int cv_value = *(unsigned int *)(data); */

	/* TO DO */
	battery_log(BAT_LOG_FULL, "[%s][charger_rt9536] NO Action! \r\n", __func__);

	return status;
}

static unsigned int charging_get_current(void *data)
{
	unsigned int status = STATUS_OK;

	/* TO DO */
	battery_log(BAT_LOG_CRTI, "[%s][charger_rt9536] g_charging_current = %d! \r\n", __func__,
		    g_charging_current);

	return status;
}

static unsigned int charging_set_current(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int current_value = *(unsigned int *)data;

	if ((current_value == AC_CHARGER_CURRENT) || (current_value == USB_CHARGER_CURRENT))
		g_charging_current = current_value;
#if 0
	else if (current_value == CHARGING_HOST_CHARGER_CURRENT) {
		battery_log(BAT_LOG_CRTI,
			    "[%s][charger_rt9536] temp patch by bcct setting!!!!!!!!\n", __func__);
		g_charging_current = AC_CHARGER_CURRENT;
	}
#endif				/* 0 */
	else
		g_charging_current = USB_CHARGER_CURRENT;


	battery_log(BAT_LOG_CRTI,
		    "[%s][charger_rt9536] current_value = %d,     g_charging_current = %d \r\n",
		    __func__, current_value, g_charging_current);

	return status;
}

static unsigned int charging_set_input_current(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static unsigned int charging_get_charging_status(void *data)
{
	unsigned int status = STATUS_OK;

	*(unsigned int *)data = rt9536_check_eoc_status();

	battery_xlog_printk(BAT_LOG_CRTI, "[%s][charger_rt9536] EOC_status = %d \r\n", __func__,
			    *(unsigned int *)data);

	return status;
}

static unsigned int charging_reset_watch_dog_timer(void *data)
{
	/* nothing to do */
	return STATUS_OK;
}

static unsigned int charging_set_hv_threshold(void *data)
{
#if 0
	unsigned int status = STATUS_OK;
	unsigned int set_hv_voltage;
	unsigned int array_size;
	unsigned short register_value;
	unsigned int voltage = *(unsigned int *)(data);

	array_size = GETARRAYNUM(VCDT_HV_VTH);
	set_hv_voltage = bmt_find_closest_level(VCDT_HV_VTH, array_size, voltage);
	register_value = charging_parameter_to_value(VCDT_HV_VTH, array_size, set_hv_voltage);
	pmic_set_register_value(PMIC_RG_VCDT_HV_VTH, register_value);
	return status;
#else
	return STATUS_OK;
#endif				/* 0 */
}

static unsigned int charging_get_hv_status(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;
	pr_notice("[charging_get_hv_status] charger ok for bring up.\n");
#else
	*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_VCDT_HV_DET);
#endif

	return status;
}

static unsigned int charging_get_battery_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(kal_bool *) (data) = 0;	/* battery exist */
	battery_log(BAT_LOG_CRTI, "[charging_get_battery_status] battery exist for bring up.\n");
#else
	val = pmic_get_register_value(PMIC_BATON_TDET_EN);
	battery_log(BAT_LOG_FULL, "[charging_get_battery_status] BATON_TDET_EN = %d\n", val);
	if (val) {
		pmic_set_register_value(PMIC_BATON_TDET_EN, 1);
		pmic_set_register_value(PMIC_RG_BATON_EN, 1);
		*(kal_bool *) (data) = pmic_get_register_value(PMIC_RGS_BATON_UNDET);
	} else
		*(kal_bool *) (data) = KAL_FALSE;
#endif

	return status;
}

static unsigned int charging_get_charger_det_status(void *data)
{
	unsigned int status = STATUS_OK;
	unsigned int val = 0;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	val = 1;
	battery_log(BAT_LOG_CRTI, "[charging_get_charger_det_status] chr exist for fpga.\n");
#else
	val = pmic_get_register_value(PMIC_RGS_CHRDET);
#endif

	*(kal_bool *) (data) = val;

	return status;
}

/* extern int hw_charging_get_charger_type(void); */

static unsigned int charging_get_charger_type(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
	*(CHARGER_TYPE *) (data) = STANDARD_HOST;
#else
	*(CHARGER_TYPE *) (data) = hw_charging_get_charger_type();
#endif

	return status;
}

static unsigned int charging_get_is_pcm_timer_trigger(void *data)
{
	unsigned int status = STATUS_OK;
#if 0
	if (slp_get_wake_reason() == WR_PCM_TIMER)
		*(kal_bool *) (data) = KAL_TRUE;
	else
		*(kal_bool *) (data) = KAL_FALSE;

	battery_log(BAT_LOG_CRTI, "[CHARGER] slp_get_wake_reason=%d\n", slp_get_wake_reason());
#endif
	return status;
}

static unsigned int charging_set_platform_reset(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_platform_reset\n");

	kernel_restart("battery service reboot system");
	/* arch_reset(0,NULL); */
#endif

	return status;
}

static unsigned int charging_get_platform_boot_mode(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	*(unsigned int *)(data) = get_boot_mode();

	battery_log(BAT_LOG_CRTI, "get_boot_mode=%d\n", get_boot_mode());
#endif

	return status;
}

static unsigned int charging_set_power_off(void *data)
{
	unsigned int status = STATUS_OK;

#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)
#else
	battery_log(BAT_LOG_CRTI, "charging_set_power_off\n");
	kernel_power_off();
#endif

	return status;
}

static unsigned int charging_get_power_source(void *data)
{
#if 0
	unsigned int status = STATUS_OK;

	*(kal_bool *) data = KAL_FALSE;

	return status;
#else
	return STATUS_UNSUPPORTED;
#endif				/* 0 */
}

static unsigned int charging_get_csdac_full_flag(void *data)
{
#if 0
	unsigned int status = STATUS_OK;

	*(kal_bool *) data = KAL_FALSE;

	return status;
#else
	return STATUS_UNSUPPORTED;
#endif				/* 0 */
}

static unsigned int charging_set_ta_current_pattern(void *data)
{
#if 0
	unsigned int status = STATUS_OK;

	*(kal_bool *) data = KAL_FALSE;

	return status;
#else
	return STATUS_UNSUPPORTED;
#endif				/* 0 */
}

static unsigned int charging_set_error_state(void *data)
{
	return STATUS_UNSUPPORTED;
}

/*
 * FUNCTION
 *		Internal_chr_control_handler
 *
 * DESCRIPTION
 *		 This function is called to set the charger hw
 *
 * CALLS
 *
 * PARAMETERS
 *		None
 *
 * RETURNS
 *
 *
 * GLOBALS AFFECTED
 *	   None
 */

static unsigned int (*const charging_func[CHARGING_CMD_NUMBER]) (void *data) = {
charging_hw_init,
	    charging_dump_register,
	    charging_enable,
	    charging_set_cv_voltage,
	    charging_get_current,
	    charging_set_current,
	    charging_set_input_current,
	    charging_get_charging_status,
	    charging_reset_watch_dog_timer,
	    charging_set_hv_threshold,
	    charging_get_hv_status,
	    charging_get_battery_status,
	    charging_get_charger_det_status,
	    charging_get_charger_type,
	    charging_get_is_pcm_timer_trigger,
	    charging_set_platform_reset,
	    charging_get_platform_boot_mode,
	    charging_set_power_off,
	    charging_get_power_source,
	    charging_get_csdac_full_flag,
	    charging_set_ta_current_pattern, charging_set_error_state};

signed int chr_control_interface(CHARGING_CTRL_CMD cmd, void *data)
{
	signed int status;

	if (cmd < CHARGING_CMD_NUMBER)
		status = charging_func[cmd] (data);
	else
		return STATUS_UNSUPPORTED;

	return status;
}
