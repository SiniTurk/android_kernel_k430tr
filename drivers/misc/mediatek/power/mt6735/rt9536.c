#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>

#include <mt-plat/charging.h>

#include <rt9536.h>

struct rt9536_info g_rt9536_info;

static DEFINE_MUTEX(rt9536_lock);
static DEFINE_SPINLOCK(rt9536_spin);

/* enum power_supply_type rt9536_status; */

kal_bool chargin_hw_init_done = KAL_FALSE;

/* Function Prototype */
/* static void rt9536_initialize(void); */
#if 0
static irqreturn_t rt9536_interrupt_handler(int irq, void *data);
#endif				/* 0 */

struct timer_list charging_timer;


static DECLARE_WAIT_QUEUE_HEAD(charger_set_gpio_ctrl_waiter);
#if 0
static void rt9536_set_en_set(struct rt9536_info *info, int high)
{
#if 0
	if (!high)
		pinctrl_select_state(info->pin, info->charging);
	else
		pinctrl_select_state(info->pin, info->not_charging);
#else
	if (high)
		gpio_set_value(8, 1);
	else
		gpio_set_value(8, 0);
#endif				/* 0 */
}


static int rt9536_get_en_set(struct rt9536_info *info)
{
	return gpio_get_value(info->en_set);
}
#endif				/* 0 */

static int rt9536_get_chgsb(struct rt9536_info *info)
{
	return gpio_get_value(info->chgsb);
}

static int rt9536_gpio_init(struct rt9536_info *info)
{
	pinctrl_select_state(info->pin, info->boot);

	gpio_request(info->en_set, "en_set");
	gpio_request(info->chgsb, "chgsb");

	return 0;
}

void rt9536_set_chargingmode(enum rt9536_mode mode)
{
	int i = 0;
	int pulse_cnt = 0;

	RT9536_DEBUG("[charger_rt9536] rt9536_set_chargingmode(%d)\n", g_rt9536_info.en_set);

	switch (mode) {
	case RT9536_MODE_USB500:
		pulse_cnt = 0;
		break;
	case RT9536_MODE_ISET:
		pulse_cnt = 1;
		break;
	case RT9536_MODE_USB100:
		pulse_cnt = 2;
		break;
	case RT9536_MODE_PTM:
		pulse_cnt = 3;
		break;
	case RT9536_MODE_DENACTIVE:
		pulse_cnt = 0;
		break;
	case RT9536_MODE_UNKNOWN:
	default:
		break;
	}

	if ((mode != RT9536_MODE_DENACTIVE) && (mode != RT9536_MODE_UNKNOWN)) {
		spin_lock(&rt9536_spin);

		gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ZERO);

		for (i = 0; i < pulse_cnt; i++) {
			udelay(110);
			gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ONE);
			udelay(110);
			gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ZERO);
		}

		spin_unlock(&rt9536_spin);

		udelay(1500);

		spin_lock(&rt9536_spin);

		gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ONE);
		udelay(770);	/* about 770 us */
		gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ZERO);

		spin_unlock(&rt9536_spin);
	} else
		gpio_set_value(g_rt9536_info.en_set, RT9536_PIN_ONE);

}

enum power_supply_type get_rt9536_status(void)
{
	return g_rt9536_info.status;
}

/* USB500 mode charging */
void rt9536_active_default(void)
{

	RT9536_DEBUG("[charger_rt9536] rt9536_active_default, status(%d)\n", g_rt9536_info.status);

	if (g_rt9536_info.status == POWER_SUPPLY_TYPE_USB) {
		RT9536_DEBUG("[charger_rt9536] :: it's already rt9536_active_default mode!!\n");
		return;
	}

	if (g_rt9536_info.status != POWER_SUPPLY_TYPE_BATTERY)
		rt9536_deactive();

	mutex_lock(&rt9536_lock);

	/* USB500 mode */
	rt9536_set_chargingmode(RT9536_MODE_USB500);

	g_rt9536_info.status = POWER_SUPPLY_TYPE_USB;

	mutex_unlock(&rt9536_lock);

	RT9536_DEBUG("[charger_rt9536] rt9536_active_default\n");

}
EXPORT_SYMBOL(rt9536_active_default);

/* TA connection, ISET mode */
void rt9536_set_ta_mode(void)
{

	RT9536_DEBUG("[charger_rt9536] rt9536_set_ta_mode\n");

	if (g_rt9536_info.status == POWER_SUPPLY_TYPE_MAINS) {
		RT9536_DEBUG("[charger_rt9536] it's already rt9536_set_ta_mode mode!!\n");
		return;
	}

	if (g_rt9536_info.status != POWER_SUPPLY_TYPE_BATTERY)
		rt9536_deactive();

	mutex_lock(&rt9536_lock);

	rt9536_set_chargingmode(RT9536_MODE_ISET);

	g_rt9536_info.status = POWER_SUPPLY_TYPE_MAINS;

	mutex_unlock(&rt9536_lock);

	RT9536_DEBUG("[charger_rt9536] rt9536_set_ta_mode\n");
}

void rt9536_set_usb_mode(void)
{
	rt9536_active_default();
}

void rt9536_set_factory_mode(void)
{

	RT9536_DEBUG("[charger_rt9536] rt9536_set_factory_mode\n");

	if (g_rt9536_info.status != POWER_SUPPLY_TYPE_BATTERY)
		rt9536_deactive();

	mutex_lock(&rt9536_lock);

	rt9536_set_chargingmode(RT9536_MODE_PTM);

	mutex_unlock(&rt9536_lock);

	RT9536_DEBUG("[charger_rt9536] rt9536_set_factory_mode\n");
}

void rt9536_deactive(void)
{

	if (g_rt9536_info.status == POWER_SUPPLY_TYPE_BATTERY) {
		RT9536_DEBUG("[charger_rt9536] it's already rt9536_deactive mode!!\n");
		return;
	}

	mutex_lock(&rt9536_lock);

	rt9536_set_chargingmode(RT9536_MODE_DENACTIVE);

	g_rt9536_info.status = POWER_SUPPLY_TYPE_BATTERY;

	mutex_unlock(&rt9536_lock);

	RT9536_DEBUG("[charger_rt9536] rt9536_deactive\n");
}
EXPORT_SYMBOL(rt9536_deactive);

void rt9536_charging_enable(unsigned int set_current, unsigned int enable)
{
	if (enable) {
#if 0
		if ((set_current == AC_CHARGER_CURRENT)
		    || (set_current == CHARGING_HOST_CHARGER_CURRENT))
#else
		if (set_current == AC_CHARGER_CURRENT)
#endif				/* 0 */
			rt9536_set_ta_mode();
		else if (set_current == USB_CHARGER_CURRENT)
			rt9536_set_usb_mode();
		else
			rt9536_active_default();

		RT9536_DEBUG("[charger_rt9536] :: %s, current(%d), enable(%d)\n", __func__,
			     set_current, enable);
	} else {
		rt9536_deactive();
		RT9536_DEBUG("[charger_rt9536] :: %s, enable(%d)\n", __func__, enable);
	}

}

unsigned char rt9536_check_eoc_status(void)
{
	unsigned char eoc_status = 0;

	/* TO DO */
	eoc_status = rt9536_get_chgsb(&g_rt9536_info);

	if (eoc_status == 1) {
		RT9536_DEBUG("[charger_rt9536] :: (%s) eoc_status(%d)\n", __func__, eoc_status);
		return 1;
	}

	RT9536_DEBUG("[charger_rt9536] :: (%s) eoc_status(%d)\n", __func__, eoc_status);
	return 0;
}

#if 0
static void rt9536_initialize(void)
{
	g_rt9536_info.status = POWER_SUPPLY_TYPE_BATTERY;
}

static enum power_supply_property rt9536_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static struct power_supply rt9536_psy = {
	.name = "rt9536",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties = rt9536_props,
	.num_properties = ARRAY_SIZE(rt9536_props),
};
#endif				/* 0 */

static struct of_device_id rt9536_of_device_id[] = {
	{
	 .compatible = "richtek,rt9536",
	 },
};

static int rt9536_parse_dt(struct platform_device *pdev)
{
	int rc;
	struct device_node *node = NULL;

	node = of_find_matching_node(node, rt9536_of_device_id);

	rc = of_property_read_u32(node, "en_set", &g_rt9536_info.en_set);
	if (rc) {
		RT9536_ERROR("[charger_rt9536] en_set not defined.\n");
		return rc;
	}

	rc = of_property_read_u32(node, "chgsb", &g_rt9536_info.chgsb);
	if (rc) {
		RT9536_ERROR("[charger_rt9536] chgsb not defined.\n");
		return rc;
	}

	g_rt9536_info.pin = devm_pinctrl_get(&pdev->dev);

	g_rt9536_info.boot = pinctrl_lookup_state(g_rt9536_info.pin, "default");
	g_rt9536_info.charging = pinctrl_lookup_state(g_rt9536_info.pin, "charging");
	g_rt9536_info.not_charging = pinctrl_lookup_state(g_rt9536_info.pin, "not_charging");

	return 0;
}

static int rt9536_probe(struct platform_device *pdev)
{
	int rc;

	RT9536_DEBUG("[RT9536]rt9536_probe\n");

	/* initialize device info */
	g_rt9536_info.mode = RT9536_MODE_DENACTIVE;
	g_rt9536_info.status = POWER_SUPPLY_TYPE_BATTERY;

	/* read device tree */
	rc = rt9536_parse_dt(pdev);
	if (rc) {
		pr_err("[rt9536] cannot read from fdt.\n");
		return rc;
	}

	/* initialize device */
	rt9536_gpio_init(&g_rt9536_info);

	/* register class */
#if 0
	info.psy = rt9536_psy;
	rc = power_supply_register(&pdev->dev, &info.psy);
	if (rc) {
		dev_err(&pdev->dev, "power supply register failed.\n");
		return rc;
	}
#endif				/* 0 */

	/* rt9536_initialize(); */

	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

static int rt9536_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver rt9536_driver = {
	.probe = rt9536_probe,
	.remove = rt9536_remove,
	.driver = {
		   .name = "rt9536",
		   .owner = THIS_MODULE,
		   .of_match_table = rt9536_of_device_id,
		   },
};

static int __init rt9536_init(void)
{

	RT9536_ERROR("[RT9536] rt9536_init\n");

	if (platform_driver_register(&rt9536_driver))
		return -ENODEV;

	return 0;
}

static void __exit rt9536_exit(void)
{
	platform_driver_unregister(&rt9536_driver);
}
module_init(rt9536_init);
module_exit(rt9536_exit);

MODULE_DESCRIPTION("Richtek RT9536 Driver");
MODULE_LICENSE("GPL");


#if 0
static int rt9536_probe(struct platform_device *dev)
{

	mt_set_gpio_mode(CHG_EN_SET_N, CHG_EN_MODE);
	mt_set_gpio_dir(CHG_EN_SET_N, CHG_EN_DIR);
	mt_set_gpio_out(CHG_EN_SET_N, GPIO_OUT_ONE);	/* charging off ; HIGH */
	/* mt_set_gpio_out(CHG_EN_SET_N, CHG_EN_DATA_OUT);  // charging off ; HIGH */

	mt_set_gpio_mode(CHG_EOC_N, CHG_EOC_MODE);
	mt_set_gpio_dir(CHG_EOC_N, CHG_EOC_DIR);
	mt_set_gpio_pull_enable(CHG_EOC_N, CHG_EOC_PULL_ENABLE);
	mt_set_gpio_pull_select(CHG_EOC_N, CHG_EOC_PULL_SELECT);

	RT9536_DEBUG("[charger_rt9536] :: charging IC Initialization is done\n");

	rt9536_initialize();

	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

static int rt9536_remove(struct platform_device *dev)
{
	rt9536_deactive();

	return 0;
}

#if 0
static int rt9536_suspend(struct platform_device *dev, pm_message_t state)
{
	RT9536_DEBUG("[charger_rt9536] :: rt9536_suspend\n");
	dev->dev.power.power_state = state;
	return 0;
}

static int rt9536_resume(struct platform_device *dev)
{
	RT9536_DEBUG("[charger_rt9536] :: rt9536_resume\n");
	dev->dev.power.power_state = PMSG_ON;
	return 0;
}
#endif				/* 0 */

static struct platform_driver rt9536_driver = {
	.probe = rt9536_probe,
	.remove = rt9536_remove,
/* .suspend = rt9536_suspend, */
/* .resume = rt9536_resume, */
	.driver = {
		   .name = "ext_charger",
		   .owner = THIS_MODULE,
		   },
};

static struct platform_device charger_ic_dev = {
	.name = "ext_charger",
	.id = -1,
};


static int __init rt9536_init(void)
{
	int ret = 0;

	RT9536_DEBUG("[charger_rt9536] Charging IC Driver Init\n");

	ret = platform_device_register(&charger_ic_dev);
	if (ret) {
		RT9536_ERROR("[charger_rt9536] Unable to device register(%d)\n", ret);
		return ret;
	}

	return platform_driver_register(&rt9536_driver);
}

static void __exit rt9536_exit(void)
{
	RT9536_DEBUG("[charger_rt9536] Charging IC Driver Exit\n");
	platform_driver_unregister(&rt9536_driver);
}
module_init(rt9536_init);
module_exit(rt9536_exit);
#endif				/* 0 */
