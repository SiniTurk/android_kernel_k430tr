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

//#include <mt-plat/mt_typedefs.h>
#include <mt-plat/charging.h>
#include <mach/mt_charging.h>
//#include <mt-plat/mt_gpio.h>

#include <rt9536.h>

extern kal_bool chargin_hw_init_done;

#define MSET_UDELAY_L 2000	/* delay for mode setting (1.5ms ~ ) */
#define MSET_UDELAY_H 2100

#define START_UDELAY_L 100	/* delay for mode setting ready (50us ~ ) */
#define START_UDELAY_H 150

#define ISET_UDELAY 150		/* delay for current setting (100 us ~700 us) */

#define HVSET_UDELAY 760	/* delay for voltage setting (750 us ~ 1 ms) */

#define RT9536_RETRY_MS	100

enum rt9536_mode {
	RT9536_MODE_DISABLED,
	RT9536_MODE_USB500,
	RT9536_MODE_ISET,
	RT9536_MODE_USB100,
	RT9536_MODE_FACTORY,
	RT9536_MODE_MAX,
};

struct rt9536_info {
	struct platform_device *pdev;
	struct power_supply psy;
	struct delayed_work dwork;

	/* gpio */
	int en_set;
	int chgsb;
	int pgb;

	struct pinctrl *pin;
	struct pinctrl_state *boot;
	struct pinctrl_state *charging;
	struct pinctrl_state *not_charging;

	/* current */
	int iset;

	/* status */
	int status;
	int enable;
	enum rt9536_mode mode;

	/* lock */
	struct mutex mode_lock;
	spinlock_t pulse_lock;

	int cv_voltage;		/* in uV */

	int cur_present;	/* in mA */
	int cur_next;		/* in mA */
};

static struct workqueue_struct *rt9536_wq = NULL;

static void rt9536_set_en_set(struct rt9536_info *info, int high)
{
	gpio_set_value(info->en_set, high);
}

static int rt9536_get_en_set(struct rt9536_info *info)
{
	return gpio_get_value(info->en_set);
}

static int rt9536_get_chgsb(struct rt9536_info *info)
{
	return gpio_get_value(info->chgsb);
}

static int rt9536_gpio_init(struct rt9536_info *info)
{
	gpio_request(info->en_set, "en_set");
	gpio_request(info->chgsb, "chgsb");

	if (gpio_get_value(info->en_set)) {
		pinctrl_select_state(info->pin, info->not_charging);
	} else {
		pinctrl_select_state(info->pin, info->charging);
	}

	return 0;
}

static int rt9536_gen_mode_pulse(struct rt9536_info *info, int pulses)
{
	int i;

	spin_lock(&info->pulse_lock);

	for(i = 0 ; i < pulses ; i++) {
		rt9536_set_en_set(info, 1);
		udelay(ISET_UDELAY);
		rt9536_set_en_set(info, 0);
		udelay(ISET_UDELAY);
	}

	spin_unlock(&info->pulse_lock);

	return 0;
}

static int rt9536_gen_hv_pulse(struct rt9536_info *info)
{
	unsigned long flags;	/* spin_lock_irqsave */

	spin_lock_irqsave(&info->pulse_lock, flags);

	rt9536_set_en_set(info, 1);
	udelay(HVSET_UDELAY);
	rt9536_set_en_set(info, 0);

	spin_unlock_irqrestore(&info->pulse_lock, flags);

	return 0;
}

static int rt9536_set_mode(struct rt9536_info *info, enum rt9536_mode mode)
{
	int rc = 0;

	mutex_lock(&info->mode_lock);

	if (mode == RT9536_MODE_DISABLED) {
		rt9536_set_en_set(info, 1);
		usleep_range(MSET_UDELAY_L, MSET_UDELAY_H);

		info->mode = mode;
		info->cur_present = 0;
		info->status = POWER_SUPPLY_STATUS_NOT_CHARGING;

		mutex_unlock(&info->mode_lock);
		return rc;
	}

	if (info->mode == mode) {
		mutex_unlock(&info->mode_lock);
		return rc;
	}

	/* to switch mode, need to disable charger ic */
	if (info->status == POWER_SUPPLY_STATUS_UNKNOWN) {
		/* do not disable charger ic for first command */
	} else {
		rt9536_set_en_set(info, 1);
		usleep_range(MSET_UDELAY_L, MSET_UDELAY_H);
	}

	/* start setting mode */
	rt9536_set_en_set(info, 0);
	usleep_range(START_UDELAY_L, START_UDELAY_H);

	switch(mode) {
	case RT9536_MODE_USB500:	/* 4 pulses */
		rt9536_gen_mode_pulse(info, 4);
		break;
	case RT9536_MODE_FACTORY:	/* 3 pulses */
		rt9536_gen_mode_pulse(info, 3);
		break;
	case RT9536_MODE_USB100:	/* 2 pulses */
		rt9536_gen_mode_pulse(info, 2);
		break;
	case RT9536_MODE_ISET:		/* 1 pulse  */
		rt9536_gen_mode_pulse(info, 1);
		break;
	default:
		break;

	}

	/* mode set */
	usleep_range(MSET_UDELAY_L, MSET_UDELAY_H);

	/* need extra pulse to set high voltage */
	if (info->cv_voltage > 4200000) {
		rt9536_gen_hv_pulse(info);
	}

	info->mode = mode;
	info->cur_present = info->cur_next;
	info->status = POWER_SUPPLY_STATUS_CHARGING;

	mutex_unlock(&info->mode_lock);

	return rc;
}

static int rt9536_get_status(struct rt9536_info *info)
{

	int status;

	if (!rt9536_get_chgsb(info)) {
		status = POWER_SUPPLY_STATUS_CHARGING;
	} else {
		if (!rt9536_get_en_set(info))
			status = POWER_SUPPLY_STATUS_FULL;
		else
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}

	return status;
}

static int rt9536_enable(struct rt9536_info *info, int en)
{
	enum rt9536_mode mode = RT9536_MODE_DISABLED;
	int rc = 0;

	if (en) {
		if (info->cur_next > 1000)
			mode = RT9536_MODE_FACTORY;
		else if (info->cur_next > 500)
			mode = RT9536_MODE_ISET;
		else if (info->cur_next >= 400)
			mode = RT9536_MODE_USB500;
		else
			mode = RT9536_MODE_USB100;
	}

	rc = rt9536_set_mode(info, mode);
	if (rc) {
		dev_err(&info->pdev->dev, "faild to set rt9536 mode to %d.\n", mode);
	}

	return rc;
}

static void rt9536_dwork(struct work_struct *work)
{
	struct rt9536_info *info = container_of(to_delayed_work(work), struct rt9536_info, dwork);
	int rc;

	rc = rt9536_enable(info, info->enable);
	if (rc) {
		dev_err(&info->pdev->dev, "retry after %dms\n", RT9536_RETRY_MS);
		queue_delayed_work(rt9536_wq, &info->dwork, msecs_to_jiffies(RT9536_RETRY_MS));
	}
	return;
}

static enum power_supply_property rt9536_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CURRENT_MAX,
};

static int rt9536_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct rt9536_info *info = container_of(psy, struct rt9536_info, psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rt9536_get_status(info);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->cv_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		if (info->mode == RT9536_MODE_DISABLED)
			val->intval = 0;
		else
			val->intval = info->cur_present;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int rt9536_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct rt9536_info *info = container_of(psy, struct rt9536_info, psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		switch (val->intval) {
		case POWER_SUPPLY_STATUS_CHARGING:
			info->enable = 1;
			break;
		case POWER_SUPPLY_STATUS_DISCHARGING:
		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			info->enable = 0;
			break;
		default:
			rc = -EINVAL;
			break;
		}

		if (!rc)
			queue_delayed_work(rt9536_wq, &info->dwork, 0);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		info->cv_voltage = val->intval;
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		info->cur_next = val->intval;
		break;
	default:
		rc = -EINVAL;
		break;
	}

	return rc;
}

static int rt9536_property_is_writeable(struct power_supply *psy,
				     enum power_supply_property psp)
{
	int rc = 0;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		rc = 1;
		break;
	default:
		rc = 0;
		break;
	}
	return rc;
}

static struct power_supply rt9536_psy = {
	.name = "rt9536",
	.type = POWER_SUPPLY_TYPE_UNKNOWN,
	.properties	= rt9536_props,
	.num_properties = ARRAY_SIZE(rt9536_props),
	.get_property	= rt9536_get_property,
	.set_property	= rt9536_set_property,
	.property_is_writeable = rt9536_property_is_writeable,
};

static int rt9536_parse_dt(struct platform_device *pdev)
{
	struct rt9536_info *info =
		(struct rt9536_info*)platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node;
	int rc;

	rc = of_property_read_u32(np, "en_set" ,&info->en_set);
	if (rc) {
		dev_err(&pdev->dev, "en_set not defined.\n");
		return rc;
	}

	rc = of_property_read_u32(np, "chgsb" ,&info->chgsb);
	if (rc) {
		dev_err(&pdev->dev, "chgsb not defined.\n");
		return rc;
	}

	info->pin = devm_pinctrl_get(&pdev->dev);
	info->boot = pinctrl_lookup_state(info->pin, "default");
	info->charging = pinctrl_lookup_state(info->pin, "charging");
	info->not_charging = pinctrl_lookup_state(info->pin, "not_charging");

	return 0;
}

static int rt9536_probe(struct platform_device *pdev)
{
	struct rt9536_info *info;
	int rc;

	info = (struct rt9536_info*)kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "memory allocation failed.\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, info);

	/* initialize device info */
	info->pdev = pdev;
	info->cv_voltage = 4200000;
	info->mode = RT9536_MODE_DISABLED;
	info->status = POWER_SUPPLY_STATUS_UNKNOWN;

	spin_lock_init(&info->pulse_lock);
	mutex_init(&info->mode_lock);
	INIT_DELAYED_WORK(&info->dwork, rt9536_dwork);

	/* read device tree */
	rc = rt9536_parse_dt(pdev);
	if (rc) {
		dev_err(&pdev->dev, "cannot read from fdt.\n");
		return rc;
	}

	/* initialize device */
	rt9536_gpio_init(info);

	/* register class */
	info->psy = rt9536_psy;
	rc = power_supply_register(&pdev->dev, &info->psy);
	if (rc) {
		dev_err(&pdev->dev, "power supply register failed.\n");
		return rc;
	}

	chargin_hw_init_done = KAL_TRUE;

	return 0;
}

static int rt9536_remove(struct platform_device *pdev)
{
	struct rt9536_info *info;

	info = platform_get_drvdata(pdev);
	if (info)
		kfree(info);

	return 0;
}

static struct of_device_id rt9536_of_device_id[] = {
	{
		.compatible = "richtek,rt9536",
	},
};

static struct platform_driver rt9536_driver = {
	.probe		= rt9536_probe,
	.remove		= rt9536_remove,
	.driver		= {
		.name	= "rt9536",
		.owner	= THIS_MODULE,
		.of_match_table = rt9536_of_device_id,
	},
};

static int __init rt9536_init(void)
{
	rt9536_wq = create_singlethread_workqueue("rt9536_wq");
	if (!rt9536_wq) {
		printk("cannot create workqueue for rt9536\n");
		return -ENOMEM;
	}

	if (platform_driver_register(&rt9536_driver))
		return -ENODEV;

	return 0;
}

static void __exit rt9536_exit(void)
{
	platform_driver_unregister(&rt9536_driver);

	if (rt9536_wq)
		destroy_workqueue(rt9536_wq);
}
module_init(rt9536_init);
module_exit(rt9536_exit);

MODULE_DESCRIPTION("Richtek RT9536 Driver");
MODULE_LICENSE("GPL");

