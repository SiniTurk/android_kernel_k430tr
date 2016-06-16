#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/irqchip/mt-eic.h>
#include <mt_gpio.h>
#include <mach/gpio_const.h>

#if defined(TARGET_MT6753_K7)
#define GPIO_HALL_1_PIN (GPIO3 | 0x80000000)
#define GPIO_HALL_1_PIN_M_EINT GPIO_MODE_00
#else
#define GPIO_HALL_1_PIN (GPIO7 | 0x80000000)
#define GPIO_HALL_1_PIN_M_EINT GPIO_MODE_00
#endif
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1

#if 0
#ifndef TPD_NO_GPIO
#include "cust_eint.h"
#include "cust_gpio_usage.h"
#include "cust_eint_md1.h"
#endif
#endif
#if 0
#define EINTF_TRIGGER_FALLING 0x02
#define EINTF_TRIGGER_RISING 0x01
#define GPIO_PULL_DOWN 0
#define GPIO_PULL_UP 1
#endif
/* SMART COVER Support */
#define SMARTCOVER_POUCH_CLOSED		1
#define SMARTCOVER_POUCH_OPENED		0

#ifdef CONFIG_STYLUS_PEN_DETECTION
#define STYLUS_PEN_IN   1
#define STYLUS_PEN_OUT  0
#endif

#define HALL_IC_DEV_NAME "bu52061nvx"

struct mt6xxx_cradle_platform_data {
	int hallic_pouch_detect_pin;
	unsigned int hallic_pouch_irq;
	unsigned long irq_flags;
};

struct mt6xxx_cradle {
	struct switch_dev sdev;
	struct device *dev;
	struct wake_lock wake_lock;
	int pouch;
	spinlock_t lock;
	int state;
#ifdef CONFIG_STYLUS_PEN_DETECTION
	struct switch_dev pen_sdev;
	int pen;
	int pen_state;
#endif

	//const struct mt6xxx_cradle_platform_data *pdata;
	int hallic_pouch_detect_pin;
	struct device_node *irq_node;
	int	irq;
};

static struct delayed_work pouch_work;
#ifdef CONFIG_STYLUS_PEN_DETECTION
static struct delayed_work pen_work;
#endif

static struct workqueue_struct *cradle_wq;
static struct mt6xxx_cradle *cradle;

#if defined(CONFIG_CORNER_QUICK_COVER)
extern void MIT300_Set_BootCoverMode(int status);
#endif

#if defined(TARGET_S6) || defined(CONFIG_CORNER_QUICK_COVER)
int Touch_Quick_Cover_Closed = 0;
#endif
static void boot_cradle_det_func(void)
{
	int state;
	int gpio_status;
#if 0
#ifdef CONFIG_STYLUS_PEN_DETECTION
	int pen_state;
#endif
#endif
	// cradle->pouch is set means that the pouch is closed.
	if(cradle->hallic_pouch_detect_pin)
		gpio_status = gpio_get_value(cradle->hallic_pouch_detect_pin);
	cradle->pouch = !gpio_status;

	printk("%s : boot pouch ==> %d\n", __func__, cradle->pouch);

	if(cradle->pouch == 1)
		state = SMARTCOVER_POUCH_CLOSED;
	else
		state = SMARTCOVER_POUCH_OPENED;

	printk("%s : [Cradle] boot cradle state is %d\n", __func__, state);

	cradle->state = state;
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->sdev, cradle->state);

#if defined(TARGET_S6) || defined(CONFIG_CORNER_QUICK_COVER)
	if(!gpio_status)//Cover Closed
		Touch_Quick_Cover_Closed = 1;
	else
		Touch_Quick_Cover_Closed = 0;
#endif
#if defined(CONFIG_CORNER_QUICK_COVER)
	MIT300_Set_BootCoverMode(state);
#endif
#if 0
#ifdef CONFIG_STYLUS_PEN_DETECTION
	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	printk("%s : boot pen ==> %d\n", __func__, cradle->pen);

	if(cradle->pen == 1)
		pen_state = STYLUS_PEN_IN;
	else
		pen_state = STYLUS_PEN_OUT;

	printk("%s : [Cradle] boot cradle pen_state is %d\n", __func__, pen_state);

	cradle->pen_state = pen_state;
	wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
	switch_set_state(&cradle->pen_sdev, cradle->pen_state);
#endif
#endif
}

static void mt6xxx_pouch_work_func(struct work_struct *work)
{
	int state = 0;
	int gpio_status;
	unsigned long polarity;
	u32 pull;

	spin_lock_irq(&cradle->lock);
	printk("%s : interrupt\n", __func__);
	// cradle->pouch is set means that the pouch is closed.
	if(cradle->hallic_pouch_detect_pin)
		gpio_status = gpio_get_value(cradle->hallic_pouch_detect_pin);
	cradle->pouch = !gpio_status;

	printk("%s : pouch ==> %d\n", __func__, cradle->pouch);

	if (cradle->pouch == 1)
		state = SMARTCOVER_POUCH_CLOSED;
	else if (cradle->pouch == 0)
		state = SMARTCOVER_POUCH_OPENED;
#if defined(TARGET_S6) || defined(CONFIG_CORNER_QUICK_COVER)
	if(!gpio_status)//Cover Closed
		Touch_Quick_Cover_Closed = 1;
	else
		Touch_Quick_Cover_Closed = 0;
#endif

	if (cradle->state != state) {
		cradle->state = state;
		if (gpio_status == 1) {
			polarity = CUST_EINT_POLARITY_LOW;
			pull = GPIO_PULL_UP;
		}
		else {
			polarity = CUST_EINT_POLARITY_HIGH;
			pull = GPIO_PULL_DOWN;
		}

		mt_set_gpio_pull_select(GPIO_HALL_1_PIN, pull);
		mt_eint_set_polarity(cradle->hallic_pouch_detect_pin, polarity);

#if 0
#if defined(CONFIG_OF)
		enable_irq(cradle->irq);
#else
		mt_eint_unmask(CUST_EINT_HALL_1_NUM);
#endif
#endif
		spin_unlock_irq(&cradle->lock);

		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->sdev, cradle->state);

		printk("%s : [Cradle] pouch value is %d\n", __func__ , state);
	}
	else {
		printk("%s : [Cradle] pouch value is %d (no change)\n", __func__ , state);
#if 0
#if defined(CONFIG_OF)
		enable_irq(cradle->irq);
#else
		mt_eint_unmask(CUST_EINT_HALL_1_NUM);
#endif
#endif
		spin_unlock_irq(&cradle->lock);
	}

}

static void mt6xxx_pouch_irq_handler(void)
{
#if defined(TARGET_S6) || defined(CONFIG_CORNER_QUICK_COVER)
int gpio_status;
#endif
	printk("pouch irq!!!!\n");
#if 0
#if defined(CONFIG_OF)
	disable_irq(cradle->irq);
#else	
	mt_eint_mask(CUST_EINT_HALL_1_NUM);
#endif
#endif
#if defined(TARGET_S6) || defined(CONFIG_CORNER_QUICK_COVER)
	if(cradle->hallic_pouch_detect_pin)
		gpio_status = gpio_get_value(cradle->hallic_pouch_detect_pin);
	if(!gpio_status)//Cover Closed
		Touch_Quick_Cover_Closed = 1;
	else
		Touch_Quick_Cover_Closed = 0;
#endif
#if defined(CONFIG_CORNER_QUICK_COVER)
	if(cradle->hallic_pouch_detect_pin)
		gpio_status = gpio_get_value(cradle->hallic_pouch_detect_pin);

#endif
	queue_delayed_work(cradle_wq, &pouch_work, msecs_to_jiffies(200));
}

static int smart_cover_gpio_set(void)
{
    printk("[hall_ic]gpio_set start\n");
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	u32 pull;
	int gpio_status;
	unsigned int flag;
	cradle->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,hall_1");
	if (cradle->irq_node) {
		of_property_read_u32_array(cradle->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		printk("ints[0] = %d, ints[1] = %d!!\n", ints[0], ints[1]);
		cradle->hallic_pouch_detect_pin = (int)ints[0];
		cradle->irq = irq_of_parse_and_map(cradle->irq_node, 0);
		if (cradle->irq < 0) {
			printk(KERN_ERR "irq_of_parse_and_map IRQ LINE NOT AVAILABLE!.");
			return -1;
		}
		if(cradle->hallic_pouch_detect_pin)
			gpio_status = gpio_get_value(cradle->hallic_pouch_detect_pin);
		cradle->pouch = !gpio_status;
		if(cradle->pouch == 0) {
			flag = EINTF_TRIGGER_FALLING;
			pull = GPIO_PULL_UP;
		}
		else {
			flag = EINTF_TRIGGER_RISING;
			pull = GPIO_PULL_DOWN;
		}
		mt_set_gpio_pull_enable(GPIO_HALL_1_PIN, GPIO_PULL_ENABLE);
		mt_set_gpio_pull_select(GPIO_HALL_1_PIN, pull);
		ret = request_irq(cradle->irq, mt6xxx_pouch_irq_handler, flag,"hall_1", NULL);
		if (ret > 0) {
			printk(KERN_ERR "IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		if (enable_irq_wake(cradle->irq) == 0)
			printk("%s :enable_irq_wake Enable(1)\n",__func__);
		else
			printk("%s :enable_irq_wake failed(1)\n",__func__);

	} else {
		printk(KERN_ERR "null irq node!!\n");
		return -EINVAL;
	}
	return ret;
}

static ssize_t cradle_pouch_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pouch : %d -> %s\n", cradle->pouch, cradle->pouch == 0 ? "open" : "close");

	return len;
}
#if 1 /* block_warning_message */
static ssize_t cradle_pouch_store(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
#endif

static ssize_t cradle_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "cradle state : %d -> %s\n", cradle->state, cradle->state == 1 ? "open" : "close");

	return len;
}
#if 1 /* block_warning_message */
static ssize_t cradle_state_store(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
#endif

static struct device_attribute cradle_state_attr = __ATTR(state, S_IRUGO | S_IWUSR, cradle_state_show, cradle_state_store);
static struct device_attribute cradle_pouch_attr   = __ATTR(pouch, S_IRUGO | S_IWUSR, cradle_pouch_show, cradle_pouch_store);

#ifdef CONFIG_STYLUS_PEN_DETECTION
static void mt6xxx_pen_work_func(struct work_struct *work)
{
	int pen_state = 0;
	int gpio_status;
	unsigned long polarity;
	u32 pull;

	spin_lock_irq(&cradle->lock);

	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	printk("%s : pen ==> %d\n", __func__, cradle->pen);

	if (cradle->pen == 1)
		pen_state = STYLUS_PEN_IN;
	else if (cradle->pen == 0)
		pen_state = STYLUS_PEN_OUT;

	if (cradle->pen_state != pen_state) {
		cradle->pen_state = pen_state;
		if (gpio_status == 1) {
			polarity = CUST_EINT_POLARITY_LOW;
			pull = GPIO_PULL_UP;
		}
		else {
			polarity = CUST_EINT_POLARITY_HIGH;
			pull = GPIO_PULL_DOWN;
		}
		mt_set_gpio_pull_select(GPIO_HALL_2_PIN, pull);
		mt_eint_set_polarity(CUST_EINT_HALL_2_NUM, polarity);

		mt_eint_unmask(CUST_EINT_HALL_2_NUM);
		spin_unlock_irq(&cradle->lock);
	
		wake_lock_timeout(&cradle->wake_lock, msecs_to_jiffies(3000));
		switch_set_state(&cradle->pen_sdev, cradle->pen_state);
		printk("%s : [Cradle] pen value is %d\n", __func__ , pen_state);
	}
	else {
				mt_eint_unmask(CUST_EINT_HALL_2_NUM);
				spin_unlock_irq(&cradle->lock);
				printk("%s : [Cradle] pen value is %d (no change)\n", __func__ , pen_state);
	}


}

static void mt6xxx_pen_irq_handler(void)
{
	printk("pen irq!!!!\n");
	mt_eint_mask(CUST_EINT_HALL_2_NUM);
	queue_delayed_work(cradle_wq, &pen_work, msecs_to_jiffies(200));
}

static void stylus_pen_gpio_set(void)
{
	int gpio_status;
	unsigned int flag;
	u32 pull;

	gpio_status = mt_get_gpio_in(GPIO_HALL_2_PIN);
	cradle->pen = !gpio_status;

	if(cradle->pen == 0) {
		flag = EINTF_TRIGGER_FALLING;
		pull = GPIO_PULL_UP;
	}
	else {
		flag = EINTF_TRIGGER_RISING;
		pull = GPIO_PULL_DOWN;
	}

	/* initialize irq of gpio_hall */
	mt_set_gpio_mode(GPIO_HALL_2_PIN, GPIO_HALL_2_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_HALL_2_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_HALL_2_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_HALL_2_PIN, pull);

	mt_eint_set_hw_debounce(CUST_EINT_HALL_2_NUM, CUST_EINT_HALL_2_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_HALL_2_NUM, flag, mt6xxx_pen_irq_handler, 0);

	mt_eint_unmask(CUST_EINT_HALL_2_NUM);
}

static ssize_t cradle_pen_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pen : %d -> %s\n", cradle->pen, cradle->pen == 0 ? "out" : "in");

	return len;
}


static ssize_t cradle_pen_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "pen state : %d -> %s\n", cradle->pen_state, cradle->pen_state == 1 ? "out" : "in");

	return len;
}

static struct device_attribute cradle_pen_state_attr = __ATTR(pen_state, S_IRUGO | S_IWUSR, cradle_pen_state_show, NULL);
static struct device_attribute cradle_pen_attr   = __ATTR(pen, S_IRUGO | S_IWUSR, cradle_pen_show, NULL);
#endif

static ssize_t cradle_print_name(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(sdev)) {
		case 0:
			return sprintf(buf, "UNDOCKED\n");
		case 2:
			return sprintf(buf, "CARKET\n");
	}
	return -EINVAL;
}
#if 0
static void mt6xxx_parse_dt(struct device *dev,
		struct mt6xxx_cradle_platform_data *pdata)
{
	struct device_node *np = dev->of_node;

	if ((pdata->hallic_pouch_detect_pin = of_get_named_gpio_flags(np, "hallic-pouch-irq-gpio", 0, NULL)) > 0)
		pdata->hallic_pouch_irq = gpio_to_irq(pdata->hallic_pouch_detect_pin);

	printk("[Hall IC] hallic_pouch_gpio: %d\n", pdata->hallic_pouch_detect_pin);

	pdata->irq_flags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;
}
#endif
static int mt6xxx_cradle_probe(struct platform_device *pdev)
{
	int ret;
#if 0
	unsigned int hall_pouch_gpio_irq = 0;
	struct mt6xxx_cradle_platform_data *pdata;
	
	if (pdev->dev.of_node) {
		pdata = devm_kzalloc(&pdev->dev,
				sizeof(struct mt6xxx_cradle_platform_data),
				GFP_KERNEL);
		if (pdata == NULL) {
			printk(KERN_ERR "%s: no pdata\n", __func__);
			return -ENOMEM;
		}
		pdev->dev.platform_data = pdata;
		mt6xxx_parse_dt(&pdev->dev, pdata);
	} else {
		pdata = pdev->dev.platform_data;
	}
	if (!pdata) {
		printk(KERN_ERR "%s: no pdata\n", __func__);
		return -ENOMEM;
	}
#endif
	cradle = kzalloc(sizeof(*cradle), GFP_KERNEL);
	if (!cradle)
		return -ENOMEM;

	cradle->sdev.name = "smartcover";
	cradle->sdev.print_name = cradle_print_name;
	cradle->pouch = 0;
	spin_lock_init(&cradle->lock);

	ret = switch_dev_register(&cradle->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	wake_lock_init(&cradle->wake_lock, WAKE_LOCK_SUSPEND, "hall_ic_wakeups");

	INIT_DELAYED_WORK(&pouch_work, mt6xxx_pouch_work_func);

	smart_cover_gpio_set();
#if 0//not yet edit for pen detect (los->mos : irq setting change). if you want use this feature, you must fix.
#ifdef CONFIG_STYLUS_PEN_DETECTION 
	cradle->pen_sdev.name = "styluspen";
	cradle->pen_sdev.print_name = cradle_print_name;

	ret = switch_dev_register(&cradle->pen_sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	INIT_DELAYED_WORK(&pen_work, mt6xxx_pen_work_func);

	stylus_pen_gpio_set();
#endif
#endif
	printk("%s : init cradle\n", __func__);

	printk("%s :boot_cradle_det_func START\n",__func__);
	boot_cradle_det_func();

	ret = device_create_file(&pdev->dev, &cradle_state_attr);
	if (ret)
		goto err_request_irq;

	ret = device_create_file(&pdev->dev, &cradle_pouch_attr);
	if (ret)
		goto err_request_irq;
#if 0
#ifdef CONFIG_STYLUS_PEN_DETECTION
	ret = device_create_file(&pdev->dev, &cradle_pen_state_attr);
	if (ret)
		goto err_request_irq;

	ret = device_create_file(&pdev->dev, &cradle_pen_attr);
	if (ret)
		goto err_request_irq;
#endif
#endif

    printk("[hall_ic]probe done\n");

	return 0;

err_request_irq:
err_switch_dev_register:
	switch_dev_unregister(&cradle->sdev);
#ifdef CONFIG_STYLUS_PEN_DETECTION
	switch_dev_unregister(&cradle->pen_sdev);
#endif
	kfree(cradle);
	return ret;
}

static int mt6xxx_cradle_remove(struct platform_device *pdev)
{
	cancel_delayed_work_sync(&pouch_work);
	switch_dev_unregister(&cradle->sdev);
#ifdef CONFIG_STYLUS_PEN_DETECTION
	cancel_delayed_work_sync(&pen_work);
	switch_dev_unregister(&cradle->pen_sdev);
#endif
	platform_set_drvdata(pdev, NULL);
	kfree(cradle);

	return 0;
}
static int mt6xxx_cradle_suspend(struct device *dev)
{
	return 0;
}

static int mt6xxx_cradle_resume(struct device *dev)
{
	return 0;
}

#ifdef CONFIG_PM
static const struct dev_pm_ops mt6xxx_cradle_pm_ops = {
	.suspend = mt6xxx_cradle_suspend,
	.resume = mt6xxx_cradle_resume,
};
#endif

#ifdef CONFIG_OF
static struct of_device_id mt6xxx_match_table[] = {
	{ .compatible = "mediatek,hall_1", },
	{ },
};
#endif

static struct platform_driver mt6xxx_cradle_driver = {
	.probe  = mt6xxx_cradle_probe,
	.remove = mt6xxx_cradle_remove,
	.driver	= {
		.name	= HALL_IC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = mt6xxx_match_table,
#endif
#ifdef CONFIG_PM
		.pm = &mt6xxx_cradle_pm_ops,
#endif

	},
};


static int __init mt6xxx_cradle_init(void)
{
	cradle_wq = create_singlethread_workqueue("cradle_wq");
	printk(KERN_INFO "cradle init \n");
	if (!cradle_wq) {
		printk(KERN_ERR "fail to create workqueue\n");
		return -ENOMEM;
	}
	return platform_driver_register(&mt6xxx_cradle_driver);
}

static void __exit mt6xxx_cradle_exit(void)
{
	if (cradle_wq)
		destroy_workqueue(cradle_wq);

	platform_driver_unregister(&mt6xxx_cradle_driver);
}

module_init(mt6xxx_cradle_init);
module_exit(mt6xxx_cradle_exit);

MODULE_AUTHOR("LG Electronics Inc.");
MODULE_DESCRIPTION("BU52061NVX HALL IC Driver for MTK platform");
MODULE_LICENSE("GPL");
