/* touch_hwif_mtk.c¬
 *
 * Copyright (C) 2015 LGE.¬
 *
 * Author: hoyeon.jang@lge.com¬
 *
 * This software is licensed under the terms of the GNU General Public¬
 * License version 2, as published by the Free Software Foundation, and¬
 * may be copied, distributed, and modified under those terms.¬
 *
 * This program is distributed in the hope that it will be useful,¬
 * but WITHOUT ANY WARRANTY; without even the implied warranty of¬
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the¬
 * GNU General Public License for more details.¬
 *
 */

#include <linux/dma-mapping.h>
#include <linux/workqueue.h>
#include <mt_gpio.h>
#include <upmu_common.h>
#include <linux/of_irq.h>

//#include <mach/mt_pm_ldo.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include "touch_hwif.h"
#include "touch_i2c.h"
#include "touch_spi.h"
#include "touch_core.h"

/* -- gpio -- */
static bool touch_gpio_is_valid(int pin)
{
	if (pin >= 0)
		return true;

	return false;
}

int touch_gpio_init(int pin, const char *name)
{
	unsigned long mt_pin = (unsigned long) pin | 0x80000000;

	TOUCH_I("%s - pin:%u, name:%s\n", __func__, pin & ~0x80000000, name);

	if (touch_gpio_is_valid(pin))
		mt_set_gpio_mode(mt_pin, GPIO_MODE_00);

	return 0;
}

void touch_gpio_direction_input(int pin)
{
	unsigned long mt_pin = (unsigned long) pin | 0x80000000;

	TOUCH_I("%s - pin:%u\n", __func__, pin & ~0x80000000);
	if (touch_gpio_is_valid(pin))
		mt_set_gpio_dir(mt_pin , GPIO_DIR_IN);
}

void touch_gpio_direction_output(int pin, int value)
{
	unsigned long mt_pin = (unsigned long) pin | 0x80000000;

	TOUCH_I("%s - pin:%u, value:%d\n", __func__, pin & ~0x80000000, value);

	if (touch_gpio_is_valid(pin)) {
		mt_set_gpio_dir(mt_pin, GPIO_DIR_OUT);
		mt_set_gpio_out(mt_pin, value);
	}
}
/* -- power -- */
int touch_power_init(struct device *dev)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (touch_gpio_is_valid(ts->vdd_pin)) {
		mt_set_gpio_dir(ts->vdd_pin, GPIO_DIR_OUT);
	} else {
		TOUCH_I("%s vdd - id: %d, vol: %d\n", __func__,
				ts->vdd_id, ts->vdd_vol);
	}

	if (touch_gpio_is_valid(ts->vio_pin)) {
		mt_set_gpio_dir(ts->vio_pin, GPIO_DIR_OUT);
	} else {
		TOUCH_I("%s vio - id: %d, vol: %d\n", __func__,
				ts->vio_id, ts->vio_vol);
	}

	return 0;
}

void touch_power_vdd(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (touch_gpio_is_valid(ts->vdd_pin)) {
		touch_gpio_direction_output(ts->vdd_pin, value);
	} else if (ts->vdd_id >= 0) {
		TOUCH_I("%s vdd - id: %d, vol: %d\n", __func__,
				ts->vdd_id, ts->vdd_vol);
		if (value)
			hwPowerOn(ts->vdd_id, ts->vdd_vol, "TP");
		else
			hwPowerDown(ts->vdd_id, "TP");
	}

}

void touch_power_vio(struct device *dev, int value)
{
	struct touch_core_data *ts = to_touch_core(dev);

	if (touch_gpio_is_valid(ts->vio_pin)) {
		touch_gpio_direction_output(ts->vio_pin, value);
	} else if (ts->vio_id >= 0) {
		TOUCH_I("%s vio - id: %d, vol: %d\n", __func__,
				ts->vio_id, ts->vio_vol);
		if (value)
			hwPowerOn(ts->vio_id, ts->vio_vol, "TP");
		else
			hwPowerDown(ts->vio_id, "TP");
	}
}

int touch_bus_init(struct device *dev, int buf_size)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	TOUCH_TRACE();

	if (buf_size) {
		ts->tx_buf = (u8 *)dma_alloc_coherent(NULL,
				buf_size, &ts->tx_pa, GFP_KERNEL);

		if (!ts->tx_buf)
			TOUCH_E("fail to allocate tx_buf\n");

		ts->rx_buf = (u8 *)dma_alloc_coherent(NULL,
				buf_size, &ts->rx_pa, GFP_KERNEL);
		if (!ts->rx_buf)
			TOUCH_E("fail to allocate rx_buf\n");

		TOUCH_I("tx_buf:%p, dma[pa:%08x]\n",
				ts->tx_buf, ts->tx_pa);
		TOUCH_I("rx_buf:%p, dma[pa:%08x]\n",
				ts->rx_buf, ts->rx_pa);
	}

	if (ts->bus_type == HWIF_SPI) {
		ret = spi_setup(to_spi_device(dev));

		if (ret < 0) {
			TOUCH_E("Failed to perform SPI setup\n");
			return -ENODEV;
		}
	}

	return ret;
}

int touch_bus_read(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_read(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI)
		ret = touch_spi_read(to_spi_device(dev), msg);

	return ret;
}

int touch_bus_write(struct device *dev, struct touch_bus_msg *msg)
{
	struct touch_core_data *ts = to_touch_core(dev);
	int ret = 0;

	if (ts->bus_type == HWIF_I2C)
		ret = touch_i2c_write(to_i2c_client(dev), msg);
	else if (ts->bus_type == HWIF_SPI)
		ret = touch_spi_write(to_spi_device(dev), msg);

	return ret;
}

struct touch_core_data *touch_ts;
static irq_handler_t touch_handler_fn;
static irq_handler_t touch_thread_fn;
static struct delayed_work work_irq;

static void touch_irq_work_func(struct work_struct *work_irq)
{
	TOUCH_TRACE();
	touch_thread_fn(touch_ts->irq, touch_ts);
}

static void touch_irq_handler(void)
{
	TOUCH_TRACE();
	if (touch_handler_fn(touch_ts->irq, touch_ts) == IRQ_WAKE_THREAD)
		queue_delayed_work(touch_ts->wq, &work_irq, 0);
}

void touch_enable_irq_wake(unsigned int irq)
{
}

void touch_disable_irq_wake(unsigned int irq)
{
}

void touch_enable_irq(unsigned int irq)
{
	TOUCH_TRACE();
	enable_irq(irq);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
}

void touch_disable_irq(unsigned int irq)
{
	TOUCH_TRACE();
	disable_irq(irq);
	//mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
}


int touch_request_irq(unsigned int irq, irq_handler_t handler,
		     irq_handler_t thread_fn,
		     unsigned long flags, const char *name, void *dev)
{
	struct touch_core_data *ts = (struct touch_core_data *)dev;
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	TOUCH_TRACE();

	touch_ts = dev;
	touch_handler_fn = handler;
	touch_thread_fn = thread_fn;
	INIT_DELAYED_WORK(&work_irq, touch_irq_work_func);

	/* TBD
	   1. MTK does't not seem to support threaded irq. *
	 *    Following code is from unified touch v4.0.   *
 	 *    Check this later. 2015.12.04.                *
 	 * 2. Interrupt pin is read from dt during probe.  *
 	      Check parsing of dt is ok and remove below. */
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6735-touch");
	//node = of_find_compatible_node(NULL, NULL, "mediatek, TOUCH_PANEL-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		ts->irq = irq_of_parse_and_map(node, 0);
		ret = request_irq(ts->irq, touch_irq_handler, flags | IRQF_TRIGGER_FALLING,
				name, NULL);
		if (ret > 0) {
			ret = ENXIO;
			TOUCH_E("tpd request_irq IRQ LINE NOT AVAILABLE!.\n");
		}
	} else {
		TOUCH_E("tpd request_irq can not find touch eint device node!.\n");
		ret = ENXIO;
	}
	TOUCH_I("irq:%d, debounce:%d-%d:\n", ts->irq, ints[0], ints[1]);
	return ret;
}

void touch_resend_irq(unsigned int irq)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
}

void touch_set_irq_pending(unsigned int irq)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
}

int touch_boot_mode(void)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
	return 0;
}

int touch_boot_mode_check(struct device *dev)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
	return 0;
}

int touch_bus_xfer(struct device *dev, struct touch_xfer_msg *xfer)
{
	TOUCH_D(BASE_INFO, "%s is not supported!\n", __func__);
	return 0;
}


static struct i2c_board_info touch_i2c_board_info __initdata;
static struct spi_board_info touch_spi_board_info __initdata;

int touch_bus_device_init(struct touch_hwif *hwif, void *driver)
{
	if (hwif->bus_type == HWIF_I2C) {
/* TBD														*
 * Need to check blocking this block is acceptable or not.	*
 * When CONFIG_USE_OF is enabled, it seems to find i2c 		*
 * board info according to the id info.						*/
#if !defined(CONFIG_USE_OF)
		struct i2c_board_info board_info = {
			I2C_BOARD_INFO(LGE_TOUCH_NAME, 0x28)
		};

		touch_i2c_board_info = board_info;
		i2c_register_board_info(1 /*touch_i2c_bus_num*/,
				&touch_i2c_board_info, 1);
#endif
		return touch_i2c_device_init(hwif, driver);
	} else if (hwif->bus_type == HWIF_SPI) {
		struct spi_board_info board_info = {
			.modalias = LGE_TOUCH_NAME,
			/*
			 * TODO
			.bus_num = touch_spi_bus_num,
			*/
		};

		touch_spi_board_info = board_info;
		spi_register_board_info(&touch_spi_board_info, 1);
		return touch_spi_device_init(hwif, driver);
	}

	TOUCH_E("Unknown touch interface : %d\n", hwif->bus_type);

	return -ENODEV;
}

void touch_bus_device_exit(struct touch_hwif *hwif)
{
	if (hwif->bus_type == HWIF_I2C)
		touch_i2c_device_exit(hwif);
	else if (hwif->bus_type == HWIF_SPI)
		touch_spi_device_exit(hwif);
}
