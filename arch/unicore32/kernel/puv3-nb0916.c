/*
 * linux/arch/unicore32/kernel/puv3-nb0916.c
 *
 * Code specific to PKUnity SoC and UniCore ISA
 *
 *	Maintained by GUAN Xue-tao <gxt@mprc.pku.edu.cn>
 *	Copyright (C) 2001-2010 Guan Xuetao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/platform_device.h>
#include <linux/mtd/physmap.h>
#include <linux/io.h>
#include <linux/reboot.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>

#include <mach/hardware.h>

static struct physmap_flash_data physmap_flash_data = {
	.width		= 1,
};

static struct resource physmap_flash_resource = {
	.start		= 0xFFF80000,
	.end		= 0xFFFFFFFF,
	.flags		= IORESOURCE_MEM,
};

static struct platform_device physmap_flash = {
	.name		= "physmap-flash",
	.id		= 0,
	.dev		= {
		.platform_data	= &physmap_flash_data,
	},
	.num_resources	= 1,
	.resource	= &physmap_flash_resource,
};

static struct resource puv3_i2c_resources[] = {
	[0] = {
		.start = PKUNITY_I2C_BASE,
		.end   = PKUNITY_I2C_BASE + 0xff,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_I2C,
		.end   = IRQ_I2C,
		.flags = IORESOURCE_IRQ,
	}
};

static struct platform_device puv3_device_i2c = {
	.name	= "PKUnity-v3-I2C",
	.id	= -1,
	.num_resources	  = ARRAY_SIZE(puv3_i2c_resources),
	.resource	  = puv3_i2c_resources,
};

static struct platform_pwm_backlight_data nb0916_backlight_data = {
	.pwm_id		= 0,
	.max_brightness	= 100,
	.dft_brightness	= 100,
	.pwm_period_ns	= 70 * 1024,
};

static struct platform_device nb0916_device_backlight = {
	.name   = "pwm-backlight",
	.id     = 0,
	.dev    = {
		.platform_data	= &nb0916_backlight_data,
	},
};

static struct gpio_keys_button nb0916_gpio_keys[] = {
	{
		.type	= EV_KEY,
		.code	= KEY_POWER,
		.gpio	= GPI_SOFF_REQ,
		.desc	= "Power Button",
		.wakeup = 1,
		.active_low = 1,
	},
	{
		.type	= EV_KEY,
		.code	= BTN_TOUCH,
		.gpio	= GPI_BTN_TOUCH,
		.desc	= "Touchpad Button",
		.wakeup = 1,
		.active_low = 1,
	},
};

static struct gpio_keys_platform_data nb0916_gpio_button_data = {
	.buttons	= nb0916_gpio_keys,
	.nbuttons	= ARRAY_SIZE(nb0916_gpio_keys),
};

static struct platform_device nb0916_device_gpio_button = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &nb0916_gpio_button_data,
	},
};

static struct platform_device *mach_nb0916_devices[] __initdata = {
	&puv3_device_i2c,
	&physmap_flash,
	&nb0916_device_backlight,
	&nb0916_device_gpio_button,
};

static irqreturn_t nb0916_lcdcaseoff_handler(int irq, void *dev_id)
{
	if (gpio_get_value(GPI_LCD_CASE_OFF))
		gpio_set_value(GPO_LCD_EN, 1);
	else
		gpio_set_value(GPO_LCD_EN, 0);

	return IRQ_HANDLED;
}

static irqreturn_t nb0916_overheat_handler(int irq, void *dev_id)
{
	machine_halt();
	/* SYSTEM HALT, NO RETURN */
	return IRQ_HANDLED;
}

static struct i2c_board_info __initdata puv3_i2c_devices[] = {
	{	I2C_BOARD_INFO("lm75",		I2C_TAR_THERMAL),	},
	{	I2C_BOARD_INFO("bq27200",	I2C_TAR_PWIC),		},
	{	I2C_BOARD_INFO("24c02",		I2C_TAR_EEPROM),	},
};

int __init mach_nb0916_init(void)
{
	i2c_register_board_info(0, puv3_i2c_devices,
			ARRAY_SIZE(puv3_i2c_devices));

	platform_add_devices(mach_nb0916_devices,
			ARRAY_SIZE(mach_nb0916_devices));

	if (request_irq(gpio_to_irq(GPI_LCD_CASE_OFF),
		&nb0916_lcdcaseoff_handler,
		IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"NB0916 lcd case off", NULL) < 0) {

		printk(KERN_DEBUG "LCD-Case-OFF IRQ %d not available\n",
			gpio_to_irq(GPI_LCD_CASE_OFF));
	}

	if (request_irq(gpio_to_irq(GPI_OTP_INT), &nb0916_overheat_handler,
		IRQF_DISABLED | IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
		"NB0916 overheating protection", NULL) < 0) {

		printk(KERN_DEBUG "Overheating Protection IRQ %d not available\n",
			gpio_to_irq(GPI_OTP_INT));
	}

	return 0;
}

subsys_initcall_sync(mach_nb0916_init);
