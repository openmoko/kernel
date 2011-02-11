/*
 * linux/arch/unicore32/kernel/puv3-smw0919.c
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

static struct gpio_keys_button smw0919_gpio_keys[] = {
	{
		.type	= EV_KEY,
		.code	= KEY_POWER,
		.gpio	= GPI_SOFF_REQ,
		.desc	= "Power Button",
		.wakeup = 1,
		.active_low = 1,
	},
};

static struct gpio_keys_platform_data smw0919_power_button_data = {
	.buttons	= smw0919_gpio_keys,
	.nbuttons	= ARRAY_SIZE(smw0919_gpio_keys),
};

static struct platform_device smw0919_device_power_button = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &smw0919_power_button_data,
	},
};

static struct platform_device *mach_smw0919_devices[] __initdata = {
	&puv3_device_i2c,
	&physmap_flash,
	&smw0919_device_power_button,
};

static struct i2c_board_info __initdata puv3_i2c_devices[] = {
	{	I2C_BOARD_INFO("lm75",		I2C_TAR_THERMAL),	},
	{	I2C_BOARD_INFO("24c02",		I2C_TAR_EEPROM),	},
};

int __init mach_smw0919_init(void)
{
	i2c_register_board_info(0, puv3_i2c_devices,
			ARRAY_SIZE(puv3_i2c_devices));

	platform_add_devices(mach_smw0919_devices,
			ARRAY_SIZE(mach_smw0919_devices));

	return 0;
}

subsys_initcall_sync(mach_smw0919_init);
