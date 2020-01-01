/*
 *  linux/arch/arm/mach-parrot6/parrot6idev.c
 *
 *  Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2008-11-05
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/usb.h>
#include <mach/usb-p6i.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

static unsigned int pins_init_p6i[] = {
	/* uart */
	P6I_UART0_DEFAULT,
	P6I_UART1_DEFAULT,
	P6I_UART2_RXTX_DEFAULT,
	P6I_NAND8_DEFAULT,
	P6I_I2CM1_DEFAULT,
	P6I_SD0_DEFAULT,
	P6I_AAI_I2S_SYNC_DEFAULT,
	P6I_AAI_PCM0_DEFAULT,
	P6I_I2S_IN1,
	P6I_I2S_IN2,
	P6I_I2S_OUT0,
	P6I_I2S_OUT1,

	P6I_GPIO_002, /* marvell reset */
	P6I_GPIO_003,
	P6I_USB_PWR_ON,
	0,
};

static struct platform_device *p6i_devices[] __initdata = {
	&p6_uart0_device,
	&p6_uart1_device,
	&p6_uart2_device,
	&p6_dmac_device,
	&p6_nand_device,
	&p6_gpio,
	&p6_aai_device,
	&p6_i2cm1_device,
	&p6_sdhci0_device,
	&p6_spi2_device, /* for jtag-eth */
	&p6i_usb0_device,
};

static struct parrot_mmc_platform_data p6i_mmc_wifi_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	/* wifi : not cd and wp pins */
};

static void __init p6idev_init(void)
{
	p6i_init();
	parrot_init_pin(pins_init_p6i);
	p6i_set_pads_i2c(1);
	p6i_set_pads_sdcard(0);

	/* disable NAND flash write protection on P6 dev */
	gpio_direction_output(29, 1);

	/*Marvell reset*/
	gpio_direction_output(2, 0);
	p6_sdhci0_device.dev.platform_data = &p6i_mmc_wifi_platform_data;

	/*USB ON*/
	gpio_direction_output(3, 1);

	p6i_eth_on_jtag_init();

	platform_add_devices(p6i_devices, ARRAY_SIZE(p6i_devices));
}

MACHINE_START(PARROT_FC6050, "FC6050 Parrot platform")
	/* Maintainer: Parrot S.A. */
	.phys_io	= PARROT6_UART0,
	.io_pg_offst	= (PARROT6_VA_UART0 >> 18) & 0xfffc,
	.boot_params	= PARROT6_DDR_BASE+0x100,
	.map_io		= p6i_map_io,
	.init_irq	= p6_init_irq,
	.timer		= &p6_timer,
	.init_machine	= p6idev_init,
MACHINE_END
