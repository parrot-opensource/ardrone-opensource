/*
 *  linux/arch/arm/mach-parrot6/p6.c
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
#include <linux/sysrq.h>
#include <linux/mmc/host.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <asm/system.h>
#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>
#include <mach/i2c.h>
#include <mach/dma-pl08x.h>
#include <mach/fb.h>
#include <mach/gpio.h>
#include <mach/mmc.h>
#include <mach/spi.h>
#include <mach/gpio_parrot.h>
#include <mach/regs-rtc-p6i.h>
#include <mach/usb-p6i.h>

#include "timer.h"
#include "devs.h"
#include "pm.h"
#include "parrot6.h"
#include "parrot6i.h"

static struct map_desc p6i_io_desc[] __initdata = {
	{
		.virtual	= PARROT6_VA_VIC,
		.pfn		= __phys_to_pfn(PARROT6_VIC),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_MPMC,
		.pfn		= __phys_to_pfn(PARROT6_MPMC),
		.length		= SZ_1M,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_SYS,
		.pfn		= __phys_to_pfn(PARROT6_SYS),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_GPIO,
		.pfn		= __phys_to_pfn(PARROT6_GPIO),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART0,
		.pfn		= __phys_to_pfn(PARROT6_UART0),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART1,
		.pfn		= __phys_to_pfn(PARROT6_UART1),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6_VA_UART2,
		.pfn		= __phys_to_pfn(PARROT6_UART2),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
	{
		.virtual	= PARROT6I_VA_RTC,
		.pfn		= __phys_to_pfn(PARROT6I_RTC),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
};


static struct parrot5_i2cm_platform i2cm_platform_p6 = {
	.bus_freq	= 10*1000,
	.retries        = 1,
};

struct p6i_usb2_platform_data_s p6i_usb2_platform_data = {
	/* board specific information */
	.operating_mode = P6P_USB2_DR_HOST,
	.phy_mode       = P6P_USB2_PHY_UTMI,
	.port_enables   = 1
};

#define _F(_type)   _PL080_CXCONFIG_FLOWCNTRL_ ## _type
#define _P(_periph) _PL080_PERIPH_ ## _periph

/* PL08x DMA controller flow control table */
static const int p6i_dma_flowctrl[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX] = {
	/* default is invalid */
	[0 ... _PL080_PERIPH_MAX-1][0 ... _PL080_PERIPH_MAX-1] = -1,
	/* source is a peripheral, flow controller = DMAC or peripheral */
	[_P(PARINT)][_P(MEM)] = _F(P2M_DMAC),
	[_P(NAND)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI0)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI1)][_P(MEM)]   = _F(P2M_DMAC),
	[_P(SPI2)][_P(MEM)]   = _F(P2M_DMAC),
	/* source is memory, flow controller = DMAC or peripheral */
	[_P(MEM)][_P(PARINT)] = _F(M2P_DMAC),
	[_P(MEM)][_P(NAND)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI0)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI1)]   = _F(M2P_DMAC),
	[_P(MEM)][_P(SPI2)]   = _F(M2P_DMAC),
	/* source and dest are memory */
	[_P(MEM)][_P(MEM)]    = _F(M2M_DMAC),
};

static struct p6_spi_config p6i_eth_spi_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static void p6i_jtag_spi_mode(int enable)
{
	if (enable) {
		u32 reg = __raw_readl(PARROT6_VA_SYS + _P6_SYS_IOCR11);
		__raw_writel(reg | P6_SYS_NJTAG_SPI, PARROT6_VA_SYS + _P6_SYS_IOCR11);
		parrot_select_pin(P6I_SPI2a);
		parrot_select_pin(GPIO_000);
		gpio_direction_output(0, 0);
		msleep(10);
		gpio_direction_input(0);
		printk("JTAG has been disabled\n");
	}
	else {
		parrot_select_pin(P6I_nTRST);
		parrot_select_pin(P6I_JTAG);
		printk("JTAG is now enable\n");
	}
}

static struct spi_board_info p6i_eth_spi_board_info[] = {
	{
		.modalias = "ksz8851snl",
		.max_speed_hz = 40000000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = p6i_jtag_spi_mode,
		.controller_data = &p6i_eth_spi_controller_data,
		.mode = SPI_MODE_0,
	},
};

static struct parrot_mmc_platform_data p6i_mmc_platform_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.wp_pin = 1,
	.cd_pin = 2,
#warning cd_pin not used
};

static void sysrq_handle_jtag(int key, struct tty_struct *tty)
{
	p6i_jtag_spi_mode(0);
}

static struct sysrq_key_op sysrq_showlocks_op = {
	.handler	= sysrq_handle_jtag,
	.help_msg	= "Jtag",
	.action_msg	= "Switch from spi to jtag mode",
};

void __init p6i_eth_on_jtag_init(void)
{
	printk("ethernet is on jtag\n");
	
	p6i_eth_spi_board_info[0].irq = gpio_interrupt_register(0, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
	spi_register_board_info(p6i_eth_spi_board_info,
				ARRAY_SIZE(p6i_eth_spi_board_info));
}


void sys_set_bit(u32 reg, u32 bit, u32 val)
{
	u32 value = __raw_readl(PARROT6_VA_SYS + reg);
	value = (value & ~(1<<bit)) | ((val & 0x1) << bit);
	__raw_writel(value, PARROT6_VA_SYS + reg);
}


/**
 * high speed should only be enabled if you are using SDcards
 */
void p6i_set_pads_sdcard(int hs)
{
	// normal operation
	sys_set_bit(_P6I_SYS_P1_1,  7, 0);
	sys_set_bit(_P6I_SYS_P1_1,  8, 0);
	sys_set_bit(_P6I_SYS_P1_1,  9, 0);
	sys_set_bit(_P6I_SYS_P1_1, 10, 0);
	sys_set_bit(_P6I_SYS_P1_1, 11, 0);
	sys_set_bit(_P6I_SYS_P1_1, 12, 0);

	sys_set_bit(_P6I_SYS_P2_2, 18, 0);
	sys_set_bit(_P6I_SYS_P2_2, 19, 0);
	sys_set_bit(_P6I_SYS_P2_2, 20, 0);
	sys_set_bit(_P6I_SYS_P2_2, 21, 0);
	sys_set_bit(_P6I_SYS_P2_2, 22, 0);
	sys_set_bit(_P6I_SYS_P2_2, 23, 0);

	// driver one eighth for every one except SD_CLK
	sys_set_bit(_P6I_SYS_E1_2, 19, 0);
	sys_set_bit(_P6I_SYS_E1_2, 20, 0);
	sys_set_bit(_P6I_SYS_E1_2, 21, 0);
	sys_set_bit(_P6I_SYS_E1_2, 22, 0);
	sys_set_bit(_P6I_SYS_E1_2, 23, 0);

	sys_set_bit(_P6I_SYS_E2_2, 19, 0);
	sys_set_bit(_P6I_SYS_E2_2, 20, 0);
	sys_set_bit(_P6I_SYS_E2_2, 21, 0);
	sys_set_bit(_P6I_SYS_E2_2, 22, 0);
	sys_set_bit(_P6I_SYS_E2_2, 23, 0);

	if (hs) {
		// half drive
		sys_set_bit(_P6I_SYS_E1_2, 18, 0);
		sys_set_bit(_P6I_SYS_E2_2, 18, 1);
		// set max SD clock to 39MHz
		__raw_writel(P6_SYS_156MHZ_SDIO_39_0MHZ, PARROT6_VA_SYS+_P6_SYS_SDCARD);
	} else {
		// quarter drive
		sys_set_bit(_P6I_SYS_E1_2, 18, 1);
		sys_set_bit(_P6I_SYS_E2_2, 18, 0);
	}
}

void p6i_set_pads_i2c(int num)
{
	if (num == 1) {
		//pull-up i2c-1
		__raw_writel(__raw_readl(PARROT6_VA_SYS+_P6I_SYS_P1_0)|(1<<16),   PARROT6_VA_SYS+_P6I_SYS_P1_0);
		__raw_writel(__raw_readl(PARROT6_VA_SYS+_P6I_SYS_P1_0)|(1<<17),   PARROT6_VA_SYS+_P6I_SYS_P1_0);

		__raw_writel(__raw_readl(PARROT6_VA_SYS+_P6I_SYS_P2_1)&(~(1<<27)),   PARROT6_VA_SYS+_P6I_SYS_P2_1);
		__raw_writel(__raw_readl(PARROT6_VA_SYS+_P6I_SYS_P2_1)&(~(1<<28)),   PARROT6_VA_SYS+_P6I_SYS_P2_1);
	}
}

static char *board_name(int board)
{
	char *name;
	switch(board) {
		default:
			name = "Unknow";
	}
	return name;
}

static void print_board_version(void)
{
	u32 reset = __raw_readl(PARROT6_VA_SYS+_P6_SYS_RESET);
	int board = (reset >> 16) & 0x7;
	int rev = (reset >> 23) & 0x7;

	printk(KERN_INFO "board %s (0x%02x) rev 0x%02x (rev tag 0x%04x)\n",
			board_name(board), board, rev, system_rev);
	if (system_rev == 0)
		system_rev = (board<<8)|rev;
}


void __init p6i_init(void)
{
	BUG_ON(!parrot_chip_is_p6i());
	clocks_init_p6();
	print_board_version();
	register_sysrq_key('j', &sysrq_showlocks_op);

	p6_i2cm0_device.dev.platform_data = &i2cm_platform_p6;
	p6_i2cm1_device.dev.platform_data = &i2cm_platform_p6;
	p6_dmac_device.dev.platform_data = &p6i_dma_flowctrl;
	//p6i_usb2_platform_data.operating_mode = P6P_USB2_DR_DEVICE;
	if (parrot_force_usb_device)
		p6i_usb0_device.name = "p6i-ehci-dev";
	p6i_usb0_device.dev.platform_data = &p6i_usb2_platform_data;

	p6_sdhci0_device.dev.platform_data = &p6i_mmc_platform_data;

	/* power management default operator */
	suspend_set_ops(&parrot6_pm_ops_never);

	/* ldo enable */
	__raw_writel(__raw_readl(PARROT6I_VA_RTC+_RTC_PWR_CTRL) | RTC_PWR_CTRL_CODEC_LDO_EN, PARROT6I_VA_RTC+_RTC_PWR_CTRL);
	msleep(1);
	__raw_writel(__raw_readl(PARROT6I_VA_RTC+_RTC_PWR_CTRL) | RTC_PWR_CTRL_USB_LDO_EN, PARROT6I_VA_RTC+_RTC_PWR_CTRL);
}

void __init p6i_map_io(void)
{
	iotable_init(p6i_io_desc, ARRAY_SIZE(p6i_io_desc));
	/* this should be done after static mapping */
}
