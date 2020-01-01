/*
 *  linux/arch/arm/mach-versatile/versatile_pb.c
 *
 *  Copyright (C) 2004 ARM Limited
 *  Copyright (C) 2000 Deep Blue Solutions Ltd
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
#include <linux/amba/bus.h>

#include <mach/hardware.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/mach-types.h>

#include <asm/mach/map.h>
#include <asm/hardware/vic.h>
#include <asm/hardware/icst307.h>
#include <asm/mach/irq.h>
#include <linux/irq.h>
#include <linux/mtd/partitions.h>
#include <linux/spi/flash.h>

#include <asm/mach/arch.h>
#include <asm/mach/mmc.h>

/* parrot include */
#include <../arch/arm/plat-parrot/include/mach/fb.h>
#include <mach/platform-p6-fpga.h>
#include <mach/regs-i2c-p5.h>
//#include <mach/spi.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include <mach/spi.h>

#include "core.h"
#include "devs.h"
#include "clock.h"

#ifndef __io_address
#define __io_address(n)		__io(IO_ADDRESS(n))
#endif

#if 1
#define IRQ_MMCI1A	IRQ_VICSOURCE23
#else
#define IRQ_MMCI1A	IRQ_SIC_MMCI1A
#endif

//#define SPI_FLASH
//#define SPI_ETHERNET_ENC28J60
#define SPI_ETHERNET_KSZ8851SNL
#define USE_SD

#if defined(SPI_ETHERNET_KSZ8851SNL) || defined(SPI_ETHERNET_ENC28J60)
#ifndef USE_SPI
#define USE_SPI
#endif
#ifndef USE_PL08X
#define USE_PL08X
#endif
#endif


static struct mmc_platform_data mmc1_plat_data = {
	.ocr_mask	= MMC_VDD_32_33|MMC_VDD_33_34,
	.status		= mmc_status,
};

#define UART3_IRQ	{ IRQ_SIC_UART3, NO_IRQ }
#define UART3_DMA	{ 0x86, 0x87 }
#define SCI1_IRQ	{ IRQ_SIC_SCI3, NO_IRQ }
#define SCI1_DMA	{ 0x88, 0x89 }
#define MMCI1_IRQ	{ IRQ_MMCI1A, IRQ_SIC_MMCI1B }
#define MMCI1_DMA	{ 0x85, 0 }

/*
 * These devices are connected via the core APB bridge
 */
#define GPIO2_IRQ	{ IRQ_GPIOINT2, NO_IRQ }
#define GPIO2_DMA	{ 0, 0 }
#define GPIO3_IRQ	{ IRQ_GPIOINT3, NO_IRQ }
#define GPIO3_DMA	{ 0, 0 }

/*
 * These devices are connected via the DMA APB bridge
 */

/* FPGA Primecells */
AMBA_DEVICE(uart3, "fpga:09", UART3,    NULL);
AMBA_DEVICE(sci1,  "fpga:0a", SCI1,     NULL);
AMBA_DEVICE(mmc1,  "fpga:0b", MMCI1,    &mmc1_plat_data);

/* DevChip Primecells */
AMBA_DEVICE(gpio2, "dev:e6",  GPIO2,    NULL);
AMBA_DEVICE(gpio3, "dev:e7",  GPIO3,    NULL);

static struct amba_device *amba_devs[] __initdata = {
	&uart3_device,
	&gpio2_device,
	&gpio3_device,
	&sci1_device,
	&mmc1_device,
};

/* P6 static mapping virtual should be in VMALLOC_END     0xfeffffff */
static struct map_desc versatile_p6_io_desc[] __initdata = {
	{
		.virtual	=  IO_ADDRESS(VERSATILE_TIC_BASE),
		.pfn		= __phys_to_pfn(VERSATILE_TIC_BASE),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(PARROT6_PWM),
		.pfn		= __phys_to_pfn(PARROT6_PWM),
		.length		= SZ_4K,
		.type		= MT_DEVICE
	},
	{
		.virtual	=  IO_ADDRESS(PARROT6_GPIO),
		.pfn		= __phys_to_pfn(PARROT6_GPIO),
		.length		= SZ_64K,
		.type		= MT_DEVICE
	},
};

static void __init versatile_p6_map_io(void)
{
	versatile_map_io();
	iotable_init(versatile_p6_io_desc, ARRAY_SIZE(versatile_p6_io_desc));
}

static void
vic_p6_handle_irq(unsigned int irq, struct irq_desc *desc)
{
	unsigned long status = __raw_readl(VA_TIC_BASE + VIC_IRQ_STATUS);

	if (status == 0) {
		do_bad_IRQ(irq, desc);
		return;
	}

	do {
		irq = ffs(status) - 1;
		status &= ~(1 << irq);

		irq += IRQ_TIC_START;

		desc = irq_desc + irq;
		desc_handle_irq(irq, desc);
	} while (status);
}

static void __init versatile_p6_init_irq(void)
{
	versatile_init_irq();
	set_irq_chained_handler(IRQ_VICSOURCE30, vic_p6_handle_irq);
	vic_init((void*) VA_TIC_BASE, IRQ_TIC_START, ~0);
}

#ifdef USE_I2C
static struct parrot5_i2cm_platform i2cm_platform_parrot5 = {
	.bus_freq	= 10*1000,
	.retries        = 1,
};
#endif

#ifdef USE_SPI
/* Parrot6 SPI devices (board specific) */
#ifdef SPI_FLASH
static struct mtd_partition versatile_board_spi_flash_partitions[] = {
	{
		.name	= "file system",
		.size	= 0x20000,
		.offset	= 0x0,
	},
};
static struct flash_platform_data versatile_board_spi_flash_data = {
	.name		= "m25p80",
	.parts		= versatile_board_spi_flash_partitions,
	.nr_parts	= ARRAY_SIZE(versatile_board_spi_flash_partitions),
	.type		= "m45pe10",
};

static struct spi_board_info versatile_spi_board_info[] __initdata = {
    {
        .modalias = "m25p80",
        .max_speed_hz = 75000000,
        .bus_num = 0,
        .chip_select = 0,
        .platform_data = &versatile_board_spi_flash_data,
        .controller_data = NULL,
        .mode = SPI_MODE_3,
    },
};
#endif

#ifdef SPI_ETHERNET_ENC28J60
static struct p6_spi_config p6_spi_controller_data = {
	.tsetupcs_ns = 80,
	.tholdcs_ns  = 210,
};

static struct spi_board_info versatile_spi_board_info[] __initdata = {
    {
        .modalias = "enc28j60",
        .max_speed_hz = 20000000,
        .bus_num = 0,
        .chip_select = 0,
        .platform_data = NULL,
        .controller_data = &p6_spi_controller_data,
        .mode = SPI_MODE_0,
    },
};
#endif

#ifdef SPI_ETHERNET_KSZ8851SNL
static struct p6_spi_config p6_spi_controller_data = {
	.tsetupcs_ns = 10,
	.tholdcs_ns  = 10,
};

static struct spi_board_info versatile_spi_board_info[] __initdata = {
    {
        .modalias = "ksz8851snl",
        .max_speed_hz = 50000000,
        .bus_num = 0,
        .chip_select = 0,
        .platform_data = NULL,
        .controller_data = &p6_spi_controller_data,
        .mode = SPI_MODE_0,
    },
};
#endif

static struct p6_spi_info p6_spi_platform_data = {
	.board_info = versatile_spi_board_info,
	.board_size = ARRAY_SIZE(versatile_spi_board_info),
};
#endif

#ifdef USE_LCD
/* LCD driver info */

static struct p6fb_mach_info parrot6_lcd_devices_info /*__initdata*/ = {
	/* does it make sense to have min & max ? */
	.yres		= {
		.min	= 480,
		.max	= 480,
		.defval	= 480,
	},

	.xres		= {
		.min	= 800,
		.max	= 800,
		.defval = 800,
	},

	.bpp		= {
		.min	= 16,
		.max	= 32,
		.defval = 16,
	},

	.pixclock	= {
#if 0
		.min	= 37037,
		.max	= 37037,
		.defval = 37037,
#else
		.min	= 100000*4,
		.max	= 100000*4,
		.defval = 100000*4,
#endif
	},
	.left_margin = 1,
	.right_margin = 1,
	.upper_margin = 1,
	.lower_margin = 1,
	.hsync_len = 20,
	.vsync_len = 100,

	.panel = 0x10000,
};
#endif


/* enable what you have on your fpga */
static struct platform_device *parrot6_devices[] __initdata = {
	&parrot6_i2cm_device,
	&parrot6_gpio_device,
#ifdef USE_SPI
	&parrot6_spi_device,
#endif
#ifdef USE_SD
	&parrot6_sdhci_device,
#endif
#ifdef USE_PL08X
	&parrot6_dmac_device,
#endif
	//&parrot6_lcd_device,
	&parrot6_usb_device,
#ifdef CONFIG_P6_NANDMC
	&parrot6_nandmc_device
#endif
};

/*
 * dirty hack to init GLOBALCLK clock:
 *  - versatile_oscvco_params is a duplicate (see core.c) and should be made
 *    available through an extern declaration
 *  - the mach-versatile clock framework does not define an API to init
 *    "read-only" clocks, i.e., clocks which rate are not modified
 */
static struct icst307_params const versatile_oscvco_params = {
	.ref		= 24000,
	.vco_max	= 200000,
	.vd_min		= 4 + 8,
	.vd_max		= 511 + 8,
	.rd_min		= 1 + 2,
	.rd_max		= 127 + 2
};

static struct clk versatile_global_clk = {
	.name	= "GLOBALCLK",
	.params	= &versatile_oscvco_params
};

static struct clk parrot_i2cm_clk = {
   .name   = "i2cm",
   .params = &versatile_oscvco_params
};

static struct clk parrot_lcdc_clk = {
   .name   = "lcdc",
   .rate   = 40000000,
};

static
void __init versatile_p6_init_clock(void)
{
    unsigned int const          osc0 = readl(IO_ADDRESS(VERSATILE_SYS_BASE) +
                                             VERSATILE_SYS_OSC0_OFFSET);
    struct icst307_vco const    vco = {
        .v = osc0 & 0x1ff,
        .r = (osc0 >> 9) & 0x7f,
        .s = (osc0 >> 16) & 0x7
    };

    versatile_global_clk.rate = icst307_khz(versatile_global_clk.params, vco) * 1000;
    clk_register(&versatile_global_clk);
	parrot_i2cm_clk.rate = icst307_khz(parrot_i2cm_clk.params, vco) * 1000;
	clk_register(&parrot_i2cm_clk);
	clk_register(&parrot_lcdc_clk);
}

arch_initcall(versatile_p6_init_clock);

#ifdef USE_PL08X
#include <mach/dma-pl08x.h>
#define _F(_type)   _PL080_CXCONFIG_FLOWCNTRL_ ## _type
#define _P(_periph) _PL080_PERIPH_ ## _periph
static const int p6_dma_flowctrl[_PL080_PERIPH_MAX][_PL080_PERIPH_MAX] = {
	/* default is invalid */
	[0 ... _PL080_PERIPH_MAX-1][0 ... _PL080_PERIPH_MAX-1] = -1,
	/* source is a peripheral, flow controller = DMAC or peripheral */
	[_P(SPI0)][_P(MEM)]   = _F(P2M_DMAC),
	/* source is memory, flow controller = DMAC or peripheral */
	[_P(MEM)][_P(SPI0)]   = _F(M2P_DMAC),
	/* source and dest are memory */
	[_P(MEM)][_P(MEM)]    = _F(M2M_DMAC),
};
#endif

static void __init versatile_pb_init(void)
{
	int i;

	versatile_init();

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}

	/* pwm for lcd */
#if 0
	__raw_writel(0x200, VA_P6_PWM_BASE+4);
	__raw_writel(0xFF, VA_P6_PWM_BASE+0x14);
	__raw_writel(0x1, VA_P6_PWM_BASE+0);
#endif

#ifdef SPI_ETHERNET_ENC28J60
	// init for the ethernet PicTail card (ENC28J60 chip)
	gpio_direction_output(12, 1); // SPI CS for 25LC256
	gpio_direction_input(11); // INT for ENC28J60
	versatile_spi_board_info[0].irq = gpio_interrupt_register(11, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
#endif
#ifdef SPI_ETHERNET_KSZ8851SNL
	gpio_direction_input(12); // SPI INT
	versatile_spi_board_info[0].irq = gpio_interrupt_register(12, GPIO_IRQ_NEG, GPIO_DEBOUNCE_NONE);
#endif
#ifdef USE_SPI
	parrot6_spi_device.dev.platform_data = &p6_spi_platform_data;
	spi_register_board_info(versatile_spi_board_info,
				ARRAY_SIZE(versatile_spi_board_info));
#endif

#ifdef USE_I2C
	parrot6_i2cm_device.dev.platform_data = &i2cm_platform_parrot5;
#endif
#ifdef USE_LCD
	parrot6_lcd_device.dev.platform_data = &parrot6_lcd_devices_info;
#endif
#ifdef USE_PL08X
	parrot6_dmac_device.dev.platform_data = &p6_dma_flowctrl;
#endif
	platform_add_devices(parrot6_devices, ARRAY_SIZE(parrot6_devices));
}

MACHINE_START(VERSATILE_PB, "ARM-Versatile PB")
	/* Maintainer: ARM Ltd/Deep Blue Solutions Ltd */
	.phys_io	= 0x101f1000,
	.io_pg_offst	= ((0xf11f1000) >> 18) & 0xfffc,
	.boot_params	= 0x00000100,
	.map_io		= versatile_p6_map_io,
	.init_irq	= versatile_p6_init_irq,
	.timer		= &versatile_timer,
	.init_machine	= versatile_pb_init,
MACHINE_END
