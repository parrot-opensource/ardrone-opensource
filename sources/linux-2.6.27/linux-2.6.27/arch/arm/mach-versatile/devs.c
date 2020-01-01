/*
 *  linux/arch/arm/mach-versatile/devs.c
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     florent.bayendrian@parrot.com
 * @date       2008-03-11
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

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <mach/hardware.h>
#include <../arch/arm/plat-parrot/include/mach/irqs-p6.h>
#include <mach/platform-p6-fpga.h>

#include "devs.h"

/* XXX TODO merge this with arch/arm/mach-parrot6/dev.c */
/** Parrot6 devices **/
#define P6_FPGA(x) (x+IRQ_TIC_START)


/* Parrot6 GPIO controller */
static struct resource parrot6_gpio_resource[] = {
	[0] = {
		.start = PARROT6_GPIO,
		.end   = PARROT6_GPIO + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_GPIO),
		.end   = P6_FPGA(IRQ_P6_GPIO),
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device parrot6_gpio_device = {
	.name		= "parrot6-gpio",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(parrot6_gpio_resource),
	.resource	= parrot6_gpio_resource,
};


/* Parrot6 SPI controller */
static struct resource parrot6_spi_resource[] = {
	[0] = {
		.start = PARROT6_SPI,
		.end   = PARROT6_SPI + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_SPIX),
		.end   = P6_FPGA(IRQ_P6_SPIX),
		.flags = IORESOURCE_IRQ,
	},
};

static u64 parrot6_dmamask = DMA_32BIT_MASK;

struct platform_device parrot6_spi_device = {
	.name		=	"p6-spi",
	.id		=	0,
	.dev		= {
		.dma_mask = &parrot6_dmamask,
		.coherent_dma_mask = DMA_32BIT_MASK,
	},
	.num_resources	= ARRAY_SIZE(parrot6_spi_resource),
	.resource	= parrot6_spi_resource,
};

/* Parrot6 DMA controller (PL080)*/
static struct resource parrot6_dmac_resource[] = {

	[0] = {
		.start = PARROT6_DMAC,
		.end   = PARROT6_DMAC + SZ_1K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_DMAC),
		.end   = P6_FPGA(IRQ_P6_DMAC),
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device parrot6_dmac_device = {
	.name = "dma-pl08x",
	.id = 0,
	.num_resources	= ARRAY_SIZE(parrot6_dmac_resource),
	.resource	= parrot6_dmac_resource,
};

/* Parrot I2C master driver */
static struct resource parrot6_i2cm_resources[] = {
	[0] = {
		.start		= PARROT6_I2CM,
		.end		= PARROT6_I2CM + SZ_64K - 1,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= P6_FPGA(IRQ_P6_I2CM),
		.end		= P6_FPGA(IRQ_P6_I2CM),
		.flags		= IORESOURCE_IRQ,
	},
};

struct platform_device parrot6_i2cm_device = {
	.name		= "parrot5-i2cm",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(parrot6_i2cm_resources),
	.resource	= parrot6_i2cm_resources,
};

/* SDIO driver */
static struct resource versatile_sdhci_parrot6_resource[] = {
	[0] = {
		.start = PARROT6_SDIO,
		.end   = PARROT6_SDIO+SZ_4K-1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_SDIO),
		.end   = P6_FPGA(IRQ_P6_SDIO),
		.flags = IORESOURCE_IRQ,
	}
};

struct platform_device parrot6_sdhci_device = {
	.name			= "p6-sdhci",
	.id			= 0,
	.num_resources		= ARRAY_SIZE(versatile_sdhci_parrot6_resource),
	.resource		= versatile_sdhci_parrot6_resource,
};

/* LCD Controller */

static struct resource p6_lcd_resource[] = {
	[0] = {
		.start = PARROT6_LCDC,
		.end   = PARROT6_LCDC + SZ_64K - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_LCDC),
		.end   = P6_FPGA(IRQ_P6_LCDC),
		.flags = IORESOURCE_IRQ,
	}

};

struct platform_device parrot6_lcd_device = {
	.name		  = "p6-lcd",
	.id		  = -1,
	.num_resources	  = ARRAY_SIZE(p6_lcd_resource),
	.resource	  = p6_lcd_resource,
	.dev = {
		.coherent_dma_mask	= 0xffffffffUL
	},
};

/* Parrot6 USB controller (Synospys) */
static struct resource parrot6_usb_resource[] = {
	[0] = {
		.start = PARROT6_USB0,
		.end   = PARROT6_USB0 + SZ_1M - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = P6_FPGA(IRQ_P6_USB0),
		.end   = P6_FPGA(IRQ_P6_USB0),
		.flags = IORESOURCE_IRQ,
	},
};

struct platform_device parrot6_usb_device = {
	.name		= "dwc_otg",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(parrot6_usb_resource),
	.resource	= parrot6_usb_resource,
};

/* Parrot6 NAND memory controller (Barco-Silex BA315) */

#ifdef CONFIG_P6_NANDMC

#define SZ_2K 0x800
#define SZ_64 0x40

static
struct resource parrot6_nandmc_resource[] = {
    [0] = {
        .start	= PARROT6_NANDMC_REGS,
        .end	= PARROT6_NANDMC_REGS + SZ_64 - 1,
        .flags	= IORESOURCE_IO,
    },
    [1] = {
        .start	= PARROT6_NANDMC_MEM,
        .end	= PARROT6_NANDMC_MEM + SZ_2K - 1,
        .flags	= IORESOURCE_MEM,
    },
    [2] = {
        .start	= P6_FPGA(IRQ_P6_NANDMC),
        .end	= P6_FPGA(IRQ_P6_NANDMC),
        .flags	= IORESOURCE_IRQ,
    }
};

struct platform_device parrot6_nandmc_device = {
    .name           = "parrot6-nandmc",
    .id             = 0,
	.num_resources  = ARRAY_SIZE(parrot6_nandmc_resource),
	.resource       = parrot6_nandmc_resource
};

#endif /* CONFIG_P6_NANDMC */
