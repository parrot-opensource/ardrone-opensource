/*
 *  linux/arch/arm/mach-versatile/devs.h
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
#ifndef __ASM_ARCH_VERSATILE_DEVS_H
#define __ASM_ARCH_VERSATILE_DEVS_H

#include <linux/platform_device.h>

extern struct platform_device parrot6_gpio_device;
extern struct platform_device parrot6_spi_device;
extern struct platform_device parrot6_dmac_device;
extern struct platform_device parrot6_i2cm_device;
extern struct platform_device parrot6_sdhci_device;
extern struct platform_device parrot6_lcd_device;
extern struct platform_device parrot6_usb_device;
extern struct platform_device parrot6_dmac_device;
#ifdef CONFIG_P6_NANDMC
extern struct platform_device parrot6_nandmc_device;
#endif

#endif
