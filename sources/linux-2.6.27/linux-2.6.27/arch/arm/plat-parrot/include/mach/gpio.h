/**
 * @file linux/include/asm-arm/arch-parrot/gpio.h
 * @brief Parrot ASICs GPIO interface
 *
 * Copyright (C) 2007 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
 * @date       2006-02-15
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
 *
 */
#ifndef __ARCH_ARM_PARROT_GPIO_REGS_H
#define __ARCH_ARM_PARROT_GPIO_REGS_H

#include "regs-gpio-p5.h"
#include "regs-gpio-p5p.h"
#include "regs-gpio-p6.h"

/* Useful macros for internal use */

#ifndef __ASSEMBLY__

#include <asm/io.h>
#include <mach/platform.h>
#include <mach/map.h>
#include <mach/parrot.h>

#define GPIO_IOCR_OFFSET(_gpio) (((_gpio) >> 4) << 2)

#ifdef CONFIG_ARCH_PARROT5

#define DDR_BASE  (parrot_chip_is_p5p()? _P5P_GPIO_P0007DDR : _P5_GPIO_P0007DDR)
#define DR_BASE   _P5_GPIO_P0007DR
#define GPIO_MODE 0
#define GPIO_IOCR(_gpio)						\
	(((_gpio) < 48)?						\
	 _P5_SYS_IOCR1  + GPIO_IOCR_OFFSET(_gpio) :			\
	 _P5P_SYS_IOCR5 + GPIO_IOCR_OFFSET((_gpio)-48))


#else /* Parrot6 */

#define DDR_BASE             _P6_GPIO_P0007DDR
#define DR_BASE              _P6_GPIO_P0007DR
#define GPIO_MODE            1
#define GPIO_IOCR(_gpio)     (_P6_SYS_IOCR1 + GPIO_IOCR_OFFSET(_gpio))

#endif

#define __DR(_x)             (DR_BASE + 0x400*((_x) >> 3))
#define __DR_SHIFT(_x)       ((_x) & 0x7)
#define __ADDR(_x)           (__DR(_x)+(1 << (2+__DR_SHIFT(_x))))
#define __DDR_SHIFT(_x)      ((_x) & 0x7)
#define __DDR(_x)            (DDR_BASE + 4*((_x) >> 3))

/* Select GPIO multiplexing */
static inline u32 parrot_gpio_to_pin(unsigned gpio)
{
	u32 iocr = GPIO_IOCR(gpio);
	if (parrot_chip_is_p6i()) {
		/* rtc gpio */
		if (gpio >= 1 && gpio <= 12) {
			iocr = 0x19006C;
			gpio--;
		}
	}
	return (iocr << 8)|((gpio & 0xf) << 4)|GPIO_MODE;
}

static inline int parrot_gpio_select(unsigned gpio)
{
#if !defined(CONFIG_ARCH_VERSATILE)
	u32 pin = parrot_gpio_to_pin(gpio);
	if (!parrot_is_select_pin(pin)) {
		printk(KERN_INFO "gpio %d is not selected.\n" 
				"Please do it in your board code\n", gpio);
		/*XXX remove this when everybody select gpio pin in board code */
		/* for p6i gpio should be selected ... */
#if 1
		if ((parrot_chip_is_p6() && gpio < 34) ||
			 parrot_chip_is_p6i()) {
			printk(KERN_ERR "not auto selecting this critical pin\n");
			return 1;
		}
		parrot_select_pin(pin);
#else
		return 1;
#endif
	}
#endif
	return 0;
}

/* GPIO interface */

static inline int gpio_request(unsigned gpio, const char *label)
{
	return 0;
}

static inline void gpio_free(unsigned gpio)
{
	return;
}

static inline int gpio_get_value(unsigned gpio)
{
	return __raw_readl(PARROT_VA_GPIO+__ADDR(gpio));
}

static inline void gpio_set_value(unsigned gpio, int value)
{
	__raw_writel((value)? 0xff : 0x00, PARROT_VA_GPIO+__ADDR(gpio));
}

static inline int gpio_direction_input(unsigned gpio)
{
	int ret = 0;
	u32 val, ddr;
	unsigned long flags;

	local_irq_save(flags);

	if (parrot_gpio_select(gpio)) {
		ret = -ENODEV;
		goto exit;
	}

	ddr = PARROT_VA_GPIO+__DDR(gpio);
	val = __raw_readl(ddr) & ~(1 << __DDR_SHIFT(gpio));
	__raw_writel(val, ddr);

exit:
	local_irq_restore(flags);

	return ret;
}

static inline int gpio_direction_output(unsigned gpio, int value)
{
	int ret = 0;
	u32 val, ddr;
	unsigned long flags;

	local_irq_save(flags);

	if (parrot_gpio_select(gpio)) {
		ret = -ENODEV;
		goto exit;
	}

	ddr = PARROT_VA_GPIO+__DDR(gpio);
	val = __raw_readl(ddr) | (1 << __DDR_SHIFT(gpio));
	__raw_writel(val, ddr);

	/* gpio is now in output mode; if it was previously in input mode,
	   its present value is the same as before its direction changed */

	/* now set new value */
	gpio_set_value(gpio, value);

exit:
	local_irq_restore(flags);

	return ret;
}

int gpio_to_irq(unsigned gpio);

int irq_to_gpio(unsigned irq);


#endif /* !__ASSEMBLY__ */

#endif /* __ARCH_ARM_PARROT_GPIO_REGS */
