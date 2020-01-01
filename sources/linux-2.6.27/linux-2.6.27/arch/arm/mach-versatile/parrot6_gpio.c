/*
 * linux/arch/arm/mach-versatile/parrot6_gpio.c
 *
 * Parrot6 GPIO driver
 *
 * Copyright (C) 2008 Parrot S.A.
 *
 * @author     ivan.djelic@parrot.com
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

#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <mach/gpio.h>
#include <mach/gpio_parrot.h>
#include "devs.h"

#define _P6_GPIO_COUNT              162

//! GPIO IRQ source
struct gpio_src {
	unsigned gpio;
	bool active;
};

static struct {
	bool			init;
	void __iomem		*membase;
	struct gpio_src		src[_P6_GPIO_INTC_COUNT];
	u32			rot_status;
} pgpio;

const uint32_t p6_irq_mode[] = {
	[GPIO_IRQ_POS]		= _P6_GPIO_INTCX_POS,
	[GPIO_IRQ_NEG]		= _P6_GPIO_INTCX_NEG,
	[GPIO_IRQ_BOTH]		= _P6_GPIO_INTCX_BOTH,
	[GPIO_IRQ_LEVEL_H]	= _P6_GPIO_INTCX_LEV,
	[GPIO_IRQ_LEVEL_L]	= _P6_GPIO_INTCX_LEV|_P6_GPIO_INTCX_LOW,
};

const uint32_t p6_debounce_mode[] = {
	[GPIO_DEBOUNCE_NONE]	= 0,
	[GPIO_DEBOUNCE_NOISE]	= _P6_GPIO_INTCX_DEBONCE,
	[GPIO_DEBOUNCE_FILTER]	= _P6_GPIO_INTCX_DEBONCE|_P6_GPIO_INTCX_FILTER
};

#define p6_mode(irq_mode, debounce_mode) \
    ((p6_irq_mode[irq_mode]|p6_debounce_mode[debounce_mode])|_P6_GPIO_INTCX_ENA)

/* Useful macros for internal use */

#include <asm/io.h>
#include <mach/platform.h>


/* Select GPIO multiplexing */
static inline void parrot6_gpio_select(unsigned gpio)
{
	// nothing to do on versatile
}

#if 0
int gpio_to_irq(unsigned gpio)
{
	int i;
	int ret = -EINVAL;

	for (i = 0; i < _P6_GPIO_INTC_COUNT; i++) {
		if (pgpio.src[i].active && pgpio.src[i].gpio == gpio) {
			ret = i + IRQ_GPIO_START;
			break;
		}
	}

	return ret;
}
EXPORT_SYMBOL(gpio_to_irq);

int irq_to_gpio(unsigned irq)
{
	int ret = -EINVAL;

	if (irq >= IRQ_GPIO_START && irq <= IRQ_GPIO_END) {
		int id = irq - IRQ_GPIO_START;
		if (pgpio.src[id].active)
			ret = pgpio.src[id].gpio;
	}

	return ret;
}
EXPORT_SYMBOL(irq_to_gpio);
#endif

int gpio_set_debounce_value(uint32_t value)
{
	int ret = 0;
	unsigned long flags;

	local_irq_save(flags);
	if (value <= 31) {
		__raw_writel(value, pgpio.membase + _P6_GPIO_DEBOUNCE);
	} else {
		ret = -EINVAL;
	}
	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(gpio_set_debounce_value);

int gpio_interrupt_register(unsigned gpio,
			    GPIO_IRQ_MODE irq_mode,
			    GPIO_DEBOUNCE_MODE debounce_mode)
{
	int src, i;
	unsigned long flags;

	// sanity check
	if (!pgpio.init || gpio >= _P6_GPIO_COUNT)
		return -ENODEV;
	if (irq_mode < GPIO_IRQ_POS || irq_mode > GPIO_IRQ_LEVEL_L)
		return -EINVAL;
	if (debounce_mode < GPIO_DEBOUNCE_NONE ||
	    debounce_mode > GPIO_DEBOUNCE_FILTER)
		return -EINVAL;

	local_irq_save(flags);

	// allocate an interrupt source
	src = -1;
	if (debounce_mode == GPIO_DEBOUNCE_NONE) {
		for (i = _P6_GPIO_NODEBOUNCE_START;
				i <= _P6_GPIO_NODEBOUNCE_END; i++ ) {
			if (!pgpio.src[i].active) {
				src = i;
				break;
			}
		}
	}

	if (src == -1) {
		int start = _P6_GPIO_DEBOUNCE_START;
#ifdef CONFIG_VERSATILE_PARROT6
		start += 2; // the two first slots are used by the rotator
#endif
		for (i = start; i <= _P6_GPIO_DEBOUNCE_END; i++ ) {
			if (!pgpio.src[i].active) {
				src = i;
				break;
			}
		}
	}

	if (src >= 0) {
		pgpio.src[src].gpio = gpio;
		pgpio.src[src].active = true;

		__raw_writel(p6_mode(irq_mode, debounce_mode)|gpio,
			     pgpio.membase + _P6_GPIO_INTC1 + src*4);
	} else {
		src = -EINVAL;
	}

	local_irq_restore(flags);

	return src+IRQ_GPIO_START;
}
EXPORT_SYMBOL(gpio_interrupt_register);

int gpio_interrupt_unregister(unsigned gpio)
{
	int i, ret = -EINVAL;
	unsigned long flags;

	if (!pgpio.init || gpio >= _P6_GPIO_COUNT)
		return -ENODEV;

	local_irq_save(flags);

	for (i = 0; i < _P6_GPIO_COUNT; i++) {
		if (pgpio.src[i].active && pgpio.src[i].gpio == gpio) {
			pgpio.src[i].active = false;
			__raw_writel(0, pgpio.membase + _P6_GPIO_INTC1 + i*4);
			ret = 0;
			break;
		}
	}

	local_irq_restore(flags);

	return ret;
}
EXPORT_SYMBOL(gpio_interrupt_unregister);

static void gpio_unmask_interrupt(unsigned int src)
{
	void __iomem *addr;
	uint32_t reg;

	addr = pgpio.membase + _P6_GPIO_INTC1 + src*4;
	reg = __raw_readl(addr);
	__raw_writel(reg & ~_P6_GPIO_INTCX_CLR, addr);
}

static void gpio_mask_ack_interrupt(unsigned int src)
{
	void __iomem *addr;
	uint32_t reg;

	addr = pgpio.membase + _P6_GPIO_INTC1 + src*4;
	reg = __raw_readl(addr);
	__raw_writel(reg | _P6_GPIO_INTCX_CLR, addr);
}

enum ROTATOR_EVENT_TAG rotator_get_status(unsigned int rot)
{
	enum ROTATOR_EVENT_TAG ret;

	if (pgpio.rot_status& (1<<rot)) {
		ret = ROTATOR_EVENT_RIGHT;
	} else {
		ret = ROTATOR_EVENT_LEFT;
	}
	return ret;
}
EXPORT_SYMBOL(rotator_get_status);

static void gpio_handle_virq(unsigned int irq, struct irq_desc *desc)
{
	uint32_t status;

	status =  __raw_readl(pgpio.membase + _P6_GPIO_STATUS);

	if (!status)
		return;

	do {
		irq = ffs(status) -1;
		status &= ~(1 << irq);

		irq += IRQ_GPIO_START;
		desc = irq_desc + irq;
		desc_handle_irq(irq, desc);

	} while(status);
}

static void gpio_irq_mask_ack(unsigned int irq)
{
	int id;

	if (irq <= IRQ_GPIO_END) {
		id = irq - IRQ_GPIO_START;
		gpio_mask_ack_interrupt(id);
	} else {
		id = irq - IRQ_ROTATOR_START;
		pgpio.rot_status = __raw_readl(pgpio.membase + _P6_GPIO_ROTATOR_STATUS);
		gpio_mask_ack_interrupt(2*id);
		gpio_mask_ack_interrupt(2*id+1);
	}
}

static void gpio_irq_unmask(unsigned int irq)
{
	int id;

	if (irq <= IRQ_GPIO_END) {
		id = irq - IRQ_GPIO_START;
		gpio_unmask_interrupt(id);
	} else {
		id = irq - IRQ_ROTATOR_START;
		gpio_unmask_interrupt(2*id);
		gpio_unmask_interrupt(2*id+1);
	}
}

static struct irq_chip vchip = {
	.name		= "p6-gpio",
	.mask_ack	= gpio_irq_mask_ack,
	.unmask		= gpio_irq_unmask,
};

static int __init parrot6_gpio_init(void)
{
	int ret = 0, i;
	struct resource *res;
	unsigned char __iomem *membase;
	struct platform_device *dev = &parrot6_gpio_device;

	printk(KERN_INFO "Parrot6 GPIO driver $Revision: 1.8 $\n");

	res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noregs;
	}

	membase = ioremap(res->start, res->end-res->start+1);
	if (!membase) {
		ret = -ENOMEM;
		goto nomap;
	}

	res = platform_get_resource(dev, IORESOURCE_IRQ, 0);
	if (res == NULL) {
		ret = -ENOENT;
		goto noirq;
	}

	set_irq_chained_handler(res->start, gpio_handle_virq);
	for (i = IRQ_GPIO_START; i <= IRQ_ROTATOR_END; i++) {
		set_irq_chip(i, &vchip);
		set_irq_handler(i, handle_level_irq);
		set_irq_flags(i, IRQF_VALID | IRQF_PROBE);
	}

	pgpio.membase = membase;
	for (i = 0; i < _P6_GPIO_INTC_COUNT; i++)
		pgpio.src[i].active = 0;
	pgpio.init = true;

#ifdef CONFIG_VERSATILE_PARROT6
	// This part is HW specific for Rotator 0 on Versatile
	// GPIO 0, 1 are used for the rotator (4 for the button)
	__raw_writel(p6_mode(GPIO_IRQ_BOTH, GPIO_DEBOUNCE_FILTER)|0,
		     pgpio.membase + _P6_GPIO_INTC1);
	__raw_writel(p6_mode(GPIO_IRQ_BOTH, GPIO_DEBOUNCE_FILTER)|1,
		     pgpio.membase + _P6_GPIO_INTC1 + 4);
	__raw_writel(_P6_GPIO_ROTATOR_BOTH, pgpio.membase + _P6_GPIO_ROTATOR);
#endif

	return 0;
noirq:
	iounmap(membase);
nomap:
noregs:
	pgpio.init = false;
	return ret;
}

//XXX this should be conditional for fpga without gpio...
//core_initcall(parrot6_gpio_init);
