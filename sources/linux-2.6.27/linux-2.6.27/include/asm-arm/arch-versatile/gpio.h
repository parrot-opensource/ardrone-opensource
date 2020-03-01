/*
 * linux/include/asm-arm/arch-versatile/gpio.h
 *
 * Parrot6 GPIO interface
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
#ifndef __ASM_ARCH_VERSATILE_GPIO_H
#define __ASM_ARCH_VERSATILE_GPIO_H

#include <asm/irq.h>
#include <asm/hardware.h>

/* XXX move this */
#define __io_address(n)		__io(IO_ADDRESS(n))
#define P6_GPIO_BASE 0xD0110000
#define VA_P6_GPIO_BASE		__io_address(P6_GPIO_BASE)

#define _P6_GPIO_INTC_COUNT          20
#define _P6_ROTATOR_COUNT             2

#define IRQ_GPIO_START		(IRQ_TIC_END+1)
#define IRQ_GPIO_END		(IRQ_GPIO_START+_P6_GPIO_INTC_COUNT-1)
#define IRQ_ROTATOR_START	(IRQ_GPIO_END+1)
#define IRQ_ROTATOR_END         (IRQ_ROTATOR_START+_P6_ROTATOR_COUNT-1)

int gpio_request(unsigned gpio, const char *label);
void gpio_free(unsigned gpio);
int gpio_direction_input(unsigned gpio);
int gpio_get_value(unsigned gpio);
int gpio_direction_output(unsigned gpio, int value);
void gpio_set_value(unsigned gpio, int value);
int gpio_to_irq(unsigned gpio);
int irq_to_gpio(unsigned irq);

typedef enum ROTATOR_EVENT_TAG {
	ROTATOR_EVENT_LEFT,
	ROTATOR_EVENT_RIGHT,
} ROTATOR_EVENT;

typedef enum GPIO_IRQ_MODE_TAG {
	GPIO_IRQ_POS,
	GPIO_IRQ_NEG,
	GPIO_IRQ_BOTH,
	GPIO_IRQ_LEVEL_H,
	GPIO_IRQ_LEVEL_L,
} GPIO_IRQ_MODE;

typedef enum GPIO_DEBOUNCE_MODE_TAG {
	GPIO_DEBOUNCE_NONE,
	GPIO_DEBOUNCE_NOISE,  /* noise detection + blind period */
	GPIO_DEBOUNCE_FILTER, /* filter input with GPIO_Debounce delay added */
} GPIO_DEBOUNCE_MODE;

int gpio_set_debounce_value(uint32_t value);

int gpio_interrupt_register(unsigned gpio,
                            GPIO_IRQ_MODE irq_mode,
			    GPIO_DEBOUNCE_MODE debounce_mode);

int gpio_interrupt_unregister(unsigned gpio);

enum ROTATOR_EVENT_TAG rotator_get_status(unsigned int rot);

#endif
