/**
 * @file dwc_otg_fiq.S
 *
 * @brief FIQ handler for USB controller
 *
 * Copyright (C) 2010 Parrot S.A.
 *
 * @author     david.guilloteau@parrot.com
 * @date       2010-02-16
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

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <mach/map.h>

        .text

		.global	dwc_otg_fiq_handler_end
ENTRY(dwc_otg_fiq_handler_start)

        /* sofint filtering */
        ldr   r8, .usbcore_base
        ldr   r9, [r8, #0x14]
        ldr   r10, [r8, #0x18]
        ands   r9, r9, r10
        /* test if no int */
        beq   no_irq
        /* test if sofint */
        tst   r9, #8
        beq   hchint

        /* sof counter process */
        ldr   r10, [r8, #0x408]
        mov   r10, r10, lsl #16
        sub   r11, r10, r13
        cmp   r11, #0x70000
        movhi r13, r10
        bhi   en_irq

        /* test if sofint only */
        eors  r9, r9, #8
        beq   sof_only

hchint:
        teq   r9, #(1<<25)
        bne   en_irq

        /* haint */
        ldr   r9, [r8, #0x414]
        ldr   r10, [r8, #0x418]
        add   r8, #0x500
        and   r9, r9, r10
haint:
        clz   r10, r9
        cmp   r10, #32
        beq   no_irq
        rsb   r10, r10, #31
        mov   r11, #1
        bic   r9, r9, r11, lsl r10

        /* hcintn : test if nak */
        add   r11, r8, r10, lsl #5
        ldr   r10, [r11, #0x08]
        and   r10, r10, #0x12
        cmp   r10, #0x12
        bne   en_irq

        /* hcchar : test if interrupt transaction */
        ldr   r10, [r11]
        mov   r12, r10
        and   r12, r12, #(3<<18)
        cmp   r12, #(3<<18)
        bne   en_irq

        /* ack hcint and reschedule */
        mvn   r12, #0
        str   r12, [r11, #0x08]
        ldr   r12, [r11, #0x10]
        eor   r10, r10, #(5<<29)
        bic   r12, r12, #(1<<31)
        str   r12, [r11, #0x10]
        str   r10, [r11]
        b     haint

sof_only:
        /* ack sofint */
        mov   r9, #8
        str   r9, [r8, #0x14]

no_irq:
        /* return */
        subs  pc, lr, #4

en_irq:
        /* mask usb fiq */
        ldr   r8, .vic_base
        mov   r9, #(1<<30)
        str   r9, [r8, #0x14]
        /* set most soft irq */
        mov   r9, #8
        str   r9, [r8, #0x18]
        /* test fiq status */
loop:
        ldr   r9, [r8, #4]
        tst   r9, #(1<<30)
        bne   loop

        /* return */
        subs  pc, lr, #4


/* registers */
.vic_base:
        .word 0xfc000000

.usbcore_base:
        .word 0xfc500000

dwc_otg_fiq_handler_end:


