#ifndef PLATFORM_P6_FPGA_H
#define PLATFORM_P6_FPGA_H 1
#include <asm/irq.h>
#include <mach/hardware.h>

/* XXX move this */
#define __io_address(n)		(u32)(__io(IO_ADDRESS(n)))
#define PARROT6_GPIO	0xD0110000

#define PARROT6_NANDMC_REGS    0xD00D0000
#define PARROT6_NANDMC_MEM     0xC0300000
#define PARROT6_SPI	    0xD00B0000
#define PARROT6_DMAC	0xC0600000
#define PARROT6_I2CM	0xD0130000
#define PARROT6_SDIO	0xC0700000
#define PARROT6_LCDC	0xD00C0000
#define PARROT6_USB0    0xc0400000
#define PARROT6_PWM 	0xD0100000
#define VERSATILE_TIC_BASE 0xc0000000

#define PARROT6_VA_GPIO		__io_address(PARROT6_GPIO)
#define VA_TIC_BASE		__io_address(VERSATILE_TIC_BASE)
#define VA_P6_PWM_BASE		__io_address(PARROT6_PWM)

#endif
