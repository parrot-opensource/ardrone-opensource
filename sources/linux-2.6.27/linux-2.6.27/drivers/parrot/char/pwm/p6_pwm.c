/**
********************************************************************************
* @file p6_pwm.c 
* @brief kernel driver for p6 pwm
*
* Copyright (C) 2008 Parrot S.A.
*
* @author     François Guillemé <francois.guilleme@parrot.com>
* @date       1-Oct-2008
********************************************************************************
*/

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#include <linux/err.h>
#include <linux/clk.h>

#include <mach/hardware.h>
#include <mach/parrot.h>
#include <mach/map-p6.h>
#include <mach/regs-pwm-p6.h>

#include "pwm_ops.h"
#include "pwm_ioctl.h"

#define PWM_NB_TIMER 16

#define BFMASK(_start, _width) (((1 << (_width))-1) << (_start))

#define BFSET(_v, _start, _width, _val) \
	do { \
		_v &= ~(BFMASK(_start, _width)); \
		_v |= ((_val & ((1 << (_width))-1)) << _start); \
	} while (0); 

#define BFGET(_v,_start,_width) \
	(( _v  & BFMASK(_start, _width)) >> _start)

static struct clk *pwm_clk;
static char __iomem *pwm_regbase;
static uint32_t pwm_active;		// we can handle up to 32 timers

static bool px_pwm_is_active(uint32_t n) 
{ 
	return n < PWM_NB_TIMER ? BFGET(pwm_active, n, 1) != 0 : false; 
}

static inline void px_pwm_set_active(uint32_t n, bool active) 
{ 
	// activate the clock if we start the first timer
	if (active && pwm_active == 0) {
		printk(KERN_DEBUG "start pwm clock\n");
		clk_enable(pwm_clk);
	}

	BFSET(pwm_active, n, 1, active);

#ifdef STOP_ON_EXIT
	// stop the clock if we stop the last timer
	if (!active && pwm_active == 0) {
		printk(KERN_DEBUG "stop pwm clock\n");
		clk_disable(pwm_clk);
	}
#endif
}

/** 
 * @brief set the clock mode of a channel
 * 
 * @param ntimer 
 * @param clock mode 
 */
static void px_pwm_set_mode(int ntimer, bool mode)
{
	int offset = 4 + (ntimer & 0x03) + (ntimer >> 2) * 8;
	uint32_t val;

	val = __raw_readl(pwm_regbase + P6_PWM_CTL);

	BFSET(val, offset, 1, mode);

	__raw_writel(val, pwm_regbase + P6_PWM_CTL);
}

/** 
 * @brief enable or disable a pwm channel
 * 
 * @param ntimer 
 * @param start 
 */
static void px_pwm_set_start(int ntimer, bool start)
{
	uint32_t val;
	int offset = (ntimer & 0x03) + (ntimer >> 2) * 8;

	val = __raw_readl(pwm_regbase + P6_PWM_CTL);

	BFSET(val, offset, 1, start);

	__raw_writel(val, pwm_regbase + P6_PWM_CTL);
}

/* Note to get a good ratio precision, frequency should be choosen so that
   pclk/(2*freq) got lot's of 0 at the end.
   Ie freq =pclk/(2*val), where the number of leading 0 are the ratio precision.
   But there is an overflow for 156Mhz starting for freq = pclk/(2*0x3000)
   ie freq < 9521Hz.
   */
static unsigned int compute_speed(unsigned int speed, unsigned int *length)
{
	unsigned int pclk = clk_get_rate(pwm_clk);
	unsigned int val = (pclk + speed) / ( speed * 2);
	unsigned int tmp;

	if (val <= 1)
		val = 1;

	/* *length >= 0 */
	*length = ffs(val) - 1;

	/* fallback to p5+ config */
	if (*length < 8 && val >= 0xf00)
		*length = 8;

	tmp = fls(val >> *length);
	if (tmp > 16) {
		*length += tmp - 16;
	}

	if (*length > 0) {
		/* round it */
		val += 1 << (*length - 1);
		val >>= *length;
	}

	val -= 1;
	/* should never happen because val is a 32 bits int */
	WARN_ON(*length & ~(0x1f));
	WARN_ON(val > 0xffff);
	return val;
}

static unsigned int compute_ratio(unsigned int ratio, unsigned int length)
{
	unsigned int val;
    if (ratio > PWM_WIDTH_MAX)
        return -EINVAL;
	/* XXX overflow happen if length > 16 */

	val = ((ratio << length) + PWM_WIDTH_MAX/2) / PWM_WIDTH_MAX;
	WARN_ON(val > 0xffff);
	return val;
}

/** 
 * @brief set the speed of a channel
 * 
 * @param ntimer 
 * @param speed 
 */
static void px_pwm_set_speed(int ntimer, unsigned int speed)
{
	unsigned int length, freq;
	uint32_t ratio;
	freq = compute_speed(speed, &length);

	__raw_writel(freq, pwm_regbase + P6_PWM_SPEED00 + ntimer*4);
	ratio = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	BFSET(ratio, 16, 5, length);
	__raw_writel(ratio, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
}

/** 
 * @brief get the speed of a channel
 * 
 * @param ntimer 
 * 
 * @return 
 */
static unsigned int px_pwm_get_speed(int ntimer)
{
	unsigned int freq;
	uint32_t pclk = clk_get_rate(pwm_clk);
	uint32_t val = __raw_readl(pwm_regbase + P6_PWM_SPEED00 + ntimer*4);
	uint32_t length = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4) >> 16;
	freq = pclk /((val + 1) * 2);
	freq >>= length;
	if (freq == 0)
		freq = 1;
	return freq;
}

/** 
 * @brief set the ratio of a channel
 * 
 * @param ntimer 
 * @param ratio 
 */
static void px_pwm_set_ratio(int ntimer, unsigned int ratio)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	unsigned int ra;
	if (length == 0) {
		/* TODO may be add 0/1 mode */
		/* clock mode */
		px_pwm_set_mode(ntimer, true);
		return;
	}	
	px_pwm_set_mode(ntimer, false);

	ra = compute_ratio(ratio, length);

	BFSET(ratio_val, 0, 16, ra);

	__raw_writel(ratio_val, pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
}

/** 
 * @brief get the ratio of a channel
 * 
 * @param ntimer 
 * 
 * @return 
 */
static unsigned int px_pwm_get_ratio(int ntimer)
{
	uint32_t ratio_val = __raw_readl(pwm_regbase + P6_PWM_RATIO00 + ntimer*4);
	uint32_t length = ratio_val >> 16;
	uint32_t ratio = ratio_val & 0xffff;

	if (length == 0)
		return PWM_WIDTH_MAX/2; /* clock mode */
	else
		return (ratio * PWM_WIDTH_MAX) >> length;
}

/**
 * reserve a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int px_pwm_request(unsigned int timer)
{
    if (timer >= PWM_NB_TIMER)
        return -EINVAL;

    if (px_pwm_is_active(timer))
        return -EBUSY;
    
	px_pwm_set_active(timer, true);

    return 0;
}

/**
 * release a pwm. Note that it is to the caller to make sure, there is no
 * concurent access
 *
 * @param timer : timer number
 * @return 0 or an error code
 */
static int px_pwm_release(unsigned int timer)
{
    if (timer >= PWM_NB_TIMER)
        return -EINVAL;

    if (!px_pwm_is_active(timer))
        return -EBUSY;
    
	px_pwm_set_active(timer, false);
    
	return 0;
}

/**
 * start a timer
 *
 * @param timer the timer to use
 *
 * @return error code
 */
static int px_pwm_start(unsigned int timer)
{
    unsigned long flags;

    if (timer > PWM_NB_TIMER)
        return -EINVAL;

    local_irq_save(flags);

	px_pwm_set_start(timer, 1);

    local_irq_restore(flags);

    return 0;
}

/** stop a timer
 *
 * @param timer
 * @return error code
 */
static int px_pwm_stop(unsigned int timer)
{
    unsigned long flags;

    if (timer > PWM_NB_TIMER)
        return -EINVAL;

    local_irq_save(flags);

	px_pwm_set_start(timer, 0);
    
	local_irq_restore(flags);
    
	return 0;
}

/** configure a timer freq
 *
 * if the timer if not stopped, the value will taken in account a the next 
 * reload (if autoreload is on)
 *
 * @param timer
 * @param freq in HZ
 *
 * @return error code
 */
static int px_pwm_set_freq(unsigned int timer, unsigned int freq)
{

    if (timer > PWM_NB_TIMER)
        return -EINVAL;

	px_pwm_set_speed(timer, freq);

    return 0;
}

static int px_pwm_get_freq(unsigned int timer, unsigned int *freq)
{
    if (timer > PWM_NB_TIMER)
        return -EINVAL;

	*freq = px_pwm_get_speed(timer);
    return 0;
}

/* ratio = 0 ... 100 00 */
/**
 * configure the duty cycle
 *
 * @param timer
 * @param ratio (percentage * 100 of the dc)
 *
 * @return error code
 */
static int px_pwm_set_width(unsigned int timer, unsigned int ratio)
{
    if (ratio > PWM_WIDTH_MAX)
        return -EINVAL;
    
	px_pwm_set_ratio(timer, ratio);

    return 0;
}

static int px_pwm_get_width(unsigned int timer, unsigned int *ratio)
{
    if (timer > PWM_NB_TIMER)
        return -EINVAL;

    *ratio = px_pwm_get_ratio(timer);

    return 0;
}

struct pwm_ops px_pwm_ops = {
    .pwm_max = PWM_NB_TIMER-1,
    .pwm_start = px_pwm_start,
    .pwm_stop = px_pwm_stop,
    .pwm_request = px_pwm_request,
    .pwm_release = px_pwm_release,
    .pwm_set_width = px_pwm_set_width,
    .pwm_set_freq = px_pwm_set_freq,
    .pwm_get_width = px_pwm_get_width,
    .pwm_get_freq = px_pwm_get_freq,
    .owner = THIS_MODULE,
};

static int __devinit px_pwm_init(void)
{
	/* XXX this should use platform stuff... */
	pwm_regbase = ioremap(PARROT6_PWM, 256);
	if (pwm_regbase == NULL) {
		printk( KERN_ERR "ioremap failed\n");
		return -ENOMEM;
	}
	pwm_clk = clk_get(NULL, "pwm");
	
	if (IS_ERR(pwm_clk)) {
		printk(KERN_ERR "PWM clock not found");
		return -EBUSY;
	}

	pwm_active = 0;

    return register_pwm(&px_pwm_ops);
}

static void __exit px_pwm_exit(void)
{
	clk_disable(pwm_clk);
	clk_put(pwm_clk);

    unregister_pwm(&px_pwm_ops);
}

module_init(px_pwm_init);
module_exit(px_pwm_exit);

MODULE_AUTHOR("PARROT SA");
MODULE_DESCRIPTION("p6 pwm driver");
MODULE_LICENSE("GPL");



