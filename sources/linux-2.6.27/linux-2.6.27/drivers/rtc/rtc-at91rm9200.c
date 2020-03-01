/*
 *	Real Time Clock interface for Linux on Atmel AT91RM9200
 *
 *	Copyright (C) 2002 Rick Bronson
 *
 *	Converted to RTC class model by Andrew Victor
 *
 *	Ported to Linux 2.6 by Steven Scholz
 *	Based on s3c2410-rtc.c Simtec Electronics
 *
 *	Based on sa1100-rtc.c by Nils Faerber
 *	Based on rtc.c by Paul Gortmaker
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>

#include <asm/uaccess.h>
//#include <mach/at91_rtc.h>
#define I2C_MODE
#define I2C_MODE_POLL
#define AT91_RTC 0
#include <../../mach-at91/include/mach/at91_rtc.h>

#if 1
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kthread.h>
struct i2c_adapter *i2c_adapter;
static irqreturn_t at91_rtc_interrupt(int irq, void *dev_id);
static struct timer_list irq_timer;
static struct completion irq_event;
static struct task_struct *irq_thread;

static int i2c0_read(int addr, int reg)
{
	struct i2c_adapter *a = i2c_adapter;//i2c_get_adapter(0);
	struct i2c_msg msg[2];
	int ret = 0;

	u_int8_t buf[2] = {0xaa, 0x55};

	if (a == NULL)
		return -1;

	msg[0].addr = addr;
	msg[0].buf = buf;
	msg[0].len = 1;
	msg[0].flags = 0;

	buf[0] = reg & 0xff;

#if 0
	ret = i2c_transfer(a, msg, 1);
	if (ret <= 0)
		printk("error : %d\n", ret);
#endif

	msg[1].addr = addr;
	msg[1].buf = buf+1;
	msg[1].len = 1;
	msg[1].flags = I2C_M_RD;

	ret = i2c_transfer(a, msg+0, 2);
	if (ret <= 0) {
		printk("error r : %d(%x %x)\n", ret, addr, reg);
		return -1;
	}

	//printk("read %x %x\n", reg, buf[1]);
	return buf[1];
}
static int i2c0_write(int addr, int reg, int val)
{
	struct i2c_adapter *a = i2c_adapter;//i2c_get_adapter(0);
	struct i2c_msg msg;
	int ret = 0;
	u_int8_t buf[2];

	if (a == NULL)
		return -1;

	msg.addr = addr;
	msg.buf = buf;
	msg.len = 2;
	msg.flags = 0;

	buf[0] = reg & 0xff;
	buf[1] = val & 0xff;

	ret = i2c_transfer(a, &msg, 1);
	if (ret <= 0)
		printk("error w : %d\n", ret);

	return ret;
}

#define RTC_CTRL 0x50
#define RTC_CTRL_RTC_EN 1
#define RTC_CTRL_RTC_SEL 2
#define RTC_CTRL_RTC_WRITE 4
#define RTC_ADDR 0x51
#define RTC_DATA0 0x52
#define RTC_DATA1 0x53
#define RTC_DATA2 0x54
#define RTC_DATA3 0x55
static inline unsigned int at91_sys_read(unsigned int reg_offset)
{
	unsigned int val;
	int ret;
	char buf0[] = { RTC_ADDR, reg_offset};
	static char buf1[] = { RTC_CTRL, RTC_CTRL_RTC_SEL};
	static char buf2[] = { RTC_CTRL, RTC_CTRL_RTC_SEL|RTC_CTRL_RTC_EN};
	static char buf3[] = { RTC_CTRL, 0};
	static char buf4[] = { RTC_DATA0, RTC_DATA1, RTC_DATA2, RTC_DATA3};
	char buf5[4];
	struct i2c_adapter *a = i2c_adapter;//i2c_get_adapter(0);
	struct i2c_msg msg[] = {
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf0,
		},
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf1,
		},
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf2,
		},
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf1,
		},
		/* data3 */
		{
			.addr = 0x49,
			.len = 1,
			.buf = &buf4[3],
		},
		{
			.addr = 0x49,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &buf5[3],
		},
		/* data2 */
		{
			.addr = 0x49,
			.len = 1,
			.buf = &buf4[2],
		},
		{
			.addr = 0x49,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &buf5[2],
		},
		/* data1 */
		{
			.addr = 0x49,
			.len = 1,
			.buf = &buf4[1],
		},
		{
			.addr = 0x49,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &buf5[1],
		},
		/* data0 */
		{
			.addr = 0x49,
			.len = 1,
			.buf = &buf4[0],
		},
		{
			.addr = 0x49,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = &buf5[0],
		},

		/* end */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf3,
		},

	};
	if (a == NULL)
		return -1;

	ret= i2c_transfer(a, msg, ARRAY_SIZE(msg)); 
	if (ret != ARRAY_SIZE(msg)) {
		printk("failed!!!!!!\n");
	}
	val = buf5[0]|(buf5[1]<<8)|(buf5[2]<<16)|(buf5[3]<<24);
	//printk("reading %02x : %08x\n", reg_offset, val);
	return val;
}

static inline void at91_sys_write(unsigned int reg_offset, unsigned long value)
{
	int ret;
	char buf0[] = { RTC_ADDR, reg_offset};
	static char buf1[] = { RTC_CTRL, RTC_CTRL_RTC_WRITE|RTC_CTRL_RTC_SEL};
	static char buf2[] = { RTC_CTRL, RTC_CTRL_RTC_WRITE|RTC_CTRL_RTC_SEL|RTC_CTRL_RTC_EN};
	static char buf3[] = { RTC_CTRL, 0};
	char buf7[] = { RTC_DATA3, (value>>24) & 0xff};
	char buf6[] = { RTC_DATA2, (value>>16) & 0xff};
	char buf5[] = { RTC_DATA1, (value>>8) & 0xff};
	char buf4[] = { RTC_DATA0, (value>>0) & 0xff};
	struct i2c_adapter *a = i2c_adapter;//i2c_get_adapter(0);
	struct i2c_msg msg[] = {
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf0,
		},
		/* data0 */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf4,
		},
		/* data1 */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf5,
		},
		/* data2 */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf6,
		},
		/* data3 */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf7,
		},
		/* ctrl */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf1,
		},
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf2,
		},
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf1,
		},

		/* end */
		{
			.addr = 0x49,
			.len = 2,
			.buf = buf3,
		},

	};
	if (a == NULL)
		return;

	//printk("writing %02x %08x\n", reg_offset, value);
	ret= i2c_transfer(a, msg, ARRAY_SIZE(msg)); 
	if (ret != ARRAY_SIZE(msg)) {
		printk("failed!!!!!!\n");
	}

	if (reg_offset == AT91_RTC_IER || reg_offset == AT91_RTC_IDR) {
		unsigned long imr = at91_sys_read(AT91_RTC_IMR);
		if (imr) {
			if (timer_pending(&irq_timer) == 0)
				mod_timer(&irq_timer, jiffies + HZ);
		}
		else {
			del_timer_sync(&irq_timer);
		}
	}
	return;
}

/* TODO implement irq mode */
static int twl4030_irq_thread(void *data)
{
	while (!kthread_should_stop()) {

		wait_for_completion_interruptible(&irq_event);
		if (i2c0_read(0x49, 0xd) & 3)
			at91_rtc_interrupt(0, data);
	}
	return 0;
}

static void irq_timer_call(unsigned long data)
{
	complete(&irq_event);
	mod_timer(&irq_timer, jiffies + HZ);
}
#endif


#define AT91_RTC_FREQ		1
#define AT91_RTC_EPOCH		1900UL	/* just like arch/arm/common/rtctime.c */

static DECLARE_COMPLETION(at91_rtc_updated);
static unsigned int at91_alarm_year = AT91_RTC_EPOCH;
struct rtc_device *at91_rtc;

/*
 * Decode time/date into rtc_time structure
 */
static void at91_rtc_decodetime(unsigned int timereg, unsigned int calreg,
				struct rtc_time *tm)
{
	unsigned int time, date;

	/* must read twice in case it changes */
	do {
		time = at91_sys_read(timereg);
		date = at91_sys_read(calreg);
	} while ((time != at91_sys_read(timereg)) ||
			(date != at91_sys_read(calreg)));

	tm->tm_sec  = BCD2BIN((time & AT91_RTC_SEC) >> 0);
	tm->tm_min  = BCD2BIN((time & AT91_RTC_MIN) >> 8);
	tm->tm_hour = BCD2BIN((time & AT91_RTC_HOUR) >> 16);

	/*
	 * The Calendar Alarm register does not have a field for
	 * the year - so these will return an invalid value.  When an
	 * alarm is set, at91_alarm_year wille store the current year.
	 */
	tm->tm_year  = BCD2BIN(date & AT91_RTC_CENT) * 100;	/* century */
	tm->tm_year += BCD2BIN((date & AT91_RTC_YEAR) >> 8);	/* year */

	tm->tm_wday = BCD2BIN((date & AT91_RTC_DAY) >> 21) - 1;	/* day of the week [0-6], Sunday=0 */
	tm->tm_mon  = BCD2BIN((date & AT91_RTC_MONTH) >> 16) - 1;
	tm->tm_mday = BCD2BIN((date & AT91_RTC_DATE) >> 24);
}

/*
 * Read current time and date in RTC
 */
static int at91_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	at91_rtc_decodetime(AT91_RTC_TIMR, AT91_RTC_CALR, tm);
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	tm->tm_year = tm->tm_year - 1900;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

/*
 * Set current time and date in RTC
 */
static int at91_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	unsigned long cr;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	/* Stop Time/Calendar from counting */
	cr = at91_sys_read(AT91_RTC_CR);
	at91_sys_write(AT91_RTC_CR, cr | AT91_RTC_UPDCAL | AT91_RTC_UPDTIM);

	at91_sys_write(AT91_RTC_IER, AT91_RTC_ACKUPD);
	wait_for_completion(&at91_rtc_updated);	/* wait for ACKUPD interrupt */
	at91_sys_write(AT91_RTC_IDR, AT91_RTC_ACKUPD);

	at91_sys_write(AT91_RTC_TIMR,
			  BIN2BCD(tm->tm_sec) << 0
			| BIN2BCD(tm->tm_min) << 8
			| BIN2BCD(tm->tm_hour) << 16);

	at91_sys_write(AT91_RTC_CALR,
			  BIN2BCD((tm->tm_year + 1900) / 100)	/* century */
			| BIN2BCD(tm->tm_year % 100) << 8	/* year */
			| BIN2BCD(tm->tm_mon + 1) << 16		/* tm_mon starts at zero */
			| BIN2BCD(tm->tm_wday + 1) << 21	/* day of the week [0-6], Sunday=0 */
			| BIN2BCD(tm->tm_mday) << 24);

	/* Restart Time/Calendar */
	cr = at91_sys_read(AT91_RTC_CR);
	at91_sys_write(AT91_RTC_CR, cr & ~(AT91_RTC_UPDCAL | AT91_RTC_UPDTIM));

	return 0;
}

/*
 * Read alarm time and date in RTC
 */
static int at91_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time *tm = &alrm->time;

	at91_rtc_decodetime(AT91_RTC_TIMALR, AT91_RTC_CALALR, tm);
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);
	tm->tm_year = at91_alarm_year - 1900;

	alrm->enabled = (at91_sys_read(AT91_RTC_IMR) & AT91_RTC_ALARM)
			? 1 : 0;

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
		1900 + tm->tm_year, tm->tm_mon, tm->tm_mday,
		tm->tm_hour, tm->tm_min, tm->tm_sec);

	return 0;
}

/*
 * Set alarm time and date in RTC
 */
static int at91_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct rtc_time tm;

	at91_rtc_decodetime(AT91_RTC_TIMR, AT91_RTC_CALR, &tm);

	at91_alarm_year = tm.tm_year;

	tm.tm_hour = alrm->time.tm_hour;
	tm.tm_min = alrm->time.tm_min;
	tm.tm_sec = alrm->time.tm_sec;

	at91_sys_write(AT91_RTC_IDR, AT91_RTC_ALARM);
	at91_sys_write(AT91_RTC_TIMALR,
		  BIN2BCD(tm.tm_sec) << 0
		| BIN2BCD(tm.tm_min) << 8
		| BIN2BCD(tm.tm_hour) << 16
		| AT91_RTC_HOUREN | AT91_RTC_MINEN | AT91_RTC_SECEN);
	at91_sys_write(AT91_RTC_CALALR,
		  BIN2BCD(tm.tm_mon + 1) << 16		/* tm_mon starts at zero */
		| BIN2BCD(tm.tm_mday) << 24
		| AT91_RTC_DATEEN | AT91_RTC_MTHEN);

	if (alrm->enabled) {
		at91_sys_write(AT91_RTC_SCCR, AT91_RTC_ALARM);
		at91_sys_write(AT91_RTC_IER, AT91_RTC_ALARM);
	}

	pr_debug("%s(): %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
		at91_alarm_year, tm.tm_mon, tm.tm_mday, tm.tm_hour,
		tm.tm_min, tm.tm_sec);

	return 0;
}

/*
 * Handle commands from user-space
 */
static int at91_rtc_ioctl(struct device *dev, unsigned int cmd,
			unsigned long arg)
{
	int ret = 0;

	pr_debug("%s(): cmd=%08x, arg=%08lx.\n", __func__, cmd, arg);

	/* important:  scrub old status before enabling IRQs */
	switch (cmd) {
	case RTC_AIE_OFF:	/* alarm off */
		at91_sys_write(AT91_RTC_IDR, AT91_RTC_ALARM);
		break;
	case RTC_AIE_ON:	/* alarm on */
		at91_sys_write(AT91_RTC_SCCR, AT91_RTC_ALARM);
		at91_sys_write(AT91_RTC_IER, AT91_RTC_ALARM);
		break;
	case RTC_UIE_OFF:	/* update off */
		at91_sys_write(AT91_RTC_IDR, AT91_RTC_SECEV);
		break;
	case RTC_UIE_ON:	/* update on */
		at91_sys_write(AT91_RTC_SCCR, AT91_RTC_SECEV);
		at91_sys_write(AT91_RTC_IER, AT91_RTC_SECEV);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

/*
 * Provide additional RTC information in /proc/driver/rtc
 */
static int at91_rtc_proc(struct device *dev, struct seq_file *seq)
{
	unsigned long imr = at91_sys_read(AT91_RTC_IMR);

	seq_printf(seq, "update_IRQ\t: %s\n",
			(imr & AT91_RTC_ACKUPD) ? "yes" : "no");
	seq_printf(seq, "periodic_IRQ\t: %s\n",
			(imr & AT91_RTC_SECEV) ? "yes" : "no");
	seq_printf(seq, "periodic_freq\t: %ld\n",
			(unsigned long) AT91_RTC_FREQ);

	return 0;
}

/*
 * IRQ handler for the RTC
 */
static irqreturn_t at91_rtc_interrupt(int irq, void *dev_id)
{
	struct rtc_device *rtc = at91_rtc;
	unsigned int rtsr;
	unsigned long events = 0;

	rtsr = at91_sys_read(AT91_RTC_SR) & at91_sys_read(AT91_RTC_IMR);
	if (rtsr) {		/* this interrupt is shared!  Is it ours? */
		if (rtsr & AT91_RTC_ALARM)
			events |= (RTC_AF | RTC_IRQF);
		if (rtsr & AT91_RTC_SECEV)
			events |= (RTC_UF | RTC_IRQF);
		if (rtsr & AT91_RTC_ACKUPD)
			complete(&at91_rtc_updated);

		at91_sys_write(AT91_RTC_SCCR, rtsr);	/* clear status reg */

		rtc_update_irq(rtc, 1, events);

		pr_debug("%s(): num=%ld, events=0x%02lx\n", __func__,
			events >> 8, events & 0x000000FF);

		return IRQ_HANDLED;
	}
	return IRQ_NONE;		/* not handled */
}

static const struct rtc_class_ops at91_rtc_ops = {
	.ioctl		= at91_rtc_ioctl,
	.read_time	= at91_rtc_readtime,
	.set_time	= at91_rtc_settime,
	.read_alarm	= at91_rtc_readalarm,
	.set_alarm	= at91_rtc_setalarm,
	.proc		= at91_rtc_proc,
};

/*
 * Initialize and install RTC driver
 */
static int __init at91_rtc_probe(struct device *dev, const char *name)
{
	struct rtc_device *rtc;
	int ret;

	at91_sys_write(AT91_RTC_CR, 0);
	at91_sys_write(AT91_RTC_MR, 0);		/* 24 hour mode */

	/* Disable all interrupts */
	at91_sys_write(AT91_RTC_IDR, AT91_RTC_ACKUPD | AT91_RTC_ALARM |
					AT91_RTC_SECEV | AT91_RTC_TIMEV |
					AT91_RTC_CALEV);

#ifndef I2C_MODE_POLL
	ret = request_irq(AT91_ID_SYS, at91_rtc_interrupt,
				IRQF_DISABLED | IRQF_SHARED,
				"at91_rtc", &at91_rtc);
	if (ret) {
		printk(KERN_ERR "at91_rtc: IRQ %d already in use.\n",
				AT91_ID_SYS);
		return ret;
	}
#else
	setup_timer(&irq_timer, irq_timer_call, (unsigned long)NULL);
	init_completion(&irq_event);
	irq_thread = kthread_run(twl4030_irq_thread, NULL, "p6mu-irq");
	//mod_timer(&irq_timer, jiffies + HZ/10);
	/* timer will start when writting IER */
#endif

	/* cpu init code should really have flagged this device as
	 * being wake-capable; if it didn't, do that here.
	 */
	if (!device_can_wakeup(dev))
		device_init_wakeup(dev, 1);

	rtc = rtc_device_register(name, dev,
				&at91_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
#ifndef I2C_MODE_POLL
		free_irq(AT91_ID_SYS, &at91_rtc);
#else
		complete(&irq_event);
		kthread_stop(irq_thread);
		del_timer_sync(&irq_timer);
#endif
		return PTR_ERR(rtc);
	}
	at91_rtc = rtc;

	printk(KERN_INFO "AT91 Real Time Clock driver.\n");
	return 0;
}

/*
 * Disable and remove the RTC driver
 */
static int __exit at91_rtc_remove(struct device *dev)
{
	struct rtc_device *rtc = at91_rtc;

	/* Disable all interrupts */
	at91_sys_write(AT91_RTC_IDR, AT91_RTC_ACKUPD | AT91_RTC_ALARM |
					AT91_RTC_SECEV | AT91_RTC_TIMEV |
					AT91_RTC_CALEV);
#ifndef I2C_MODE_POLL
	free_irq(AT91_ID_SYS, &at91_rtc);
#else
	/* THIS is racy */
	complete(&irq_event);
	kthread_stop(irq_thread);
	del_timer_sync(&irq_timer);
#endif

	rtc_device_unregister(rtc);

	return 0;
}

#ifndef I2C_MODE
static int __init at91_rtc_probe_plat(struct platform_device *pdev)
{
	return at91_rtc_probe(&pdev->dev, pdev->name);
}


static int __exit at91_rtc_remove_plat(struct platform_device *pdev)
{
	return at91_rtc_remove(&pdev->dev);
}

#ifdef CONFIG_PM

/* AT91RM9200 RTC Power management control */

static u32 at91_rtc_imr;

static int at91_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* this IRQ is shared with DBGU and other hardware which isn't
	 * necessarily doing PM like we are...
	 */
	at91_rtc_imr = at91_sys_read(AT91_RTC_IMR)
			& (AT91_RTC_ALARM|AT91_RTC_SECEV);
	if (at91_rtc_imr) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(AT91_ID_SYS);
		else
			at91_sys_write(AT91_RTC_IDR, at91_rtc_imr);
	}
	return 0;
}

static int at91_rtc_resume(struct platform_device *pdev)
{
	if (at91_rtc_imr) {
		if (device_may_wakeup(&pdev->dev))
			disable_irq_wake(AT91_ID_SYS);
		else
			at91_sys_write(AT91_RTC_IER, at91_rtc_imr);
	}
	return 0;
}
#else
#define at91_rtc_suspend NULL
#define at91_rtc_resume  NULL
#endif

static struct platform_driver at91_rtc_driver = {
	.remove		= __exit_p(at91_rtc_remove),
	.suspend	= at91_rtc_suspend,
	.resume		= at91_rtc_resume,
	.driver		= {
		.name	= "at91_rtc",
		.owner	= THIS_MODULE,
	},
};
#endif

#ifdef I2C_MODE
static int __init x1205_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ver;
	i2c_adapter = client->adapter;
	ver = at91_sys_read(0xFC);
	printk("rtc version %x valid %x\n", ver,
			at91_sys_read(AT91_RTC_VER));
	if (ver != 0x222 && ver != 0)
		return -ENODEV;

	if (!(i2c0_read(0x49, 0x56) & 2)) {
		i2c0_write(0x49, 0x56, 0xc);
		udelay(200);
		i2c0_write(0x49, 0x56, 0x4);
		if (i2c0_read(0x49, 0x56) != 0x6)
			printk("starting rtc oscilator failed\n");
	}

	/* unmask rtc irq */
	i2c0_write(0x49, 0xc, i2c0_read(0x49, 0xc) & ~(3));

	return at91_rtc_probe(&client->dev, "p6mu");
}
static int __exit x1205_remove(struct i2c_client *client)
{
	/* mask rtc irq */
	i2c0_write(0x49, 0xc, i2c0_read(0x49, 0xc) | 3);
	/* XXX disable oscilator ? */
	/* we want the rtc to run across reboot ... */
	return at91_rtc_remove(&client->dev);
}

static const struct i2c_device_id x1205_id[] = {
	{ "p6mu_rtc", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, x1205_id);

static struct i2c_driver x1205_driver = {
	.driver		= {
		.name	= "rtc-p6mu",
	},
	.probe		= x1205_probe,
	.remove		= __exit_p(x1205_remove),
	.id_table	= x1205_id,
};
#endif

static int __init at91_rtc_init(void)
{
#ifdef I2C_MODE
	return i2c_add_driver(&x1205_driver);
#else
	return platform_driver_probe(&at91_rtc_driver, at91_rtc_probe);
#endif
}

static void __exit at91_rtc_exit(void)
{
#ifdef I2C_MODE
	i2c_del_driver(&x1205_driver);
#else
	platform_driver_unregister(&at91_rtc_driver);
#endif
}

module_init(at91_rtc_init);
module_exit(at91_rtc_exit);

MODULE_AUTHOR("Rick Bronson");
MODULE_DESCRIPTION("RTC driver for Atmel AT91RM9200");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:at91_rtc");
