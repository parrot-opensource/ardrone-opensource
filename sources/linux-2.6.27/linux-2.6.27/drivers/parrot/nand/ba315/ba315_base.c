#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/io.h>
#include <linux/mtd/partitions.h>

#include "../onfi.h"
#include "kba315.h"
#include "ba315_mtd.h"
#include "ba315_base.h"

/*
 * FIXME :
 * chip->errstat on operation failure
 * cached read/program
 * merge curr and next work
 * do not send command to nand if next work is related to the page cached in the
 * controller RAM
 * copy back operations
 * read verify
 * random data read (when accessing data spread over the same page, for data/oob
 * data/ecc for example)
 * random data write...
 */

/* BA315 registers */

#define BA315_STATUS                         0x00
#define BA315_START                          0x04
#define BA315_TIM0                           0x08
#define BA315_TIM1                           0x0C
#define BA315_CFG                            0x10
#define BA315_CTRL0                          0x14
#define BA315_CTRL1                          0x18
#define BA315_ADDR_LO                        0x1C
#define BA315_TIMEOUT                        0x20
#define BA315_ECC_CFG                        0x24
#define BA315_IRQ_ENABLE                     0x28
#define BA315_IRQ_DISABLE                    0x2C
#define BA315_IRQ_STATUS                     0x30
#define BA315_ADDR_HI                        0x34

/* BA315_STATUS */

#define BA315_STATUS_RnB                     (1 << 0)
#define BA315_STATUS_READY_CMD               (1 << 1)
#define BA315_STATUS_DEC_ERR                 (1 << 2)
#define BA315_STATUS_DEC_FAIL                (1 << 9)
#define BA315_STATUS_WRITE_FAULT             (1 << 10)

#define fsBA315_STATUS_NB_ERRS               (3)
#define fwBA315_STATUS_NB_ERRS               (6)

/* BA315_START */

#define BA315_START_START_PULSE              (1 << 0)
#define BA315_START_RESET_PULSE              (1 << 1)

/* BA315_TIM0  */

#define fsBA315_TIM0_TWP                     (0)
#define fwBA315_TIM0_TWP                     (4)

#define fsBA315_TIM0_TWH                     (4)
#define fwBA315_TIM0_TWH                     (4)

#define fsBA315_TIM0_TRP                     (8)
#define fwBA315_TIM0_TRP                     (4)

#define fsBA315_TIM0_TREH                    (12)
#define fwBA315_TIM0_TREH                    (4)

#define fsBA315_TIM0_TWSETUP                 (16)
#define fwBA315_TIM0_TWSETUP                 (4)

#define fsBA315_TIM0_TRSETUP                 (20)
#define fwBA315_TIM0_TRSETUP                 (4)

#define fsBA315_TIM0_TWHR                    (24)
#define fwBA315_TIM0_TWHR                    (4)

#define fsBA315_TIM0_TCEH                    (28)
#define fwBA315_TIM0_TCEH                    (3)

#define BA315_TIM0_EDO                       (1 << 31)

/* BA315_TIM1 */

#define fsBA315_TIM1_BTA                     (0)
#define fwBA315_TIM1_BTA                     (6)

#define fsBA315_TIM1_TBUSY                   (6)
#define fwBA315_TIM1_TBUSY                   (5)

/* BA315_CFG */

#define BA315_CFG_MODE_8_16                  (1 << 0)
#define BA315_CFG_COL_ADDR_NUM               (1 << 1)
#define BA315_CFG_WP_ENABLE                  (1 << 2)

#define fsBA315_CFG_ROW_ADDR_PROT            (3)
#define fwBA315_CFG_ROW_ADDR_PROT            (24)

#define fsBA315_CFG_DEC_CLK_DIV              (27)
#define fwBA315_CFG_DEC_CLK_DIV              (2)

#define BA315_CFG_BOOT_ENABLE                (1 << 29)
#define BA315_CFG_CE_INTERCEPT               (1 << 30)

/* BA315_CTRL0 */

#define fsBA315_CTRL0_CMD1                   (0)
#define fwBA315_CTRL0_CMD1                   (8)

#define fsBA315_CTRL0_CMD2                   (8)
#define fwBA315_CTRL0_CMD2                   (8)

#define fsBA315_CTRL0_CMD3                   (16)
#define fwBA315_CTRL0_CMD3                   (8)

#define BA315_CTRL0_CMD1_EN                  (1 << 24)
#define BA315_CTRL0_CMD2_EN                  (1 << 25)
#define BA315_CTRL0_CMD3_EN                  (1 << 26)
#define BA315_CTRL0_RW                       (1 << 27)
#define BA315_CTRL0_DATA_EN                  (1 << 28)

#define fsBA315_CTRL0_ADDR_EN                (29)
#define fwBA315_CTRL0_ADDR_EN                (3)

/* BA315_CTRL1 */

#define fsBA315_CTRL1_NB_DATA                (8)
#define fwBA315_CTRL1_NB_DATA                (13)

#define BA315_CTRL1_ECC_ENABLE               (1 << 21)
#define BA315_CTRL1_WAIT_READY               (1 << 22)
#define BA315_CTRL1_CE_BUSY                  (1 << 23)
#define BA315_CTRL1_CE_SELECT                (3 << 24)
#define BA315_CTRL1_DMA_ENABLE               (1 << 26)

/* BA315_ADDR_LO */

#define fsBA315_ADDR_LO_ADDR0                (0)
#define fwBA315_ADDR_LO_ADDR0                (8)

#define fsBA315_ADDR_LO_ADDR1                (8)
#define fwBA315_ADDR_LO_ADDR1                (8)

#define fsBA315_ADDR_LO_ADDR2                (16)
#define fwBA315_ADDR_LO_ADDR2                (8)

#define fsBA315_ADDR_LO_ADDR3                (24)
#define fwBA315_ADDR_LO_ADDR3                (8)

/* BA315_ADDR_HI */

#define fsBA315_ADDR_HI_ADDR4                (0)
#define fwBA315_ADDR_HI_ADDR4                (8)

/* BA315_TIMEOUT */

#define fsBA315_TIMEOUT_DELAY                (0)
#define fwBA315_TIMEOUT_DELAY                (20)

/* BA315_ECC_CFG */

#define fsBA315_ECC_CFG_ECC_OFFSET           (0)
#define fwBA315_ECC_CFG_ECC_OFFSET           (13)

#define fsBA315_ECC_CFG_ECC_INTER            (13)
#define fwBA315_ECC_CFG_ECC_INTER            (4)

#define fsBA315_ECC_CFG_PACKET_SIZE          (17)
#define fwBA315_ECC_CFG_PACKET_SIZE          (10)

#define fsBA315_ECC_CFG_NB_PACKETS           (27)
#define fwBA315_ECC_CFG_NB_PACKETS           (4)

#define BA315_ECC_CFG_ECC_TYPE               (1 << 31)

/* BA315_IRQ_ENABLE */

#define BA315_IRQ_ENABLE_READY               (1 << 0)
#define BA315_IRQ_ENABLE_OP_COMPLETE         (1 << 1)
#define BA315_IRQ_ENABLE_TIMEOUT             (1 << 2)

/* BA315_IRQ_DISABLE */

#define BA315_IRQ_DISABLE_READY              (1 << 0)
#define BA315_IRQ_DISABLE_OP_COMPLETE        (1 << 1)
#define BA315_IRQ_DISABLE_TIMEOUT            (1 << 2)

/* BA315_IRQ_STATUS */

#define BA315_IRQ_STATUS_READY               (1 << 0)
#define BA315_IRQ_STATUS_OP_COMPLETE         (1 << 1)
#define BA315_IRQ_STATUS_TIMEOUT             (1 << 2)

/* build a mask for the specified bits field */
#define F_MASK(_x) ((u32)(((fw##_x)==32)? ~0U:((1U << (fw##_x))-1)) << (fs##_x))

/* extract the bits field specified by _x from the _d word */
#define F_GET(_d,_x)	  ((u32)(((_d) & F_MASK(_x)) >> (fs ## _x)))

/* set bits field specified by _x to _v */
#define F_VAL(_v,_x)	  ((u32) ((u32)(_v) << (u32)(fs ## _x)) & F_MASK(_x))

/* set the bits field specified by _x to _v within the _d word */
#define F_SET(_v,_d,_x)   ((_d) = ((u32)(_d) & ~F_MASK(_x)) | F_VAL(_v,_x))

#define pr_warning(fmt,arg...) \
	printk(KERN_WARNING fmt,##arg)

#ifdef STAT
#include <mach/hardware.h>
static
void
ba315_start_survey(void)
{
    writeb(1, IO_ADDRESS(VERSATILE_GPIO0_BASE) + (0x1 << 2));
}

static
void
ba315_stop_survey(void)
{
    writeb(0, IO_ADDRESS(VERSATILE_GPIO0_BASE) + (0x1 << 2));
}

static
void
ba315_restart_survey(void)
{
    writeb(0, IO_ADDRESS(VERSATILE_GPIO0_BASE) + (0x1 << 2));
    writeb(1, IO_ADDRESS(VERSATILE_GPIO0_BASE) + (0x1 << 2));
}

static
int
ba315_proc_show(struct seq_file* seq, void* data)
{
    struct ba315_ctrl* const ctrl = seq->private;
    struct ba315_stats          stats;

    spin_lock(&ctrl->read_stats_lock);
    stats = ctrl->read_stats;
    spin_unlock(&ctrl->read_stats_lock);

    if (unlikely(! stats.samples))
        seq_printf(seq, "no read samples\n");
    else
        seq_printf(seq, "read samples: %u aligned: %u%% pages: %u%% size: %u same: %u%% chained: %u%%\n",
                   stats.samples,
                   stats.aligned * 100 / stats.samples,
                   stats.pages * 100 / stats.samples,
                   stats.size,
                   stats.same_page * 100 / stats.samples,
                   stats.chained * 100 / stats.samples);

    spin_lock(&ctrl->write_stats_lock);
    stats = ctrl->write_stats;
    spin_unlock(&ctrl->write_stats_lock);

    if (unlikely(! stats.samples))
        seq_printf(seq, "no write samples\n");
    else
        seq_printf(seq, "write samples: %u aligned: %u%% pages: %u%% size: %u same: %u%% chained: %u%%\n",
                   stats.samples,
                   stats.aligned * 100 / stats.samples,
                   stats.pages * 100 / stats.samples,
                   stats.size,
                   stats.same_page * 100 / stats.samples,
                   stats.chained * 100 / stats.samples);

    return 0;
}

static
int
ba315_proc_open(struct inode* inode, struct file* file)
{
	return single_open(file, ba315_proc_show, PDE(inode)->data);
}

static
struct file_operations const ba315_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ba315_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#else
static void ba315_start_survey(void) {}
static void ba315_stop_survey(void) {}

static void ba315_restart_survey(void) {}
#endif

struct ba315_status_checker {
    int (*is_expected_irq)(u32 status);
    int (*is_expected_status)(u32 status);
};

static inline
struct nand_chip*
bc_nand_chip(struct ba315_ctrl const* ctrl)
{
    return (struct nand_chip*) &ctrl->__base;
}

static inline
struct ba315_ctrl*
bc_ba315_ctrl(struct nand_chip* chip)
{
    return (struct ba315_ctrl*) chip;
}

static inline
u32
ba315_readl(const struct ba315_ctrl* controller, unsigned int reg_off)
{
    return readl(controller->bc_regs + reg_off);
}

static inline
void
ba315_writel(u32 value, struct ba315_ctrl* controller, unsigned int reg_off)
{
    writel(value, controller->bc_regs + reg_off);
    return;
}

/*****************************************************************************
 * interrupts handling related function
 *****************************************************************************/
static inline
int
ba315_is_ready_irq(u32 irq_status)
{
    return irq_status & BA315_IRQ_STATUS_READY;
}

static inline
int
ba315_is_ready_status(u32 flash_status)
{
    return flash_status & BA315_STATUS_READY_CMD;
}

static inline
int
ba315_is_complete_irq(u32 irq_status)
{
    return irq_status & BA315_IRQ_STATUS_OP_COMPLETE;
}

static inline
int
ba315_is_complete_status(u32 flash_status)
{
    return flash_status & BA315_STATUS_RnB;
}


static
const struct ba315_status_checker ready_checker = {
    .is_expected_irq = ba315_is_ready_irq,
    .is_expected_status = ba315_is_ready_status
};

void
setup_ready_checker(struct ba315_ctrl* controller)
{
    controller->status_checker = &ready_checker;
    ba315_writel(BA315_IRQ_ENABLE_READY, controller, BA315_IRQ_ENABLE);
}

static
irqreturn_t
ba315_interrupt(int irq, void* dev_id)
{
    struct ba315_ctrl* const ctrl = bc_ba315_ctrl(((struct mtd_info*) dev_id)->priv);
    u32                      irq_status = ba315_readl(ctrl, BA315_IRQ_STATUS);

    if (unlikely(! irq_status))
        return IRQ_NONE;

    /* acknowledge interrupts */
    ba315_writel(irq_status, ctrl, BA315_IRQ_STATUS);

    ba315_restart_survey();

    if (likely(ctrl->status_checker->is_expected_irq(irq_status)))
        complete(&ctrl->irq_evt);

    return IRQ_HANDLED;
}

/*****************************************************************************
 * controller commands related functions
 *****************************************************************************/

/**
 * ba315_reset -        request controller to reset nand device
 *
 * @controller:         controller
 */
static inline
void
ba315_reset(struct ba315_ctrl* controller)
{
    unsigned long uninitialized_var(delay);

    /* reset controller (blocked if waiting on the RnB pin) */
    ba315_writel(BA315_START_RESET_PULSE, controller, BA315_START);

    /* reset flash device */
    controller->dev.ctrl0 = BA315_CTRL0_CMD1_EN | F_VAL(NAND_CMD_RESET, BA315_CTRL0_CMD1);
    controller->dev.ctrl1 = BA315_CTRL1_WAIT_READY;
    ba315_flush(controller, &setup_ready_checker);

    switch (controller->state) {
    case ba315_sleeping:
    case ba315_reading:
        delay = (unsigned long) controller->settings.toRST;
        break;
    case ba315_writing:
        delay = (unsigned long) controller->settings.twRST;
        break;
    case ba315_erasing:
        delay = (unsigned long) controller->settings.teRST;
        break;
    default:
        BUG();
    }

    /* wait till an interrupt is handled or timeout expired */
    wait_for_completion_timeout(&controller->irq_evt, delay);
}

/**
 * ba315_flush -        request controller to start processing commands
 *
 * @controller:         controller
 * @setup_handler:      function to setup interrupt handler which checks controller
 *                      and nand device status upon completion notification
 */
void
ba315_flush(struct ba315_ctrl* controller, void (*setup_handler)(struct ba315_ctrl*))
{
    struct ba315_device const* dev = &controller->dev;

    /* install interrupt handler if specified */
    if (setup_handler) {
        (*setup_handler)(controller);
        BUG_ON(! controller->status_checker);
    }
    else
        ba315_writel(0x7, controller, BA315_IRQ_DISABLE);
    /* we should never end up with a null handler !! */

    ba315_writel(dev->ctrl0, controller, BA315_CTRL0);
    ba315_writel(dev->ctrl1, controller, BA315_CTRL1);
    ba315_writel(dev->addr_lo, controller, BA315_ADDR_LO);
    ba315_writel(dev->addr_hi, controller, BA315_ADDR_HI);

#if 0
    if (controller->state == ba315_reading  || controller->state == ba315_sleeping) {
        uint32_t ecc = ba315_readl(controller, BA315_ECC_CFG);
        printk("\n");
        pr_info("ctrl: ctrl0=0x%08x ctrl1=0x%08x\n", dev->ctrl0, dev->ctrl1);
        pr_info("address: addr_hi=0x%02x addr_lo=0x%08x\n", dev->addr_hi & 0xff, dev->addr_lo);
        pr_info("ecc: offset=0x%03x inter=0x%01x size=0x%03x nb=0x%01x type=%s\n",
                F_GET(ecc, BA315_ECC_CFG_ECC_OFFSET),
                F_GET(ecc, BA315_ECC_CFG_ECC_INTER),
                F_GET(ecc, BA315_ECC_CFG_PACKET_SIZE),
                F_GET(ecc, BA315_ECC_CFG_NB_PACKETS),
                (ecc & BA315_ECC_CFG_ECC_TYPE) ? "Hamming" : "Reed-Solomon");
        pr_info("commands: cmd1: %02x | cmd2: %02x | addr: %d | ECC: %s | "
                "write: %d | cmd3: %02x | wait: %s | read: %d | ECC: %s\n",
                (dev->ctrl0 & BA315_CTRL0_CMD1_EN) ?  F_GET(dev->ctrl0, BA315_CTRL0_CMD1) : 0xff,
                (dev->ctrl0 & BA315_CTRL0_CMD2_EN) ?  F_GET(dev->ctrl0, BA315_CTRL0_CMD2) : 0xff,
                F_GET(dev->ctrl0, BA315_CTRL0_ADDR_EN),
                ((dev->ctrl0 & BA315_CTRL0_RW) &&
                 (dev->ctrl1 & BA315_CTRL1_ECC_ENABLE)) ? "encode" : "no",
                ((dev->ctrl0 & BA315_CTRL0_RW) && 
                 (dev->ctrl0 & BA315_CTRL0_DATA_EN)) ? F_GET(dev->ctrl1, BA315_CTRL1_NB_DATA) : 0,
                (dev->ctrl0 & BA315_CTRL0_CMD3_EN) ?  F_GET(dev->ctrl0, BA315_CTRL0_CMD3) : 0xff,
                (dev->ctrl1 & BA315_CTRL1_WAIT_READY) ?  "yes" : "no",
                (! (dev->ctrl0 & BA315_CTRL0_RW) && 
                 (dev->ctrl0 & BA315_CTRL0_DATA_EN)) ? F_GET(dev->ctrl1, BA315_CTRL1_NB_DATA) : 0,
                (! (dev->ctrl0 & BA315_CTRL0_RW) &&
                 (dev->ctrl1 & BA315_CTRL1_ECC_ENABLE)) ? "decode" : "no");
    }
#endif

    led_trigger_event(nand_led_trigger, LED_FULL);

    ba315_restart_survey();

    /* launch access to NAND */
    ba315_writel(BA315_START_START_PULSE, controller, BA315_START);
}

/**
 * ba315_wait -         wait controller commands completion
 *
 * @controller:         controller
 */
int
ba315_wait(struct ba315_ctrl* controller)
{
    int             status;
	unsigned long   tmout;
    int             err;


    /* wait till an interrupt is handled or timeout expired */
    tmout = wait_for_completion_timeout(&controller->irq_evt, (unsigned long) HZ);
    ba315_restart_survey();

    /* get the controller/flash status word */
    status = ba315_readl(controller, BA315_STATUS);

	led_trigger_event(nand_led_trigger, LED_OFF);

    memset(&controller->dev, 0, sizeof(controller->dev));

#if 0
    pr_info("status: %03x\n", status);
#endif

    /* 
     * the interrupt we were waiting for never happened : flash(es) and/or the
     * controller are likely to be stalled
     * therefore, soft reset the controller
     * FIXME: we should handle multiple reset retries (and give up after a fixed
     * number of retries)
     * FIXME: account ECC failures and correction (ecc_stats)
     */
    err = -ETIME;
    if (unlikely(! tmout))
        goto reset;

    /* the controller status word is not consistent with the interrupt we had */
    err = -EBUSY;
    if (unlikely(! controller->status_checker->is_expected_status(status)))
        goto reset;

    /* the page we were trying to write to was protected against modification */
    err = -EPERM;
    if (unlikely(status & BA315_STATUS_WRITE_FAULT))
        goto err;
    
    /* 
     * the ECC was not able to correct all errors. rotten data...
     */
    err = -EIO;
    if (unlikely(status & BA315_STATUS_DEC_FAIL)) {
        printk(KERN_INFO "ba315 : BA315_STATUS_DEC_FAIL\n");
        goto err;
	}

    if (status & BA315_STATUS_DEC_ERR) {
        printk(KERN_INFO "ba315 : BA315_STATUS_DEC_ERR\n");
        if (controller->state != ba315_reading)
            goto err;
        controller->corrected++;
	}

    return 0;

reset:
    ba315_reset(controller);

err:
    return err;
}

/**
 * ba315_busy_wait -    request the controller to wait for the nand device to
 *                      be ready to handle next command
 * @controller:         controller
 */
void
ba315_busy_wait(struct ba315_ctrl* controller)
{
    /*
     * keep CE high during busy phase for large page devices only
     */
    controller->dev.ctrl1 |= BA315_CTRL1_WAIT_READY;
    if (bc_nand_chip(controller)->page_shift > 9)
        controller->dev.ctrl1 |= BA315_CTRL1_CE_BUSY;
        
}

/**
 * ba315_setup_read -   send a read command to nand device
 *
 * @controller:         controller
 * @pageno:             request page number
 * @column:             index within the requested page
 *
 * Description:
 * Request controller to send a read command to nand device,
 * which in turn load requested data in its internal buffer
 */
void
ba315_setup_read(struct ba315_ctrl* controller, off_t pageno, off_t column)
{
    off_t const                 psz = (1 << bc_nand_chip(controller)->page_shift);
    struct ba315_device* const  dev = &controller->dev;

    if (psz > 512) {
        dev->ctrl0 = F_VAL(NAND_CMD_READ0, BA315_CTRL0_CMD1) |
            F_VAL(NAND_CMD_READSTART, BA315_CTRL0_CMD3) | BA315_CTRL0_CMD3_EN;
        dev->addr_lo = (pageno << 16) | (column & 0xffff);
        dev->addr_hi = (pageno >> 16) & 0xff;
    }
    else {
        if (column >= psz) {
            column -= psz;
            dev->ctrl0 = F_VAL(NAND_CMD_READOOB, BA315_CTRL0_CMD1);
        }
        else if (column >= 256) {
            column -= 256;
            dev->ctrl0 = F_VAL(NAND_CMD_READ1, BA315_CTRL0_CMD1);
        }
        else
            dev->ctrl0 = F_VAL(NAND_CMD_READ0, BA315_CTRL0_CMD1);

        dev->addr_lo = pageno << 8 | (column & 0xff);
    }

    dev->ctrl0 |= BA315_CTRL0_CMD1_EN | F_VAL(controller->cycles,
                                              BA315_CTRL0_ADDR_EN);
}

static
void
ba315_init_ecc(struct ba315_ctrl* controller, off_t ecc_start, size_t nb_packet)
{
    u32 ecc_ctl = ba315_readl(controller, BA315_ECC_CFG);

    F_SET(ecc_start, ecc_ctl, BA315_ECC_CFG_ECC_OFFSET);
    F_SET(nb_packet, ecc_ctl, BA315_ECC_CFG_NB_PACKETS);
    ba315_writel(ecc_ctl, controller, BA315_ECC_CFG);
}

off_t
ba315_last_eccbyte(struct ba315_ctrl const* controller, int subpage)
{
    int             bytes = bc_nand_chip(controller)->ecc.bytes;
    uint32_t const* pos = bc_nand_chip(controller)->ecc.layout->eccpos;

    return (1 << bc_nand_chip(controller)->page_shift) + pos[((subpage + 1) * bytes) - 1];
}

off_t
ba315_first_eccbyte(struct ba315_ctrl const* controller, int subpage)
{
    int const       bytes = bc_nand_chip(controller)->ecc.bytes;
    uint32_t const* pos = bc_nand_chip(controller)->ecc.layout->eccpos;

    return (1 << bc_nand_chip(controller)->page_shift) + pos[subpage * bytes];
}

off_t
ba315_subpage(off_t column, size_t subpagesize)
{
    return (column / subpagesize);
}

static
off_t
ba315_align_col(off_t column, size_t subpagesize)
{
    return column ? ba315_subpage(column, subpagesize) * subpagesize : 0;
}

/*
 * raw data commands handling
 */
static
size_t
ba315_raw_data_cmd(struct mtd_info const* mtd,
                   struct mtd_oob_ops const* ops,
                   struct ba315_cmd* cmd,
                   size_t data_bytes)
{
    off_t const off = cmd->data_off;
    size_t      data_len;

    BUG_ON(! mtd);
    BUG_ON(! ops);
    BUG_ON(! ops->len);
    BUG_ON(! cmd);
    BUG_ON(! ops->datbuf);
    BUG_ON(off < 0);
    BUG_ON(off > mtd->writesize);

    cmd->oob_len = 0;

    data_len = min_t(size_t, mtd->writesize - off, ops->len - data_bytes);
    cmd->len = cmd->data_len = data_len;

    if (! data_len)
        return 0;

    cmd->data = ops->datbuf + data_bytes;
    cmd->col = off;
    return data_len;
}

/*
 * out of band commands handling
 */
static
size_t
ba315_raw_oob_cmd(struct mtd_info const* mtd,
                struct mtd_oob_ops const* ops,
                struct ba315_cmd* cmd,
                size_t oob_bytes)
{
    off_t const off = ops->ooboffs;
    off_t       oob_off = mtd->writesize;
    size_t      oob_len = ops->ooblen - oob_bytes;

    BUG_ON(! mtd);
    BUG_ON(! ops);
    BUG_ON(! ops->ooblen);
    BUG_ON(! cmd);
    BUG_ON(! ops->oobbuf);
    BUG_ON(off < 0);

    cmd->data_len = 0;

    switch (ops->mode) {
    case MTD_OOB_PLACE:
    case MTD_OOB_RAW:
        BUG_ON(off > mtd->oobsize);

        oob_off += off;
        oob_len = min_t(size_t, mtd->oobsize - off, oob_len);
        break;

    case MTD_OOB_AUTO:
        {
            struct nand_ecclayout const* const  layout =
                ((struct nand_chip*) mtd->priv)->ecc.layout;

            BUG_ON(off > layout->oobavail);

            oob_off += layout->oobfree[0].offset + off;
            oob_len = min_t(size_t, layout->oobavail - off, oob_len);
        }
        break;

    default:
        BUG();
    }

    cmd->len = cmd->oob_len = oob_len;

    if (! oob_len)
        return 0;

    cmd->col = oob_off;
    cmd->oob_off = 0;
    cmd->oob = ops->oobbuf + oob_bytes;
    return oob_len;
}

/*****************************************************************************
 * write handling
 *****************************************************************************/
#ifdef memcpy_toio
#undef memcpy_toio
#endif

static inline
void
memcpy_toio(void __iomem volatile* to, void const* from, size_t count)
{
    memcpy((void __force*) __mem_pci(to), from, count);
}

void
ba315_write_buf(struct ba315_ctrl* controller,
                off_t to,
                void const* from,
                size_t length)
{
    memcpy_toio(controller->bc_mem + to, from, length);
}

#ifdef memset_io
#undef memset_io
#endif

static inline
void
memset_io(void __iomem volatile* to, int c, size_t count)
{
    memset((void __force*) __mem_pci(to), c, count);
}

void
ba315_set_buf(struct ba315_ctrl* controller,
              off_t to,
              int byte,
              size_t count)
{
    memset_io(controller->bc_mem + to, byte, count);
}

static
void
ba315_raw_data_write(struct mtd_info const* mtd,
                     struct mtd_oob_ops const* ops,
                     struct ba315_cmd* cmd,
                     size_t data_written)
{
    if (ba315_raw_data_cmd(mtd, ops, cmd, data_written))
        cmd->type = ba315_raw_write;
    else
        cmd->type = ba315_cmd_end;
}

static
void
ba315_raw_oob_write(struct mtd_info const* mtd,
                    struct mtd_oob_ops const* ops,
                    struct ba315_cmd* cmd,
                    size_t oob_written)
{
    if (ba315_raw_oob_cmd(mtd, ops, cmd, oob_written))
        cmd->type = ba315_raw_write;
    else
        cmd->type = ba315_cmd_end;
}

#define NOTALIGNED(column)	((column & (ssz - 1)) != 0)

static
void
ba315_write_ecc(struct mtd_info const* mtd,
                struct mtd_oob_ops const* ops,
                struct ba315_cmd* cmd,
                size_t data_written,
                size_t oob_written)
{
    size_t const    ssz = ((struct nand_chip*) mtd->priv)->ecc.size;
    off_t const     off = cmd->data_off;
    size_t const    wsz = mtd->writesize;
    size_t const    len = min_t(size_t, wsz - off, ops->len - data_written);
    size_t          oob_off = 0;
    size_t          oob_len = 0;

    BUG_ON(! mtd);
    BUG_ON(! ops);
    /* FIXME: the standard MTD layer returns -EINVAL here */
    BUG_ON(NOTALIGNED(off));
    BUG_ON(NOTALIGNED(ops->len));
    BUG_ON(! cmd);
    BUG_ON(! ops->datbuf);

    if (ops->oobbuf) {
        oob_off = ops->ooboffs;

        switch (ops->mode) {
        case MTD_OOB_PLACE:
            BUG_ON(oob_off > mtd->oobsize);

            oob_off += wsz;
            oob_len = min(mtd->oobsize - oob_off, ops->ooblen - oob_written);
            break;

        case MTD_OOB_AUTO:
            {
                struct nand_ecclayout* const layout =
                    ((struct nand_chip*) mtd->priv)->ecc.layout;

                BUG_ON(off > layout->oobavail);

                oob_off += wsz + layout->oobfree[0].offset;
                oob_len = min_t(size_t, layout->oobavail - off, ops->ooblen - oob_written);
            }
            break;

        default:
            BUG();
        }

        cmd->oob = ops->oobbuf + oob_written;
    }
    else
        cmd->oob = 0;

    if (! (len || oob_len)) {
        cmd->type = ba315_cmd_end;
        return;
    }

    cmd->data_off = 0;
    cmd->data_len = len;
    cmd->data = ops->datbuf + data_written;

    cmd->oob_off = oob_off - off;
    cmd->oob_len = oob_len;

    cmd->col = off;
    cmd->len = max_t(size_t,
                     ba315_last_eccbyte((struct ba315_ctrl*) mtd->priv,
                                        ba315_subpage(off + len - 1, ssz)) + 1,
                     oob_off + oob_len) - off;
    cmd->type = ba315_ecc_write;
}

void
ba315_encode_ecc(struct ba315_ctrl* controller, off_t ecc_start, size_t nb_packet)
{
    struct ba315_device* dev = &controller->dev;

#if 0
    pr_info("%s: start=%ld, nb=%u\n",
            __FUNCTION__,
            ecc_start,
            nb_packet);
#endif

    ba315_init_ecc(controller, ecc_start, nb_packet);
    dev->ctrl0 |= BA315_CTRL0_RW;
    dev->ctrl1 |= BA315_CTRL1_ECC_ENABLE;
}

static
void
ba315_write_cmd(struct mtd_info const* mtd,
               struct mtd_oob_ops const* ops,
               struct ba315_cmd* cmd,
               size_t data_written,
               size_t oob_written)
{
    if (likely((ops->mode != MTD_OOB_RAW) && ops->datbuf))
        return ba315_write_ecc(mtd, ops, cmd, data_written, oob_written);

    if (! ops->datbuf)
        return ba315_raw_oob_write(mtd, ops, cmd, oob_written);

    if (! ops->oobbuf)
        return ba315_raw_data_write(mtd, ops, cmd, data_written);

    ba315_raw_data_write(mtd, ops, cmd, data_written);
    {
        struct ba315_cmd next;

        next.pageno = cmd->pageno;
        ba315_raw_oob_write(mtd, ops, &next, oob_written);

        /*
         * merge both data and oob operations into one command
         */
        cmd->oob = next.oob;
        cmd->oob_off = next.oob_off;
        cmd->oob_len = next.oob_len;
        cmd->len = next.oob_off + next.oob_len - cmd->col;
    }
}

void
ba315_setup_write(struct ba315_ctrl* controller,
                  off_t pageno,
                  off_t column,
                  size_t length)
{
    off_t const                 psz = (1 << bc_nand_chip(controller)->page_shift);
    struct ba315_device* const  dev = &controller->dev;

    dev->ctrl0 = BA315_CTRL0_CMD2_EN | F_VAL(NAND_CMD_SEQIN, BA315_CTRL0_CMD2) |
        BA315_CTRL0_DATA_EN | BA315_CTRL0_RW |
        F_VAL(controller->cycles, BA315_CTRL0_ADDR_EN) |
        BA315_CTRL0_CMD3_EN | F_VAL(NAND_CMD_PAGEPROG, BA315_CTRL0_CMD3);
    dev->ctrl1 = F_VAL(length, BA315_CTRL1_NB_DATA);

    if (psz > 512) {
        dev->addr_lo = (pageno << 16) | ((unsigned int)column & 0xffff);
        dev->addr_hi = (pageno >> 16) & 0xff;
    }
    else {
        if (column >= psz) {
            column -= psz;
            dev->ctrl0 |= BA315_CTRL0_CMD1_EN |
                F_VAL(NAND_CMD_READOOB, BA315_CTRL0_CMD1);
        }
        else if (column >= 256) {
            column -= 256;
            dev->ctrl0 |= BA315_CTRL0_CMD1_EN |
                F_VAL(NAND_CMD_READ1, BA315_CTRL0_CMD1);
        }
        else
            dev->ctrl0 |= BA315_CTRL0_CMD1_EN |
                F_VAL(NAND_CMD_READ0, BA315_CTRL0_CMD1);
        dev->addr_lo = pageno << 8 | (column & 0xff);
    }
}

static
int
ba315_write(struct ba315_ctrl* controller,
            struct ba315_cmd const* curr)
{
    int err;

    BUG_ON(controller->state != ba315_sleeping && controller->state != ba315_writing);
    BUG_ON(curr->type != ba315_raw_write &&
           curr->type != ba315_ecc_write &&
           curr->type != ba315_cmd_end);

    if (controller->state == ba315_writing) {
        err = ba315_wait(controller);
        if (unlikely(err)) {
            controller->state = ba315_sleeping;
            return err;
        }
    }

    if (! (curr->type == ba315_ecc_write || curr->type == ba315_raw_write)) {
        controller->state = ba315_sleeping;
        return 0;
    }

    controller->state = ba315_writing;

#if 0
    if (curr->pageno == 0)
    pr_info("%s: write command: page=%ld, col=%ld, len=%u, data_off=%ld, "
            "data_len=%u, oob_off=%ld, oob_len=%u\n",
            __FUNCTION__,
            curr->pageno,
            curr->col, curr->len, curr->data_off, curr->data_len, curr->oob_off,
            curr->oob_len);
#endif
    if (curr->data_len)
        ba315_write_buf(controller, curr->data_off, curr->data, curr->data_len);

    ba315_setup_write(controller, curr->pageno, curr->col, curr->len);

    {
        size_t const data_end = curr->data_off + curr->data_len;
        size_t const ssz = bc_nand_chip(controller)->ecc.size;

        if (curr->type == ba315_ecc_write) {
            size_t const first = ba315_first_eccbyte(controller, curr->col / ssz) - curr->col;

            ba315_set_buf(controller, data_end, 0xff, first - data_end);
            ba315_encode_ecc(controller,
                             first,
                             ba315_subpage(data_end + ssz - 1, ssz));
        }

        if (curr->oob_len) {
            size_t const last = (curr->type == ba315_ecc_write) ?
                ba315_last_eccbyte(controller, (curr->col + curr->data_len) / ssz) - curr->col :
                data_end;

            if (last)
                ba315_set_buf(controller, last, 0xff, curr->oob_off - last);
            ba315_write_buf(controller, curr->oob_off, curr->oob, curr->oob_len);
        }
    }

    ba315_busy_wait(controller);
    ba315_flush(controller, &setup_ready_checker);

    return 0;
}

#ifdef STAT
static
void
ba315_account(struct ba315_stats* stats,
              size_t length,
              off_t off,
              unsigned short chained,
              unsigned short pagesize,
              unsigned int page)
{
    unsigned int const sample = stats->samples;

    if (length && ! (length % pagesize) && ! off)
        stats->aligned++;
    if ((size_t) (off + length) > (size_t) pagesize)
        stats->pages++;
    stats->size = (sample * stats->size + length) / (sample + 1);
    if (stats->last_page == page)
        stats->same_page++;
    if (chained)
        stats->chained++;
    stats->last_page = page;
    stats->samples = sample + 1;
}
#endif

static
int
_ba315_handle_write(struct kba315_work* curr_work, struct kba315_work* next_work)
{
    int                             err = -EINVAL;
    struct mtd_info*                mtd = curr_work->set->mtd;
    struct ba315_ctrl*              ctrl = mtd->priv;
    struct mtd_oob_ops* const       ops = (struct mtd_oob_ops*) curr_work->ops;

    uint32_t const          wsz = mtd->writesize;   /* page size */
    int const               psft = bc_nand_chip(ctrl)->page_shift;
    int const               pmsk = bc_nand_chip(ctrl)->pagemask;

    size_t const            data_len = ops->datbuf ? ops->len : 0;    /* requested data length */
    size_t const            oob_len = ops->oobbuf ? ops->ooblen : 0;  /* requested data length */
    size_t                  data_written = 0;       /* total data length write so far */
    size_t                  oob_written = 0;        /* total oob length write so far */

    struct ba315_cmd* const cmd = ctrl->cmds;
    unsigned short          curr = ctrl->curr_cmd;
    unsigned short          next = ctrl->next_cmd;

#ifdef STAT
    spin_lock(&ctrl->write_stats_lock);
    ba315_account(&ctrl->write_stats,
                  data_len,
                  curr_work->offset & (wsz - 1),
                  (next_work && curr_work->set->type == next_work->set->type) ? 1 : 0,
                  wsz,
                  (curr_work->offset >> psft) & pmsk);
    spin_unlock(&ctrl->write_stats_lock);

    pr_info("***%s ops***: mode=%d, len=%u, ooblen=%u, ooboffs=%u, datbuf=0x%08x, oobbuf=0x%08x\n",
            __FUNCTION__, ops->mode, ops->len, ops->ooblen, ops->ooboffs,
            (unsigned int) ops->datbuf,
            (unsigned int) ops->oobbuf);
#endif

    if (ctrl->state == ba315_sleeping) {
        /*
         * build and flush the first prepare command
         */
        cmd[curr].pageno = (curr_work->offset >> psft) & pmsk;
        cmd[curr].data_off = curr_work->offset & (wsz - 1);
        ba315_write_cmd(mtd, ops, &cmd[curr], 0, 0);
        ba315_write(ctrl, &cmd[curr]);
    }

    while (data_written < data_len || oob_written < oob_len) {
#if 0
        pr_info("%s data_len=%u, data_written=%u, oob_len=%u, oob_written=%u\n",
                __FUNCTION__, data_len, data_written, oob_len, oob_written);
#endif
        cmd[next].data_off = 0;
        cmd[next].pageno = cmd[curr].pageno + 1;
        ba315_write_cmd(mtd, ops, &cmd[next],
                        data_written + cmd[curr].data_len,
                        oob_written + cmd[curr].oob_len);

        if (cmd[next].type == ba315_cmd_end) {
            if (next_work &&
                curr_work->set->type == next_work->set->type) {
                cmd[next].pageno = (next_work->offset >> psft) & pmsk;
                cmd[next].data_off = next_work->offset & (wsz - 1);

                ba315_write_cmd(mtd, (struct mtd_oob_ops const*)next_work->ops, &cmd[next], 0, 0);
            }
        }

        err = ba315_write(ctrl, &cmd[next]);
        if (err) 
            goto out;

        data_written += cmd[curr].data_len;
        oob_written += cmd[curr].oob_len;

        curr = (curr + 1) % 2;
        next = (next + 1) % 2;
    }

out:
    ctrl->curr_cmd = curr;
    ctrl->next_cmd = next;

    ops->retlen = data_written;
    ops->oobretlen = oob_written;

    if (err)
        return err;

	return 0;
}

#define ba315_handle_write _ba315_handle_write

/*****************************************************************************
 * read handling
 *****************************************************************************/
#ifdef memcpy_fromio
#undef memcpy_fromio
#endif

static inline
void
memcpy_fromio(void* to, void __iomem const volatile* from, size_t count)
{
    memcpy(to, (void __force const*) __mem_pci(from), count);
}

void
ba315_read_buf(void* to,
               struct ba315_ctrl* controller,
               off_t from,
               size_t length)
{
    memcpy_fromio(to, controller->bc_mem + from, length);
}

static
void
ba315_raw_data_read(struct mtd_info const* mtd,
                    struct mtd_oob_ops const* ops,
                    struct ba315_cmd* cmd,
                    size_t data_read)
{
    if (ba315_raw_data_cmd(mtd, ops, cmd, data_read))
        cmd->type = ba315_raw_read;
    else
        cmd->type = ba315_cmd_end;
}

static
void
ba315_raw_oob_read(struct mtd_info const* mtd,
                   struct mtd_oob_ops const* ops,
                   struct ba315_cmd* cmd,
                   size_t oob_read)
{
    if (ba315_raw_oob_cmd(mtd, ops, cmd, oob_read))
        cmd->type = ba315_raw_read;
    else
        cmd->type = ba315_cmd_end;
}

static
void
ba315_read_ecc(struct mtd_info const* mtd,
               struct mtd_oob_ops const* ops,
               struct ba315_cmd* cmd,
               size_t data_read,
               size_t oob_read)
{
    size_t const    ssz = ((struct nand_chip*) mtd->priv)->ecc.size;
    off_t const     off = cmd->data_off;
    size_t const    start_col = ba315_align_col(off, ssz);
    size_t const    wsz = mtd->writesize;
    size_t const    len = min_t(size_t, wsz - off, ops->len - data_read);
    size_t          oob_off = 0;
    size_t          oob_len = 0;

    BUG_ON(! mtd);
    BUG_ON(! ops);
    BUG_ON(! ops->len);
    BUG_ON(! cmd);
    BUG_ON(! cmd->data);

#if 0
    pr_info("%s: read ecc: data_off=%ld, data_len=%u, oob_off=%u, oob_len=%u\n",
            __FUNCTION__,
            off, len, oob_off, oob_len);
#endif

    if (ops->oobbuf) {
        oob_off = ops->ooboffs;

        switch (ops->mode) {
        case MTD_OOB_PLACE:
            BUG_ON(oob_off > mtd->oobsize);

            oob_off += wsz;
            oob_len = min(mtd->oobsize - oob_off, ops->ooblen - oob_read);
            break;

        case MTD_OOB_AUTO:
            {
                struct nand_ecclayout* const layout =
                    ((struct nand_chip*) mtd->priv)->ecc.layout;

                BUG_ON(off > layout->oobavail);

                oob_off += wsz + layout->oobfree[0].offset;
                oob_len = min_t(size_t, layout->oobavail - off, ops->ooblen - oob_read);
            }
            break;

        default:
            BUG();
        }

        cmd->oob = ops->oobbuf + oob_read;
    }
    else
        cmd->oob = 0;

    if (! (len || oob_len)) {
        cmd->type = ba315_cmd_end;
        return;
    }

    cmd->data_off = off - start_col;
    cmd->data_len = len;
    cmd->data = ops->datbuf + data_read;

    cmd->oob_off = oob_off - start_col;
    cmd->oob_len = oob_len;

    cmd->col = start_col;
    cmd->len = max_t(size_t,
                     ba315_last_eccbyte((struct ba315_ctrl*) mtd->priv,
                                        ba315_subpage(off + len - 1, ssz)) + 1,
                     oob_off + oob_len) - start_col;
    cmd->type = ba315_ecc_read;
}

static
void
ba315_read_cmd(struct mtd_info const* mtd,
               struct mtd_oob_ops const* ops,
               struct ba315_cmd* cmd,
               size_t data_read,
               size_t oob_read)
{
    /*
     * FIXME: we could also merge curr and next command if working on the same
     * page
     */

    if (likely((ops->mode != MTD_OOB_RAW) && ops->datbuf))
        return ba315_read_ecc(mtd, ops, cmd, data_read, oob_read);

    if (! ops->datbuf)
        return ba315_raw_oob_read(mtd, ops, cmd, oob_read);

    if (! ops->oobbuf)
        return ba315_raw_data_read(mtd, ops, cmd, data_read);

    ba315_raw_data_read(mtd, ops, cmd, data_read);
    {
        struct ba315_cmd next;

        next.pageno = cmd->pageno;
        ba315_raw_oob_read(mtd, ops, &next, oob_read);
        if (ops->mode == MTD_OOB_RAW)
            next.oob = ops->datbuf + data_read + oob_read + cmd->data_len;

        /*
         * merge both data and oob operations into one command
         */
        cmd->oob = next.oob;
        cmd->oob_off = next.col - cmd->col;
        cmd->oob_len = next.oob_len;
        cmd->len = cmd->oob_off + next.len;
    }
}

/**
 * ba315_do_read -      request controller to get data from nand internal buffer
 *
 * @controller:         controller
 * @length:             size of data to read in bytes
 */
void
ba315_do_read(struct ba315_ctrl* controller, size_t length)
{
    struct ba315_device* dev = &controller->dev;

    F_SET(length, dev->ctrl1, BA315_CTRL1_NB_DATA);
    dev->ctrl0 |= BA315_CTRL0_DATA_EN;
}

void
ba315_decode_ecc(struct ba315_ctrl* controller, off_t ecc_start, size_t nb_packet)
{
    ba315_init_ecc(controller, ecc_start, nb_packet);
    controller->dev.ctrl0 &= ~BA315_CTRL0_RW;
    controller->dev.ctrl1 |= BA315_CTRL1_ECC_ENABLE;
}

static
int
ba315_empty_ecc(struct ba315_ctrl* controller, off_t ecc_start, size_t nb_packet)
{

    int const   ecc_bytes = (int) bc_nand_chip(controller)->ecc.bytes * nb_packet;
    int const   data_bytes = (int) nb_packet * bc_nand_chip(controller)->ecc.size;
    int         cnt;
    int         nok;

    for (cnt = 0; cnt < ecc_bytes; cnt++)
        if (readb(controller->bc_mem + ecc_start + cnt) != 0xff) {
            return -1;
        }

    for (cnt = 0, nok = 3; cnt < data_bytes && nok; cnt += 4) {
        if (readl(controller->bc_mem + cnt) == ~0UL)
            nok--;
        else
            nok = 3;
    }

    if (nok) {
        return -1;
	}

    if (bc_nand_chip(controller)->ecc.bytes == 10) {
        for (cnt = 0; cnt < nb_packet; cnt++) {
            writeb(0xff,
                   controller->bc_mem +
                   (cnt * bc_nand_chip(controller)->ecc.size) +
                   243);
        }
    }

    return 0;
}

static
int
ba315_read(struct ba315_ctrl* controller,
           struct ba315_cmd const* curr,
           struct ba315_cmd const* next)
{
    int err;
    int unc_error = 0;

    BUG_ON(controller->state != ba315_sleeping && controller->state != ba315_reading);
#if 0
    BUG_ON(curr->type != ba315_raw_read &&
           curr->type != ba315_ecc_read &&
           curr->type != ba315_cmd_end);
#endif

    if (controller->state != ba315_sleeping) {
        err = ba315_wait(controller);
        if (unlikely(err)) {
            size_t const    ssz = bc_nand_chip(controller)->ecc.size;

            if (err != -EIO) {
                controller->state = ba315_sleeping;
                return err;
            }

            err = ba315_empty_ecc(controller,
                            ba315_first_eccbyte(controller,
                                                curr->col / ssz) -
                            curr->col,
                            ba315_subpage(curr->data_off +
                                          curr->data_len +
                                          ssz - 1,
                                          ssz));
            if (err)
                unc_error = -EBADMSG;
        }

        if (curr->data_len)
            ba315_read_buf(curr->data, controller, curr->data_off, curr->data_len);
        if (curr->oob_len)
            ba315_read_buf(curr->oob, controller, curr->oob_off, curr->oob_len);
        ba315_restart_survey();
    }

    if (! (next->type == ba315_ecc_read || next->type == ba315_raw_read)) {
        controller->state = ba315_sleeping;
        return unc_error;
    }

#if 0
    if (next->pageno == 0)
    pr_info("%s: read command: page=%ld, col=%ld, len=%u, data_off=%ld, "
            "data_len=%u, oob_off=%ld, oob_len=%u\n",
            __FUNCTION__,
            next->pageno,
            next->col, next->len, next->data_off, next->data_len, next->oob_off,
            next->oob_len);
#endif

    controller->state = ba315_reading;

    ba315_setup_read(controller, next->pageno, next->col);
    ba315_busy_wait(controller);
    ba315_do_read(controller, next->len);
    if (next->type == ba315_ecc_read) {
        size_t const    ssz = bc_nand_chip(controller)->ecc.size;

        ba315_decode_ecc(controller,
                         ba315_first_eccbyte(controller, next->col / ssz) - next->col,
                         ba315_subpage(next->data_off + next->data_len + ssz - 1, ssz));
    }
    ba315_flush(controller, &setup_ready_checker);

    return unc_error;
}


static
int
_ba315_handle_read(struct kba315_work* curr_work, struct kba315_work* next_work)
{
    int                             err = -EINVAL;
    struct mtd_info*                mtd = curr_work->set->mtd;
    struct ba315_ctrl*              ctrl = mtd->priv;
    struct mtd_oob_ops* const       ops = (struct mtd_oob_ops*) curr_work->ops;

    uint32_t const          wsz = mtd->writesize;   /* page size */
    int const               psft = bc_nand_chip(ctrl)->page_shift;
    int const               pmsk = bc_nand_chip(ctrl)->pagemask;

    size_t const            data_len = ops->datbuf ? ops->len : 0;    /* requested data length */
    size_t const            oob_len = ops->oobbuf ? ops->ooblen : 0;  /* requested data length */
    size_t                  data_read = 0;          /* total data length read so far */
    size_t                  oob_read = 0;           /* total oob length read so far */

    struct ba315_cmd* const cmd = ctrl->cmds;
    unsigned short          curr = ctrl->curr_cmd;
    unsigned short          next = ctrl->next_cmd;

    ctrl->corrected = 0;

#ifdef STAT
    spin_lock(&ctrl->read_stats_lock);
    ba315_account(&ctrl->read_stats,
                  data_len,
                  curr_work->offset & (wsz - 1),
                  (next_work && curr_work->set->type == next_work->set->type) ? 1 : 0,
                  wsz,
                  (curr_work->offset >> psft) & pmsk);
    spin_unlock(&ctrl->read_stats_lock);

    if (next_work)
        pr_info("next: yes\n");

    pr_info("***%s ops***: state=%d, mode=%d, len=%u, ooblen=%u, ooboffs=%u, datbuf=0x%08x, oobbuf=0x%08x\n",
            __FUNCTION__, ctrl->state, ops->mode, ops->len, ops->ooblen, ops->ooboffs,
            (unsigned int) ops->datbuf,
            (unsigned int) ops->oobbuf);
#endif

    if (ctrl->state == ba315_sleeping) {
        /*
         * build and flush the first prepare command
         */
        cmd[curr].pageno = (curr_work->offset >> psft) & pmsk;
        cmd[curr].data_off = curr_work->offset & (wsz - 1);
        ba315_read_cmd(mtd, ops, &cmd[curr], 0, 0);

        ba315_read(ctrl, 0, &cmd[curr]);
    }

    while (data_read < data_len || oob_read < oob_len) {
#if 0
        pr_info("%s data_len=%u, data_read=%u, oob_len=%u, oob_read=%u\n",
                __FUNCTION__, data_len, data_read, oob_len, oob_read);
#endif
        cmd[next].data_off = 0;
        cmd[next].pageno = cmd[curr].pageno + 1;
        ba315_read_cmd(mtd, ops, &cmd[next],
                       data_read + cmd[curr].data_len,
                       oob_read + cmd[curr].oob_len);

        if (cmd[next].type == ba315_cmd_end) {
            if (next_work &&
                curr_work->set->type == next_work->set->type) {
                cmd[next].pageno = (next_work->offset >> psft) & pmsk;
                cmd[next].data_off = next_work->offset & (wsz - 1);

                ba315_read_cmd(mtd,
                               (struct mtd_oob_ops const*) next_work->ops,
                               &cmd[next],
                               0,
                               0);
            }
        }

        err = ba315_read(ctrl, &cmd[curr], &cmd[next]);
        if (err) 
            goto out;

        data_read += cmd[curr].data_len;
        oob_read += cmd[curr].oob_len;

        curr = (curr + 1) % 2;
        next = (next + 1) % 2;
    }

out:
    ctrl->curr_cmd = curr;
    ctrl->next_cmd = next;

    ops->retlen = data_read;
    ops->oobretlen = oob_read;

    if (ctrl->corrected) {
        printk(KERN_INFO"detected ecc error num=%d, ret=%d\n", ctrl->corrected, err);
        mtd->ecc_stats.corrected += ctrl->corrected;
        if (! err)
            err = -EUCLEAN;
    }

    if (err == -EBADMSG)
        mtd->ecc_stats.failed++;

    return err;
}

static
int
ba315_handle_read(struct kba315_work* curr_work, struct kba315_work* next_work)
{
    int ret;

    ba315_start_survey();
    ret = _ba315_handle_read(curr_work, next_work);
    ba315_stop_survey();

    return ret;
}

/*****************************************************************************
 * erase handling
 *****************************************************************************/
static
void
ba315_setup_erase(struct ba315_ctrl* controller, off_t pageno)
{
    struct ba315_device* const  dev = &controller->dev;

    dev->ctrl0 = BA315_CTRL0_CMD1_EN | F_VAL(NAND_CMD_ERASE1, BA315_CTRL0_CMD1) |
        BA315_CTRL0_CMD3_EN | F_VAL(NAND_CMD_ERASE2, BA315_CTRL0_CMD3);

    if ((1 << bc_nand_chip(controller)->page_shift) > 512)
        dev->ctrl0 |= F_VAL(controller->cycles - 2, BA315_CTRL0_ADDR_EN);
    else
        dev->ctrl0 |= F_VAL(controller->cycles - 1, BA315_CTRL0_ADDR_EN);

    dev->addr_lo = pageno;
}

/*
 * see nand_block_checkbad
 */
static
int
ba315_block_checkbad(struct mtd_info* mtd, loff_t page, int allowbbt)
{
    struct ba315_ctrl* const    ctrl = mtd->priv;

#if 0
    //return 0;

#else
    /* FIXME: remove #if */
	/* Return info from the table if available */
	if (bc_nand_chip(ctrl)->bbt)
        return nand_isbad_bbt(mtd, page << bc_nand_chip(ctrl)->page_shift, allowbbt);
#endif

    ba315_setup_read(ctrl, page, mtd->writesize +
                     bc_nand_chip(ctrl)->badblockpos);
    ba315_do_read(ctrl, 1);
    ba315_busy_wait(ctrl);
    ba315_flush(ctrl, &setup_ready_checker);

    if (ba315_wait(ctrl))
        return 1;

    if (readb(ctrl->bc_mem) != 0xff)
        return 1;

    return 0;
}

#define BBT_PAGE_MASK	0xffffff3f

static
int
ba315_handle_erase(struct kba315_work* curr_work, struct kba315_work* next_work)
{
    struct mtd_info* const      mtd = curr_work->set->mtd;
    struct ba315_ctrl* const    ctrl = mtd->priv;
    struct erase_info* const    ops = (struct erase_info*) curr_work->ops;
    int const                   psft = bc_nand_chip(ctrl)->page_shift;
    int const                   esft = bc_nand_chip(ctrl)->phys_erase_shift;
    loff_t                      pageno = ops->addr >> psft;
    size_t                      len = ops->len;
    loff_t                      rewrite_bbt = 0;
    unsigned int                bbt_masked_page = ~0UL;
	int const                   pages_per_block = 1 << (esft - psft);
    int                         ret;

    BUG_ON(ctrl->state != ba315_sleeping);

#if 0
    pr_info("ba315_handle_erase\n");
#endif

	if (bc_nand_chip(ctrl)->options & BBT_AUTO_REFRESH && ! curr_work->erase_bbt)
		bbt_masked_page = bc_nand_chip(ctrl)->bbt_td->pages[0] & BBT_PAGE_MASK;

    ops->fail_addr = ~0UL;
    ops->state = MTD_ERASING;
    ctrl->state = ba315_erasing;

    while (len) {
        /*
		 * heck if we have a bad block, we do not erase bad blocks !
		 */

		if (ba315_block_checkbad(mtd, pageno, curr_work->erase_bbt)) {
			printk(KERN_WARNING "nand_erase: attempt to erase a "
			       "bad block at page %lld\n", pageno);
			ops->state = MTD_ERASE_FAILED;
			goto out;
		}

        ba315_setup_erase(ctrl, pageno);
        ba315_busy_wait(ctrl);
        ba315_flush(ctrl, &setup_ready_checker);

        if (ba315_wait(ctrl)) {
            ops->state = MTD_ERASE_FAILED;
            /* save failing address */
            ops->fail_addr = pageno << psft;
            goto out;
        }

		if (bbt_masked_page != 0xffffffff &&
		    (pageno & BBT_PAGE_MASK) == bbt_masked_page)
			    rewrite_bbt = (pageno << psft);

		len -= (1 << esft);
		pageno += pages_per_block;
    }

    ops->state = MTD_ERASE_DONE;

out:
    ctrl->state = ba315_sleeping;
	ret = (ops->state == MTD_ERASE_DONE) ? 0 : -EIO;

    if(! ret)
        mtd_erase_callback(ops);

	if (bbt_masked_page == 0xffffffff)
        return ret;

    if (rewrite_bbt)
        nand_update_bbt(mtd, rewrite_bbt);

    return ret;
}

static
void
ba315_bug(void)
{
    BUG();
}

static
kba315_op_t* const
ba315_ctrl_ops[] = {
    [read_op]  = (void*) &ba315_handle_read,
    [write_op] = (void*) &ba315_handle_write,
    [erase_op] = (void*) &ba315_handle_erase
};

static
void
ba315_select_chip(struct mtd_info* mtd, int chipnr)
{
    return;
}

static
int
ba315_block_bad(struct mtd_info* mtd, loff_t offset, int getchip)
{
#if 1
	struct nand_chip*   chip = mtd->priv;
    uint8_t             buf = 0;
    struct mtd_oob_ops  ops = {
        .mode = MTD_OOB_RAW,
        .len = 0,
        .ooblen = 1,
        .ooboffs = chip->badblockpos,
        .datbuf = 0,
        .oobbuf = &buf
    };

    ba315_mtd_read_oob(mtd, offset, &ops);

    return (buf != 0xff);
#else
    return 0;
#endif
}

static
int
ba315_block_markbad(struct mtd_info* mtd, loff_t offset)
{
	struct nand_chip*   chip = mtd->priv;
	int                 block, ret;

	/* Get block number */
	block = ((int) offset) >> chip->bbt_erase_shift;
	if (chip->bbt)
		chip->bbt[block >> 2] |= 0x01 << ((block & 0x03) << 1);

	/* Do we have a flash based bad block table ? */
	if (chip->options & NAND_USE_FLASH_BBT)
		ret = nand_update_bbt(mtd, offset);
	else {
        uint8_t             buf = 0;
        struct mtd_oob_ops  ops = {
            .mode = MTD_OOB_RAW,
            .len = 0,
            .ooblen = 1,
            .ooboffs = chip->badblockpos,
            .datbuf = 0,
            .oobbuf = &buf
        };

        ret = ba315_mtd_write_oob(mtd, offset, &ops);
	}

	if (! ret)
		mtd->ecc_stats.badblocks++;

	return ret;
}

/*****************************************************************************
 * init and exit related functions
 *****************************************************************************/

static
int __devinit
ba315_identify_flash(struct ba315_ctrl* controller, char* id_bytes, size_t size)
{
    /*
     * adding wait ready cycles does generate no interrupts at the end of
     * read cycles.
     * i found no infos about wether or not the flash is busy during read id
     * command
     */
    ba315_writel(BA315_CTRL0_CMD1_EN |
                 F_VAL(NAND_CMD_READID, BA315_CTRL0_CMD1) |
                 F_VAL(1, BA315_CTRL0_ADDR_EN) |
                 BA315_CTRL0_DATA_EN,
                 controller, BA315_CTRL0);
    ba315_writel(F_VAL(size, BA315_CTRL1_NB_DATA),
                 controller, BA315_CTRL1);
    ba315_writel(0, controller, BA315_ADDR_LO);
    ba315_writel(0, controller, BA315_ADDR_HI);

    /*
     * flush commands
     */
    setup_ready_checker(controller);
    ba315_writel(BA315_START_START_PULSE, controller, BA315_START);

    /* 
     * wait for command ready do work as well
     * but for the sake of reliability, ensure operation is completed
     */
    if (ba315_wait(controller))
        return -1;

    memcpy_fromio(id_bytes, controller->bc_mem, size);
    return 0;
}

static
int __devinit
ba315_scan_nand(struct mtd_info* mtd, struct nand_chip* chip)
{
	struct nand_flash_dev*  type;
    char                    id_bytes[4];
    int                     t;

    /* get flash id bytes */
    if (ba315_identify_flash(bc_ba315_ctrl(chip), id_bytes, sizeof(id_bytes)))
        return -ENODEV;

	/* Lookup the flash id */
	for (t = 0, type = 0; nand_flash_ids[t].name; t++) {
		/* The 2nd id byte holds device id */
		if (id_bytes[1] == nand_flash_ids[t].id) {
			type = &nand_flash_ids[t];
			break;
		}
	}
	if (! type)
		return -ENODEV;

    /*
     * Newer devices have all the information in additional id bytes
     */
	if (! type->pagesize) {
		/* The 3rd id byte holds MLC / multichip data */
		chip->cellinfo = id_bytes[2];

		/*
         * The 4th id byte is the important one
         */

		/* Calc pagesize */
		mtd->writesize = 1024 << (id_bytes[3] & 0x3);

		/* Calc oobsize */
		id_bytes[3] >>= 2;
		mtd->oobsize = (8 << (id_bytes[3] & 0x01)) * (mtd->writesize >> 9);

		/* Calc blocksize. Blocksize is multiples of 64KiB */
		id_bytes[3] >>= 2;
		mtd->erasesize = (64 * 1024) << (id_bytes[3] & 0x03);

		/* Get buswidth information */
		id_bytes[3] >>= 2;
		chip->options |= ((id_bytes[3] & 0x01) ? NAND_BUSWIDTH_16 : 0);
	}
    /*
     * Old devices have chip data hardcoded in the device id table
     */
    else {
		mtd->erasesize = type->erasesize;
		mtd->writesize = type->pagesize;
		mtd->oobsize = mtd->writesize / 32;
	}

    mtd->name = "nand0";
	mtd->size = chip->chipsize = type->chipsize << 20;
    chip->numchips = 1;
	chip->page_shift = ffs(mtd->writesize) - 1;
	chip->pagemask = (chip->chipsize >> chip->page_shift) - 1;
	chip->bbt_erase_shift = chip->phys_erase_shift = ffs(mtd->erasesize) - 1;
	chip->chip_shift = ffs(chip->chipsize) - 1;

	/* Set the bad block position */
	chip->badblockpos = (mtd->writesize > 512) ?
		NAND_LARGE_BADBLOCK_POS : NAND_SMALL_BADBLOCK_POS;

	/* Get chip options, preserve non chip based options */
	chip->options &= ~NAND_CHIPOPTIONS_MSK;
	chip->options |= type->options & NAND_CHIPOPTIONS_MSK;

	/* Set chip as a default. Board drivers can override it, if necessary */
	chip->options |= NAND_NO_AUTOINCR;

	/* Check if chip is a not a samsung device. Do not clear the
	 * options for chips which are not having an extended id.
	 */
	if (id_bytes[0] != NAND_MFR_SAMSUNG &&
        ! type->pagesize)
		chip->options &= ~NAND_SAMSUNG_LP_OPTIONS;

	/* Try to identify manufacturer : first id byte holds manufacturer id */
	for (t = 0; nand_manuf_ids[t].id; t++)
		if (nand_manuf_ids[t].id == id_bytes[0])
			break;

	pr_info("NAND device: Manufacturer ID: 0x%02x, Chip ID: 0x%02x (%s %s)\n",
            nand_manuf_ids[t].id,
            type->id,
            nand_manuf_ids[t].name,
            type->name);

	return 0;
}

static
struct clk* ba315_clk;

static
int __devinit
ba315_init_clock(void)
{
    int ret;

    ba315_clk = clk_get(NULL, "nand");
	if (IS_ERR(ba315_clk)) {
        ret = PTR_ERR(ba315_clk);
        goto out;
    }

    ret = clk_enable(ba315_clk);
    if (unlikely(ret))
        goto put;

    return 0;

put:
    clk_put(ba315_clk);
out:
    return ret;
}

static
void
ba315_fini_clock(void)
{
    clk_disable(ba315_clk);
    clk_put(ba315_clk);
}


static
int
ba315_safe_timing(int timing)
{
    return timing;
}

static
unsigned char
ba315_ns2t(int clk_t, int timing)
{
    /* controller automatically adds one clock cycle */
    return timing / clk_t;
}

static
void __devinit
ba315_init(struct mtd_info* mtd, struct onfi_setting* setting)
{
    struct ba315_ctrl*          controller = mtd->priv;
    struct nand_ecc_ctrl const* ecc = &bc_nand_chip(controller)->ecc;

    /* 
     * Review all BA315 registers and set them up with regard to parameters
     * configured at nand probing time
     */
    ba315_writel(0x7, controller, BA315_IRQ_STATUS);

    if (! setting) {
        ba315_writel(0x7fffffff, controller, BA315_TIM0);
        ba315_writel(0xfff, controller, BA315_TIM1);
        controller->settings.toRST = usecs_to_jiffies(1000);
        controller->settings.twRST = usecs_to_jiffies(1000);
        controller->settings.teRST = usecs_to_jiffies(1000);
    }
    else {
        /*
         * For timing requirements, see http://www.onfi.org/docs/ONFI_1_0_Gold.pdf
         * and controller documentation (table 14 and 44)
         */

        struct onfi_timings*    timings = &setting->timings;
        unsigned int            tim0 = 0, tim1 = 0;
        int                     tmp, twp, trp, trsetup;
        int                     hclk_t = NSEC_PER_SEC / clk_get_rate(ba315_clk);

        twp = max(ba315_safe_timing(timings->tWP),
                  ba315_safe_timing(timings->tDS));
        twp = max(10, twp);
        twp = min(50, twp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, twp), BA315_TIM0_TWP);

        tmp = max(ba315_safe_timing(timings->tCLH),
                  ba315_safe_timing(timings->tCH));
        tmp = max(tmp, ba315_safe_timing(timings->tALH));
        tmp = max(tmp, ba315_safe_timing(timings->tDH));
        tmp = max(tmp, ba315_safe_timing(timings->tWH));
        tmp = max(tmp, ba315_safe_timing(timings->tWC) - twp);
        tmp = max(10, tmp);
        tmp = min(50, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM0_TWH);

        trp = max(ba315_safe_timing(timings->tRP),
                  ba315_safe_timing(timings->tREA));
        trp = max(16, trp);
        trp = min(50, trp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, trp), BA315_TIM0_TRP);

        tmp = max(ba315_safe_timing(timings->tREH),
                  ba315_safe_timing(timings->tRC) - trp);
        tmp = max(10, tmp);
        tmp = min(50, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM0_TREH);

        tmp = max(100, ba315_safe_timing(timings->tRHZ));
        tmp = min(200, tmp);
        tim1 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM1_BTA);

        tmp = max(ba315_safe_timing(timings->tCLS),
                  ba315_safe_timing(timings->tCS));
        tmp = max(tmp, ba315_safe_timing(timings->tALS)) - twp;
        tmp = max(5, tmp);
        tmp = min(60, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM0_TWSETUP);

        trsetup = max(ba315_safe_timing(timings->tCEA) -
                      ba315_safe_timing(timings->tREA),
                      ba315_safe_timing(timings->tCLR));
        trsetup = max(9, tmp);
        trsetup = min(60, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, trsetup), BA315_TIM0_TRSETUP);

        tmp = ba315_safe_timing(timings->tRR) - trsetup;
        tmp = max(tmp, 0);
        tmp = min(10, tmp);
        tim1 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM1_TBUSY);

        tmp = ba315_safe_timing(timings->tWHR) - trsetup;
        tmp = max(tmp, 20);
        tmp = min(70, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM0_TWHR);

        tmp = ba315_safe_timing(timings->tWHR) - trsetup;
        tmp = max(tmp, 20);
        tmp = min(110, tmp);
        tim0 |= F_VAL(ba315_ns2t(hclk_t, tmp), BA315_TIM0_TWHR);

        tim0 |= F_VAL(ba315_ns2t(hclk_t, ba315_safe_timing(timings->tCEH)), BA315_TIM0_TCEH);

        controller->settings.toRST = usecs_to_jiffies(timings->toRST);
        controller->settings.twRST = usecs_to_jiffies(timings->twRST);
        controller->settings.teRST = usecs_to_jiffies(timings->teRST);

        if (setting->edo)
            tim0 |= BA315_TIM0_EDO;

        ba315_writel(tim0, controller, BA315_TIM0);
        ba315_writel(tim1, controller, BA315_TIM1);
    }

    ba315_writel(0, controller, BA315_CTRL0);
    ba315_writel(0, controller, BA315_CTRL1);
    ba315_writel(0, controller, BA315_ADDR_LO);
    ba315_writel(0, controller, BA315_TIMEOUT);
    ba315_writel(((bc_nand_chip(controller)->options & NAND_BUSWIDTH_16) ?
                 BA315_CFG_MODE_8_16 : 0) |
                 F_VAL(0x3, BA315_CFG_DEC_CLK_DIV) /*|
                 BA315_CFG_CE_INTERCEPT*/,
                 controller, BA315_CFG);

    /*
     * ECC configuration handling
     */
    if (ecc->mode == NAND_ECC_HW) {
        /*
         * Hamming support : 3 bytes / ECC step
         * Reed-Solomon : 10
         * Any other value is a programming error
         */
        BUG_ON(ecc->bytes != 3 && ecc->bytes != 10);

        /* no support for ECC intervals */
        ba315_writel(F_VAL(ecc->size, BA315_ECC_CFG_PACKET_SIZE) |
                     ((ecc->bytes == 3) ?
                     BA315_ECC_CFG_ECC_TYPE :   /* hamming */
                     0),                         /* reed-solomon */
                     controller, BA315_ECC_CFG);
    }
    ba315_writel(0x7, controller, BA315_IRQ_DISABLE);
    ba315_writel(0, controller, BA315_ADDR_HI);

    return;
}

static
struct nand_ecclayout ba315_hamming_oob512 = {
    .eccbytes = 3,
    .eccpos = { 0, 1, 2 },
    .oobfree = {
        {
            .offset = 6,
            .length = 10
        }
    }
};

static
struct nand_ecclayout ba315_hamming_oob2048 = {
    .eccbytes = 12,
    .eccpos = {
        2,  3,  4,
        5,  6,  7,
        8,  9,  10,
        11, 12, 13
    },
    .oobfree = {
        {
            .offset = 14,
            .length = 50
        }
    }
};

static
struct nand_ecclayout ba315_rs_oob2048 = {
    .eccbytes = 40,
    .eccpos = {
         2,  3,  4,  5,  6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
        22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
        32, 33, 34, 35, 36, 37, 38, 39, 40, 41
    },
    .oobfree = {
        {
            .offset = 42,
            .length = 24
        }
    }
};

#ifdef CONFIG_MTD_PARTITIONS
static
int __devinit
ba315_add_mtd(struct platform_device* device, struct mtd_info* mtd)
{
    char const*             probes[] =  {
                                            "parrotpart",
#ifdef CONFIG_MTD_CMDLINE_PARTS
                                            "cmdlinepart",
#endif
                                            NULL
                                        };
    struct mtd_partition*   parts;
    int                     nr = 0;

    nr = parse_mtd_partitions(mtd, probes, &parts, 0);
    if (nr > 0)
        return add_mtd_partitions(mtd, parts, nr);
    else
        return add_mtd_device(mtd);
}
#else
static
int __devinit
ba315_add_mtd(struct mtd_info* mtd)
{
    return add_mtd_device(mtd);
}
#endif

static
int
ba315_read_onfi(char* buffer,
                struct mtd_info* mtd,
                int command,
                int address,
                size_t size)
{
    int                         ret;
    struct ba315_ctrl* const    ctrl = mtd->priv;

    if (command != ONFI_NOOP_CMD)
        ba315_writel(BA315_CTRL0_CMD1_EN |
                     F_VAL(command, BA315_CTRL0_CMD1) |
                     F_VAL(1, BA315_CTRL0_ADDR_EN) |
                     BA315_CTRL0_DATA_EN,
                     ctrl, BA315_CTRL0);
    else
        ba315_writel(BA315_CTRL0_DATA_EN, ctrl, BA315_CTRL0);

    ba315_writel(((command == ONFI_READPARM_CMD) ? BA315_CTRL1_WAIT_READY : 0) |
                 F_VAL(size, BA315_CTRL1_NB_DATA),
                 ctrl, BA315_CTRL1);
    ba315_writel((u32) address, ctrl, BA315_ADDR_LO);
    ba315_writel(0, ctrl, BA315_ADDR_HI);

    /*
     * flush commands
     */
    setup_ready_checker(ctrl);
    ba315_writel(BA315_START_START_PULSE, ctrl, BA315_START);

    /* 
     * wait for command ready do work as well
     * but for the sake of reliability, ensure operation is completed
     */
    ret = ba315_wait(ctrl);
    if (ret)
        return ret;

    memcpy_fromio(buffer, ctrl->bc_mem, size);
    return 0;
}

static
struct onfi_operations const ba315_onfi_ops = {
    .read_mtd = ba315_read_onfi,
    .init_mtd = ba315_init
};

/*
 * No need to call either request_region or request_mem_region to ensure proper
 * resources reservation since this will already be performed by the
 * platform_device_register framework
 */
int __devinit
ba315_probe_ctrl(struct platform_device* pdev)
{
    struct mtd_info*    mtd;
    struct ba315_ctrl*  ctrl;
    struct resource*    res;
    char*               msg;
    int                 ret = -EINVAL;

    dev_dbg(&pdev->dev, "probe start\n");

    msg = "Invalid device resources descriptor";
    if (unlikely(pdev->num_resources != 3))
        goto out;

    /*
     * allocate controller structure
     */
    ret = -ENOMEM;
    msg = "failed to allocate controller structure";
    mtd = kzalloc(sizeof(*mtd) + sizeof(*ctrl), GFP_KERNEL);
    if (! mtd)
        goto out;

    /* the controller structure is allocated just behind its related mtd */
    ctrl = (struct ba315_ctrl*) &mtd[1];
    ctrl->state = ba315_sleeping;
    memset(&ctrl->dev, 0, sizeof(ctrl->dev));
    ctrl->curr_cmd = 0;
    ctrl->next_cmd = 1;
#ifdef STAT
    spin_lock_init(&ctrl->read_stats_lock);
    ctrl->read_stats.last_page = -1;
    spin_lock_init(&ctrl->write_stats_lock);
    ctrl->write_stats.last_page = -1;
#endif
    init_completion(&ctrl->irq_evt);

    mtd->priv = ctrl;
	mtd->owner = THIS_MODULE;
    platform_set_drvdata(pdev, mtd);

    /*
     * mapping registers
     */
    ret = -ENXIO;
    msg = "no I/O resource descriptor found";
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (unlikely(! res))
        goto free;

    ret = -ENOMEM;
    msg = "failed to remap I/O resource";
    ctrl->bc_regs = ioremap(res->start, res->end - res->start + 1);
    if (unlikely(! ctrl->bc_regs))
        goto free;

    /*
     * mapping memory
     */
    ret = -ENXIO;
    msg = "no memory resource descriptor found";
    res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
    if (unlikely(! res))
        goto unmap_regs;

    ret = -ENOMEM;
    msg = "failed to remap memory resource";
    ctrl->bc_mem = ioremap(res->start, res->end - res->start + 1);
    if (unlikely(! ctrl->bc_mem))
        goto unmap_regs;

    msg = "failed to init clock";
    ret = ba315_init_clock();
    if (unlikely(ret))
        goto unmap_mem;

    /*
     * install the interrupts handler
     */
    ret = -ENXIO;
    msg = "no interrupt resource descriptor found";
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (unlikely(! res))
        goto fini_clock;

	/* clear irq before unmasking irq */
    ba315_init(mtd, NULL);

    msg = "failed to reserve interrupt line";
    ret = request_irq(res->start, &ba315_interrupt, 0,
                      "BA315 NAND controller", mtd);
    if (unlikely(ret))
        goto fini_clock;

    /*
     * bootstrap controller with fail safe configuration then,
     * auto detect underlying flash type
     */
    ba315_reset(ctrl);
    msg = "failed to detect NAND flash type";
    ret = ba315_scan_nand(mtd, bc_nand_chip(ctrl));
    if (ret)
        goto free_irq;

    /*
     * mandatory nand_chip structure setup befor calling
     * nand_scan_tail NAND flash to finalize MTD initialization
     */

    {
        /*
         * ECC parms setup
         */
        struct nand_ecc_ctrl*   ecc = &bc_nand_chip(ctrl)->ecc;

        /*
         * Hamming computes ECC on a fixed 512 bytes large page basis ;
         * therefore, it is not suitable for 256 bytes pages.
         * Reed-Solomon needs 10 bytes per data ECC step : unsuitable for small
         * oob area.
         * We should then compute ECC bytes in a software manner.
         * Unsupported for instance, hence the BUG_ON.
         */
        msg = "invalid writesize or oobsize";
        if ((mtd->writesize < 512) || (mtd->oobsize < 16)) {
            ret = -ENXIO;
            goto free_irq;
        }

	    if (bc_nand_chip(ctrl)->cellinfo & NAND_CI_CELLTYPE_MSK)
            /*
             * MLC NAND flashes require Reed-Solomon
             * for proper data reliability
             */
            ecc->bytes = 10;
        else
            /*
             * SLC NAND flashes do not require
             * Reed-Solomon (overhead ?)
             */
            ecc->bytes = 3;

		switch (mtd->writesize) {
		case 512:
            ecc->layout = &ba315_hamming_oob512;
            break;

		case 2048:
            if (ecc->bytes == 3)
                ecc->layout = &ba315_hamming_oob2048;
            else
                ecc->layout = &ba315_rs_oob2048;
			break;

		default:
            msg = "unsupported writesize";
            ret = -ENXIO;
            goto free_irq;
		}

        ecc->mode = NAND_ECC_HW;
        ecc->size = 512;

        ecc->calculate = (void*) &ba315_bug;
        ecc->correct = (void*) &ba315_bug;
        ecc->hwctl = (void*) &ba315_bug;
        ecc->read_page_raw = (void*) &ba315_bug;
        ecc->write_page_raw = (void*) ba315_bug;
        ecc->read_page = (void*) &ba315_bug;
        ecc->write_page = (void*) &ba315_bug;
        ecc->read_oob = (void*) &ba315_bug;
        ecc->write_oob = (void*) &ba315_bug;
    }

    /* compute column cycles */
    ctrl->cycles = (bc_nand_chip(ctrl)->page_shift > 9) ? 2 : 1;
    /* and add row cycles */
    ctrl->cycles += 
        (((bc_nand_chip(ctrl)->chip_shift -
          bc_nand_chip(ctrl)->page_shift) > 16) ?  3 : 2);

    /* FIXME: nand_get_device is needed ? */
    spin_lock_init(&bc_nand_chip(ctrl)->controller->lock);
    bc_nand_chip(ctrl)->select_chip = &ba315_select_chip;
    bc_nand_chip(ctrl)->controller = &bc_nand_chip(ctrl)->hwcontrol;
    init_waitqueue_head(&bc_nand_chip(ctrl)->controller->wq);

    /*
     * re-init controller with optimized settings now that nand
     * device was discovered
     */
    msg = "failed to setup ONFI settings";
    ret = onfi_enable(mtd, &ba315_onfi_ops);
    if (unlikely(ret))
        goto free_irq;

    msg = "failed to init operations queue daemon";
    ret = kba315_init(ba315_ctrl_ops);
    if (unlikely(ret))
        goto free_irq;

    /*
     * run final nand layer generic initialization:
     *
     * force nand_scan_tail to skip BBT scanning. we will do it ourself a bit
     * later
     * we do not need any memory buffers to operate ; we use the controller RAM
     * instead
     */
    bc_nand_chip(ctrl)->options |= NAND_USE_FLASH_BBT | NAND_SKIP_BBTSCAN;
    msg = "failed to finalize nand scanning operation";
    ret = nand_scan_tail(mtd);
    if (unlikely(ret))
        goto exit_kba315;

    kfree(bc_nand_chip(ctrl)->buffers);
    bc_nand_chip(ctrl)->buffers = 0;

    /*
     * install our own MTD methods
     */
	mtd->read = &ba315_mtd_read;
	mtd->read_oob = &ba315_mtd_read_oob;
	mtd->write = &ba315_mtd_write;
	mtd->write_oob = &ba315_mtd_write_oob;
	mtd->erase = &ba315_mtd_erase_nobbt;
	mtd->erase_bbt = &ba315_mtd_erase_bbt;
    /* FIXME (optional): replace with a proper suspend function (see nand_suspend) */
	mtd->suspend = (void*) &ba315_bug;
    /* FIXME (optional): replace with a proper resume function (see nand_resume) */
	mtd->resume = (void*) &ba315_bug;

    /*
     * prepare for BBT scanning
     */
    bc_nand_chip(ctrl)->scan_bbt = nand_default_bbt;
    bc_nand_chip(ctrl)->block_bad = &ba315_block_bad;
    bc_nand_chip(ctrl)->block_markbad = &ba315_block_markbad;

    /*
     * scan BBT
     */
    ret = nand_default_bbt(mtd);
    if (unlikely(ret))
        goto exit_kba315;

#ifdef STAT
    if (unlikely(! proc_create_data("driver/ba315stats", 0, NULL,
                                    &ba315_proc_fops, ctrl)))
        goto exit_kba315;
#endif

    if (likely(! ba315_add_mtd(pdev, mtd)))
        return 0;

#ifdef STAT
    remove_proc_entry("driver/ba315stats", NULL);
#endif

exit_kba315:
    kba315_fini();
free_irq:
    free_irq(res->start, mtd);
fini_clock:
    ba315_fini_clock();
unmap_mem:
    iounmap(ctrl->bc_mem);
unmap_regs:
    iounmap(ctrl->bc_regs);
free:
    kfree(mtd);
out:
    dev_notice(&pdev->dev, "%s\n", msg);
    return ret;
}

int __devexit
ba315_remove_ctrl(struct platform_device* pdev)
{
    struct mtd_info*    mtd = platform_get_drvdata(pdev);
    struct ba315_ctrl*  ctrl = mtd->priv;
    struct resource*    res;

    dev_dbg(&pdev->dev, "device removal start\n");

	nand_release(mtd);

    kba315_fini();
    ba315_reset(ctrl);
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (likely(res))
        free_irq(res->start, mtd);

    iounmap(ctrl->bc_mem);
    iounmap(ctrl->bc_regs);
    ba315_fini_clock();
#ifdef STAT
    remove_proc_entry("driver/ba315stats", NULL);
#endif
    platform_set_drvdata(pdev, 0);
    kfree(mtd);

    return 0;
}
