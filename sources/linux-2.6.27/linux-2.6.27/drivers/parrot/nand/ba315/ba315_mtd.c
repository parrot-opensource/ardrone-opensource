/*
 *
 * description  ba315 MTD adaptation layer implementation
 *
 *      author  Gregor Boirie <gregor.boirie@parrot.com>
 *        date  06-Oct-2008
 *
 *        $Id: ba315_mtd.c,v 1.1 2008-12-19 13:04:19 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include "kba315.h"

size_t
ba315_mtd_retlen(struct kba315_work* units, size_t count)
{
    size_t len = 0;

    BUG_ON(! units);
    BUG_ON(! count);

    while (count--)
        len += ((struct mtd_oob_ops const*) units[count].ops)->retlen;

    return len;
}

size_t
ba315_mtd_oobretlen(struct kba315_work* units, size_t count)
{
    size_t len = 0;

    BUG_ON(! units);
    BUG_ON(! count);

    while (count--)
        len += ((struct mtd_oob_ops const*) units[count].ops)->oobretlen;

    return len;
}

static
int
ba315_mtd_data(struct mtd_info* mtd, loff_t addr,
               size_t len, size_t* retlen, uint8_t* buf,
               enum kba315_ops_type cmd)
{
    int ret = -EINVAL;

    if ((addr + len) > mtd->size)
        goto out;

    ret = 0;
    if (! len)
        goto out;

    {
        KBA315_DEFINE_WORKSET(set, mtd, cmd);
        struct mtd_oob_ops ops = {
            .mode   = MTD_OOB_PLACE,
            .len    = len,
            .datbuf = buf,
            .oobbuf = NULL,
            .retlen = 0,
            .oobretlen = 0
        };
        struct kba315_work  work = {
            .ops = &ops,
            .offset = addr
        };

        kba315_wait_complete(&set, &work, 1);
        *retlen = ((struct mtd_oob_ops const*) work.ops)->retlen;
        ret = set.ret;
    }

out:
    return ret;
}

int
ba315_mtd_read(struct mtd_info* mtd, loff_t from,
               size_t len, size_t* retlen, uint8_t* buf)
{
    return ba315_mtd_data(mtd, from, len, retlen, buf, read_op);
}

int
ba315_mtd_write(struct mtd_info* mtd, loff_t to,
                size_t len, size_t* retlen, uint8_t const* buf)
{
    return ba315_mtd_data(mtd, to, len, retlen, (uint8_t*) buf, write_op);
}

static
int
ba315_mtd_oob(struct mtd_info* mtd, loff_t addr,
              struct mtd_oob_ops* ops,
              enum kba315_ops_type cmd)
{
    int     ret = -EINVAL;
    size_t  max_oob_len;

    if (unlikely(addr >= mtd->size))
        goto out;

    if (ops->datbuf && unlikely((addr + ops->len) > mtd->size))
        goto out;

    switch(ops->mode) {
	case MTD_OOB_AUTO:
        max_oob_len = mtd->ecclayout->oobavail;
        break;

	case MTD_OOB_PLACE:
	case MTD_OOB_RAW:
        max_oob_len = mtd->oobsize;
		break;

	default:
        ret = -ENOTSUPP;
		goto out;
	}

    if (! ops->datbuf)
        if (unlikely(ops->ooboffs >= max_oob_len))
            goto out;

    {
        KBA315_DEFINE_WORKSET(set, mtd, cmd);
        struct kba315_work  work = {
            /*
             * FIXME: ops should be passed by reference with the kba315_work
             * struct
             */
            .ops = ops,
            .offset = addr
        };

        ops->retlen = 0;
        ops->oobretlen = 0;

        kba315_wait_complete(&set, &work, 1);
        ret = set.ret;
    }

out:
    return ret;
}

int
ba315_mtd_read_oob(struct mtd_info* mtd, loff_t from,
                   struct mtd_oob_ops* ops)
{
    size_t          max_oob_len;
    size_t const    psft = ffs(mtd->writesize) - 1;

    if (ops->mode == MTD_OOB_AUTO)
        max_oob_len = mtd->ecclayout->oobavail;
    else
        max_oob_len = mtd->oobsize;

    if (! ops->datbuf)
        if (unlikely((ops->ooboffs + ops->ooblen) >
                     (((mtd->size >> psft) - (from >> psft)) * max_oob_len)))
            return -EINVAL;

    return ba315_mtd_oob(mtd, from, ops, read_op);
}

int
ba315_mtd_write_oob(struct mtd_info* mtd, loff_t to,
                    struct mtd_oob_ops* ops)
{
    size_t max_oob_len;

    if (ops->mode == MTD_OOB_AUTO)
        max_oob_len = mtd->ecclayout->oobavail;
    else
        max_oob_len = mtd->oobsize;

    if (! ops->datbuf)
        if (unlikely((ops->ooboffs + ops->ooblen) > max_oob_len))
            return -EINVAL;

    return ba315_mtd_oob(mtd, to, ops, write_op);
}

static
int
ba315_mtd_erase(struct mtd_info* mtd, struct erase_info* ops, int erase_bbt)
{
    struct nand_chip* const chip = mtd->priv;

	if (ops->addr & ((1 << chip->phys_erase_shift) - 1))
		return -EINVAL;

	if (ops->len & ((1 << chip->phys_erase_shift) - 1))
		return -EINVAL;

	if ((ops->len + ops->addr) > mtd->size)
		return -EINVAL;

    {
        KBA315_DEFINE_WORKSET(set, mtd, erase_op);
        struct kba315_work  work = {
            /*
             * FIXME: ops should be passed by reference with the kba315_work
             * struct
             */
            .ops = ops,
            .erase_bbt = erase_bbt
        };

        ops->state = MTD_ERASING;
        kba315_wait_complete(&set, &work, 1);

        return set.ret;
    }
}

int
ba315_mtd_erase_nobbt(struct mtd_info* mtd, struct erase_info* ops)
{
    return ba315_mtd_erase(mtd, ops, 0);
}

int
ba315_mtd_erase_bbt(struct mtd_info* mtd, struct erase_info* ops)
{
    return ba315_mtd_erase(mtd, ops, 1);
}

/* vim:ts=4:sw=4:et:syn=c */
