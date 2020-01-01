/*
 *
 * description  BA315 controller operations queue interface
 *
 *      author  Gregor Boirie <gregor.boirie@parrot.com>
 *        date  18-Sep-2008
 *
 *        $Id: kba315.h,v 1.2 2008-12-26 18:17:40 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#ifndef _KBA315_H
#define _KBA315_H

#include <linux/list.h>
#include <linux/completion.h>
#include <linux/mtd/mtd.h>

enum kba315_ops_type {
    read_op,
    write_op,
    erase_op,
    max_op
};

struct kba315_workset {
    struct completion       done;
    struct mtd_info*        mtd;
    enum kba315_ops_type    type;
    size_t                  uncomplete;
    int                     ret;
};

struct kba315_work {
    struct list_head        node;
    struct kba315_workset*  set;
    void*                   ops;
    loff_t                  offset;
    int                     erase_bbt;
};

extern void     kba315_wait_complete(struct kba315_workset*, struct kba315_work*, size_t);
extern size_t   kba315_retlen(struct kba315_work*, size_t);
extern size_t   kba315_oobretlen(struct kba315_work*, size_t);

#define KBA315_DEFINE_WORKSET(_name, _mtd, _type)   \
    struct kba315_workset _name = {                         \
        .mtd = _mtd,                                        \
        .type = _type                                       \
    }

typedef int (kba315_op_t)(struct kba315_work*, struct kba315_work*);

extern int  kba315_init(kba315_op_t* const*) __init;
extern void kba315_fini(void);

#endif

/* vim:ts=4:sw=4:et:syn=c */
