/**
 *
 *       @file  ba315_drv.h
 *
 *      @brief  BA315 driver module interface
 *
 *     @author  Gregor Boirie <gregor.boirie@parrot.com>
 *       @date  09-Dec-2008
 *
 *        $Id: ba315_base.h,v 1.6 2009-06-16 09:21:46 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/mtd/nand.h>

struct ba315_device {
    u32 ctrl0;
    u32 ctrl1;
    u32 addr_lo;
    u32 addr_hi;
};

enum ba315_state {
    ba315_sleeping,
    ba315_reading,
    ba315_writing,
    ba315_erasing
};

enum ba315_cmd_type {
    ba315_cmd_end,
    ba315_ecc_read,
    ba315_raw_read,
    ba315_ecc_write,
    ba315_raw_write
};

struct ba315_cmd {
    enum ba315_cmd_type type;
    off_t               pageno;
    off_t               col;
    size_t              len;
    uint8_t*            data;
    off_t               data_off;
    size_t              data_len;
    uint8_t*            oob;
    off_t               oob_off;
    size_t              oob_len;
};

struct ba315_stats {
    unsigned int    samples;
    unsigned int    aligned;
    unsigned int    pages;
    unsigned int    size;
    unsigned int    same_page;
    unsigned int    chained;
    off_t           last_page;
};

struct ba315_status_checker;

struct ba315_settings {
    unsigned long  toRST;
    unsigned long  twRST;
    unsigned long  teRST;
};

struct ba315_ctrl {
    struct nand_chip                    __base;
#define bc_regs __base.IO_ADDR_R
#define bc_mem  __base.IO_ADDR_W
    struct completion                   irq_evt;
    struct ba315_status_checker const*  status_checker;
    struct ba315_device                 dev;
    unsigned char                       cycles;
    enum ba315_state                    state;
#if 0
    spinlock_t                          read_stats_lock;
    struct ba315_stats                  read_stats;
    spinlock_t                          write_stats_lock;
    struct ba315_stats                  write_stats;
#endif
    struct ba315_settings               settings;
    unsigned char                       curr_cmd;
    unsigned char                       next_cmd;
    unsigned int                        corrected;
    struct ba315_cmd                    cmds[2];
};

extern void ba315_set_buf(struct ba315_ctrl*, off_t, int, size_t);
extern void ba315_write_buf(struct ba315_ctrl*, off_t, void const*, size_t);
extern void ba315_encode_ecc(struct ba315_ctrl*, off_t, size_t);
extern void ba315_flush(struct ba315_ctrl*, void (*)(struct ba315_ctrl*));
extern void setup_ready_checker(struct ba315_ctrl*);
extern int  ba315_wait(struct ba315_ctrl*);
extern int ba315_poll(struct ba315_ctrl*);
extern void ba315_busy_wait(struct ba315_ctrl*);
extern void ba315_setup_write(struct ba315_ctrl*, off_t, off_t, size_t);
extern off_t ba315_last_eccbyte(struct ba315_ctrl const*, int);
extern off_t ba315_first_eccbyte(struct ba315_ctrl const*, int);
extern off_t ba315_subpage(off_t, size_t);

extern void ba315_setup_read(struct ba315_ctrl*, off_t, off_t);
extern void ba315_do_read(struct ba315_ctrl*, size_t);
extern void ba315_decode_ecc(struct ba315_ctrl*, off_t, size_t);
extern void ba315_read_buf(void*, struct ba315_ctrl*, off_t, size_t);

struct platform_device;

extern int __devinit ba315_probe_ctrl(struct platform_device*);
extern int __devexit ba315_remove_ctrl(struct platform_device*);

/* vim:ts=4:sw=4:et:syn=c */
