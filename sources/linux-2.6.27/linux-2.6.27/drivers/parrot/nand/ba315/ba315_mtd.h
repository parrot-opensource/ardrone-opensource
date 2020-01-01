/*
 *
 * description  ba315 MTD adaptation layer interface
 *
 *      author  Gregor Boirie <gregor.boirie@parrot.com>
 *        date  20-Nov-2008
 *
 *        $Id: ba315_mtd.h,v 1.1 2008-12-19 13:04:19 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

extern int ba315_mtd_read(struct mtd_info*, loff_t, size_t, size_t*, uint8_t*);
extern int ba315_mtd_read_oob(struct mtd_info*, loff_t, struct mtd_oob_ops*);
extern int ba315_mtd_write(struct mtd_info*, loff_t, size_t, size_t*, uint8_t const*);
extern int ba315_mtd_write_oob(struct mtd_info*, loff_t, struct mtd_oob_ops*);
extern int ba315_mtd_erase_bbt(struct mtd_info*, struct erase_info*);
extern int ba315_mtd_erase_nobbt(struct mtd_info*, struct erase_info*);
extern size_t ba315_mtd_oobretlen(struct kba315_work*, size_t);
extern size_t ba315_mtd_retlen(struct kba315_work*, size_t);

/* vim:ts=4:sw=4:et:syn=c */
