/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Linkedsemi LS2010 eSPI virtual wire (VW) driver
 *
 * ioctl ABI shared with userspace, and the mailbox wire format shared
 * with the E906 firmware (espi_ls.c).
 */
#ifndef __UAPI_LINKEDSEMI_ESPI_VW_H
#define __UAPI_LINKEDSEMI_ESPI_VW_H

#include <linux/types.h>
#include <linux/ioctl.h>

#define LS_ESPI_VW_MS_NUM		8

/* eSPI virtual wire indices (Intel eSPI base specification). */
#define LS_ESPI_VW_SYS_EVT_2		0x02
#define LS_ESPI_VW_SYS_EVT_3		0x03
#define LS_ESPI_VW_SYS_EVT_7		0x07
#define LS_ESPI_SYS_EVT_3_PLTRST_N	(1 << 1)

/* Mailbox wire format shared with the E906 firmware (espi_ls.c). */
struct espi_vwire_msg {
	__u8 vw_idx;
	__u8 vw_val;
};

/*
 * VW channel ioctls.  Same magic as the ASPEED_ESPI_VW_* commands in
 * drivers/soc/aspeed/espi/aspeed-espi-comm.h; 0x10..0x15 are taken by the
 * ASPEED-compatible set, so the Linkedsemi-specific commands continue at 0x16.
 */
#define LS_ESPI_VW_IOC_MAGIC		0xb8
#define LS_ESPI_VW_GET_S02_MS		_IOR(LS_ESPI_VW_IOC_MAGIC, 0x16, __u8)
#define LS_ESPI_VW_GET_S03_MS		_IOR(LS_ESPI_VW_IOC_MAGIC, 0x17, __u8)
#define LS_ESPI_VW_GET_S07_MS		_IOR(LS_ESPI_VW_IOC_MAGIC, 0x18, __u8)

/*
 * Argument for LS_ESPI_VW_GET_MS_VW_VAL: idx (0..LS_ESPI_VW_MS_NUM-1)
 * selects the ms_vw_val[] entry, val is filled in by the kernel.
 */
struct ls_espi_vw_ms_vw_arg {
	__u8 idx;
	__u8 val;
};
#define LS_ESPI_VW_GET_MS_VW_VAL	_IOWR(LS_ESPI_VW_IOC_MAGIC, 0x19, \
					      struct ls_espi_vw_ms_vw_arg)
/* Send {vw_idx, vw_val} downstream over the VW channel. */
#define LS_ESPI_VW_PUT_VW		_IOW(LS_ESPI_VW_IOC_MAGIC, 0x1a, \
					     struct espi_vwire_msg)

#endif /* __UAPI_LINKEDSEMI_ESPI_VW_H */
