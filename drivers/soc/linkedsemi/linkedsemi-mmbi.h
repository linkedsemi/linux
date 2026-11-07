/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Linkedsemi LS2010 MMBI (Memory Mapped Bus Interface) common definitions
 *
 * The descriptor layout below is shared with the E906 firmware
 * (soc/linkedsemi/common/espi_lpc/espi_lpc_host_mmbi.c).  Keep struct
 * ls_mmbi_desc / ls_mmbi_host_ros / ls_mmbi_host_rws in sync with
 * struct mmbi_desc / host_ros / host_rws there.  All fields are stored
 * little-endian; the host CPU reads the region verbatim.
 */

#ifndef __LINKEDSEMI_MMBI_H
#define __LINKEDSEMI_MMBI_H

#include <linux/bits.h>
#include <linux/types.h>

#define LS_MMBI_SIGNATURE	"#MMBI$"
#define LS_MMBI_VERSION_1_0	1

#define LS_MMBI_MAX_CHANNELS	7
#define LS_MMBI_MIN_SIZE	1024

/* Byte offsets of the fixed regions inside the shared MMBI space */
#define LS_MMBI_DESC_OFF	0	/* struct ls_mmbi_desc */
#define LS_MMBI_DESC_SIZE	64
#define LS_MMBI_ROS_OFF		LS_MMBI_DESC_SIZE /* struct ls_mmbi_host_ros */
#define LS_MMBI_ROS_SIZE	8
#define LS_MMBI_RWS_OFF		(LS_MMBI_ROS_OFF + LS_MMBI_ROS_SIZE)
#define LS_MMBI_RWS_SIZE	8
#define LS_MMBI_BUF_OFF		(LS_MMBI_RWS_OFF + LS_MMBI_RWS_SIZE)

/* The *_p and *_ba descriptor fields hold byte offsets divided by 8 */
#define LS_MMBI_UNITS(off)	((off) >> 3)

/*
 * MMBI state bits ride in the low two bits of the two write-pointer
 * registers (the pointer part covers bits [31:2]).
 */
#define LS_MMBI_PTR_MASK	~0x3U
#define LS_MMBI_STATE_B_R	BIT(0)	/* BMC reset, in host_ros.b2h_wp */
#define LS_MMBI_STATE_B_U	BIT(1)	/* BMC up, in host_ros.b2h_wp */
#define LS_MMBI_STATE_H_R	BIT(0)	/* Host reset, in host_rws.h2b_wp */
#define LS_MMBI_STATE_H_U	BIT(1)	/* Host up, in host_rws.h2b_wp */

enum ls_mmbi_host_int_type {
	LS_MMBI_HOST_INT_NONE = 0,
	LS_MMBI_HOST_INT_PCIE = 1,
	LS_MMBI_HOST_INT_ESPI_VW = 3,
};

/* Buffer type, same value as enum mmbi_buffer_type of the MMBI spec */
#define LS_MMBI_BUF_TYPE_VPSCB	1

struct ls_mmbi_desc {
	u8 signature[6];
	u8 version;
	u8 os_use;
	u32 b2h_ba;		/* units of 8 bytes */
	u32 h2b_ba;		/* units of 8 bytes */
	u32 b2h_l;
	u32 h2b_l;
	u8 buft;
	u8 reserved0[7];
	u32 h_ros_p;		/* units of 8 bytes */
	u32 h_rws_p;		/* units of 8 bytes */
	u8 h_int_t;
	u8 h_int_l;
	u8 reserved1[3];
	u8 h_int_v;
	u8 bmc_int_t;
	u32 bmc_int_l;
	u8 reserved2[4];
	u8 bmc_int_v;
	u8 reserved3[8];
} __packed;

/* BMC written, host read only */
struct ls_mmbi_host_ros {
	u32 b2h_wp;		/* B2H write pointer + BMC state bits */
	u32 h2b_rp;		/* H2B read pointer */
} __packed;

/* Host written, BMC reads; host writes here trigger MMBI_HOST_RWS_WR */
struct ls_mmbi_host_rws {
	u32 h2b_wp;		/* H2B write pointer + Host state bits */
	u32 b2h_rp;		/* B2H read pointer */
} __packed;

/* The E906 firmware relies on this exact layout */
static_assert(sizeof(struct ls_mmbi_desc) == 64);
static_assert(sizeof(struct ls_mmbi_host_ros) == 8);
static_assert(sizeof(struct ls_mmbi_host_rws) == 8);

struct ls_mmbi_ins;

struct ls_mmbi_chan_desc {
	u32 b2h_ba;		/* B2H buffer byte offset */
	u32 h2b_ba;		/* H2B buffer byte offset */
	u32 b2h_l;		/* B2H buffer length */
	u32 h2b_l;		/* H2B buffer length */
	u8 index;
	struct ls_mmbi_ins *mmbi;
	void *priv;		/* struct ls_mmbi_chan_priv, private */
};

struct ls_mmbi_ins {
	struct device *dev;
	void *virt;		/* kernel mapping of the shared MMBI space */
	size_t size;		/* total size of the shared space in bytes */
	int ins_id;		/* instance id, used for the miscdev name */
	struct ls_mmbi_chan_desc chan_desc[LS_MMBI_MAX_CHANNELS];
	u8 num_of_channels;
	u8 host_int_type;	/* enum ls_mmbi_host_int_type */
	u8 host_int_location;	/* eSPI: vw_idx, PCIe: MSI number */
	u8 host_int_value;	/* eSPI: vw_data, PCIe: 0 */
	u8 bmc_int_type;	/* 0: MMBI_HOST_RWS_WR / doorbell notify */
	u8 bmc_int_value;
	u32 bmc_int_location;
	/*
	 * Transport hook used to report an interrupt to the host after the
	 * BMC advanced its B2H write pointer.  May be NULL when the
	 * transport has no host interrupt support.
	 */
	int (*host_int_raise)(struct ls_mmbi_ins *mmbi);
};

int ls_mmbi_init(struct ls_mmbi_ins *mmbi);
void ls_mmbi_remove(struct ls_mmbi_ins *mmbi);

/*
 * Called by the transport when the host wrote the shared region
 * (eSPI: MMBI_HOST_RWS_WR mailbox message, PCIe: local doorbell
 * interrupt).  Refreshes the per-channel rx/tx state and wakes pollers.
 */
void ls_mmbi_host_write_notify(struct ls_mmbi_ins *mmbi);

#endif /* __LINKEDSEMI_MMBI_H */
