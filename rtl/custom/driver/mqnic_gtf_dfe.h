/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * GTF DFE Optimization - Kernel/Userspace Shared Header
 * Register offsets, ioctl definitions, and data structures.
 */

#ifndef MQNIC_GTF_DFE_H
#define MQNIC_GTF_DFE_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* Register offsets (from app BAR base) */
#define GTF_DFE_REG_CTRL           0x0000
#define GTF_DFE_REG_STATUS         0x0004
#define GTF_DFE_REG_IRQ_EN         0x0008
#define GTF_DFE_REG_IRQ_STATUS     0x000C
#define GTF_DFE_REG_TAP_ACTIVE     0x0010
#define GTF_DFE_REG_TAP_COUNT      0x0014
#define GTF_DFE_REG_TAP_VAL_BASE   0x0020
#define GTF_DFE_REG_TAP_OVR_BASE   0x0060
#define GTF_DFE_REG_EYE_HEIGHT     0x00A0
#define GTF_DFE_REG_EYE_WIDTH      0x00A4
#define GTF_DFE_REG_EYE_BER        0x00A8
#define GTF_DFE_REG_EYE_PRESCALE   0x00AC
#define GTF_DFE_REG_EYE_H_RANGE    0x00B0
#define GTF_DFE_REG_EYE_V_RANGE    0x00B4
#define GTF_DFE_REG_TX_ALIGN_HIT   0x00C0
#define GTF_DFE_REG_TX_ALIGN_MISS  0x00C4
#define GTF_DFE_REG_TX_ALIGN_CTRL  0x00C8
#define GTF_DFE_REG_LPM_THRESHOLD  0x00D0
#define GTF_DFE_REG_CAL_TIMEOUT    0x00D4
#define GTF_DFE_REG_CAL_STABILITY  0x00D8
#define GTF_DFE_REG_DRP_ADDR       0x00E0
#define GTF_DFE_REG_DRP_DATA       0x00E4
#define GTF_DFE_REG_DRP_CTRL       0x00E8
#define GTF_DFE_REG_DRP_STATUS     0x00EC
#define GTF_DFE_REG_SCAN_POINTS    0x00F0
#define GTF_DFE_REG_SCAN_BRAM_ADDR 0x00F4
#define GTF_DFE_REG_SCAN_BRAM_DATA 0x00F8
#define GTF_DFE_REG_VERSION        0x0100
#define GTF_DFE_REG_SCRATCH        0x0104

/* CTRL register bits */
#define GTF_DFE_CTRL_CAL_START     (1 << 0)
#define GTF_DFE_CTRL_EYE_SCAN      (1 << 1)
#define GTF_DFE_CTRL_OVERRIDE      (1 << 2)
#define GTF_DFE_CTRL_BUF_BYPASS    (1 << 3)
#define GTF_DFE_CTRL_FCS_CHECK     (1 << 4)
#define GTF_DFE_CTRL_FCS_DELETE    (1 << 5)
#define GTF_DFE_CTRL_FCS_TX_INS    (1 << 6)
#define GTF_DFE_CTRL_LPM_FORCE     (1 << 7)
#define GTF_DFE_CTRL_DFE_FORCE     (1 << 8)
#define GTF_DFE_CTRL_AUTO_MODE     (1 << 9)
#define GTF_DFE_CTRL_QUICK_SCAN    (1 << 10)

/* STATUS register bits */
#define GTF_DFE_STATUS_CAL_DONE    (1 << 0)
#define GTF_DFE_STATUS_CAL_BUSY    (1 << 1)
#define GTF_DFE_STATUS_EYE_DONE    (1 << 2)
#define GTF_DFE_STATUS_EYE_BUSY    (1 << 3)
#define GTF_DFE_STATUS_CDR_LOCK    (1 << 4)
#define GTF_DFE_STATUS_LPM_ACTIVE  (1 << 6)
#define GTF_DFE_STATUS_DFE_ACTIVE  (1 << 7)
#define GTF_DFE_STATUS_BUF_ALIGNED (1 << 8)

/* Version magic */
#define GTF_DFE_VERSION_MAGIC      0x47464401  /* "GFD\x01" */

/* Number of DFE taps */
#define GTF_DFE_NUM_TAPS           15

/* Max eye scan points */
#define GTF_DFE_MAX_SCAN_POINTS    8192

/* ioctl definitions */
#define GTF_DFE_IOC_MAGIC          'G'

enum gtf_dfe_mode {
	GTF_DFE_MODE_LPM  = 0,
	GTF_DFE_MODE_DFE  = 1,
	GTF_DFE_MODE_AUTO = 2,
};

struct gtf_dfe_taps {
	__u8  count;
	__u8  values[GTF_DFE_NUM_TAPS];
	__u16 active_mask;
};

struct gtf_dfe_drp_xfer {
	__u16 addr;
	__u16 data;
};

struct gtf_dfe_status {
	__u32 ctrl;
	__u32 status;
	__u16 eye_height;
	__u16 eye_width;
	__u32 eye_ber;
	__u32 tx_align_hits;
	__u32 tx_align_misses;
	__u8  active_taps;
	__u16 active_mask;
	__u8  mode;      /* enum gtf_dfe_mode */
	__u8  buf_bypass;
	__u8  fcs_check;
};

#define GTF_DFE_CALIBRATE       _IO(GTF_DFE_IOC_MAGIC, 1)
#define GTF_DFE_EYE_SCAN        _IO(GTF_DFE_IOC_MAGIC, 2)
#define GTF_DFE_GET_TAPS        _IOR(GTF_DFE_IOC_MAGIC, 3, struct gtf_dfe_taps)
#define GTF_DFE_SET_TAPS        _IOW(GTF_DFE_IOC_MAGIC, 4, struct gtf_dfe_taps)
#define GTF_DFE_SET_MODE        _IOW(GTF_DFE_IOC_MAGIC, 5, int)
#define GTF_DFE_GET_STATUS      _IOR(GTF_DFE_IOC_MAGIC, 6, struct gtf_dfe_status)
#define GTF_DFE_DRP_READ        _IOWR(GTF_DFE_IOC_MAGIC, 7, struct gtf_dfe_drp_xfer)
#define GTF_DFE_DRP_WRITE       _IOW(GTF_DFE_IOC_MAGIC, 8, struct gtf_dfe_drp_xfer)
#define GTF_DFE_QUICK_SCAN      _IO(GTF_DFE_IOC_MAGIC, 9)

#endif /* MQNIC_GTF_DFE_H */
