/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __QCOM_KONA_RPMH_H
#define __QCOM_KONA_RPMH_H

#include <linux/bitops.h>
#include <linux/types.h>

#define KONA_BCM_VOTE_MASK	GENMASK(13, 0)
#define KONA_BCM_VOTE_X_SHIFT	14
#define KONA_BCM_VALID		BIT(29)
#define KONA_BCM_COMMIT		BIT(30)

#define KONA_BCM_TCS_CMD(commit, valid, x, y) \
	(((commit) ? KONA_BCM_COMMIT : 0) | \
	 ((valid) ? KONA_BCM_VALID : 0) | \
	 (((x) & KONA_BCM_VOTE_MASK) << KONA_BCM_VOTE_X_SHIFT) | \
	 ((y) & KONA_BCM_VOTE_MASK))

struct kona_bcm_db {
	__le32 unit;
	__le16 width;
	u8 vcd;
	u8 reserved;
} __packed;

struct kona_bcm_state {
	const char *name;
	u32 addr;
	u32 unit;
	u16 width;
	u8 vcd;
	u64 requested_x;
	u64 requested_y;
	u64 raw_x;
	u64 raw_y;
	u32 dry_run_data;
	u64 dry_run_generation;
	u64 last_diagnosed_x;
	u64 last_diagnosed_y;
	bool saturated_x;
	bool saturated_y;
	u64 committed_x;
	u64 committed_y;
	u32 requested_data;
	u32 committed_data;
	u64 requested_generation;
	u64 committed_generation;
	bool metadata_valid;
	bool dirty;
	bool fallback;
	bool fallback_logged;
	u32 retry_count;
	u32 failure_count;
	u32 saturation_count;
	u64 submission_count;
	int last_error;
};

struct kona_qnode_vote {
	u64 avg;
	u64 peak;
	u16 buswidth;
	u16 channels;
};

enum kona_cpu_bcm_id {
	KONA_CPU_BCM_SH4,
	KONA_CPU_BCM_SH0,
	KONA_CPU_BCM_MC0,
	KONA_CPU_BCM_COUNT,
};

#endif
