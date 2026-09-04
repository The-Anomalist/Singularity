/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_INTERCONNECT_KONA_H
#define __LINUX_INTERCONNECT_KONA_H

#include <linux/kconfig.h>
#include <linux/types.h>

enum kona_icc_gpu_source {
	KONA_ICC_GPU_SOURCE_NONE,
	KONA_ICC_GPU_SOURCE_GMU_HFI,
	KONA_ICC_GPU_SOURCE_MSM_BUS,
};

enum kona_icc_gpu_publish_phase {
	KONA_ICC_GPU_PHASE_REQUESTED,
	KONA_ICC_GPU_PHASE_APPLIED,
};

/* The authoritative KGSL command row. This interface never performs I/O. */
struct kona_icc_gpu_contribution {
	enum kona_icc_gpu_source source;
	u32 selected_level;
	u32 requested_level;
	u32 applied_level;
	u32 mc0_addr;
	u32 sh0_addr;
	u32 acv_addr;
	u32 mc0_data;
	u32 sh0_data;
	u32 acv_data;
	bool applied_valid;
	enum kona_icc_gpu_publish_phase phase;
};

#if IS_ENABLED(CONFIG_INTERCONNECT_QCOM_KONA)
void kona_icc_gpu_publish_contribution(
		const struct kona_icc_gpu_contribution *contribution);
void kona_icc_gpu_clear_contribution(enum kona_icc_gpu_source source,
		int error);
void kona_icc_ipa_shadow_note_selected(unsigned int idx);
#else
static inline void kona_icc_gpu_publish_contribution(
		const struct kona_icc_gpu_contribution *contribution) { }
static inline void kona_icc_gpu_clear_contribution(
		enum kona_icc_gpu_source source, int error) { }
static inline void kona_icc_ipa_shadow_note_selected(unsigned int idx) { }
#endif

#endif
