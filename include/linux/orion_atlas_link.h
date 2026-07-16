/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_ORION_ATLAS_LINK_H
#define _LINUX_ORION_ATLAS_LINK_H

#include <linux/types.h>
#include <linux/cpumask.h>
#include <linux/string.h>

#define ATLAS_MAX_CPU_POLICIES 8

struct atlas_cpu_policy_telemetry {
	cpumask_t cpus;
	unsigned int util_pct;
	unsigned int freq_khz;
	unsigned int thermal_pct;
	unsigned int capacity;
	bool active;
};

struct atlas_telemetry_snapshot {
	unsigned int mem_pressure_pct;
	unsigned int mem_contention_pct;
	unsigned int mem_reclaim_pct;
	unsigned int mem_swap_pct;
	unsigned int mem_workingset_refault_pct;
	struct atlas_cpu_policy_telemetry cpu[ATLAS_MAX_CPU_POLICIES];
	unsigned int nr_cpu_policies;
	unsigned int cpu_util_pct;
	unsigned int cpu_freq_khz;
	unsigned int cpu_thermal_pct;
	unsigned int gpu_util_pct;
	unsigned int gpu_freq_khz;
	unsigned int gpu_thermal_pct;
	unsigned int npu_util_pct;
	unsigned int npu_bw_kbps;
	unsigned int npu_thermal_pct;
	bool display_active;
	u64 timestamp_ns;
	unsigned int seq;
};

#if IS_ENABLED(CONFIG_CPU_FREQ_GOV_ATLAS)
void atlas_update_gpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct);
void atlas_get_gpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct);
void atlas_update_npu_telemetry(unsigned int util_pct, unsigned int bw_kbps,
				unsigned int thermal_pct);
void atlas_get_npu_telemetry(unsigned int *util_pct, unsigned int *bw_kbps,
			     unsigned int *thermal_pct);
void atlas_update_cpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct);
void atlas_update_cpu_policy_telemetry(const struct cpumask *cpus,
				       unsigned int util_pct,
				       unsigned int freq_khz,
				       unsigned int thermal_pct,
				       unsigned int capacity);
void atlas_get_snapshot(struct atlas_telemetry_snapshot *snapshot);
void atlas_get_cpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct);
void atlas_update_mem_telemetry(unsigned int pressure_pct,
				unsigned int contention_pct);
void atlas_get_mem_telemetry(unsigned int *pressure_pct,
			     unsigned int *contention_pct);
void atlas_update_mem_stats(unsigned int reclaim_pct, unsigned int swap_pct,
			    unsigned int workingset_refault_pct);
void atlas_get_mem_stats(unsigned int *reclaim_pct, unsigned int *swap_pct,
			 unsigned int *workingset_refault_pct);
void atlas_update_display_state(bool active);
bool atlas_display_state_active(void);
#else
static inline void atlas_update_gpu_telemetry(unsigned int util_pct,
				      unsigned int freq_khz,
				      unsigned int thermal_pct)
{
}

static inline void atlas_get_gpu_telemetry(unsigned int *util_pct,
				   unsigned int *freq_khz,
				   unsigned int *thermal_pct)
{
	if (util_pct)
		*util_pct = 0;
	if (freq_khz)
		*freq_khz = 0;
	if (thermal_pct)
		*thermal_pct = 0;
}

static inline void atlas_update_npu_telemetry(unsigned int util_pct,
					      unsigned int bw_kbps,
					      unsigned int thermal_pct)
{
}

static inline void atlas_get_npu_telemetry(unsigned int *util_pct,
					   unsigned int *bw_kbps,
					   unsigned int *thermal_pct)
{
	if (util_pct)
		*util_pct = 0;
	if (bw_kbps)
		*bw_kbps = 0;
	if (thermal_pct)
		*thermal_pct = 0;
}

static inline void atlas_update_cpu_telemetry(unsigned int util_pct,
				      unsigned int freq_khz,
				      unsigned int thermal_pct)
{
}

static inline void atlas_update_cpu_policy_telemetry(const struct cpumask *cpus,
				      unsigned int util_pct,
				      unsigned int freq_khz,
				      unsigned int thermal_pct,
				      unsigned int capacity)
{
}

static inline void atlas_get_snapshot(struct atlas_telemetry_snapshot *snapshot)
{
	if (snapshot)
		memset(snapshot, 0, sizeof(*snapshot));
}

static inline void atlas_get_cpu_telemetry(unsigned int *util_pct,
				   unsigned int *freq_khz,
				   unsigned int *thermal_pct)
{
	if (util_pct)
		*util_pct = 0;
	if (freq_khz)
		*freq_khz = 0;
	if (thermal_pct)
		*thermal_pct = 0;
}

static inline void atlas_update_mem_telemetry(unsigned int pressure_pct,
					      unsigned int contention_pct)
{
}

static inline void atlas_get_mem_telemetry(unsigned int *pressure_pct,
					   unsigned int *contention_pct)
{
	if (pressure_pct)
		*pressure_pct = 0;
	if (contention_pct)
		*contention_pct = 0;
}

static inline void atlas_update_mem_stats(unsigned int reclaim_pct,
					  unsigned int swap_pct,
					  unsigned int workingset_refault_pct)
{
}

static inline void atlas_get_mem_stats(unsigned int *reclaim_pct,
				       unsigned int *swap_pct,
				       unsigned int *workingset_refault_pct)
{
	if (reclaim_pct)
		*reclaim_pct = 0;
	if (swap_pct)
		*swap_pct = 0;
	if (workingset_refault_pct)
		*workingset_refault_pct = 0;
}

static inline void atlas_update_display_state(bool active)
{
}

static inline bool atlas_display_state_active(void)
{
	return true;
}
#endif

#endif /* _LINUX_ORION_ATLAS_LINK_H */
