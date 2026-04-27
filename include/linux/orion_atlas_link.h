/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_ORION_ATLAS_LINK_H
#define _LINUX_ORION_ATLAS_LINK_H

#include <linux/types.h>

#if IS_ENABLED(CONFIG_CPU_FREQ_GOV_ATLAS)
void atlas_update_gpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct);
void atlas_get_gpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct);
void atlas_update_cpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct);
void atlas_get_cpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct);
void atlas_update_mem_telemetry(unsigned int pressure_pct,
				unsigned int contention_pct);
void atlas_get_mem_telemetry(unsigned int *pressure_pct,
			     unsigned int *contention_pct);
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

static inline void atlas_update_cpu_telemetry(unsigned int util_pct,
				      unsigned int freq_khz,
				      unsigned int thermal_pct)
{
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
#endif

#endif /* _LINUX_ORION_ATLAS_LINK_H */
