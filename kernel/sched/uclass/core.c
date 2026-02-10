// SPDX-License-Identifier: GPL-2.0
/*
 * UCLASS (Unified Capacity and Latency Aware Scheduling System)
 *
 * This file intentionally keeps UCLASS policy in one place so fair/walt paths
 * can call helpers while preserving stock scheduler control flow.
 */

#include "../sched.h"
#include <linux/sched/sysctl.h>

#include "uclass.h"

bool uclass_enabled(void)
{
	return sched_feat(UCLASS);
}

bool uclass_wakeup_preempt_enabled(void)
{
	return sched_feat(UCLASS) && sched_feat(UCLASS_WAKEUP_PREEMPT) &&
	       sysctl_sched_uclass_wakeup_boost;
}

unsigned long uclass_adjust_wakeup_gran(struct task_struct *curr,
					struct task_struct *p,
					unsigned long gran)
{
	unsigned int boost_pct;

	if (!uclass_wakeup_preempt_enabled())
		return gran;

	if (!uclamp_latency_sensitive(p) || uclamp_boosted(curr))
		return gran;

	boost_pct = min_t(unsigned int, sysctl_sched_uclass_gran_boost_pct, 100);
	gran = mult_frac(gran, 100 - boost_pct, 100);

	return max_t(unsigned long, gran, sysctl_sched_min_granularity / 2);
}

bool uclass_idle_candidate_is_better(unsigned long cpu_cap,
				     unsigned long target_cap,
				     struct cpuidle_state *idle,
				     unsigned int min_exit_lat)
{
	if (!idle || cpu_cap != target_cap)
		return true;

	if (uclass_enabled() && sysctl_sched_uclass_idle_bias)
		return idle->exit_latency < min_exit_lat;

	return idle->exit_latency <= min_exit_lat;
}
