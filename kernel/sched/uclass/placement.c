// SPDX-License-Identifier: GPL-2.0
/*
 * UCLASS placement policy helpers
 */

#include "../sched.h"
#include <linux/sched/sysctl.h>

#include "uclass.h"

bool uclass_placement_enabled(void)
{
	return uclass_enabled() && sched_feat(UCLASS_PLACEMENT);
}

bool uclass_pick_idle_cpu_first(struct task_struct *p)
{
	return uclass_placement_enabled() && sysctl_sched_uclass_idle_bias &&
	       uclamp_latency_sensitive(p);
}

bool uclass_prefer_prev_cpu(struct task_struct *p)
{
	return uclass_placement_enabled() && sysctl_sched_uclass_prefer_prev_cpu &&
	       uclamp_latency_sensitive(p);
}

unsigned int uclass_prev_cpu_energy_margin_pct(void)
{
	if (!uclass_placement_enabled())
		return 6;

	return min_t(unsigned int, sysctl_sched_uclass_prev_cpu_energy_margin_pct,
		     50);
}

bool uclass_idle_candidate_is_better(unsigned long cpu_cap,
				     unsigned long target_cap,
				     struct cpuidle_state *idle,
				     unsigned int min_exit_lat)
{
	if (!idle || cpu_cap != target_cap)
		return true;

	if (uclass_placement_enabled() && sysctl_sched_uclass_idle_bias)
		return idle->exit_latency < min_exit_lat;

	return idle->exit_latency <= min_exit_lat;
}
