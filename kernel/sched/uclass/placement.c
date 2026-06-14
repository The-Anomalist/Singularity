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
	if (uclass_auto_tune_enabled() && uclass_task_high_util(p))
		return false;

	return uclass_placement_enabled() &&
	       READ_ONCE(sysctl_sched_uclass_idle_bias) &&
	       uclass_task_active(p);
}

bool uclass_prefer_prev_cpu(struct task_struct *p)
{
	return uclass_placement_enabled() &&
	       READ_ONCE(sysctl_sched_uclass_prefer_prev_cpu) &&
	       uclass_task_active(p);
}

unsigned int uclass_prev_cpu_energy_margin_pct(void)
{
	if (!uclass_placement_enabled())
		return 6;

	return uclass_pct(READ_ONCE(sysctl_sched_uclass_prev_cpu_energy_margin_pct),
			  50);
}

bool uclass_idle_candidate_is_better(unsigned long cpu_cap,
				     unsigned long target_cap,
				     struct task_struct *p,
				     struct cpuidle_state *idle,
				     unsigned int min_exit_lat)
{
	unsigned int limit;

	if (unlikely(!idle || cpu_cap != target_cap))
		return true;

	limit = uclass_idle_exit_latency_limit_us(p);
	if (limit && idle->exit_latency > limit)
		return false;

	if (uclass_placement_enabled() && READ_ONCE(sysctl_sched_uclass_idle_bias))
		return idle->exit_latency < min_exit_lat;

	return idle->exit_latency <= min_exit_lat;
}
