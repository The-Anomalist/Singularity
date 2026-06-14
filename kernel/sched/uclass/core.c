// SPDX-License-Identifier: GPL-2.0
/*
 * UCLASS (Unified Capacity and Latency Aware Scheduling System)
 *
 * Shared feature/gating helpers used by wakeup and placement units.
 */

#include "../sched.h"
#include <linux/sched/sysctl.h>

#include "uclass.h"

bool uclass_enabled(void)
{
	return sched_feat(UCLASS);
}

bool uclass_auto_tune_enabled(void)
{
	return uclass_enabled() && READ_ONCE(sysctl_sched_uclass_auto_tune);
}

bool uclass_task_high_util(struct task_struct *p)
{
	unsigned long util, cap;
	unsigned int high_pct;

	if (unlikely(!uclass_enabled() || !p))
		return false;

	high_pct = uclass_pct(READ_ONCE(sysctl_sched_uclass_high_util_pct), 100);
	if (unlikely(!high_pct))
		return false;

	util = task_util_est(p);
#ifdef CONFIG_UCLAMP_TASK
	util = clamp(util,
		     uclamp_eff_value(p, UCLAMP_MIN),
		     uclamp_eff_value(p, UCLAMP_MAX));
#endif
	cap = capacity_orig_of(task_cpu(p));
	if (unlikely(!cap))
		cap = SCHED_CAPACITY_SCALE;

	return util * 100 >= cap * high_pct;
}

unsigned int uclass_effective_gran_boost_pct(struct task_struct *curr,
					     struct task_struct *p)
{
	unsigned int boost_pct;

	boost_pct = uclass_pct(READ_ONCE(sysctl_sched_uclass_gran_boost_pct), 100);
	if (unlikely(!uclass_auto_tune_enabled() || !curr || !p))
		return boost_pct;

	if (uclass_task_high_util(p))
		boost_pct += 10;

	if (uclamp_boosted(p))
		boost_pct += 5;

	return min_t(unsigned int, boost_pct,
		     uclass_pct(READ_ONCE(sysctl_sched_uclass_auto_boost_max_pct),
				100));
}

unsigned int uclass_idle_exit_latency_limit_us(struct task_struct *p)
{
	unsigned int limit;

	limit = READ_ONCE(sysctl_sched_uclass_idle_exit_latency_limit_us);
	if (unlikely(!limit || !uclass_auto_tune_enabled() || !p))
		return limit;

	if (uclass_task_high_util(p))
		limit = max_t(unsigned int, limit / 2, 50);

	return limit;
}
