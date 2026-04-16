// SPDX-License-Identifier: GPL-2.0
/*
 * UCLASS wakeup policy helpers
 */

#include "../sched.h"
#include <linux/sched/sysctl.h>

#include "uclass.h"

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

	boost_pct = uclass_effective_gran_boost_pct(curr, p);
	gran = mult_frac(gran, 100 - boost_pct, 100);

	return max_t(unsigned long, gran, sysctl_sched_min_granularity / 2);
}
