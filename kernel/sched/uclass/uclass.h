/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _KERNEL_SCHED_UCLASS_H
#define _KERNEL_SCHED_UCLASS_H

#include <linux/types.h>
#include <linux/cpuidle.h>

struct cpuidle_state;
struct task_struct;

#ifdef CONFIG_SCHED_UCLASS
bool uclass_enabled(void);
bool uclass_wakeup_preempt_enabled(void);
bool uclass_placement_enabled(void);

unsigned long uclass_adjust_wakeup_gran(struct task_struct *curr,
					struct task_struct *p,
					unsigned long gran);

bool uclass_pick_idle_cpu_first(struct task_struct *p);
bool uclass_prefer_prev_cpu(struct task_struct *p);
unsigned int uclass_prev_cpu_energy_margin_pct(void);

bool uclass_idle_candidate_is_better(unsigned long cpu_cap,
				     unsigned long target_cap,
				     struct cpuidle_state *idle,
				     unsigned int min_exit_lat);
#else
static inline bool uclass_enabled(void)
{
	return false;
}

static inline bool uclass_wakeup_preempt_enabled(void)
{
	return false;
}

static inline bool uclass_placement_enabled(void)
{
	return false;
}

static inline unsigned long
uclass_adjust_wakeup_gran(struct task_struct *curr,
			  struct task_struct *p,
			  unsigned long gran)
{
	(void)curr;
	(void)p;
	return gran;
}

static inline bool uclass_pick_idle_cpu_first(struct task_struct *p)
{
	(void)p;
	return false;
}

static inline bool uclass_prefer_prev_cpu(struct task_struct *p)
{
	(void)p;
	return false;
}

static inline unsigned int uclass_prev_cpu_energy_margin_pct(void)
{
	return 6;
}

static inline bool uclass_idle_candidate_is_better(unsigned long cpu_cap,
					    unsigned long target_cap,
					    struct cpuidle_state *idle,
					    unsigned int min_exit_lat)
{
	(void)cpu_cap;
	(void)target_cap;
	if (!idle || cpu_cap != target_cap)
		return true;

	return idle->exit_latency <= min_exit_lat;
}
#endif

#endif /* _KERNEL_SCHED_UCLASS_H */
