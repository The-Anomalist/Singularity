/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include "sched.h"

#include <linux/sched/cpufreq.h>
#include <trace/events/power.h>
#include <linux/sched/sysctl.h>
#include <linux/mm.h>
#include <linux/atomic.h>
#include <linux/vmstat.h>
#include <linux/orion_atlas_link.h>
#include <linux/seqlock.h>
#include <linux/workqueue.h>

#ifdef OPLUS_FEATURE_POWER_CPUFREQ
/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 80
#define DEFAULT_RATE_LIMIT_US 0
static unsigned int default_above_hispeed_delay[] = {
                  DEFAULT_RATE_LIMIT_US };
static unsigned int default_target_loads[] = { DEFAULT_TARGET_LOAD };
#endif

struct sugov_auto_cfg {
	unsigned int		up_rate_limit_us;
	unsigned int		down_rate_limit_us;
	unsigned int		down_hysteresis_us;
	unsigned int		hispeed_load;
	unsigned int		hispeed_freq;
	unsigned int		rtg_boost_freq;
	unsigned int		mem_boost_util;
	unsigned int		mem_boost_hyst_us;
	unsigned int		auto_boost_high_load;
	unsigned int		auto_boost_low_load;
	unsigned int		auto_boost_min_util;
	unsigned int		auto_boost_max_util;
	unsigned int		auto_boost_decay_us;
	unsigned int		auto_boost_heavy_util;
	unsigned int		auto_boost_heavy_tasks;
	unsigned int		auto_boost_prime_util;
	unsigned int		auto_boost_gold_util;
	unsigned int		auto_boost_efficiency_load;
	unsigned int		auto_boost_efficiency_util;
	unsigned int		auto_profile;
	bool			uclamp_helper;
	bool			auto_boost;
	bool			pl;
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	spinlock_t        target_loads_lock;
	unsigned int            *target_loads;
	int               ntarget_loads;
	/*
	 * Wait this long before raising speed above hispeed, by default a
	 * single timer interval.
	 */
	spinlock_t above_hispeed_delay_lock;
	unsigned int *above_hispeed_delay;
	int nabove_hispeed_delay;
#endif
};

struct sugov_tunables {
	struct gov_attr_set	attr_set;
	struct sugov_auto_cfg	auto_cfg;
};

struct sugov_policy {
	struct cpufreq_policy	*policy;

	u64 last_ws;
	u64 curr_cycles;
	u64 last_cyc_update_time;
	unsigned long avg_cap;
	struct sugov_tunables	*tunables;
	struct list_head	tunables_hook;
	unsigned long hispeed_util;
	unsigned long rtg_boost_util;
	unsigned long max;

	raw_spinlock_t		update_lock;	/* For shared policies */
	u64			last_freq_update_time;
	s64			min_rate_limit_ns;
	s64			up_rate_delay_ns;
	s64			down_rate_delay_ns;
	s64			down_hyst_ns;
	unsigned int		next_freq;
	unsigned int		cached_raw_freq;
	unsigned int		prev_cached_raw_freq;
	u64			freq_hold_until_ns;
	u64			auto_boost_until_ns;
	u64			efficiency_until_ns;
	unsigned long		auto_boost_avg_util;
	unsigned int		cpu_signal_ema;
	unsigned int		gpu_signal_ema;
	unsigned int		npu_signal_ema;
	unsigned int		mem_signal_ema;
	unsigned int		fusion_signal_ema;
	unsigned int		thermal_signal_ema;
	unsigned int		thermal_rise_ema;
	int			atlas_cpu_slot;
	unsigned int		atlas_policy_capacity;
	u64			atlas_last_publish_ns;
	bool			has_prime_cpu;

	/* The next fields are only needed if fast switch cannot be used: */
	struct			irq_work irq_work;
	struct			kthread_work work;
	struct			mutex work_lock;
	struct			kthread_worker worker;
	struct task_struct	*thread;
	bool			work_in_progress;

	bool			limits_changed;
	bool			need_freq_update;
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	u64			hispeed_validate_time;
	u64			update_time;
	/* used to detect freq locked */
	ktime_t			start_time;
	bool			freq_locked;
	unsigned int		min_freq;
	bool			after_limits_changed;
#endif
};

struct sugov_cpu {
	struct update_util_data	update_util;
	struct sugov_policy	*sg_policy;
	unsigned int		cpu;

	bool			iowait_boost_pending;
	unsigned int		iowait_boost;
	u64			last_update;

	struct sched_walt_cpu_load walt_load;

	unsigned long util;
	unsigned int flags;

	unsigned long		bw_dl;
	unsigned long		min;
	unsigned long		max;

	/* The field below is for single-CPU policies only: */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long		saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct sugov_cpu, sugov_cpu);
static unsigned int stale_ns;
static DEFINE_PER_CPU(struct sugov_tunables *, cached_tunables);



/*
 * Producers update their own slot in the cpufreq callback.  Publication is
 * rate limited and is the only writer of atlas_snapshot, so a snapshot never
 * combines fields from two producer updates.  Do not take atlas_snapshot_lock
 * from a cpufreq callback.
 */
struct atlas_raw_policy_telemetry {
	seqcount_t seq;
	unsigned int util_pct, freq_khz, thermal_pct, capacity;
	u64 timestamp_ns;
	bool active;
};
static DEFINE_SEQLOCK(atlas_snapshot_lock);
static struct atlas_raw_policy_telemetry atlas_raw_cpu[ATLAS_MAX_CPU_POLICIES];
static cpumask_t atlas_policy_masks[ATLAS_MAX_CPU_POLICIES];
static struct atlas_telemetry_snapshot atlas_snapshot = {
	.display_active = true,
};
static struct delayed_work atlas_mem_work;
static atomic_t atlas_active_policies = ATOMIC_INIT(0);
static bool atlas_mem_work_initialized;
static unsigned long atlas_mem_prev_pgscan;
static unsigned long atlas_mem_prev_pswpout;
static unsigned long atlas_mem_prev_refault;
static u64 atlas_mem_prev_sample_ns;
static unsigned int atlas_mem_prev_interval_ms;
static bool atlas_mem_baseline_valid;
#define ATLAS_MEM_ACTIVE_MS 32
#define ATLAS_MEM_IDLE_MS 250
#define ATLAS_CPU_PUBLISH_NS (6 * NSEC_PER_MSEC)

static void atlas_publish_domain_timestamp(u64 *timestamp)
{
	u64 now = ktime_get_ns();

	*timestamp = now;
	atlas_snapshot.timestamp_ns = now;
	atlas_snapshot.seq++;
}

static void atlas_recompute_cpu_aggregate(void)
{
	unsigned int i, max_util = 0, max_thermal = 0;
	u64 weighted_freq = 0, weight = 0;

	for (i = 0; i < atlas_snapshot.nr_cpu_policies; i++) {
		struct atlas_cpu_policy_telemetry *cpu = &atlas_snapshot.cpu[i];
		struct atlas_raw_policy_telemetry *raw = &atlas_raw_cpu[i];
		unsigned int seq, cap;

		do {
			seq = read_seqcount_begin(&raw->seq);
			cpu->util_pct = READ_ONCE(raw->util_pct);
			cpu->freq_khz = READ_ONCE(raw->freq_khz);
			cpu->thermal_pct = READ_ONCE(raw->thermal_pct);
			cpu->capacity = READ_ONCE(raw->capacity);
			cpu->timestamp_ns = READ_ONCE(raw->timestamp_ns);
			cpu->active = READ_ONCE(raw->active);
		} while (read_seqcount_retry(&raw->seq, seq));
		if (!cpu->active)
			continue;
		cap = cpu->capacity ?: 1;
		max_util = max(max_util, (unsigned int) cpu->util_pct);
		max_thermal = max(max_thermal, (unsigned int) cpu->thermal_pct);
		weighted_freq += (u64) cpu->freq_khz * cap;
		weight += cap;
	}
	atlas_snapshot.cpu_util_pct = max_util;
	atlas_snapshot.cpu_freq_khz = weight ? div64_u64(weighted_freq, weight) : 0;
	atlas_snapshot.cpu_thermal_pct = max_thermal;
}

static int atlas_find_cpu_policy_slot(const struct cpumask *cpus)
{
	unsigned int i;

	for (i = 0; i < atlas_snapshot.nr_cpu_policies; i++)
		if (cpumask_equal(&atlas_policy_masks[i], cpus))
			return i;
	if (atlas_snapshot.nr_cpu_policies >= ATLAS_MAX_CPU_POLICIES)
		return -ENOSPC;
	return atlas_snapshot.nr_cpu_policies++;
}

static unsigned int atlas_policy_capacity(const struct cpumask *cpus)
{
	unsigned int cpu, capacity = 1;

	for_each_cpu(cpu, cpus)
		capacity = max_t(unsigned int, capacity,
			arch_scale_cpu_capacity(NULL, cpu));

	return capacity;
}

static int atlas_register_cpu_policy(const struct cpumask *cpus,
				     unsigned int capacity)
{
	int slot;

	if (!cpus)
		return -EINVAL;

	write_seqlock(&atlas_snapshot_lock);
	slot = atlas_find_cpu_policy_slot(cpus);
	if (slot >= 0) {
		struct atlas_cpu_policy_telemetry *cpu = &atlas_snapshot.cpu[slot];

		cpumask_copy(&atlas_policy_masks[slot], cpus);
		seqcount_init(&atlas_raw_cpu[slot].seq);
		write_seqcount_begin(&atlas_raw_cpu[slot].seq);
		atlas_raw_cpu[slot].util_pct = 0;
		atlas_raw_cpu[slot].freq_khz = 0;
		atlas_raw_cpu[slot].thermal_pct = 0;
		atlas_raw_cpu[slot].capacity = capacity ?: 1;
		atlas_raw_cpu[slot].timestamp_ns = ktime_get_ns();
		atlas_raw_cpu[slot].active = true;
		write_seqcount_end(&atlas_raw_cpu[slot].seq);
		cpu->util_pct = 0;
		cpu->freq_khz = 0;
		cpu->thermal_pct = 0;
		cpu->capacity = capacity ?: 1;
		cpu->timestamp_ns = atlas_raw_cpu[slot].timestamp_ns;
		cpu->active = true;
		atlas_recompute_cpu_aggregate();
		atlas_publish_domain_timestamp(&atlas_snapshot.cpu_timestamp_ns);
	}
	write_sequnlock(&atlas_snapshot_lock);

	if (slot >= 0 && atomic_inc_return(&atlas_active_policies) == 1 &&
	    atlas_mem_work_initialized)
		mod_delayed_work(system_power_efficient_wq, &atlas_mem_work, 0);

	return slot;
}

static void atlas_unregister_cpu_policy(int slot)
{
	if (slot < 0 || slot >= ATLAS_MAX_CPU_POLICIES)
		return;

	write_seqlock(&atlas_snapshot_lock);
	write_seqcount_begin(&atlas_raw_cpu[slot].seq);
	atlas_raw_cpu[slot].active = false;
	atlas_raw_cpu[slot].util_pct = 0;
	atlas_raw_cpu[slot].freq_khz = 0;
	atlas_raw_cpu[slot].thermal_pct = 0;
	atlas_raw_cpu[slot].timestamp_ns = ktime_get_ns();
	write_seqcount_end(&atlas_raw_cpu[slot].seq);
	atlas_recompute_cpu_aggregate();
	atlas_publish_domain_timestamp(&atlas_snapshot.cpu_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);

	if (atomic_dec_and_test(&atlas_active_policies) && atlas_mem_work_initialized) {
		cancel_delayed_work_sync(&atlas_mem_work);
		atlas_mem_baseline_valid = false;
	}
}

static void atlas_update_cpu_policy_slot(int slot, unsigned int util_pct,
					 unsigned int freq_khz,
					 unsigned int thermal_pct)
{
	struct atlas_raw_policy_telemetry *raw;

	if (slot < 0 || slot >= ATLAS_MAX_CPU_POLICIES)
		return;
	raw = &atlas_raw_cpu[slot];
	if (!READ_ONCE(raw->active))
		return;
	write_seqcount_begin(&raw->seq);
	raw->util_pct = min_t(unsigned int, util_pct, 100);
	raw->freq_khz = freq_khz;
	raw->thermal_pct = min_t(unsigned int, thermal_pct, 100);
	raw->timestamp_ns = ktime_get_ns();
	write_seqcount_end(&raw->seq);
}

static void atlas_maybe_publish_cpu_policy(int slot, u64 time, u64 *last_publish)
{
	if (slot < 0 || slot >= ATLAS_MAX_CPU_POLICIES)
		return;
	if (*last_publish && time - *last_publish < ATLAS_CPU_PUBLISH_NS)
		return;

	write_seqlock(&atlas_snapshot_lock);
	atlas_recompute_cpu_aggregate();
	atlas_publish_domain_timestamp(&atlas_snapshot.cpu_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
	*last_publish = time;
}

void atlas_get_snapshot(struct atlas_telemetry_snapshot *snapshot)
{
	unsigned int seq;

	if (!snapshot)
		return;
	do {
		seq = read_seqbegin(&atlas_snapshot_lock);
		memcpy(snapshot, &atlas_snapshot, sizeof(*snapshot));
	} while (read_seqretry(&atlas_snapshot_lock, seq));
}
EXPORT_SYMBOL_GPL(atlas_get_snapshot);

void atlas_update_gpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct)
{
	write_seqlock(&atlas_snapshot_lock);
	atlas_snapshot.gpu_util_pct = min_t(unsigned int, util_pct, 100);
	atlas_snapshot.gpu_freq_khz = freq_khz;
	atlas_snapshot.gpu_thermal_pct = min_t(unsigned int, thermal_pct, 100);
	atlas_publish_domain_timestamp(&atlas_snapshot.gpu_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_gpu_telemetry);

void atlas_get_gpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	if (util_pct) *util_pct = s.gpu_util_pct;
	if (freq_khz) *freq_khz = s.gpu_freq_khz;
	if (thermal_pct) *thermal_pct = s.gpu_thermal_pct;
}
EXPORT_SYMBOL_GPL(atlas_get_gpu_telemetry);

void atlas_update_npu_telemetry(unsigned int util_pct, unsigned int bw_kbps,
				unsigned int thermal_pct)
{
	write_seqlock(&atlas_snapshot_lock);
	atlas_snapshot.npu_util_pct = min_t(unsigned int, util_pct, 100);
	atlas_snapshot.npu_bw_kbps = bw_kbps;
	atlas_snapshot.npu_thermal_pct = min_t(unsigned int, thermal_pct, 100);
	atlas_publish_domain_timestamp(&atlas_snapshot.npu_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_npu_telemetry);

void atlas_get_npu_telemetry(unsigned int *util_pct, unsigned int *bw_kbps,
			     unsigned int *thermal_pct)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	if (util_pct) *util_pct = s.npu_util_pct;
	if (bw_kbps) *bw_kbps = s.npu_bw_kbps;
	if (thermal_pct) *thermal_pct = s.npu_thermal_pct;
}
EXPORT_SYMBOL_GPL(atlas_get_npu_telemetry);

void atlas_update_cpu_policy_telemetry(const struct cpumask *cpus,
				       unsigned int util_pct,
				       unsigned int freq_khz,
				       unsigned int thermal_pct,
				       unsigned int capacity)
{
	int slot;

	if (!cpus)
		return;
	write_seqlock(&atlas_snapshot_lock);
	slot = atlas_find_cpu_policy_slot(cpus);
	if (slot >= 0) {
		cpumask_copy(&atlas_policy_masks[slot], cpus);
		write_seqcount_begin(&atlas_raw_cpu[slot].seq);
		atlas_raw_cpu[slot].util_pct = min_t(unsigned int, util_pct, 100);
		atlas_raw_cpu[slot].freq_khz = freq_khz;
		atlas_raw_cpu[slot].thermal_pct = min_t(unsigned int, thermal_pct, 100);
		atlas_raw_cpu[slot].capacity = capacity ?: 1;
		atlas_raw_cpu[slot].timestamp_ns = ktime_get_ns();
		atlas_raw_cpu[slot].active = true;
		write_seqcount_end(&atlas_raw_cpu[slot].seq);
		atlas_recompute_cpu_aggregate();
		atlas_publish_domain_timestamp(&atlas_snapshot.cpu_timestamp_ns);
	}
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_cpu_policy_telemetry);

void atlas_update_cpu_telemetry(unsigned int util_pct, unsigned int freq_khz,
				unsigned int thermal_pct)
{
	write_seqlock(&atlas_snapshot_lock);
	if (!atomic_read(&atlas_active_policies)) {
		atlas_snapshot.cpu_util_pct = min_t(unsigned int, util_pct, 100);
		atlas_snapshot.cpu_freq_khz = freq_khz;
		atlas_snapshot.cpu_thermal_pct = min_t(unsigned int, thermal_pct, 100);
		atlas_publish_domain_timestamp(&atlas_snapshot.cpu_timestamp_ns);
	}
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_cpu_telemetry);

void atlas_get_cpu_telemetry(unsigned int *util_pct, unsigned int *freq_khz,
			     unsigned int *thermal_pct)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	if (util_pct) *util_pct = s.cpu_util_pct;
	if (freq_khz) *freq_khz = s.cpu_freq_khz;
	if (thermal_pct) *thermal_pct = s.cpu_thermal_pct;
}
EXPORT_SYMBOL_GPL(atlas_get_cpu_telemetry);

void atlas_update_mem_telemetry(unsigned int pressure_pct,
				unsigned int contention_pct)
{
	write_seqlock(&atlas_snapshot_lock);
	atlas_snapshot.mem_pressure_pct = min_t(unsigned int, pressure_pct, 100);
	atlas_snapshot.mem_contention_pct = min_t(unsigned int, contention_pct, 100);
	atlas_publish_domain_timestamp(&atlas_snapshot.mem_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_mem_telemetry);

void atlas_get_mem_telemetry(unsigned int *pressure_pct,
			     unsigned int *contention_pct)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	if (pressure_pct) *pressure_pct = s.mem_pressure_pct;
	if (contention_pct) *contention_pct = s.mem_contention_pct;
}
EXPORT_SYMBOL_GPL(atlas_get_mem_telemetry);

void atlas_update_mem_stats(unsigned int reclaim_pct, unsigned int swap_pct,
			    unsigned int workingset_refault_pct)
{
	write_seqlock(&atlas_snapshot_lock);
	atlas_snapshot.mem_reclaim_pct = min_t(unsigned int, reclaim_pct, 100);
	atlas_snapshot.mem_swap_pct = min_t(unsigned int, swap_pct, 100);
	atlas_snapshot.mem_workingset_refault_pct = min_t(unsigned int, workingset_refault_pct, 100);
	atlas_publish_domain_timestamp(&atlas_snapshot.mem_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
}
EXPORT_SYMBOL_GPL(atlas_update_mem_stats);

void atlas_get_mem_stats(unsigned int *reclaim_pct, unsigned int *swap_pct,
			 unsigned int *workingset_refault_pct)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	if (reclaim_pct) *reclaim_pct = s.mem_reclaim_pct;
	if (swap_pct) *swap_pct = s.mem_swap_pct;
	if (workingset_refault_pct) *workingset_refault_pct = s.mem_workingset_refault_pct;
}
EXPORT_SYMBOL_GPL(atlas_get_mem_stats);

void atlas_update_display_state(bool active)
{
	write_seqlock(&atlas_snapshot_lock);
	atlas_snapshot.display_active = active;
	atlas_publish_domain_timestamp(&atlas_snapshot.display_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);
	if (atlas_mem_work_initialized && atomic_read(&atlas_active_policies))
		mod_delayed_work(system_power_efficient_wq, &atlas_mem_work, 0);
}
EXPORT_SYMBOL_GPL(atlas_update_display_state);

bool atlas_display_state_active(void)
{
	struct atlas_telemetry_snapshot s;

	atlas_get_snapshot(&s);
	return s.display_active;
}
EXPORT_SYMBOL_GPL(atlas_display_state_active);

static unsigned int atlas_delta_pct(unsigned long now, unsigned long prev,
					 unsigned int shift, u64 elapsed_ns)
{
	u64 delta;

	if (now <= prev || !elapsed_ns)
		return 0;

	delta = now - prev;
	delta = div64_u64(delta * ATLAS_MEM_ACTIVE_MS * NSEC_PER_MSEC,
			elapsed_ns);
	return min_t(unsigned int, 100, delta >> shift);
}

static void atlas_mem_sample_work(struct work_struct *work)
{
	long mem_total = totalram_pages, mem_avail;
	unsigned long pgscan_now, pswpout_now, refault_now;
	unsigned long vm_events[NR_VM_EVENT_ITEMS] = { 0 };
	unsigned int pressure, contention, reclaim, swap, refault;
	unsigned int interval_ms;
	bool active;
	u64 now, elapsed_ns;

	if (!atomic_read(&atlas_active_policies))
		return;

	mem_avail = si_mem_available();
	pressure = (!mem_total || mem_avail >= mem_total) ? 0 :
		100 - mult_frac(mem_avail, 100, mem_total);
	pgscan_now = global_node_page_state(NR_VMSCAN_WRITE) +
		global_node_page_state(NR_VMSCAN_IMMEDIATE);
	all_vm_events(vm_events);
	pswpout_now = vm_events[PSWPOUT];
	refault_now = global_node_page_state(WORKINGSET_REFAULT);
	now = ktime_get_ns();

	write_seqlock(&atlas_snapshot_lock);
	active = atlas_snapshot.display_active;
	interval_ms = active ? ATLAS_MEM_ACTIVE_MS : ATLAS_MEM_IDLE_MS;
	if (!atlas_mem_baseline_valid ||
	    atlas_mem_prev_interval_ms != interval_ms) {
		reclaim = 0;
		swap = 0;
		refault = 0;
		atlas_mem_baseline_valid = true;
	} else {
		elapsed_ns = now - atlas_mem_prev_sample_ns;
		reclaim = atlas_delta_pct(pgscan_now, atlas_mem_prev_pgscan, 7,
					 elapsed_ns);
		swap = atlas_delta_pct(pswpout_now, atlas_mem_prev_pswpout, 4,
				      elapsed_ns);
		refault = atlas_delta_pct(refault_now, atlas_mem_prev_refault, 5,
					 elapsed_ns);
	}
	atlas_mem_prev_pgscan = pgscan_now;
	atlas_mem_prev_pswpout = pswpout_now;
	atlas_mem_prev_refault = refault_now;
	atlas_mem_prev_sample_ns = now;
	atlas_mem_prev_interval_ms = interval_ms;
	contention = min_t(unsigned int, 100,
		((atlas_snapshot.cpu_util_pct + atlas_snapshot.gpu_util_pct) / 2) +
		(atlas_snapshot.gpu_util_pct >= 50 ? 10 : 0));
	atlas_snapshot.mem_pressure_pct = pressure;
	atlas_snapshot.mem_contention_pct = contention;
	atlas_snapshot.mem_reclaim_pct = reclaim;
	atlas_snapshot.mem_swap_pct = swap;
	atlas_snapshot.mem_workingset_refault_pct = refault;
	atlas_publish_domain_timestamp(&atlas_snapshot.mem_timestamp_ns);
	write_sequnlock(&atlas_snapshot_lock);

	if (atomic_read(&atlas_active_policies))
		queue_delayed_work(system_power_efficient_wq, &atlas_mem_work,
			msecs_to_jiffies(interval_ms));
}

static unsigned int atlas_cpu_thermal_pct_for_cpu(int cpu)
{
	unsigned long max_cap = arch_scale_cpu_capacity(NULL, cpu);
	unsigned long therm_cap = thermal_cap(cpu);

	if (!max_cap || therm_cap >= max_cap)
		return 0;

	return mult_frac(max_cap - therm_cap, 100, max_cap);
}

/************************ Governor internals ***********************/

static bool sugov_should_update_freq(struct sugov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	/*
	 * Since cpufreq_update_util() is called with rq->lock held for
	 * the @target_cpu, our per-CPU data is fully serialized.
	 *
	 * However, drivers cannot in general deal with cross-CPU
	 * requests, so while get_next_freq() will work, our
	 * sugov_update_commit() call may not for the fast switching platforms.
	 *
	 * Hence stop here for remote requests if they aren't supported
	 * by the hardware, as calculating the frequency is pointless if
	 * we cannot in fact act on it.
	 *
	 * This is needed on the slow switching platforms too to prevent CPUs
	 * going offline from leaving stale IRQ work items behind.
	 */
	if (!cpufreq_this_cpu_can_update(sg_policy->policy))
		return false;

	if (unlikely(READ_ONCE(sg_policy->limits_changed))) {
		WRITE_ONCE(sg_policy->limits_changed, false);
		sg_policy->need_freq_update = true;

		/*
		 * The above limits_changed update must occur before the reads
		 * of policy limits in cpufreq_driver_resolve_freq() or a policy
		 * limits update might be missed, so use a memory barrier to
		 * ensure it.
		 *
		 * This pairs with the write memory barrier in sugov_limits().
		 */
		smp_mb();

		return true;
	}

	/* No need to recalculate next freq for min_rate_limit_us
	 * at least. However we might still decide to further rate
	 * limit once frequency change direction is decided, according
	 * to the separate rate limits.
	 */

	delta_ns = time - sg_policy->last_freq_update_time;
	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static inline bool use_pelt(void)
{
#ifdef CONFIG_SCHED_WALT
	return false;
#else
	return true;
#endif
}

static inline bool conservative_pl(void)
{
#ifdef CONFIG_SCHED_WALT
	return sysctl_sched_conservative_pl;
#else
	return false;
#endif
}

static void sugov_clear_display_off_boosts(struct sugov_policy *sg_policy)
{
	sg_policy->freq_hold_until_ns = 0;
	sg_policy->auto_boost_until_ns = 0;
	sg_policy->efficiency_until_ns = 0;
	sg_policy->auto_boost_avg_util = 0;
	sg_policy->cpu_signal_ema = 0;
	sg_policy->gpu_signal_ema = 0;
	sg_policy->npu_signal_ema = 0;
	sg_policy->mem_signal_ema = 0;
	sg_policy->fusion_signal_ema = 0;
	sg_policy->thermal_signal_ema = 0;
	sg_policy->thermal_rise_ema = 0;
}

static bool sugov_up_down_rate_limit(struct sugov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	if (!atlas_display_state_active()) {
		sugov_clear_display_off_boosts(sg_policy);
		return false;
	}

	delta_ns = time - sg_policy->last_freq_update_time;

	if (next_freq > sg_policy->next_freq)
		return false;

	if (next_freq < sg_policy->next_freq && time < sg_policy->freq_hold_until_ns)
		return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
			return true;

	return false;
}

static inline bool sugov_is_transition_event(unsigned int flags)
{
	return flags & (SCHED_CPUFREQ_EARLY_DET |
			SCHED_CPUFREQ_MIGRATION |
			SCHED_CPUFREQ_INTERCLUSTER_MIG);
}

static inline void sugov_note_transition_boost(struct sugov_policy *sg_policy,
					      u64 time, unsigned int flags)
{
	if (!atlas_display_state_active() || !sugov_is_transition_event(flags))
		return;

	/*
	 * Scene changes tend to generate short migration bursts. Keep the
	 * frequency high for at least one down hysteresis window to avoid
	 * up/down ping-pong that can surface as frame hitching.
	 */
	sg_policy->freq_hold_until_ns = max_t(u64, sg_policy->freq_hold_until_ns,
					    time + sg_policy->down_hyst_ns);
}

static bool sugov_update_next_freq(struct sugov_policy *sg_policy, u64 time,
				   unsigned int next_freq)
{
	if (sg_policy->next_freq == next_freq)
		return false;

	if (sugov_up_down_rate_limit(sg_policy, time, next_freq)) {
		/* Restore cached freq as next_freq is not changed */
		sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
		return false;
	}

	sg_policy->next_freq = next_freq;
	if (next_freq > sg_policy->policy->cur)
		sg_policy->freq_hold_until_ns = time + sg_policy->down_hyst_ns;
	sg_policy->last_freq_update_time = time;

	return true;
}

static unsigned long freq_to_util(struct sugov_policy *sg_policy,
				  unsigned int freq)
{
	return mult_frac(sg_policy->max, freq,
			 sg_policy->policy->cpuinfo.max_freq);
}

#define KHZ 1000
static void sugov_track_cycles(struct sugov_policy *sg_policy,
				unsigned int prev_freq,
				u64 upto)
{
	u64 delta_ns, cycles;
	u64 next_ws = sg_policy->last_ws + sched_ravg_window;

	if (use_pelt())
		return;

	upto = min(upto, next_ws);
	/* Track cycles in current window */
	delta_ns = upto - sg_policy->last_cyc_update_time;
	delta_ns *= prev_freq;
	do_div(delta_ns, (NSEC_PER_SEC / KHZ));
	cycles = delta_ns;
	sg_policy->curr_cycles += cycles;
	sg_policy->last_cyc_update_time = upto;
}

static void sugov_calc_avg_cap(struct sugov_policy *sg_policy, u64 curr_ws,
				unsigned int prev_freq)
{
	u64 last_ws = sg_policy->last_ws;
	unsigned int avg_freq;

	if (use_pelt())
		return;

	BUG_ON(curr_ws < last_ws);
	if (curr_ws <= last_ws)
		return;

	/* If we skipped some windows */
	if (curr_ws > (last_ws + sched_ravg_window)) {
		avg_freq = prev_freq;
		/* Reset tracking history */
		sg_policy->last_cyc_update_time = curr_ws;
	} else {
		sugov_track_cycles(sg_policy, prev_freq, curr_ws);
		avg_freq = sg_policy->curr_cycles;
		avg_freq /= sched_ravg_window / (NSEC_PER_SEC / KHZ);
	}
	sg_policy->avg_cap = freq_to_util(sg_policy, avg_freq);
	sg_policy->curr_cycles = 0;
	sg_policy->last_ws = curr_ws;
}

static void sugov_fast_switch(struct sugov_policy *sg_policy, u64 time,
			      unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int cpu;

	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	sugov_track_cycles(sg_policy, sg_policy->policy->cur, time);
	next_freq = cpufreq_driver_fast_switch(policy, next_freq);
	if (!next_freq)
		return;

	policy->cur = next_freq;

	if (trace_cpu_frequency_enabled()) {
		for_each_cpu(cpu, policy->cpus)
			trace_cpu_frequency(next_freq, cpu);
	}
}

static void sugov_deferred_update(struct sugov_policy *sg_policy, u64 time,
				  unsigned int next_freq)
{
	if (!sugov_update_next_freq(sg_policy, time, next_freq))
		return;

	if (use_pelt())
		sg_policy->work_in_progress = true;
	irq_work_queue(&sg_policy->irq_work);
}

#define TARGET_LOAD 80

#ifdef OPLUS_FEATURE_POWER_CPUFREQ
static unsigned int freq_to_targetload(
	struct sugov_tunables *tunables, unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&tunables->auto_cfg.target_loads_lock, flags);

	for (i = 0; i < tunables->auto_cfg.ntarget_loads - 1 &&
		     freq >= tunables->auto_cfg.target_loads[i+1]; i += 2)
		;

	ret = tunables->auto_cfg.target_loads[i];
	spin_unlock_irqrestore(&tunables->auto_cfg.target_loads_lock, flags);
	return ret;
}


static unsigned int choose_freq(struct sugov_policy *sg_policy,
		unsigned int loadadjfreq)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = policy->cur;
	unsigned int prevfreq, freqmin, freqmax;
	unsigned int tl;
	int index;

	freqmin = 0;
	freqmax = UINT_MAX;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(sg_policy->tunables, freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		index = cpufreq_frequency_table_target(policy,
						       loadadjfreq / tl,
						       CPUFREQ_RELATION_L);
		freq = policy->freq_table[index].frequency;

		trace_choose_freq(freq, prevfreq, freqmax, freqmin, tl, index);

		if (freq > prevfreq) {
			/* The previous frequency is too low. */
			freqmin = prevfreq;

			if (freq >= freqmax) {
				/*
				 * Find the highest frequency that is less
				 * than freqmax.
				 */
				index = cpufreq_frequency_table_target(
					    policy,
					    freqmax - 1, CPUFREQ_RELATION_H);
				freq = policy->freq_table[index].frequency;

				if (freq == freqmin) {
					/*
					 * The first frequency below freqmax
					 * has already been found to be too
					 * low.  freqmax is the lowest speed
					 * we found that is fast enough.
					 */
					freq = freqmax;
					break;
				}
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq <= freqmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				index = cpufreq_frequency_table_target(
					    policy,
					    freqmin + 1, CPUFREQ_RELATION_L);
				freq = policy->freq_table[index].frequency;

				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				if (freq == freqmax)
					break;
			}
		}

		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

	return freq;
}

static unsigned int freq_to_above_hispeed_delay(struct sugov_tunables *tunables,
						unsigned int freq)
{
	unsigned long flags;
	unsigned int ret;
	int i;

	spin_lock_irqsave(&tunables->auto_cfg.above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->auto_cfg.nabove_hispeed_delay - 1 &&
		freq >= tunables->auto_cfg.above_hispeed_delay[i + 1]; i += 2)
		;

	ret = tunables->auto_cfg.above_hispeed_delay[i];
	spin_unlock_irqrestore(&tunables->auto_cfg.above_hispeed_delay_lock, flags);
	return ret;
}

static bool sugov_time_limit(struct sugov_policy *sg_policy,
				unsigned int next_freq, unsigned int flags)
{
	u64 delta_ns;
	bool skip_hispeed_delay = false;
	unsigned int delay;

	if (flags & SCHED_CPUFREQ_EARLY_DET ||
	    flags & SCHED_CPUFREQ_MIGRATION ||
	    flags & SCHED_CPUFREQ_INTERCLUSTER_MIG)
		skip_hispeed_delay = true;

	if (sg_policy->after_limits_changed) {
		skip_hispeed_delay = true;
		sg_policy->after_limits_changed = false;
	}

	if (!skip_hispeed_delay && next_freq > sg_policy->next_freq &&
	    sg_policy->next_freq >= sg_policy->tunables->auto_cfg.hispeed_freq) {
		delta_ns = sg_policy->update_time -
				sg_policy->hispeed_validate_time;
		delay = freq_to_above_hispeed_delay(sg_policy->tunables,
							sg_policy->next_freq);
		if (delta_ns < NSEC_PER_USEC * delay) {
			trace_sugov_time_limit(cpumask_first(sg_policy->policy->cpus),
					"above_hispeed_delay", delta_ns,
					sg_policy->next_freq, next_freq);
			return true;
		}
	}

	sg_policy->hispeed_validate_time = sg_policy->update_time;
	return false;
}
#endif

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: Atlas policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * If the utilization is frequency-invariant, choose the new frequency to be
 * proportional to it, that is
 *
 * next_freq = C * max_freq * util / max
 *
 * Otherwise, approximate the would-be frequency-invariant utilization by
 * util_raw * (curr_freq / max_freq) which leads to
 *
 * next_freq = C * curr_freq * util_raw / max
 *
 * Take C = 1.25 for the frequency tipping point at (util / max) = 0.8.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct sugov_policy *sg_policy,
				  unsigned long util, unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned int freq = arch_scale_freq_invariant() ?
				policy->cpuinfo.max_freq : policy->cur;

#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	unsigned int prev_freq = freq;
	unsigned int prev_laf;
	u64 loadadjfreq;

	/*
	 * Keep the interactive-style target-load calculation in 64-bit
	 * arithmetic. Modern and overclocked CPU OPPs can be several MHz in
	 * kHz units, so prev_freq * util * 100 easily overflows 32 bits before
	 * choose_freq() gets a chance to resolve the request against the full
	 * cpufreq table. An overflow here wraps the raw target back down and
	 * prevents Atlas from ever asking for the newly exposed top OPPs.
	 */
	loadadjfreq = (u64)prev_freq * util * 100;
	loadadjfreq = div64_u64(loadadjfreq, max);
	prev_laf = min_t(u64, loadadjfreq, UINT_MAX);

	freq = choose_freq(sg_policy, prev_laf);
	trace_sugov_next_freq_tl(policy->cpu, util, max, freq, prev_laf, prev_freq);
#else
	freq = map_util_freq(util, freq, max);
	trace_sugov_next_freq(policy->cpu, util, max, freq);
#endif

	if (freq == sg_policy->cached_raw_freq && !sg_policy->need_freq_update)
		return sg_policy->next_freq;

	sg_policy->need_freq_update = false;
	sg_policy->prev_cached_raw_freq = sg_policy->cached_raw_freq;
	sg_policy->cached_raw_freq = freq;
	return cpufreq_driver_resolve_freq(policy, freq);
}

extern long
schedtune_cpu_margin_with(unsigned long util, int cpu, struct task_struct *p);

/*
 * This function computes an effective utilization for the given CPU, to be
 * used for frequency selection given the linear relation: f = u * f_max.
 *
 * The scheduler tracks the following metrics:
 *
 *   cpu_util_{cfs,rt,dl,irq}()
 *   cpu_bw_dl()
 *
 * Where the cfs,rt and dl util numbers are tracked with the same metric and
 * synchronized windows and are thus directly comparable.
 *
 * The @util parameter passed to this function is assumed to be the aggregation
 * of RT and CFS util numbers. The cases of DL and IRQ are managed here.
 *
 * The cfs,rt,dl utilization are the running times measured with rq->clock_task
 * which excludes things like IRQ and steal-time. These latter are then accrued
 * in the irq utilization.
 *
 * The DL bandwidth number otoh is not a measured metric but a value computed
 * based on the task model parameters and gives the minimal utilization
 * required to meet deadlines.
 */
#ifdef CONFIG_SCHED_WALT
static unsigned long sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);
	unsigned long max = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

	sg_cpu->max = max;
	sg_cpu->bw_dl = cpu_bw_dl(rq);

	return stune_util(sg_cpu->cpu, 0, &sg_cpu->walt_load);
}
#else
static unsigned long sugov_get_util(struct sugov_cpu *sg_cpu)
{
	struct rq *rq = cpu_rq(sg_cpu->cpu);

	unsigned long util_cfs = cpu_util_cfs(rq);
	unsigned long max = arch_scale_cpu_capacity(NULL, sg_cpu->cpu);

	sg_cpu->max = max;
	sg_cpu->bw_dl = cpu_bw_dl(rq);

	return schedutil_cpu_util(sg_cpu->cpu, util_cfs, max,
				  FREQUENCY_UTIL, NULL);
}
#endif

/**
 * sugov_iowait_reset() - Reset the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @set_iowait_boost: true if an IO boost has been requested
 *
 * The IO wait boost of a task is disabled after a tick since the last update
 * of a CPU. If a new IO wait boost is requested after more then a tick, then
 * we enable the boost starting from the minimum frequency, which improves
 * energy efficiency by ignoring sporadic wakeups from IO.
 */
static bool sugov_iowait_reset(struct sugov_cpu *sg_cpu, u64 time,
			       bool set_iowait_boost)
{
	s64 delta_ns = time - sg_cpu->last_update;

	/* Reset boost only if a tick has elapsed since last request */
	if (delta_ns <= TICK_NSEC)
		return false;

	sg_cpu->iowait_boost = set_iowait_boost ? sg_cpu->min : 0;
	sg_cpu->iowait_boost_pending = set_iowait_boost;

	return true;
}

/**
 * sugov_iowait_boost() - Updates the IO boost status of a CPU.
 * @sg_cpu: the sugov data for the CPU to boost
 * @time: the update time from the caller
 * @flags: SCHED_CPUFREQ_IOWAIT if the task is waking up after an IO wait
 *
 * Each time a task wakes up after an IO operation, the CPU utilization can be
 * boosted to a certain utilization which doubles at each "frequent and
 * successive" wakeup from IO, ranging from the utilization of the minimum
 * OPP to the utilization of the maximum OPP.
 * To keep doubling, an IO boost has to be requested at least once per tick,
 * otherwise we restart from the utilization of the minimum OPP.
 */
static void sugov_iowait_boost(struct sugov_cpu *sg_cpu, u64 time,
			       unsigned int flags)
{
	bool set_iowait_boost = flags & SCHED_CPUFREQ_IOWAIT;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sg_cpu->iowait_boost &&
	    sugov_iowait_reset(sg_cpu, time, set_iowait_boost))
		return;

	/* Boost only tasks waking up after IO */
	if (!set_iowait_boost)
		return;

	/* Ensure boost doubles only one time at each request */
	if (sg_cpu->iowait_boost_pending)
		return;
	sg_cpu->iowait_boost_pending = true;

	/* Double the boost at each request */
	if (sg_cpu->iowait_boost) {
		sg_cpu->iowait_boost =
			min_t(unsigned int, sg_cpu->iowait_boost << 1, SCHED_CAPACITY_SCALE);
		return;
	}

	/* First wakeup after IO: start with minimum boost */
	sg_cpu->iowait_boost = sg_cpu->min;
}

/**
 * sugov_iowait_apply() - Apply the IO boost to a CPU.
 * @sg_cpu: the sugov data for the cpu to boost
 * @time: the update time from the caller
 * @util: the utilization to (eventually) boost
 * @max: the maximum value the utilization can be boosted to
 *
 * A CPU running a task which woken up after an IO operation can have its
 * utilization boosted to speed up the completion of those IO operations.
 * The IO boost value is increased each time a task wakes up from IO, in
 * sugov_iowait_apply(), and it's instead decreased by this function,
 * each time an increase has not been requested (!iowait_boost_pending).
 *
 * A CPU which also appears to have been idle for at least one tick has also
 * its IO boost utilization reset.
 *
 * This mechanism is designed to boost high frequently IO waiting tasks, while
 * being more conservative on tasks which does sporadic IO operations.
 */
static unsigned long sugov_iowait_apply(struct sugov_cpu *sg_cpu, u64 time,
					unsigned long util, unsigned long max)
{
	unsigned long boost;

	/* No boost currently required */
	if (!sg_cpu->iowait_boost)
		return util;

	/* Reset boost if the CPU appears to have been idle enough */
	if (sugov_iowait_reset(sg_cpu, time, false))
		return util;

	if (!sg_cpu->iowait_boost_pending) {
		/*
		 * No boost pending; reduce the boost value.
		 */
		sg_cpu->iowait_boost >>= 1;
		if (sg_cpu->iowait_boost < sg_cpu->min) {
			sg_cpu->iowait_boost = 0;
			return util;
		}
	}

	sg_cpu->iowait_boost_pending = false;

	/*
	 * @util is already in capacity scale; convert iowait_boost
	 * into the same scale so we can compare.
	 */
	boost = (sg_cpu->iowait_boost * max) >> SCHED_CAPACITY_SHIFT;
	return max(boost, util);
}

#ifdef CONFIG_NO_HZ_COMMON
static bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls_cpu(sg_cpu->cpu);
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool sugov_cpu_is_busy(struct sugov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

#define NL_RATIO 75
#define DEFAULT_HISPEED_LOAD 82
#define DEFAULT_CPU0_RTG_BOOST_FREQ 1600000
#define DEFAULT_CPU4_RTG_BOOST_FREQ 1600000
#define DEFAULT_CPU7_RTG_BOOST_FREQ 1600000
#define DEFAULT_AUTO_BOOST_HIGH_LOAD_WALT 86
#define DEFAULT_AUTO_BOOST_LOW_LOAD_WALT 58
#define DEFAULT_AUTO_BOOST_MIN_UTIL_WALT 96
#define DEFAULT_AUTO_BOOST_MAX_UTIL_WALT 224
#define DEFAULT_AUTO_BOOST_DECAY_US_WALT 8000
#define DEFAULT_AUTO_BOOST_HIGH_LOAD_PELT 90
#define DEFAULT_AUTO_BOOST_LOW_LOAD_PELT 65
#define DEFAULT_AUTO_BOOST_MIN_UTIL_PELT 64
#define DEFAULT_AUTO_BOOST_MAX_UTIL_PELT 192
#define DEFAULT_AUTO_BOOST_DECAY_US_PELT 8000
#define DEFAULT_AUTO_BOOST_HEAVY_TASKS 4
#define DEFAULT_AUTO_BOOST_HEAVY_UTIL 92
#define DEFAULT_AUTO_BOOST_PRIME_UTIL 300
#define DEFAULT_AUTO_BOOST_GOLD_UTIL 220
#define DEFAULT_AUTO_BOOST_EFFICIENCY_LOAD 35
#define DEFAULT_AUTO_BOOST_EFFICIENCY_UTIL 128

enum sugov_auto_profile {
	SUGOV_AUTO_PROFILE_MANUAL = 0,
	SUGOV_AUTO_PROFILE_BALANCED = 1,
	SUGOV_AUTO_PROFILE_BATTERY = 2,
	SUGOV_AUTO_PROFILE_PERFORMANCE = 3,
};

static void sugov_apply_auto_profile(struct sugov_tunables *tunables,
				     unsigned int profile)
{
	bool pelt = use_pelt();

	tunables->auto_cfg.auto_boost = true;
	tunables->auto_cfg.uclamp_helper = false;
	tunables->auto_cfg.auto_boost_heavy_tasks = DEFAULT_AUTO_BOOST_HEAVY_TASKS;
	tunables->auto_cfg.auto_boost_heavy_util = DEFAULT_AUTO_BOOST_HEAVY_UTIL;
	tunables->auto_cfg.auto_boost_prime_util = DEFAULT_AUTO_BOOST_PRIME_UTIL;
	tunables->auto_cfg.auto_boost_gold_util = DEFAULT_AUTO_BOOST_GOLD_UTIL;
	tunables->auto_cfg.auto_boost_efficiency_load =
		DEFAULT_AUTO_BOOST_EFFICIENCY_LOAD;
	tunables->auto_cfg.auto_boost_efficiency_util =
		DEFAULT_AUTO_BOOST_EFFICIENCY_UTIL;

	if (pelt) {
		tunables->auto_cfg.down_rate_limit_us = 1000;
		tunables->auto_cfg.auto_boost_high_load = DEFAULT_AUTO_BOOST_HIGH_LOAD_PELT;
		tunables->auto_cfg.auto_boost_low_load = DEFAULT_AUTO_BOOST_LOW_LOAD_PELT;
		tunables->auto_cfg.auto_boost_min_util = DEFAULT_AUTO_BOOST_MIN_UTIL_PELT;
		tunables->auto_cfg.auto_boost_max_util = DEFAULT_AUTO_BOOST_MAX_UTIL_PELT;
		tunables->auto_cfg.auto_boost_decay_us = DEFAULT_AUTO_BOOST_DECAY_US_PELT;
	} else {
		tunables->auto_cfg.down_rate_limit_us = 2000;
		tunables->auto_cfg.auto_boost_high_load = DEFAULT_AUTO_BOOST_HIGH_LOAD_WALT;
		tunables->auto_cfg.auto_boost_low_load = DEFAULT_AUTO_BOOST_LOW_LOAD_WALT;
		tunables->auto_cfg.auto_boost_min_util = DEFAULT_AUTO_BOOST_MIN_UTIL_WALT;
		tunables->auto_cfg.auto_boost_max_util = DEFAULT_AUTO_BOOST_MAX_UTIL_WALT;
		tunables->auto_cfg.auto_boost_decay_us = DEFAULT_AUTO_BOOST_DECAY_US_WALT;
	}

	switch (profile) {
	case SUGOV_AUTO_PROFILE_BATTERY:
		tunables->auto_cfg.auto_boost_high_load = min(100U,
			tunables->auto_cfg.auto_boost_high_load + 4);
		tunables->auto_cfg.auto_boost_low_load = min(tunables->auto_cfg.auto_boost_high_load,
			tunables->auto_cfg.auto_boost_low_load + 4);
		tunables->auto_cfg.auto_boost_min_util = mult_frac(
			tunables->auto_cfg.auto_boost_min_util, 9, 10);
		tunables->auto_cfg.auto_boost_max_util = mult_frac(
			tunables->auto_cfg.auto_boost_max_util, 9, 10);
		tunables->auto_cfg.auto_boost_decay_us = max(1000U,
			mult_frac(tunables->auto_cfg.auto_boost_decay_us, 3, 4));
		break;
	case SUGOV_AUTO_PROFILE_PERFORMANCE:
		tunables->auto_cfg.auto_boost_high_load = max(1U,
			tunables->auto_cfg.auto_boost_high_load - 4);
		tunables->auto_cfg.auto_boost_low_load = max(1U,
			tunables->auto_cfg.auto_boost_low_load - 4);
		tunables->auto_cfg.auto_boost_min_util = min(1024U, mult_frac(
			tunables->auto_cfg.auto_boost_min_util, 11, 10));
		tunables->auto_cfg.auto_boost_max_util = min(1024U, mult_frac(
			tunables->auto_cfg.auto_boost_max_util, 11, 10));
		tunables->auto_cfg.auto_boost_decay_us = max(tunables->auto_cfg.auto_boost_decay_us,
			12000U);
		break;
	case SUGOV_AUTO_PROFILE_BALANCED:
	default:
		break;
	}
}

static bool sugov_policy_has_prime_cpu(struct sugov_policy *sg_policy)
{
	unsigned long policy_max_cap = 0, system_max_cap = 0;
	unsigned int cpu;

	for_each_possible_cpu(cpu)
		system_max_cap = max(system_max_cap,
			arch_scale_cpu_capacity(NULL, cpu));

	for_each_cpu(cpu, sg_policy->policy->cpus)
		policy_max_cap = max(policy_max_cap,
			arch_scale_cpu_capacity(NULL, cpu));

	return policy_max_cap && (policy_max_cap == system_max_cap);
}
static void sugov_walt_adjust(struct sugov_cpu *sg_cpu, unsigned long *util,
			      unsigned long *max)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	bool is_migration = sg_cpu->flags & SCHED_CPUFREQ_INTERCLUSTER_MIG;
	bool is_rtg_boost = sg_cpu->walt_load.rtgb_active;
	unsigned long nl = sg_cpu->walt_load.nl;
	unsigned long cpu_util = sg_cpu->util;
	bool is_hiload;
	unsigned long pl = sg_cpu->walt_load.pl;

	if (use_pelt())
		return;

	if (is_rtg_boost)
		*util = max(*util, sg_policy->rtg_boost_util);

	is_hiload = (cpu_util >= mult_frac(sg_policy->avg_cap,
					   sg_policy->tunables->auto_cfg.hispeed_load,
					   100));

	if (is_hiload && !is_migration)
		*util = max(*util, sg_policy->hispeed_util);

	if (is_hiload && nl >= mult_frac(cpu_util, NL_RATIO, 100))
		*util = *max;

	if (sg_policy->tunables->auto_cfg.pl) {
		if (conservative_pl())
			pl = mult_frac(pl, TARGET_LOAD, 100);
		*util = max(*util, pl);
	}
}

/*
 * Make sugov_should_update_freq() ignore the rate limit when DL
 * has increased the utilization.
 */
static inline void ignore_dl_rate_limit(struct sugov_cpu *sg_cpu, struct sugov_policy *sg_policy)
{
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl) {
		sg_policy->limits_changed = true;
		sg_policy->after_limits_changed = true;
	}
#else
	if (cpu_bw_dl(cpu_rq(sg_cpu->cpu)) > sg_cpu->bw_dl)
		WRITE_ONCE(sg_policy->limits_changed, true);
#endif
}

static inline unsigned long target_util(struct sugov_policy *sg_policy,
				  unsigned int freq)
{
	unsigned long util;

	util = freq_to_util(sg_policy, freq);
	util = mult_frac(util, TARGET_LOAD, 100);
	return util;
}

static void sugov_apply_tunable_boosts(struct sugov_cpu *sg_cpu, u64 time,
				       unsigned int flags,
				       unsigned long *util,
				       unsigned long max)
{
	(void)time;
	(void)flags;

	/* WALT path does not pass through schedutil_cpu_util(). */
	if (!use_pelt())
		*util = min(max, *util +
			schedtune_cpu_margin_with(*util, sg_cpu->cpu, NULL));
}

static void sugov_apply_auto_boost(struct sugov_policy *sg_policy, u64 time,
				   unsigned int flags, unsigned int nr_running,
				   unsigned long *util,
				   unsigned long max)
{
	struct sugov_auto_cfg *auto_cfg = &sg_policy->tunables->auto_cfg;
	unsigned long avg_util, floor_util, cap_util, min_cap_util;
	unsigned int util_pct, cpu_signal, gpu_signal, npu_signal, mem_signal;
	unsigned int shared_mem_contention_pct;
	unsigned int gpu_util_pct, gpu_freq_khz, gpu_thermal_pct;
	unsigned int npu_util_pct, npu_thermal_pct;
	unsigned int cpu_freq_khz, cpu_thermal_pct;
	unsigned int high_load, low_load;
	unsigned int min_util, max_util;
	unsigned int decay_us;
	unsigned int heavy_util_thres, heavy_tasks_thres;
	unsigned int efficiency_load, efficiency_util;
	unsigned int heavy_floor_util;
	unsigned int fairness_floor_util = 0;
	unsigned int pressure_cap_relief = 0;
	unsigned int cpu_momentum, gpu_momentum, npu_momentum, mem_momentum;
	unsigned int fusion_signal, fusion_ema, fusion_sum;
	u64 fusion_boost_until;
	unsigned int thermal_penalty = 0;
	unsigned int thermal_rise, thermal_rise_ema;
	unsigned int thermal_relief = 0;
	u64 decay_ns;
	bool transition = sugov_is_transition_event(flags);
	bool heavy_load;
	struct atlas_telemetry_snapshot snap;
	unsigned int reclaim_signal, swap_signal, refault_signal;


	if (!auto_cfg->auto_boost || !max)
		return;

	atlas_get_snapshot(&snap);
	if (!snap.display_active) {
		sugov_clear_display_off_boosts(sg_policy);
		return;
	}

	avg_util = sg_policy->auto_boost_avg_util;
	if (!avg_util)
		avg_util = *util;
	else
		avg_util = ((avg_util * 3) + *util) >> 2;

	util_pct = mult_frac(*util, 100, max);
	cpu_signal = util_pct;
	/* Scheduler migration is not evidence of graphics work. */
	gpu_signal = 0;
	gpu_util_pct = snap.gpu_util_pct;
	gpu_freq_khz = snap.gpu_freq_khz;
	gpu_thermal_pct = snap.gpu_thermal_pct;
	gpu_signal = max(gpu_signal, gpu_util_pct);
	npu_util_pct = snap.npu_util_pct;
	npu_thermal_pct = snap.npu_thermal_pct;
	npu_signal = npu_util_pct;
	cpu_freq_khz = sg_policy->policy->cur;
	cpu_thermal_pct =
		atlas_cpu_thermal_pct_for_cpu(cpumask_first(sg_policy->policy->cpus));
	atlas_update_cpu_policy_slot(sg_policy->atlas_cpu_slot, util_pct,
		cpu_freq_khz, cpu_thermal_pct);
	atlas_maybe_publish_cpu_policy(sg_policy->atlas_cpu_slot, time,
		&sg_policy->atlas_last_publish_ns);
	shared_mem_contention_pct = snap.mem_contention_pct;
	mem_signal = snap.mem_pressure_pct;
	reclaim_signal = snap.mem_reclaim_pct;
	swap_signal = snap.mem_swap_pct;
	refault_signal = snap.mem_workingset_refault_pct;

	/*
	 * Lightweight online learning with a derivative term:
	 * - cpu_signal follows utilization,
	 * - gpu_signal follows transition bursts often seen in rendering,
	 * - npu_signal follows NPU/CDSP/CVP interconnect pressure,
	 * - mem_signal follows system memory pressure.
	 *
	 * The momentum terms are computed before EWMA update, so Atlas can
	 * react to the leading edge of a CPU/GPU burst while still using the
	 * smoothed signals below for hysteresis and steady-state decisions.
	 */
	cpu_momentum = cpu_signal > sg_policy->cpu_signal_ema ?
		cpu_signal - sg_policy->cpu_signal_ema : 0;
	gpu_momentum = gpu_signal > sg_policy->gpu_signal_ema ?
		gpu_signal - sg_policy->gpu_signal_ema : 0;
	npu_momentum = npu_signal > sg_policy->npu_signal_ema ?
		npu_signal - sg_policy->npu_signal_ema : 0;
	mem_momentum = mem_signal > sg_policy->mem_signal_ema ?
		mem_signal - sg_policy->mem_signal_ema : 0;

	sg_policy->cpu_signal_ema =
		((sg_policy->cpu_signal_ema * 7) + cpu_signal) >> 3;
	sg_policy->gpu_signal_ema =
		((sg_policy->gpu_signal_ema * 3) + gpu_signal) >> 2;
	sg_policy->npu_signal_ema =
		((sg_policy->npu_signal_ema * 3) + npu_signal) >> 2;
	sg_policy->mem_signal_ema =
		((sg_policy->mem_signal_ema * 7) + mem_signal) >> 3;

	fusion_sum = cpu_signal * 3 + gpu_signal * 2 + npu_signal * 2 +
		mem_signal + cpu_momentum * 2 + gpu_momentum * 2 +
		npu_momentum * 2 + mem_momentum;
	fusion_signal = min_t(unsigned int, 100, fusion_sum / 15);
	sg_policy->fusion_signal_ema =
		((sg_policy->fusion_signal_ema * 5) + fusion_signal) / 6;
	fusion_ema = sg_policy->fusion_signal_ema;

	high_load = auto_cfg->auto_boost_high_load;
	low_load = auto_cfg->auto_boost_low_load;
	min_util = auto_cfg->auto_boost_min_util;
	max_util = auto_cfg->auto_boost_max_util;
	decay_us = auto_cfg->auto_boost_decay_us;
	heavy_util_thres = auto_cfg->auto_boost_heavy_util;
	heavy_tasks_thres = auto_cfg->auto_boost_heavy_tasks;
	efficiency_load = auto_cfg->auto_boost_efficiency_load;
	efficiency_util = auto_cfg->auto_boost_efficiency_util;
	heavy_floor_util = sg_policy->has_prime_cpu ?
		auto_cfg->auto_boost_prime_util : auto_cfg->auto_boost_gold_util;
	thermal_penalty = max3(cpu_thermal_pct, gpu_thermal_pct, npu_thermal_pct);
	thermal_rise = thermal_penalty > sg_policy->thermal_signal_ema ?
		thermal_penalty - sg_policy->thermal_signal_ema : 0;
	sg_policy->thermal_signal_ema =
		((sg_policy->thermal_signal_ema * 7) + thermal_penalty) >> 3;
	sg_policy->thermal_rise_ema =
		((sg_policy->thermal_rise_ema * 3) + thermal_rise) >> 2;
	thermal_rise_ema = sg_policy->thermal_rise_ema;
	decay_ns = decay_us * NSEC_PER_USEC;

	/*
	 * Thermal efficiency loop: if heat is building while GPU/NPU/CPU work is
	 * sustained, stop adding speculative Atlas boost before the platform hits a
	 * hard thermal clamp.  Actual scheduler utilization is still honored below,
	 * so this trims wasted chase-frequency instead of cutting demanded work.
	 */
	if (thermal_penalty >= 45 &&
	    (thermal_rise_ema >= 3 || thermal_penalty >= 60) &&
	    (gpu_util_pct >= 55 || npu_util_pct >= 55 || util_pct >= 70)) {
		thermal_relief = thermal_penalty - 40 + (thermal_rise_ema << 1);
		thermal_relief = min(thermal_relief, 32U);
		if (sg_policy->freq_hold_until_ns > time)
			sg_policy->freq_hold_until_ns = time;
		if (sg_policy->efficiency_until_ns < time + decay_ns)
			sg_policy->efficiency_until_ns = time + decay_ns;
	}

	/* Promote responsiveness when graphics bursts are detected. */
	if (sg_policy->gpu_signal_ema >= 35 || gpu_util_pct >= 45) {
		high_load = max_t(unsigned int, 1, high_load - 8);
		low_load = max_t(unsigned int, 1, low_load - 6);
		min_util = min_t(unsigned int, SCHED_CAPACITY_SCALE,
				 min_util + 32);
	}

	if (thermal_relief) {
		high_load = min_t(unsigned int, 100, high_load + (thermal_relief >> 1));
		low_load = min_t(unsigned int, high_load, low_load + (thermal_relief >> 2));
		min_util = min_util > thermal_relief ? min_util - thermal_relief : 0;
		max_util = max_util > (thermal_relief << 1) ?
			max_util - (thermal_relief << 1) : min_util;
		max_util = max(max_util, min_util);
	}

	/*
	 * GPU starvation guard: when Orion reports heavy GPU work, keep enough
	 * CPU capacity available for userspace submission, irq work, and driver
	 * housekeeping. The floor is deliberately modest unless CPU demand also
	 * rises, so Atlas helps the GPU without stealing the entire budget.
	 */
	if (gpu_util_pct >= 85)
		fairness_floor_util = max(fairness_floor_util, thermal_relief ? 112U : 160U);
	else if (gpu_util_pct >= 70 || sg_policy->gpu_signal_ema >= 55)
		fairness_floor_util = max(fairness_floor_util, thermal_relief ? 80U : 112U);
	if (fairness_floor_util && (cpu_signal >= 45 || cpu_momentum >= 18))
		fairness_floor_util = max_t(unsigned int, fairness_floor_util, 208);

	/* NPU/CDSP/CVP bursts usually share memory and CPU feed work. */
	if (sg_policy->npu_signal_ema >= 35 || npu_util_pct >= 45) {
		high_load = max_t(unsigned int, 1, high_load - 5);
		low_load = max_t(unsigned int, 1, low_load - 4);
		min_util = min_t(unsigned int, SCHED_CAPACITY_SCALE,
				 min_util + 24);
	}

	/*
	 * Cross-domain fusion: boost only when CPU, GPU and memory pressure agree
	 * or are accelerating together. This behaves like a tiny bottleneck
	 * classifier and avoids spending CPU headroom on isolated GPU load.
	 */
	if ((fusion_signal >= 75 || fusion_ema >= 60) && thermal_penalty < 65) {
		high_load = max_t(unsigned int, 1, high_load - 6);
		low_load = max_t(unsigned int, 1, low_load - 4);
		min_util = min_t(unsigned int, SCHED_CAPACITY_SCALE,
				 min_util + 40);
		if (cpu_momentum >= 20 || gpu_momentum >= 25 ||
		    npu_momentum >= 25) {
			fusion_boost_until = time +
				(auto_cfg->auto_boost_decay_us * NSEC_PER_USEC);
			sg_policy->auto_boost_until_ns =
				max(sg_policy->auto_boost_until_ns,
				    fusion_boost_until);
		}
	}

	/*
	 * When either side reports sustained thermal pressure, damp GPU-driven
	 * CPU boosts proportionally to avoid futile frequency chasing.
	 */
	if (thermal_penalty > 55) {
		unsigned int damp = min_t(unsigned int, 24,
					  (thermal_penalty - 55) / 2 +
					  (thermal_rise_ema >> 1));

		high_load = min_t(unsigned int, 100, high_load + damp);
		low_load = min_t(unsigned int, high_load, low_load + (damp >> 1));
		min_util = min_util > (damp >> 1) ? min_util - (damp >> 1) : 0;
		max_util = max_t(unsigned int, min_util,
				 max_util > damp ? max_util - damp : min_util);
	}

	/* Ease off boosts when memory pressure is sustained. */
	if (sg_policy->mem_signal_ema >= 70) {
		high_load = min_t(unsigned int, 100, high_load + 6);
		low_load = min_t(unsigned int, high_load, low_load + 4);
		max_util = max_t(unsigned int, min_util,
				 mult_frac(max_util, 9, 10));
	}

	if (reclaim_signal >= 45 || swap_signal >= 30 || refault_signal >= 35) {
		high_load = min_t(unsigned int, 100, high_load + 5);
		low_load = min_t(unsigned int, high_load, low_load + 3);
		max_util = max_t(unsigned int, min_util,
				 mult_frac(max_util, 17, 20));
		/* Reclaim needs CPU time too; keep a small non-GPU floor alive. */
		fairness_floor_util = max_t(unsigned int, fairness_floor_util, 128);
		pressure_cap_relief = max_t(unsigned int, pressure_cap_relief, 64);
	}

	if (shared_mem_contention_pct >= 65 &&
	    (gpu_util_pct >= 50 || npu_util_pct >= 50) &&
	    thermal_penalty < 70) {
		min_util = min_t(unsigned int, SCHED_CAPACITY_SCALE,
				 min_util + 32);
		fairness_floor_util = max_t(unsigned int, fairness_floor_util, 160);
		pressure_cap_relief = max_t(unsigned int, pressure_cap_relief, 96);
		sg_policy->auto_boost_until_ns = max_t(u64,
						       sg_policy->auto_boost_until_ns,
						       time + (decay_us * NSEC_PER_USEC));
	}

	/* Atlas/Orion thermal and clock coupling. */
	if (gpu_thermal_pct >= 75 || npu_thermal_pct >= 75 ||
	    cpu_thermal_pct >= 75) {
		high_load = min_t(unsigned int, 100, high_load + 8);
		low_load = min_t(unsigned int, high_load, low_load + 6);
		max_util = max_t(unsigned int, min_util,
				 mult_frac(max_util, 17, 20));
	} else if (!thermal_relief && gpu_util_pct >= 65) {
		/* Clock domains are incomparable; use utilization demand only. */
		min_util = min_t(unsigned int, SCHED_CAPACITY_SCALE, min_util + 32);
	}

	if (thermal_relief && fairness_floor_util) {
		fairness_floor_util = fairness_floor_util > thermal_relief ?
			fairness_floor_util - thermal_relief : 64;
		fairness_floor_util = max(fairness_floor_util, 64U);
	}

	if (fairness_floor_util) {
		min_util = max_t(unsigned int, min_util, fairness_floor_util);
		if (pressure_cap_relief)
			max_util = min_t(unsigned int, SCHED_CAPACITY_SCALE,
					 max_util + pressure_cap_relief);
	}

	low_load = min(low_load, high_load);
	heavy_load = util_pct >= heavy_util_thres ||
		     nr_running >= heavy_tasks_thres;

	if (heavy_load) {
		sg_policy->auto_boost_until_ns = max_t(u64,
			sg_policy->auto_boost_until_ns, time + (decay_ns << 1));
		avg_util = ((avg_util << 1) + (*util << 1)) >> 2;
	} else if (util_pct <= efficiency_load &&
		   !transition && !(flags & SCHED_CPUFREQ_IOWAIT)) {
		sg_policy->efficiency_until_ns = max_t(u64,
							sg_policy->efficiency_until_ns,
							time + decay_ns);
	}

	sg_policy->auto_boost_avg_util = avg_util;

	if (transition || (flags & SCHED_CPUFREQ_IOWAIT))
		sg_policy->auto_boost_until_ns = max_t(u64,
			sg_policy->auto_boost_until_ns, time + decay_ns);

	if (util_pct >= high_load)
		sg_policy->auto_boost_until_ns = max_t(u64,
			sg_policy->auto_boost_until_ns,
			time + (decay_ns << 1));
	else if (util_pct >= low_load)
		sg_policy->auto_boost_until_ns = max_t(u64,
			sg_policy->auto_boost_until_ns, time + decay_ns);

	if (time < sg_policy->efficiency_until_ns && !heavy_load &&
	    !transition && !(flags & SCHED_CPUFREQ_IOWAIT))
		return;

	if (time >= sg_policy->auto_boost_until_ns)
		return;

	floor_util = max_t(unsigned long, avg_util,
			  mult_frac(max, min_util,
				    SCHED_CAPACITY_SCALE));
	if (heavy_load) {
		floor_util = max_t(unsigned long, floor_util,
			mult_frac(max, heavy_floor_util,
				  SCHED_CAPACITY_SCALE));
	}
	cap_util = mult_frac(max, max_util,
			    SCHED_CAPACITY_SCALE);
	if (!heavy_load)
		cap_util = min_t(unsigned long, cap_util,
			mult_frac(max, efficiency_util,
				  SCHED_CAPACITY_SCALE));
	min_cap_util = mult_frac(max, 256, SCHED_CAPACITY_SCALE);
	min_cap_util = min_t(unsigned long, floor_util, min_cap_util);
	cap_util = max(cap_util, min_cap_util);

	*util = max(*util, min(floor_util, cap_util));
}

static void sugov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max, hs_util, boost_util;
	unsigned int next_f;
	bool busy;

	if (!sg_policy->tunables->auto_cfg.pl && flags & SCHED_CPUFREQ_PL)
		return;

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	ignore_dl_rate_limit(sg_cpu, sg_policy);

	if (!sugov_should_update_freq(sg_policy, time))
		return;

	/* Limits may have changed, don't skip frequency update */
	busy = use_pelt() && !sg_policy->need_freq_update &&
		sugov_cpu_is_busy(sg_cpu);

	sg_cpu->util = util = sugov_get_util(sg_cpu);
	max = sg_cpu->max;
	sg_cpu->flags = flags;

	if (sg_policy->max != max) {
		sg_policy->max = max;
		hs_util = target_util(sg_policy,
				       sg_policy->tunables->auto_cfg.hispeed_freq);
		sg_policy->hispeed_util = hs_util;

		boost_util = target_util(sg_policy,
				    sg_policy->tunables->auto_cfg.rtg_boost_freq);
		sg_policy->rtg_boost_util = boost_util;
	}

	util = sugov_iowait_apply(sg_cpu, time, util, max);
	sugov_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
			   sg_policy->policy->cur);
	sugov_note_transition_boost(sg_policy, time, flags);

	trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util,
				sg_policy->avg_cap, max, sg_cpu->walt_load.nl,
				sg_cpu->walt_load.pl,
				sg_cpu->walt_load.rtgb_active, flags);

	sugov_walt_adjust(sg_cpu, &util, &max);
	sugov_apply_tunable_boosts(sg_cpu, time, flags, &util, max);
	sugov_apply_auto_boost(sg_policy, time, flags,
			       cpu_rq(sg_cpu->cpu)->nr_running, &util, max);
	next_f = get_next_freq(sg_policy, util, max);
	/*
	 * Do not reduce the frequency if the CPU has not been idle
	 * recently, as the reduction is likely to be premature then.
	 */
	if (busy && next_f < sg_policy->next_freq) {
		next_f = sg_policy->next_freq;

		/* Restore cached freq as next_freq has changed */
		sg_policy->cached_raw_freq = sg_policy->prev_cached_raw_freq;
	}

	/*
	 * This code runs under rq->lock for the target CPU, so it won't run
	 * concurrently on two different CPUs for the same target and it is not
	 * necessary to acquire the lock in the fast switch case.
	 */
	if (sg_policy->policy->fast_switch_enabled) {
		sugov_fast_switch(sg_policy, time, next_f);
	} else {
		raw_spin_lock(&sg_policy->update_lock);
		sugov_deferred_update(sg_policy, time, next_f);
		raw_spin_unlock(&sg_policy->update_lock);
	}
}

static unsigned int sugov_next_freq_shared(struct sugov_cpu *sg_cpu, u64 time,
					   unsigned int flags)
{
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	u64 last_freq_update_time = sg_policy->last_freq_update_time;
	unsigned long util = 0, max = 1;
	unsigned int nr_running = 0;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct sugov_cpu *j_sg_cpu = &per_cpu(sugov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = last_freq_update_time - j_sg_cpu->last_update;
		if (delta_ns > stale_ns) {
			sugov_iowait_reset(j_sg_cpu, last_freq_update_time,
					   false);
			continue;
		}

		/*
		 * If the util value for all CPUs in a policy is 0, just using >
		 * will result in a max value of 1. WALT stats can later update
		 * the aggregated util value, causing get_next_freq() to compute
		 * freq = max_freq * 1.25 * (util / max) for nonzero util,
		 * leading to spurious jumps to fmax.
		 */
		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		j_util = sugov_iowait_apply(j_sg_cpu, time, j_util, j_max);

		if (j_util * max >= j_max * util) {
			util = j_util;
			max = j_max;
		}
		nr_running = max(nr_running, cpu_rq(j)->nr_running);

		sugov_walt_adjust(j_sg_cpu, &util, &max);
		sugov_apply_tunable_boosts(j_sg_cpu, time, j_sg_cpu->flags,
					  &util, max);
	}

	sugov_apply_auto_boost(sg_policy, time, flags, nr_running, &util, max);

	return get_next_freq(sg_policy, util, max);
}

static void
sugov_update_shared(struct update_util_data *hook, u64 time, unsigned int flags)
{
	struct sugov_cpu *sg_cpu = container_of(hook, struct sugov_cpu, update_util);
	struct sugov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long hs_util, boost_util;
	unsigned int next_f;

	if (!sg_policy->tunables->auto_cfg.pl && flags & SCHED_CPUFREQ_PL)
		return;

	sg_cpu->util = sugov_get_util(sg_cpu);
	sg_cpu->flags = flags;
	raw_spin_lock(&sg_policy->update_lock);

	if (sg_policy->max != sg_cpu->max) {
		sg_policy->max = sg_cpu->max;
		hs_util = target_util(sg_policy,
					sg_policy->tunables->auto_cfg.hispeed_freq);
		sg_policy->hispeed_util = hs_util;

		boost_util = target_util(sg_policy,
				    sg_policy->tunables->auto_cfg.rtg_boost_freq);
		sg_policy->rtg_boost_util = boost_util;
	}

	sugov_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	sugov_calc_avg_cap(sg_policy, sg_cpu->walt_load.ws,
			   sg_policy->policy->cur);
	ignore_dl_rate_limit(sg_cpu, sg_policy);
	sugov_note_transition_boost(sg_policy, time, flags);

	trace_sugov_util_update(sg_cpu->cpu, sg_cpu->util, sg_policy->avg_cap,
				sg_cpu->max, sg_cpu->walt_load.nl,
				sg_cpu->walt_load.pl,
				sg_cpu->walt_load.rtgb_active, flags);

	if (sugov_should_update_freq(sg_policy, time) &&
	    !(flags & SCHED_CPUFREQ_CONTINUE)) {
		next_f = sugov_next_freq_shared(sg_cpu, time, flags);
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
		sg_policy->update_time = time;
		if (sugov_time_limit(sg_policy, next_f, flags))
			goto out;
#endif

		if (sg_policy->policy->fast_switch_enabled)
			sugov_fast_switch(sg_policy, time, next_f);
		else
			sugov_deferred_update(sg_policy, time, next_f);
	}
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
out:
#endif

	raw_spin_unlock(&sg_policy->update_lock);
}

static void sugov_work(struct kthread_work *work)
{
	struct sugov_policy *sg_policy = container_of(work, struct sugov_policy, work);
	unsigned int freq;
	unsigned long flags;

	/*
	 * Hold sg_policy->update_lock shortly to handle the case where:
	 * incase sg_policy->next_freq is read here, and then updated by
	 * sugov_deferred_update() just before work_in_progress is set to false
	 * here, we may miss queueing the new update.
	 *
	 * Note: If a work was queued after the update_lock is released,
	 * sugov_work() will just be called again by kthread_work code; and the
	 * request will be proceed before the sugov thread sleeps.
	 */
	raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
	freq = sg_policy->next_freq;
	if (use_pelt())
		sg_policy->work_in_progress = false;
	sugov_track_cycles(sg_policy, sg_policy->policy->cur,
			   ktime_get_ns());
	raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, freq, CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);
}

static void sugov_irq_work(struct irq_work *irq_work)
{
	struct sugov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct sugov_policy, irq_work);

	kthread_queue_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct sugov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct sugov_tunables *to_sugov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct sugov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_ns(struct sugov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.up_rate_limit_us);
}

static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.down_rate_limit_us);
}

static ssize_t down_hysteresis_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.down_hysteresis_us);
}

static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->auto_cfg.up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->auto_cfg.down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_ns(sg_policy);
	}

	return count;
}

static ssize_t down_hysteresis_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	struct sugov_policy *sg_policy;
	unsigned int hyst_us;

	if (kstrtouint(buf, 10, &hyst_us))
		return -EINVAL;

	tunables->auto_cfg.down_hysteresis_us = hyst_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook)
		sg_policy->down_hyst_ns = hyst_us * NSEC_PER_USEC;

	return count;
}

static struct governor_attr up_rate_limit_us __maybe_unused =
	__ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us __maybe_unused =
	__ATTR_RW(down_rate_limit_us);
static struct governor_attr down_hysteresis_us __maybe_unused =
	__ATTR_RW(down_hysteresis_us);

static ssize_t hispeed_load_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.hispeed_load);
}

static ssize_t hispeed_load_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.hispeed_load))
		return -EINVAL;

	tunables->auto_cfg.hispeed_load = min(100U, tunables->auto_cfg.hispeed_load);

	return count;
}

static ssize_t hispeed_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.hispeed_freq);
}

static ssize_t hispeed_freq_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;
	struct sugov_policy *sg_policy;
	unsigned long hs_util;
	unsigned long flags;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->auto_cfg.hispeed_freq = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		hs_util = target_util(sg_policy,
					sg_policy->tunables->auto_cfg.hispeed_freq);
		sg_policy->hispeed_util = hs_util;
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	}

	return count;
}

static ssize_t rtg_boost_freq_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.rtg_boost_freq);
}

static ssize_t rtg_boost_freq_store(struct gov_attr_set *attr_set,
				    const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int val;
	struct sugov_policy *sg_policy;
	unsigned long boost_util;
	unsigned long flags;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	tunables->auto_cfg.rtg_boost_freq = val;
	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		boost_util = target_util(sg_policy,
					  sg_policy->tunables->auto_cfg.rtg_boost_freq);
		sg_policy->rtg_boost_util = boost_util;
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	}

	return count;
}

static ssize_t pl_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.pl);
}

static ssize_t mem_boost_util_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.mem_boost_util);
}

static ssize_t mem_boost_util_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.mem_boost_util))
		return -EINVAL;

	tunables->auto_cfg.mem_boost_util = min_t(unsigned int,
				tunables->auto_cfg.mem_boost_util, SCHED_CAPACITY_SCALE);
	return count;
}

static ssize_t mem_boost_hyst_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.mem_boost_hyst_us);
}

static ssize_t mem_boost_hyst_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.mem_boost_hyst_us))
		return -EINVAL;

	return count;
}

static ssize_t auto_boost_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost);
}

static ssize_t auto_boost_store(struct gov_attr_set *attr_set,
				const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtobool(buf, &tunables->auto_cfg.auto_boost))
		return -EINVAL;

	return count;
}

static ssize_t auto_profile_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_profile);
}

static ssize_t auto_profile_store(struct gov_attr_set *attr_set,
				  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int profile;

	if (kstrtouint(buf, 10, &profile))
		return -EINVAL;

	if (profile > SUGOV_AUTO_PROFILE_PERFORMANCE)
		return -EINVAL;

	tunables->auto_cfg.auto_profile = profile;
	if (profile != SUGOV_AUTO_PROFILE_MANUAL)
		sugov_apply_auto_profile(tunables, profile);

	return count;
}

static ssize_t auto_boost_high_load_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_high_load);
}

static ssize_t auto_boost_high_load_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_high_load))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_high_load = min(100U,
					    tunables->auto_cfg.auto_boost_high_load);
	tunables->auto_cfg.auto_boost_low_load = min(tunables->auto_cfg.auto_boost_low_load,
					   tunables->auto_cfg.auto_boost_high_load);

	return count;
}

static ssize_t auto_boost_low_load_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_low_load);
}

static ssize_t auto_boost_low_load_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_low_load))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_low_load = min(tunables->auto_cfg.auto_boost_low_load,
					   tunables->auto_cfg.auto_boost_high_load);

	return count;
}

static ssize_t auto_boost_min_util_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_min_util);
}

static ssize_t auto_boost_min_util_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_min_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_min_util = min_t(unsigned int,
				tunables->auto_cfg.auto_boost_min_util,
				tunables->auto_cfg.auto_boost_max_util);

	return count;
}

static ssize_t auto_boost_max_util_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_max_util);
}

static ssize_t auto_boost_max_util_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_max_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_max_util = min_t(unsigned int,
				tunables->auto_cfg.auto_boost_max_util,
				SCHED_CAPACITY_SCALE);
	tunables->auto_cfg.auto_boost_min_util = min(tunables->auto_cfg.auto_boost_min_util,
					   tunables->auto_cfg.auto_boost_max_util);

	return count;
}

static ssize_t auto_boost_decay_us_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_decay_us);
}

static ssize_t auto_boost_decay_us_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_decay_us))
		return -EINVAL;

	return count;
}

static ssize_t auto_boost_heavy_util_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_heavy_util);
}

static ssize_t auto_boost_heavy_util_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_heavy_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_heavy_util = min(100U,
					      tunables->auto_cfg.auto_boost_heavy_util);
	return count;
}

static ssize_t auto_boost_heavy_tasks_show(struct gov_attr_set *attr_set,
					   char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_heavy_tasks);
}

static ssize_t auto_boost_heavy_tasks_store(struct gov_attr_set *attr_set,
					    const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_heavy_tasks))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_heavy_tasks = max(1U,
					       tunables->auto_cfg.auto_boost_heavy_tasks);
	return count;
}

static ssize_t auto_boost_prime_util_show(struct gov_attr_set *attr_set,
					  char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_prime_util);
}

static ssize_t auto_boost_prime_util_store(struct gov_attr_set *attr_set,
					   const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_prime_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_prime_util = min_t(unsigned int,
				tunables->auto_cfg.auto_boost_prime_util,
				SCHED_CAPACITY_SCALE);
	return count;
}

static ssize_t auto_boost_gold_util_show(struct gov_attr_set *attr_set,
					 char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.auto_boost_gold_util);
}

static ssize_t auto_boost_gold_util_store(struct gov_attr_set *attr_set,
					  const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_gold_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_gold_util = min_t(unsigned int,
				tunables->auto_cfg.auto_boost_gold_util,
				SCHED_CAPACITY_SCALE);
	return count;
}

static ssize_t auto_boost_efficiency_load_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 tunables->auto_cfg.auto_boost_efficiency_load);
}

static ssize_t auto_boost_efficiency_load_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_efficiency_load))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_efficiency_load = min(100U,
			tunables->auto_cfg.auto_boost_efficiency_load);
	return count;
}

static ssize_t auto_boost_efficiency_util_show(struct gov_attr_set *attr_set,
					       char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 tunables->auto_cfg.auto_boost_efficiency_util);
}

static ssize_t auto_boost_efficiency_util_store(struct gov_attr_set *attr_set,
						const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtouint(buf, 10, &tunables->auto_cfg.auto_boost_efficiency_util))
		return -EINVAL;

	tunables->auto_cfg.auto_boost_efficiency_util = min_t(unsigned int,
			tunables->auto_cfg.auto_boost_efficiency_util,
			SCHED_CAPACITY_SCALE);
	return count;
}

static ssize_t uclamp_helper_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	return scnprintf(buf, PAGE_SIZE, "%u\n", tunables->auto_cfg.uclamp_helper);
}

static ssize_t uclamp_helper_store(struct gov_attr_set *attr_set,
				   const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtobool(buf, &tunables->auto_cfg.uclamp_helper))
		return -EINVAL;

	return count;
}

static ssize_t pl_store(struct gov_attr_set *attr_set, const char *buf,
				   size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	if (kstrtobool(buf, &tunables->auto_cfg.pl))
		return -EINVAL;

	return count;
}

#ifdef OPLUS_FEATURE_POWER_CPUFREQ
static ssize_t target_loads_show(struct gov_attr_set *attr_set, char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&tunables->auto_cfg.target_loads_lock, flags);
	for (i = 0; i < tunables->auto_cfg.ntarget_loads; i++)
		ret += snprintf(buf + ret, sizeof(buf), "%u%s", tunables->auto_cfg.target_loads[i],
			i & 0x1 ? ":" : " ");
	snprintf(buf + ret - 1, sizeof(buf), "\n");
	spin_unlock_irqrestore(&tunables->auto_cfg.target_loads_lock, flags);
	return ret;
}

static unsigned int *get_tokenized_data(const char *buf, int *num_tokens)
{
	const char *cp;
	int i;
	int ntokens = 1;
	unsigned int *tokenized_data;
	int err = -EINVAL;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err;

	tokenized_data = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!tokenized_data) {
		err = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &tokenized_data[i++]) != 1)
			goto err_kfree;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_kfree;

	*num_tokens = ntokens;

	return tokenized_data;
err_kfree:
	kfree(tokenized_data);
err:
	return ERR_PTR(err);
}

static ssize_t target_loads_store(struct gov_attr_set *attr_set, const char *buf,
					size_t count)
{
	int ntokens;
	unsigned int *new_target_loads = NULL;
	unsigned long flags;
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);

	new_target_loads = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_target_loads))
		return PTR_ERR(new_target_loads);

	spin_lock_irqsave(&tunables->auto_cfg.target_loads_lock, flags);
	if (tunables->auto_cfg.target_loads != default_target_loads)
		kfree(tunables->auto_cfg.target_loads);

	tunables->auto_cfg.target_loads = new_target_loads;
	tunables->auto_cfg.ntarget_loads = ntokens;
	spin_unlock_irqrestore(&tunables->auto_cfg.target_loads_lock, flags);

	return count;
}

static ssize_t above_hispeed_delay_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned int *new_above_hispeed_delay = NULL;
	unsigned long flags;
	int ntokens;

	new_above_hispeed_delay = get_tokenized_data(buf, &ntokens);
	if (IS_ERR(new_above_hispeed_delay))
		return PTR_ERR(new_above_hispeed_delay);

	spin_lock_irqsave(&tunables->auto_cfg.above_hispeed_delay_lock, flags);
	if (tunables->auto_cfg.above_hispeed_delay != default_above_hispeed_delay)
		kfree(tunables->auto_cfg.above_hispeed_delay);
	tunables->auto_cfg.above_hispeed_delay = new_above_hispeed_delay;
	tunables->auto_cfg.nabove_hispeed_delay = ntokens;
	spin_unlock_irqrestore(&tunables->auto_cfg.above_hispeed_delay_lock, flags);

	return count;
}

static ssize_t above_hispeed_delay_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct sugov_tunables *tunables = to_sugov_tunables(attr_set);
	unsigned long flags;
	ssize_t ret = 0;
	int i;

	spin_lock_irqsave(&tunables->auto_cfg.above_hispeed_delay_lock, flags);

	for (i = 0; i < tunables->auto_cfg.nabove_hispeed_delay; i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%u%s",
				tunables->auto_cfg.above_hispeed_delay[i],
				i & 0x1 ? ":" : " ");

	snprintf(buf + ret - 1, PAGE_SIZE - ret + 1, "\n");
	spin_unlock_irqrestore(&tunables->auto_cfg.above_hispeed_delay_lock, flags);

	return ret;
}
#endif

static struct governor_attr hispeed_load __maybe_unused = __ATTR_RW(hispeed_load);
static struct governor_attr hispeed_freq __maybe_unused = __ATTR_RW(hispeed_freq);
static struct governor_attr rtg_boost_freq __maybe_unused = __ATTR_RW(rtg_boost_freq);
static struct governor_attr pl __maybe_unused = __ATTR_RW(pl);
static struct governor_attr mem_boost_util __maybe_unused = __ATTR_RW(mem_boost_util);
static struct governor_attr mem_boost_hyst_us __maybe_unused = __ATTR_RW(mem_boost_hyst_us);
static struct governor_attr auto_boost __maybe_unused = __ATTR_RW(auto_boost);
static struct governor_attr auto_profile __maybe_unused = __ATTR_RW(auto_profile);
static struct governor_attr auto_boost_high_load __maybe_unused = __ATTR_RW(auto_boost_high_load);
static struct governor_attr auto_boost_low_load __maybe_unused = __ATTR_RW(auto_boost_low_load);
static struct governor_attr auto_boost_min_util __maybe_unused = __ATTR_RW(auto_boost_min_util);
static struct governor_attr auto_boost_max_util __maybe_unused = __ATTR_RW(auto_boost_max_util);
static struct governor_attr auto_boost_decay_us __maybe_unused = __ATTR_RW(auto_boost_decay_us);
static struct governor_attr auto_boost_heavy_util __maybe_unused = __ATTR_RW(auto_boost_heavy_util);
static struct governor_attr auto_boost_heavy_tasks __maybe_unused =
	__ATTR_RW(auto_boost_heavy_tasks);
static struct governor_attr auto_boost_prime_util __maybe_unused = __ATTR_RW(auto_boost_prime_util);
static struct governor_attr auto_boost_gold_util __maybe_unused = __ATTR_RW(auto_boost_gold_util);
static struct governor_attr auto_boost_efficiency_load __maybe_unused =
	__ATTR_RW(auto_boost_efficiency_load);
static struct governor_attr auto_boost_efficiency_util __maybe_unused =
	__ATTR_RW(auto_boost_efficiency_util);
static struct governor_attr uclamp_helper __maybe_unused = __ATTR_RW(uclamp_helper);
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
static struct governor_attr target_loads __maybe_unused =
	__ATTR(target_loads, 0664, target_loads_show, target_loads_store);
static struct governor_attr above_hispeed_delay __maybe_unused =
				__ATTR_RW(above_hispeed_delay);
#endif

static struct attribute *sugov_attributes[] = {
	NULL
};

static void sugov_tunables_free(struct kobject *kobj)
{
	struct gov_attr_set *attr_set = container_of(kobj, struct gov_attr_set, kobj);

	kfree(to_sugov_tunables(attr_set));
}

static ssize_t atlas_governor_show(struct kobject *kobj, struct attribute *attr,
				   char *buf)
{
	struct governor_attr *gattr = container_of(attr, struct governor_attr, attr);

	return gattr->show(container_of(kobj, struct gov_attr_set, kobj), buf);
}

static ssize_t atlas_governor_store(struct kobject *kobj, struct attribute *attr,
				    const char *buf, size_t count)
{
	/*
	 * Atlas runs in automatic mode by default; runtime behavior adapts
	 * through scheduler, thermal, memory and Orion telemetry signals.
	 * Keep sysfs tunables read-only to avoid fragile per-game retuning.
	 */
	return -EOPNOTSUPP;
}

static const struct sysfs_ops atlas_governor_sysfs_ops = {
	.show = atlas_governor_show,
	.store = atlas_governor_store,
};

static struct kobj_type sugov_tunables_ktype = {
	.default_attrs = sugov_attributes,
	.sysfs_ops = &atlas_governor_sysfs_ops,
	.release = &sugov_tunables_free,
};

/********************** cpufreq governor interface *********************/

static struct cpufreq_governor atlas_gov;

static struct sugov_policy *sugov_policy_alloc(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	sg_policy->atlas_cpu_slot = -1;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void sugov_policy_free(struct sugov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int sugov_kthread_create(struct sugov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	kthread_init_work(&sg_policy->work, sugov_work);
	kthread_init_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"sugov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create sugov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, sugov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void sugov_kthread_stop(struct sugov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	kthread_flush_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct sugov_tunables *sugov_tunables_alloc(struct sugov_policy *sg_policy)
{
	struct sugov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void sugov_tunables_save(struct cpufreq_policy *policy,
		struct sugov_tunables *tunables)
{
	int cpu;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!have_governor_per_policy())
		return;

	if (!cached) {
		cached = kzalloc(sizeof(*tunables), GFP_KERNEL);
		if (!cached)
			return;

		for_each_cpu(cpu, policy->related_cpus)
			per_cpu(cached_tunables, cpu) = cached;
	}

	cached->auto_cfg.pl = tunables->auto_cfg.pl;
	cached->auto_cfg.hispeed_load = tunables->auto_cfg.hispeed_load;
	cached->auto_cfg.rtg_boost_freq = tunables->auto_cfg.rtg_boost_freq;
	cached->auto_cfg.hispeed_freq = tunables->auto_cfg.hispeed_freq;
	cached->auto_cfg.up_rate_limit_us = tunables->auto_cfg.up_rate_limit_us;
	cached->auto_cfg.down_rate_limit_us = tunables->auto_cfg.down_rate_limit_us;
	cached->auto_cfg.auto_profile = tunables->auto_cfg.auto_profile;
	cached->auto_cfg.auto_boost = tunables->auto_cfg.auto_boost;
	cached->auto_cfg.auto_boost_high_load = tunables->auto_cfg.auto_boost_high_load;
	cached->auto_cfg.auto_boost_low_load = tunables->auto_cfg.auto_boost_low_load;
	cached->auto_cfg.auto_boost_min_util = tunables->auto_cfg.auto_boost_min_util;
	cached->auto_cfg.auto_boost_max_util = tunables->auto_cfg.auto_boost_max_util;
	cached->auto_cfg.auto_boost_decay_us = tunables->auto_cfg.auto_boost_decay_us;
	cached->auto_cfg.auto_boost_heavy_util = tunables->auto_cfg.auto_boost_heavy_util;
	cached->auto_cfg.auto_boost_heavy_tasks = tunables->auto_cfg.auto_boost_heavy_tasks;
	cached->auto_cfg.auto_boost_prime_util = tunables->auto_cfg.auto_boost_prime_util;
	cached->auto_cfg.auto_boost_gold_util = tunables->auto_cfg.auto_boost_gold_util;
	cached->auto_cfg.auto_boost_efficiency_load =
		tunables->auto_cfg.auto_boost_efficiency_load;
	cached->auto_cfg.auto_boost_efficiency_util =
		tunables->auto_cfg.auto_boost_efficiency_util;
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	cached->auto_cfg.above_hispeed_delay = tunables->auto_cfg.above_hispeed_delay;
	cached->auto_cfg.nabove_hispeed_delay = tunables->auto_cfg.nabove_hispeed_delay;
#endif
}

static void sugov_clear_global_tunables(void)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;
}

static void sugov_tunables_restore(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	struct sugov_tunables *cached = per_cpu(cached_tunables, policy->cpu);

	if (!cached)
		return;

	tunables->auto_cfg.pl = cached->auto_cfg.pl;
	tunables->auto_cfg.hispeed_load = cached->auto_cfg.hispeed_load;
	tunables->auto_cfg.rtg_boost_freq = cached->auto_cfg.rtg_boost_freq;
	tunables->auto_cfg.hispeed_freq = cached->auto_cfg.hispeed_freq;
	tunables->auto_cfg.up_rate_limit_us = cached->auto_cfg.up_rate_limit_us;
	tunables->auto_cfg.down_rate_limit_us = cached->auto_cfg.down_rate_limit_us;
	tunables->auto_cfg.auto_profile = cached->auto_cfg.auto_profile;
	tunables->auto_cfg.auto_boost = cached->auto_cfg.auto_boost;
	tunables->auto_cfg.auto_boost_high_load = cached->auto_cfg.auto_boost_high_load;
	tunables->auto_cfg.auto_boost_low_load = cached->auto_cfg.auto_boost_low_load;
	tunables->auto_cfg.auto_boost_min_util = cached->auto_cfg.auto_boost_min_util;
	tunables->auto_cfg.auto_boost_max_util = cached->auto_cfg.auto_boost_max_util;
	tunables->auto_cfg.auto_boost_decay_us = cached->auto_cfg.auto_boost_decay_us;
	tunables->auto_cfg.auto_boost_heavy_util = cached->auto_cfg.auto_boost_heavy_util;
	tunables->auto_cfg.auto_boost_heavy_tasks = cached->auto_cfg.auto_boost_heavy_tasks;
	tunables->auto_cfg.auto_boost_prime_util = cached->auto_cfg.auto_boost_prime_util;
	tunables->auto_cfg.auto_boost_gold_util = cached->auto_cfg.auto_boost_gold_util;
	tunables->auto_cfg.auto_boost_efficiency_load =
		cached->auto_cfg.auto_boost_efficiency_load;
	tunables->auto_cfg.auto_boost_efficiency_util =
		cached->auto_cfg.auto_boost_efficiency_util;
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	tunables->auto_cfg.above_hispeed_delay = cached->auto_cfg.above_hispeed_delay;
	tunables->auto_cfg.nabove_hispeed_delay = cached->auto_cfg.nabove_hispeed_delay;
#endif
}

static int sugov_init(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy;
	struct sugov_tunables *tunables;
	unsigned long util;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = sugov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = sugov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = sugov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	/*
	 * Rate-limit both directions to avoid chasing short-lived samples.
	 * A slightly longer downscale guard window improves performance per
	 * watt by preventing repeated CPU/GPU oscillation under long loops.
	 */
	tunables->auto_cfg.up_rate_limit_us = 500;
	tunables->auto_cfg.down_rate_limit_us = 4000;
	tunables->auto_cfg.down_hysteresis_us = 3000;
	tunables->auto_cfg.hispeed_load = DEFAULT_HISPEED_LOAD;
	tunables->auto_cfg.hispeed_freq = 0;
	tunables->auto_cfg.auto_profile = SUGOV_AUTO_PROFILE_BALANCED;
	sugov_apply_auto_profile(tunables, tunables->auto_cfg.auto_profile);
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	tunables->auto_cfg.target_loads = default_target_loads;
	tunables->auto_cfg.ntarget_loads = ARRAY_SIZE(default_target_loads);
	spin_lock_init(&tunables->auto_cfg.target_loads_lock);
	tunables->auto_cfg.above_hispeed_delay = default_above_hispeed_delay;
	tunables->auto_cfg.nabove_hispeed_delay =
		ARRAY_SIZE(default_above_hispeed_delay);
	spin_lock_init(&tunables->auto_cfg.above_hispeed_delay_lock);
#endif

	switch (policy->cpu) {
	default:
	case 0:
		tunables->auto_cfg.rtg_boost_freq = DEFAULT_CPU0_RTG_BOOST_FREQ;
		break;
	case 4:
		tunables->auto_cfg.rtg_boost_freq = DEFAULT_CPU4_RTG_BOOST_FREQ;
		break;
	case 7:
		tunables->auto_cfg.rtg_boost_freq = DEFAULT_CPU7_RTG_BOOST_FREQ;
		break;
	}

	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	util = target_util(sg_policy, sg_policy->tunables->auto_cfg.rtg_boost_freq);
	sg_policy->rtg_boost_util = util;

	stale_ns = sched_ravg_window + (sched_ravg_window >> 3);

	sugov_tunables_restore(policy);

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &sugov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   policy->governor->name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	kobject_put(&tunables->attr_set.kobj);
	policy->governor_data = NULL;
	sugov_clear_global_tunables();

stop_kthread:
	sugov_kthread_stop(sg_policy);
	mutex_unlock(&global_tunables_lock);

free_sg_policy:
	sugov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static void sugov_exit(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	struct sugov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count) {
		sugov_tunables_save(policy, tunables);
		sugov_clear_global_tunables();
	}

	mutex_unlock(&global_tunables_lock);

	sugov_kthread_stop(sg_policy);
	sugov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);
}

static int sugov_start(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->auto_cfg.up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->auto_cfg.down_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_hyst_ns =
		sg_policy->tunables->auto_cfg.down_hysteresis_us * NSEC_PER_USEC;
	update_min_rate_limit_ns(sg_policy);
	sg_policy->last_freq_update_time	= 0;
	sg_policy->next_freq			= 0;
	sg_policy->work_in_progress		= false;
	sg_policy->limits_changed		= false;
	sg_policy->need_freq_update		= false;
	sg_policy->cached_raw_freq		= 0;
	sg_policy->freq_hold_until_ns		= 0;
	sg_policy->auto_boost_until_ns		= 0;
	sg_policy->efficiency_until_ns		= 0;
	sg_policy->auto_boost_avg_util		= 0;
	sg_policy->cpu_signal_ema		= 0;
	sg_policy->gpu_signal_ema		= 0;
	sg_policy->npu_signal_ema		= 0;
	sg_policy->mem_signal_ema		= 0;
	sg_policy->fusion_signal_ema		= 0;
	sg_policy->thermal_signal_ema		= 0;
	sg_policy->thermal_rise_ema		= 0;
	sg_policy->atlas_policy_capacity	= atlas_policy_capacity(policy->cpus);
	sg_policy->atlas_last_publish_ns	= 0;
	sg_policy->atlas_cpu_slot		= atlas_register_cpu_policy(policy->cpus,
							 sg_policy->atlas_policy_capacity);
	sg_policy->has_prime_cpu		= sugov_policy_has_prime_cpu(sg_policy);
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	sg_policy->hispeed_validate_time	= 0;
	sg_policy->update_time	= 0;
	sg_policy->freq_locked			= false;
	sg_policy->min_freq			= policy->min;
	sg_policy->after_limits_changed		= false;
#endif
	sg_policy->prev_cached_raw_freq		= 0;

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->cpu			= cpu;
		sg_cpu->sg_policy		= sg_policy;
		sg_cpu->min			=
			(SCHED_CAPACITY_SCALE * policy->cpuinfo.min_freq) /
			policy->cpuinfo.max_freq;
	}

	for_each_cpu(cpu, policy->cpus) {
		struct sugov_cpu *sg_cpu = &per_cpu(sugov_cpu, cpu);

		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							sugov_update_shared :
							sugov_update_single);
	}
	return 0;
}

static void sugov_stop(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();
	atlas_unregister_cpu_policy(sg_policy->atlas_cpu_slot);
	sg_policy->atlas_cpu_slot = -1;

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}
}

static void sugov_limits(struct cpufreq_policy *policy)
{
	struct sugov_policy *sg_policy = policy->governor_data;
	unsigned long flags, now;
	unsigned int freq;
#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	s64 delta;
#endif

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		sugov_track_cycles(sg_policy, sg_policy->policy->cur,
				   ktime_get_ns());
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	} else {
		raw_spin_lock_irqsave(&sg_policy->update_lock, flags);
		freq = policy->cur;
		now = ktime_get_ns();

		/*
		 * cpufreq_driver_resolve_freq() has a clamp, so we do not need
		 * to do any sort of additional validation here.
		 */
		freq = cpufreq_driver_resolve_freq(policy, freq);
		sg_policy->cached_raw_freq = freq;
		sugov_fast_switch(sg_policy, now, freq);
		raw_spin_unlock_irqrestore(&sg_policy->update_lock, flags);
	}

#ifdef OPLUS_FEATURE_POWER_CPUFREQ
	if (policy->min == policy->cpuinfo.max_freq &&
	    policy->min > sg_policy->min_freq) {
		sg_policy->start_time = ktime_get();
		sg_policy->freq_locked = true;
	} else if (sg_policy->freq_locked && policy->min < policy->max) {
		now = ktime_get();
		delta = ktime_to_ns(ktime_sub(now, sg_policy->start_time));
		if (delta >= 8 * NSEC_PER_SEC)
			pr_warn("policy%d's freq locked at max_freq for %lld(ns)",
				cpumask_first(policy->related_cpus), delta);
		sg_policy->freq_locked = false;
	}
	sg_policy->min_freq = policy->min;
	sg_policy->after_limits_changed = true;
#endif
	/*
	 * The limits_changed update below must take place before the updates
	 * of policy limits in cpufreq_set_policy() or a policy limits update
	 * might be missed, so use a memory barrier to ensure it.
	 *
	 * This pairs with the memory barrier in sugov_should_update_freq().
	 */
	smp_wmb();

	WRITE_ONCE(sg_policy->limits_changed, true);
}

static struct cpufreq_governor atlas_gov = {
	.name			= "atlas",
	.owner			= THIS_MODULE,
	.dynamic_switching	= true,
	.init			= sugov_init,
	.exit			= sugov_exit,
	.start			= sugov_start,
	.stop			= sugov_stop,
	.limits			= sugov_limits,
};

static int __init sugov_register(void)
{
	int ret;

	INIT_DELAYED_WORK(&atlas_mem_work, atlas_mem_sample_work);
	atlas_mem_work_initialized = true;
	ret = cpufreq_register_governor(&atlas_gov);
	if (ret) {
		atlas_mem_work_initialized = false;
		cancel_delayed_work_sync(&atlas_mem_work);
	}
	return ret;
}
fs_initcall(sugov_register);
