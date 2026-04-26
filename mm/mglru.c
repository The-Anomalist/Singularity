// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-Gen LRU aging and reclaim machinery for the 4.19 backport.
 */

#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/mmzone.h>
#include <linux/memcontrol.h>
#include <linux/jump_label.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#include <linux/string.h>
#include <linux/swap.h>
#include <linux/sysfs.h>
#include <linux/uaccess.h>
#include <linux/mmu_notifier.h>
#include <linux/vmstat.h>
#include <trace/events/vmscan.h>
#ifdef CONFIG_PROC_FS
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#endif
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#ifdef CONFIG_LRU_GEN
static DEFINE_STATIC_KEY_FALSE(lru_gen_caps);
static bool __read_mostly lru_gen_boot_enabled =
	IS_ENABLED(CONFIG_LRU_GEN_ENABLED);
static unsigned int __read_mostly lru_gen_min_ttl_ms;
static unsigned int __read_mostly lru_gen_age_period_ms = 1000;
static unsigned int __read_mostly lru_gen_weight_anon_pct = 50;
static unsigned int __read_mostly lru_gen_dedup_window_ms = 25;
static bool __read_mostly lru_gen_pressure_normalize = true;
static unsigned int __read_mostly lru_gen_ptwalk_pages = 256;
static bool __read_mostly lru_gen_ptwalk_clear_young;
static bool __read_mostly lru_gen_reclaim_ptwalk;
static bool __read_mostly lru_gen_reclaim_feedback;
static bool __read_mostly lru_gen_reclaim_advance;
/*
 * Reclaim-time MM walk throttling must be shared by all memcg lruvecs on a
 * node; otherwise each memcg can trigger its own walk and create reclaim
 * stalls under memory pressure.
 */
static unsigned long lru_gen_mm_walk_seq[MAX_NUMNODES];
#ifdef CONFIG_DEBUG_FS
static struct dentry *mglru_debugfs_root;
#endif
#ifdef CONFIG_PROC_FS
static struct proc_dir_entry *mglru_proc_entry;
#endif
#ifdef CONFIG_SYSFS
static struct kobject *mglru_kobj;
#endif
static unsigned long lru_gen_count_old(struct lru_gen_struct *lrugen, int type);
static bool lru_gen_dedup_access(struct lru_gen_struct *lrugen, unsigned int type,
				 unsigned long nr_pages);
static bool lru_gen_should_age(struct lru_gen_struct *lrugen);
static void lru_gen_reset_lruvec(struct lruvec *lruvec);

static void lru_gen_clamp_min_seq(struct lru_gen_struct *lrugen, unsigned int type)
{
	unsigned long floor = lrugen->max_seq - (MIN_NR_GENS - 1);

	if (lrugen->min_seq[type] > lrugen->max_seq)
		lrugen->min_seq[type] = lrugen->max_seq;
	if (lrugen->min_seq[type] < floor)
		lrugen->min_seq[type] = floor;
}

#define MGLRU_MAX_PRESSURE	(ULONG_MAX / 4)

static void lru_gen_bump_pressure(struct lru_gen_struct *lrugen,
				  unsigned int type, unsigned long delta)
{
	unsigned long pressure = lrugen->pressure[type];

	if (!delta)
		return;

	if (pressure > MGLRU_MAX_PRESSURE - delta)
		pressure = MGLRU_MAX_PRESSURE;
	else
		pressure += delta;

	lrugen->pressure[type] = pressure;
}

static void lru_gen_reset_all_lruvecs(void)
{
	struct mem_cgroup *memcg = NULL;
	int nid;

	memcg = mem_cgroup_iter(NULL, NULL, NULL);
	do {
		for_each_node(nid) {
			pg_data_t *pgdat = NODE_DATA(nid);
			struct lruvec *lruvec = mem_cgroup_lruvec(pgdat, memcg);
			unsigned long flags;

			spin_lock_irqsave(&pgdat->lru_lock, flags);
			lru_gen_reset_lruvec(lruvec);
			spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		}
	} while ((memcg = mem_cgroup_iter(NULL, memcg, NULL)));

	for_each_node(nid)
		WRITE_ONCE(lru_gen_mm_walk_seq[nid], jiffies);
}

static int __init setup_lru_gen(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_boot_enabled = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_boot_enabled = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen", setup_lru_gen);

static int __init setup_lru_gen_min_ttl(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_min_ttl_ms = val;

	return 0;
}
early_param("lru_gen_min_ttl_ms", setup_lru_gen_min_ttl);

static int __init setup_lru_gen_age_period(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_age_period_ms = clamp_t(unsigned int, val, 100, 60000);

	return 0;
}
early_param("lru_gen_age_period_ms", setup_lru_gen_age_period);

static int __init setup_lru_gen_weight_anon(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_weight_anon_pct = min_t(unsigned int, val, 100);

	return 0;
}
early_param("lru_gen_weight_anon_pct", setup_lru_gen_weight_anon);

static int __init setup_lru_gen_dedup_window(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_dedup_window_ms = min_t(unsigned int, val, 1000);

	return 0;
}
early_param("lru_gen_dedup_window_ms", setup_lru_gen_dedup_window);

static int __init setup_lru_gen_normalize(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_pressure_normalize = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_pressure_normalize = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen_pressure_normalize", setup_lru_gen_normalize);

static int __init setup_lru_gen_ptwalk_pages(char *str)
{
	unsigned int val;

	if (!str || kstrtouint(str, 0, &val))
		return -EINVAL;

	lru_gen_ptwalk_pages = clamp_t(unsigned int, val, 32, 16384);
	return 0;
}
early_param("lru_gen_ptwalk_pages", setup_lru_gen_ptwalk_pages);

static int __init setup_lru_gen_ptwalk_clear_young(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_ptwalk_clear_young = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_ptwalk_clear_young = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen_ptwalk_clear_young", setup_lru_gen_ptwalk_clear_young);

static int __init setup_lru_gen_reclaim_ptwalk(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_reclaim_ptwalk = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_reclaim_ptwalk = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen_reclaim_ptwalk", setup_lru_gen_reclaim_ptwalk);

static int __init setup_lru_gen_reclaim_feedback(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_reclaim_feedback = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_reclaim_feedback = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen_reclaim_feedback", setup_lru_gen_reclaim_feedback);

static int __init setup_lru_gen_reclaim_advance(char *str)
{
	if (!str)
		return -EINVAL;

	if (!strcmp(str, "1") || !strcmp(str, "on") || !strcmp(str, "y"))
		lru_gen_reclaim_advance = true;
	else if (!strcmp(str, "0") || !strcmp(str, "off") || !strcmp(str, "n"))
		lru_gen_reclaim_advance = false;
	else
		return -EINVAL;

	return 0;
}
early_param("lru_gen_reclaim_advance", setup_lru_gen_reclaim_advance);

static int __init init_lru_gen(void)
{
	if (lru_gen_boot_enabled)
		static_branch_enable(&lru_gen_caps);

	return 0;
}
late_initcall(init_lru_gen);

static void lru_gen_advance_seq(struct lruvec *lruvec)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned long seq = lrugen->max_seq + 1;
	unsigned int gen = seq % MAX_NR_GENS;
	unsigned int type, zone;

	/*
	 * Generation slots are reused in a ring. Reset the slot for the new
	 * sequence to avoid carrying stale accounting after wrap-around.
	 */
	for (type = 0; type < ANON_AND_FILE; type++)
		for (zone = 0; zone < MAX_NR_ZONES; zone++)
			lrugen->nr_pages[gen][type][zone] = 0;

	lrugen->max_seq = seq;
	lrugen->timestamps[gen] = jiffies;

	for (type = 0; type < ANON_AND_FILE; type++) {
		if (lrugen->min_seq[type] + MIN_NR_GENS <= seq)
			lrugen->min_seq[type] = seq - MIN_NR_GENS + 1;
		lru_gen_clamp_min_seq(lrugen, type);
	}

	trace_mm_vmscan_lru_gen_advance(pgdat->node_id, lrugen->max_seq,
					lrugen->min_seq[LRU_GEN_ANON],
					lrugen->min_seq[LRU_GEN_FILE],
					lru_gen_count_old(lrugen, LRU_GEN_ANON),
					lru_gen_count_old(lrugen, LRU_GEN_FILE));

	count_vm_event(MGLRU_AGED);
}

static unsigned long lru_gen_count_old(struct lru_gen_struct *lrugen, int type)
{
	unsigned int zone;
	unsigned long total = 0;
	unsigned int gen = lrugen->min_seq[type] % MAX_NR_GENS;

	for (zone = 0; zone < MAX_NR_ZONES; zone++)
		total += max_t(long, 0, lrugen->nr_pages[gen][type][zone]);

	return total;
}

static int lru_gen_lru_type(enum lru_list lru)
{
	return is_file_lru(lru) ? LRU_GEN_FILE : LRU_GEN_ANON;
}

static void lru_gen_scan_current_mm(struct lruvec *lruvec, struct scan_control *sc)
{
	(void)lruvec;
	(void)sc;

	/*
	 * Emergency stability mode for 4.19 backports: disable MM page-table
	 * sampling entirely. Keep the hook for future re-enable work, but avoid
	 * mmap_sem/walk_page_range()/young-bit interactions in production.
	 */
	return;
}
void lru_gen_update_size(struct lruvec *lruvec, enum lru_list lru,
			 enum zone_type zid, long delta)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned long seq;
	unsigned int type, gen;
	long *nr_pages;

	if (!lru_gen_enabled() || !lru_gen_get_state() || lru == LRU_UNEVICTABLE || !delta)
		return;

	lockdep_assert_held(&pgdat->lru_lock);

	type = lru_gen_lru_type(lru);
	seq = is_active_lru(lru) ? lrugen->max_seq : lrugen->min_seq[type];
	gen = seq % MAX_NR_GENS;
	nr_pages = &lrugen->nr_pages[gen][type][zid];

	*nr_pages += delta;
	if (*nr_pages < 0)
		*nr_pages = 0;
	if (delta > 0)
		lrugen->timestamps[gen] = jiffies;
}

static void lru_gen_decay_pressure(struct lru_gen_struct *lrugen)
{
	unsigned int type;

	for (type = 0; type < ANON_AND_FILE; type++)
		lrugen->pressure[type] -= lrugen->pressure[type] >> 3;
}

static void lru_gen_normalize_pressure(struct lru_gen_struct *lrugen)
{
	unsigned long floor, type;

	if (!READ_ONCE(lru_gen_pressure_normalize))
		return;

	floor = min(lrugen->pressure[LRU_GEN_ANON], lrugen->pressure[LRU_GEN_FILE]);
	if (!floor || floor <= SWAP_CLUSTER_MAX)
		return;

	for (type = 0; type < ANON_AND_FILE; type++) {
		lrugen->pressure[type] -= floor - SWAP_CLUSTER_MAX;
		lrugen->normalized[type]++;
	}
}

static bool lru_gen_dedup_access(struct lru_gen_struct *lrugen, unsigned int type,
				 unsigned long nr_pages)
{
	unsigned long now = jiffies;
	unsigned long win = READ_ONCE(lru_gen_dedup_window_ms);

	if (!win)
		return false;

	if (time_before(now, lrugen->access_stamp[type] + msecs_to_jiffies(win))) {
		lrugen->deduped[type] += nr_pages;
		count_vm_events(MGLRU_DEDUPED, nr_pages);
		return true;
	}

	lrugen->access_stamp[type] = now;
	return false;
}

static bool lru_gen_min_ttl_protected(struct lru_gen_struct *lrugen, int type)
{
	unsigned long min_ttl = READ_ONCE(lru_gen_min_ttl_ms);
	unsigned long min_ttl_jiffies;
	unsigned int gen;
	unsigned long age;

	if (!min_ttl)
		return false;

	min_ttl_jiffies = msecs_to_jiffies(min_ttl);
	gen = lrugen->min_seq[type] % MAX_NR_GENS;
	age = jiffies - lrugen->timestamps[gen];

	return age < min_ttl_jiffies && lru_gen_count_old(lrugen, type);
}

static bool lru_gen_should_age(struct lru_gen_struct *lrugen)
{
	unsigned long gen = lrugen->max_seq % MAX_NR_GENS;
	unsigned long age = jiffies - lrugen->timestamps[gen];
	unsigned long age_period = msecs_to_jiffies(READ_ONCE(lru_gen_age_period_ms));
	unsigned long anon_old = lru_gen_count_old(lrugen, LRU_GEN_ANON);
	unsigned long file_old = lru_gen_count_old(lrugen, LRU_GEN_FILE);

	if (age >= max_t(unsigned long, 1, age_period))
		return true;

	/*
	 * Keep aging forward if the oldest generation accumulates enough
	 * pages, even under light reclaim pressure.
	 */
	if (anon_old + file_old >= SWAP_CLUSTER_MAX * 32)
		return true;

	return lrugen->pressure[LRU_GEN_ANON] + lrugen->pressure[LRU_GEN_FILE] >=
	       SWAP_CLUSTER_MAX * 16;
}

static unsigned int lru_gen_pick_scan_type(struct lru_gen_struct *lrugen)
{
	unsigned long anon = lrugen->pressure[LRU_GEN_ANON] + 1;
	unsigned long file = lrugen->pressure[LRU_GEN_FILE] + 1;
	unsigned long anon_old = lru_gen_count_old(lrugen, LRU_GEN_ANON);
	unsigned long file_old = lru_gen_count_old(lrugen, LRU_GEN_FILE);

	if (!total_swap_pages)
		return LRU_GEN_FILE;

	anon += anon_old / SWAP_CLUSTER_MAX;
	file += file_old / SWAP_CLUSTER_MAX;
	anon += lrugen->tiers[LRU_GEN_ANON] * SWAP_CLUSTER_MAX;
	file += lrugen->tiers[LRU_GEN_FILE] * SWAP_CLUSTER_MAX;

	return anon >= file ? LRU_GEN_ANON : LRU_GEN_FILE;
}

static bool lru_gen_oldest_empty(struct lru_gen_struct *lrugen, unsigned int type)
{
	unsigned int gen = lrugen->min_seq[type] % MAX_NR_GENS;
	unsigned int zid;

	for (zid = 0; zid < MAX_NR_ZONES; zid++) {
		if (!list_empty(&lrugen->lists[gen][type][zid]))
			return false;
	}

	return true;
}

bool lru_gen_enabled(void)
{
	return static_branch_likely(&lru_gen_caps);
}

int lru_gen_get_state(void)
{
	return READ_ONCE(lru_gen_boot_enabled);
}

int lru_gen_set_state(bool enable)
{
	bool old = READ_ONCE(lru_gen_boot_enabled);

	if (old == enable)
		return 0;

	WRITE_ONCE(lru_gen_boot_enabled, enable);
	if (enable)
		static_branch_enable(&lru_gen_caps);
	else
		static_branch_disable(&lru_gen_caps);

	/*
	 * Avoid carrying stale generation timestamps and pressure signals
	 * across runtime toggles; restart tracking from a clean baseline.
	 */
	lru_gen_reset_all_lruvecs();

	return 0;
}

int lru_gen_set_min_ttl(unsigned int ttl_ms)
{
	WRITE_ONCE(lru_gen_min_ttl_ms, ttl_ms);
	return 0;
}

unsigned int lru_gen_get_min_ttl(void)
{
	return READ_ONCE(lru_gen_min_ttl_ms);
}

int lru_gen_set_age_period(unsigned int period_ms)
{
	WRITE_ONCE(lru_gen_age_period_ms, clamp_t(unsigned int, period_ms, 100, 60000));
	return 0;
}

unsigned int lru_gen_get_age_period(void)
{
	return READ_ONCE(lru_gen_age_period_ms);
}

int lru_gen_set_weight_anon(unsigned int anon_pct)
{
	WRITE_ONCE(lru_gen_weight_anon_pct, min_t(unsigned int, anon_pct, 100));
	return 0;
}

unsigned int lru_gen_get_weight_anon(void)
{
	return READ_ONCE(lru_gen_weight_anon_pct);
}

int lru_gen_set_dedup_window(unsigned int window_ms)
{
	WRITE_ONCE(lru_gen_dedup_window_ms, min_t(unsigned int, window_ms, 1000));
	return 0;
}

unsigned int lru_gen_get_dedup_window(void)
{
	return READ_ONCE(lru_gen_dedup_window_ms);
}

int lru_gen_set_normalize(bool enable)
{
	WRITE_ONCE(lru_gen_pressure_normalize, enable);
	return 0;
}

int lru_gen_get_normalize(void)
{
	return READ_ONCE(lru_gen_pressure_normalize);
}

int lru_gen_set_ptwalk_pages(unsigned int pages)
{
	WRITE_ONCE(lru_gen_ptwalk_pages,
		   clamp_t(unsigned int, pages, 32, 16384));
	return 0;
}

unsigned int lru_gen_get_ptwalk_pages(void)
{
	return READ_ONCE(lru_gen_ptwalk_pages);
}

int lru_gen_set_reclaim_ptwalk(bool enable)
{
	WRITE_ONCE(lru_gen_reclaim_ptwalk, enable);
	return 0;
}

int lru_gen_get_reclaim_ptwalk(void)
{
	return READ_ONCE(lru_gen_reclaim_ptwalk);
}

int lru_gen_set_reclaim_feedback(bool enable)
{
	WRITE_ONCE(lru_gen_reclaim_feedback, enable);
	return 0;
}

int lru_gen_get_reclaim_feedback(void)
{
	return READ_ONCE(lru_gen_reclaim_feedback);
}

int lru_gen_set_reclaim_advance(bool enable)
{
	WRITE_ONCE(lru_gen_reclaim_advance, enable);
	return 0;
}

int lru_gen_get_reclaim_advance(void)
{
	return READ_ONCE(lru_gen_reclaim_advance);
}

int lru_gen_set_ptwalk_clear_young(bool enable)
{
	WRITE_ONCE(lru_gen_ptwalk_clear_young, enable);
	return 0;
}

int lru_gen_get_ptwalk_clear_young(void)
{
	return READ_ONCE(lru_gen_ptwalk_clear_young);
}

void lru_gen_init_lruvec(struct lruvec *lruvec)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen, type, zone;

	for (gen = 0; gen < MAX_NR_GENS; gen++) {
		lrugen->timestamps[gen] = jiffies;

		for (type = 0; type < ANON_AND_FILE; type++) {
			for (zone = 0; zone < MAX_NR_ZONES; zone++) {
				INIT_LIST_HEAD(&lrugen->lists[gen][type][zone]);
				lrugen->nr_pages[gen][type][zone] = 0;
			}
		}
	}

	lrugen->max_seq = MIN_NR_GENS + 1;
	lrugen->min_seq[LRU_GEN_ANON] = lrugen->max_seq - MIN_NR_GENS;
	lrugen->min_seq[LRU_GEN_FILE] = lrugen->max_seq - MIN_NR_GENS;
	lrugen->pressure[LRU_GEN_ANON] = 1;
	lrugen->pressure[LRU_GEN_FILE] = 1;
	lrugen->tiers[LRU_GEN_ANON] = 0;
	lrugen->tiers[LRU_GEN_FILE] = 0;
	lrugen->scanned[LRU_GEN_ANON] = 0;
	lrugen->scanned[LRU_GEN_FILE] = 0;
	lrugen->reclaimed[LRU_GEN_ANON] = 0;
	lrugen->reclaimed[LRU_GEN_FILE] = 0;
	lrugen->accessed[LRU_GEN_ANON] = 0;
	lrugen->accessed[LRU_GEN_FILE] = 0;
	lrugen->evicted[LRU_GEN_ANON] = 0;
	lrugen->evicted[LRU_GEN_FILE] = 0;
	lrugen->access_stamp[LRU_GEN_ANON] = jiffies;
	lrugen->access_stamp[LRU_GEN_FILE] = jiffies;
	lrugen->deduped[LRU_GEN_ANON] = 0;
	lrugen->deduped[LRU_GEN_FILE] = 0;
	lrugen->normalized[LRU_GEN_ANON] = 0;
	lrugen->normalized[LRU_GEN_FILE] = 0;
	lrugen->last_reclaim = jiffies;
	lrugen->mm_walk_seq = jiffies;
	lrugen->mm_walk_success = 0;
	lrugen->mm_walk_failures = 0;
	lrugen->mm_walk_fallback = 0;
	lrugen->mm_walk_sampled_ptes = 0;
	lrugen->mm_walk_young_cleared = 0;
	lrugen->reclaim_stall = 0;
	lrugen->last_scanned = 0;
	lrugen->last_reclaimed = 0;
	lrugen->last_efficiency = 0;
}

void lru_gen_track_page_scan(struct lruvec *lruvec, enum lru_list lru,
			     unsigned long nr_scanned, unsigned long nr_taken,
			     unsigned long nr_reclaimed)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned long gen;
	unsigned long delta;
	int type;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !nr_scanned)
		return;

	type = lru_gen_lru_type(lru);

	/*
	 * MGLRU classification stage:
	 * - treat active LRU pages as "young" (max_seq)
	 * - treat inactive LRU pages as "old" (min_seq[type])
	 * - bias pressure according to isolation and reclaim efficiency
	 */
	gen = (is_active_lru(lru) ? lrugen->max_seq : lrugen->min_seq[type]) %
	      MAX_NR_GENS;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	lrugen->timestamps[gen] = jiffies;
	delta = nr_taken + nr_scanned - min(nr_reclaimed, nr_taken);
	lru_gen_bump_pressure(lrugen, type, delta);
	lrugen->scanned[type] += nr_scanned;
	lrugen->reclaimed[type] += nr_reclaimed;
	if (is_active_lru(lru) && lrugen->pressure[type] > nr_reclaimed)
		lrugen->pressure[type] -= nr_reclaimed;
	if (!is_active_lru(lru) && nr_reclaimed)
		lrugen->evicted[type] += nr_reclaimed;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_note_access(struct lruvec *lruvec, bool file)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int type = file ? LRU_GEN_FILE : LRU_GEN_ANON;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	if (lru_gen_dedup_access(lrugen, type, 1)) {
		spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
		return;
	}
	lrugen->accessed[type]++;
	count_vm_event(MGLRU_ACTIVATED);
	/*
	 * Age generations when enough new accesses arrive, so the reclaim
	 * side can keep selecting truly older generations.
	 */
	if (lrugen->accessed[type] >= SWAP_CLUSTER_MAX * 4) {
		lrugen->accessed[type] = 0;
		lru_gen_advance_seq(lruvec);
	}
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_note_page_referenced(struct lruvec *lruvec, struct page *page,
				  bool from_reclaim)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned int type;
	unsigned long nr_pages;
	unsigned long threshold;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	type = page_is_file_cache(page) ? LRU_GEN_FILE : LRU_GEN_ANON;
	nr_pages = hpage_nr_pages(page);
	threshold = SWAP_CLUSTER_MAX * (from_reclaim ? 2 : 4);

	spin_lock_irq(&pgdat->lru_lock);
	if (lru_gen_dedup_access(lrugen, type, nr_pages)) {
		spin_unlock_irq(&pgdat->lru_lock);
		return;
	}

	lrugen->accessed[type] += nr_pages;
	count_vm_events(MGLRU_ACTIVATED, nr_pages);
	/*
	 * Reclaim-driven references are collected from page table walks
	 * (via page_referenced()), so use them as a direct aging signal.
	 */
	if (from_reclaim)
		lru_gen_bump_pressure(lrugen, type,
				      min_t(unsigned long, nr_pages,
					    SWAP_CLUSTER_MAX));

	if (lrugen->accessed[type] >= threshold) {
		lrugen->accessed[type] = 0;
		lru_gen_advance_seq(lruvec);
	}
	spin_unlock_irq(&pgdat->lru_lock);
}

void lru_gen_note_lru_move(struct lruvec *lruvec, enum lru_list old_lru,
			   enum lru_list new_lru, unsigned long nr_pages)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned int old_type, new_type;
	unsigned long old_seq;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !nr_pages)
		return;

	if (old_lru == LRU_UNEVICTABLE || new_lru == LRU_UNEVICTABLE)
		return;

	lockdep_assert_held(&pgdat->lru_lock);

	old_type = lru_gen_lru_type(old_lru);
	new_type = lru_gen_lru_type(new_lru);

	old_seq = is_active_lru(old_lru) ? lrugen->max_seq : lrugen->min_seq[old_type];
	if (!is_active_lru(new_lru) && old_type == new_type &&
	    old_seq == lrugen->min_seq[new_type])
		lrugen->evicted[new_type] += nr_pages;

	if (!is_active_lru(new_lru) && is_active_lru(old_lru))
		count_vm_events(MGLRU_EVICTED, nr_pages);
}

void lru_gen_enter_reclaim(struct lruvec *lruvec, struct scan_control *sc)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen;
	unsigned int type;

	(void)sc;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	/*
	 * Feed a bounded amount of page-table access information from
	 * direct reclaim contexts into generation aging.
	 */
	lru_gen_scan_current_mm(lruvec, sc);

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	gen = lrugen->max_seq % MAX_NR_GENS;
	if (time_before(lrugen->timestamps[gen] + HZ / 4, jiffies))
		lrugen->timestamps[gen] = jiffies;
	if (lru_gen_should_age(lrugen)) {
		lrugen->evicted[LRU_GEN_ANON] = 0;
		lrugen->evicted[LRU_GEN_FILE] = 0;
		if (READ_ONCE(lru_gen_reclaim_advance))
			lru_gen_advance_seq(lruvec);
	}

	/*
	 * Reclaim path for the oldest generations:
	 * if an oldest generation is already drained, move its floor forward so
	 * reclaim naturally shifts to the next oldest generation.
	 */
	for (type = 0; type < ANON_AND_FILE; type++) {
		if (lru_gen_oldest_empty(lrugen, type) &&
		    lrugen->min_seq[type] < lrugen->max_seq)
			lrugen->min_seq[type]++;
		lru_gen_clamp_min_seq(lrugen, type);
	}
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_adjust_scan(struct lruvec *lruvec, struct scan_control *sc,
			 unsigned long *nr)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	unsigned long flags;
	unsigned long anon_scan;
	unsigned long file_scan;
	unsigned long total;
	unsigned long denom;
	unsigned long anon_weight;
	unsigned long file_weight;
	unsigned long anon_old = 0;
	unsigned long file_old = 0;
	unsigned int preferred;
	bool feedback_enabled;
	bool emergency_fallback = false;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	anon_scan = nr[LRU_INACTIVE_ANON] + nr[LRU_ACTIVE_ANON];
	file_scan = nr[LRU_INACTIVE_FILE] + nr[LRU_ACTIVE_FILE];
	total = anon_scan + file_scan;
	if (!total)
		return;

	spin_lock_irqsave(&pgdat->lru_lock, flags);
	feedback_enabled = READ_ONCE(lru_gen_reclaim_feedback);

	/*
	 * Eviction decision stage:
	 * steer scan pressure toward colder generations using pressure[].
	 * Keep at least one cluster of each type to avoid starvation.
	 */
	anon_weight = lrugen->pressure[LRU_GEN_ANON] + 1;
	file_weight = lrugen->pressure[LRU_GEN_FILE] + 1;

	anon_old = lru_gen_count_old(lrugen, LRU_GEN_ANON);
	file_old = lru_gen_count_old(lrugen, LRU_GEN_FILE);

	/*
	 * Safety valve for legacy backports: if reclaim cycles repeatedly make
	 * no progress, stop MGLRU scan skewing and let classic vmscan reclaim
	 * balancing take over until pressure feedback recovers.
	 */
	if (feedback_enabled && lrugen->reclaim_stall >= 8) {
		emergency_fallback = true;
		goto unlock;
	}

	anon_weight += anon_old / SWAP_CLUSTER_MAX;
	file_weight += file_old / SWAP_CLUSTER_MAX;
	anon_weight = anon_weight * READ_ONCE(lru_gen_weight_anon_pct);
	file_weight = file_weight * (100 - READ_ONCE(lru_gen_weight_anon_pct));

	/*
	 * Respect optional working-set protection: until the oldest generation
	 * reaches min_ttl, hard-throttle it and let the peer type absorb scan.
	 */
	if (lru_gen_min_ttl_protected(lrugen, LRU_GEN_ANON))
		anon_weight = 0;
	if (lru_gen_min_ttl_protected(lrugen, LRU_GEN_FILE))
		file_weight = 0;

	if (anon_old > file_old)
		anon_weight += SWAP_CLUSTER_MAX / 2;
	else if (file_old > anon_old)
		file_weight += SWAP_CLUSTER_MAX / 2;

	if (!total_swap_pages)
		anon_weight = 0;
	preferred = lru_gen_pick_scan_type(lrugen);

	if (preferred == LRU_GEN_ANON)
		anon_weight += SWAP_CLUSTER_MAX;
	else
		file_weight += SWAP_CLUSTER_MAX;
unlock:
	spin_unlock_irqrestore(&pgdat->lru_lock, flags);

	if (emergency_fallback)
		return;

	denom = anon_weight + file_weight;
	if (!denom)
		return;

	anon_scan = max_t(unsigned long, anon_scan,
			  !anon_weight ? 0 : SWAP_CLUSTER_MAX);
	file_scan = max_t(unsigned long, file_scan,
			  !file_weight ? 0 : SWAP_CLUSTER_MAX);

	if (anon_weight)
		anon_scan = max_t(unsigned long, SWAP_CLUSTER_MAX,
				  div64_u64((u64)total * anon_weight, denom));
	else
		anon_scan = 0;
	anon_scan = min(anon_scan, total);
	file_scan = total - anon_scan;

	nr[LRU_INACTIVE_ANON] = min(nr[LRU_INACTIVE_ANON], anon_scan);
	nr[LRU_ACTIVE_ANON] = anon_scan - nr[LRU_INACTIVE_ANON];
	nr[LRU_INACTIVE_FILE] = min(nr[LRU_INACTIVE_FILE], file_scan);
	nr[LRU_ACTIVE_FILE] = file_scan - nr[LRU_INACTIVE_FILE];
}

void lru_gen_tune_memcg(struct lruvec *lruvec, struct scan_control *sc,
			unsigned long reclaimed, unsigned long scanned)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	struct pglist_data *pgdat = lruvec_pgdat(lruvec);
	bool ptwalk_reclaim_enabled = READ_ONCE(lru_gen_reclaim_ptwalk);
	bool feedback_enabled = READ_ONCE(lru_gen_reclaim_feedback);
	unsigned long flags;
	unsigned int anon_tier, file_tier;
	unsigned long efficiency, split;
	unsigned long now = jiffies;
	unsigned long anon_eff = 0, file_eff = 0;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !scanned)
		return;

	efficiency = reclaimed * 100 / scanned;
	spin_lock_irqsave(&pgdat->lru_lock, flags);
	lrugen->last_scanned = scanned;
	lrugen->last_reclaimed = reclaimed;
	lrugen->last_efficiency = min_t(unsigned int, efficiency, 100U);

	/*
	 * Keep reclaim feedback optional on 4.19 backports. Disabling this
	 * path removes tier/reclaim_stall feedback and reclaim-driven aging
	 * from direct reclaim, while still recording reclaim efficiency and
	 * applying pressure decay/normalization.
	 */
	if (feedback_enabled) {
		/* memcg/tier tuning stage */
		anon_tier = min_t(unsigned int, MAX_NR_GENS - 1,
				  lrugen->pressure[LRU_GEN_ANON] /
					  (8 * SWAP_CLUSTER_MAX));
		file_tier = min_t(unsigned int, MAX_NR_GENS - 1,
				  lrugen->pressure[LRU_GEN_FILE] /
					  (8 * SWAP_CLUSTER_MAX));

		if (efficiency < 10) {
			anon_tier = min_t(unsigned int, anon_tier + 1,
					  MAX_NR_GENS - 1);
			file_tier = min_t(unsigned int, file_tier + 1,
					  MAX_NR_GENS - 1);
		} else if (efficiency > 50) {
			anon_tier = anon_tier ? anon_tier - 1 : 0;
			file_tier = file_tier ? file_tier - 1 : 0;
		}

		/*
		 * When reclaim-time ptwalk is enabled, avoid feeding no-progress
		 * stalls back into pressure escalation. This prevents positive
		 * feedback loops between MM walk hints and reclaim aggressiveness.
		 */
		if (!ptwalk_reclaim_enabled && !reclaimed &&
		    scanned >= SWAP_CLUSTER_MAX)
			lrugen->reclaim_stall = min_t(unsigned int,
						      lrugen->reclaim_stall + 1,
						      16);
		else if (lrugen->reclaim_stall)
			lrugen->reclaim_stall >>= 1;

		if (!ptwalk_reclaim_enabled && lrugen->reclaim_stall >= 4) {
			lru_gen_bump_pressure(lrugen, LRU_GEN_ANON,
					      SWAP_CLUSTER_MAX / 2);
			lru_gen_bump_pressure(lrugen, LRU_GEN_FILE,
					      SWAP_CLUSTER_MAX / 2);
		}

		lrugen->tiers[LRU_GEN_ANON] = anon_tier;
		lrugen->tiers[LRU_GEN_FILE] = file_tier;

		split = max_t(unsigned long, 1, scanned / 2);
		if (reclaimed)
			anon_eff = min_t(unsigned long, reclaimed, split) * 100 /
				   split;
		if (scanned > split)
			file_eff = (reclaimed > split ? reclaimed - split : 0) *
				   100 / (scanned - split);

		if (anon_eff + 5 < file_eff)
			lru_gen_bump_pressure(lrugen, LRU_GEN_ANON,
					      SWAP_CLUSTER_MAX / 2);
		else if (file_eff + 5 < anon_eff)
			lru_gen_bump_pressure(lrugen, LRU_GEN_FILE,
					      SWAP_CLUSTER_MAX / 2);
	}

	lru_gen_decay_pressure(lrugen);
	lru_gen_normalize_pressure(lrugen);
	if (!feedback_enabled) {
		spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		return;
	}

	trace_mm_vmscan_lru_gen_feedback(pgdat->node_id,
					 lrugen->pressure[LRU_GEN_ANON],
					 lrugen->pressure[LRU_GEN_FILE],
					 lrugen->tiers[LRU_GEN_ANON],
					 lrugen->tiers[LRU_GEN_FILE],
					 lrugen->scanned[LRU_GEN_ANON],
					 lrugen->scanned[LRU_GEN_FILE],
					 lrugen->reclaimed[LRU_GEN_ANON],
					 lrugen->reclaimed[LRU_GEN_FILE]);

	/* Tier-aware aging under weak reclaim or high pressure tiers. */
	if (efficiency < 20 || anon_tier + file_tier >= MAX_NR_GENS ||
	    (!ptwalk_reclaim_enabled && lrugen->reclaim_stall >= 6) ||
	    now - lrugen->last_reclaim >= HZ)
		lru_gen_advance_seq(lruvec);

	lrugen->last_reclaim = now;
	spin_unlock_irqrestore(&pgdat->lru_lock, flags);
}

#if defined(CONFIG_DEBUG_FS) || defined(CONFIG_PROC_FS)
static void lru_gen_reset_lruvec(struct lruvec *lruvec)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int type;

	for (type = 0; type < ANON_AND_FILE; type++) {
		lrugen->pressure[type] = 1;
		lrugen->tiers[type] = 0;
		lrugen->scanned[type] = 0;
		lrugen->reclaimed[type] = 0;
		lrugen->accessed[type] = 0;
		lrugen->evicted[type] = 0;
		lrugen->access_stamp[type] = jiffies;
		lrugen->deduped[type] = 0;
		lrugen->normalized[type] = 0;
	}
	lrugen->mm_walk_seq = jiffies;
	lrugen->mm_walk_success = 0;
	lrugen->mm_walk_failures = 0;
	lrugen->mm_walk_fallback = 0;
	lrugen->mm_walk_sampled_ptes = 0;
	lrugen->mm_walk_young_cleared = 0;
	lrugen->reclaim_stall = 0;
	lrugen->last_scanned = 0;
	lrugen->last_reclaimed = 0;
	lrugen->last_efficiency = 0;
}
#endif

#ifdef CONFIG_DEBUG_FS
static unsigned long mglru_gen_total(struct lru_gen_struct *lrugen,
				     unsigned int gen, unsigned int type)
{
	unsigned int zone;
	unsigned long total = 0;

	for (zone = 0; zone < MAX_NR_ZONES; zone++)
		total += max_t(long, 0, lrugen->nr_pages[gen][type][zone]);

	return total;
}

static int mglru_stats_show(struct seq_file *m, void *v)
{
	struct pglist_data *pgdat;

	seq_printf(m,
		   "enabled=%d min_ttl_ms=%u age_period_ms=%u weight_anon_pct=%u dedup_window_ms=%u normalize=%d ptwalk_pages=%u reclaim_ptwalk=%d reclaim_feedback=%d reclaim_advance=%d ptwalk_clear_young=%d\n",
		   lru_gen_get_state(), lru_gen_get_min_ttl(),
		   lru_gen_get_age_period(), lru_gen_get_weight_anon(),
		   lru_gen_get_dedup_window(), lru_gen_get_normalize(),
		   lru_gen_get_ptwalk_pages(), lru_gen_get_reclaim_ptwalk(),
		   lru_gen_get_reclaim_feedback(),
		   lru_gen_get_reclaim_advance(),
		   lru_gen_get_ptwalk_clear_young());

	for_each_online_pgdat(pgdat) {
		struct lruvec *lruvec = node_lruvec(pgdat);
		struct lru_gen_struct *lrugen = &lruvec->lrugen;
		unsigned long flags;
		unsigned long anon_age, file_age;
		unsigned long anon_old, file_old;
		unsigned long global_mm_walk_age;

		spin_lock_irqsave(&pgdat->lru_lock, flags);
		anon_age = jiffies -
			   lrugen->timestamps[lrugen->min_seq[LRU_GEN_ANON] % MAX_NR_GENS];
		file_age = jiffies -
			   lrugen->timestamps[lrugen->min_seq[LRU_GEN_FILE] % MAX_NR_GENS];
		anon_old = lru_gen_count_old(lrugen, LRU_GEN_ANON);
		file_old = lru_gen_count_old(lrugen, LRU_GEN_FILE);
		global_mm_walk_age = jiffies - lru_gen_mm_walk_seq[pgdat->node_id];

		seq_printf(m,
			   "node=%d max_seq=%lu min_seq=(anon:%lu file:%lu) old=(anon:%lu file:%lu) age_jiffies=(anon:%lu file:%lu)\n",
			   pgdat->node_id, lrugen->max_seq,
			   lrugen->min_seq[LRU_GEN_ANON],
			   lrugen->min_seq[LRU_GEN_FILE], anon_old, file_old,
			   anon_age, file_age);
		seq_printf(m,
			   "  pressure=(anon:%lu file:%lu) tier=(anon:%u file:%u) scanned=(anon:%lu file:%lu) reclaimed=(anon:%lu file:%lu) evicted=(anon:%lu file:%lu) deduped=(anon:%lu file:%lu) normalized=(anon:%lu file:%lu)\n",
			   lrugen->pressure[LRU_GEN_ANON],
			   lrugen->pressure[LRU_GEN_FILE],
			   lrugen->tiers[LRU_GEN_ANON],
			   lrugen->tiers[LRU_GEN_FILE],
			   lrugen->scanned[LRU_GEN_ANON],
			   lrugen->scanned[LRU_GEN_FILE],
			   lrugen->reclaimed[LRU_GEN_ANON],
			   lrugen->reclaimed[LRU_GEN_FILE],
			   lrugen->evicted[LRU_GEN_ANON],
			   lrugen->evicted[LRU_GEN_FILE],
			   lrugen->deduped[LRU_GEN_ANON],
			   lrugen->deduped[LRU_GEN_FILE],
			   lrugen->normalized[LRU_GEN_ANON],
			   lrugen->normalized[LRU_GEN_FILE]);
		seq_printf(m,
			   "  mm_walk=(ok:%lu fail:%lu fallback:%lu sampled:%lu young_cleared:%lu global_age_jiffies:%lu) stall=%u last_cycle=(scanned:%lu reclaimed:%lu efficiency:%u%%)\n",
			   lrugen->mm_walk_success, lrugen->mm_walk_failures,
			   lrugen->mm_walk_fallback, lrugen->mm_walk_sampled_ptes,
			   lrugen->mm_walk_young_cleared, global_mm_walk_age,
			   lrugen->reclaim_stall,
			   lrugen->last_scanned, lrugen->last_reclaimed,
			   lrugen->last_efficiency);
		seq_printf(m,
			   "  protected=(anon:%d file:%d)\n",
			   lru_gen_min_ttl_protected(lrugen, LRU_GEN_ANON),
			   lru_gen_min_ttl_protected(lrugen, LRU_GEN_FILE));
		seq_printf(m,
			   "  gens=(g0 anon:%lu file:%lu) (g1 anon:%lu file:%lu) (g2 anon:%lu file:%lu) (g3 anon:%lu file:%lu)\n",
			   mglru_gen_total(lrugen, 0, LRU_GEN_ANON),
			   mglru_gen_total(lrugen, 0, LRU_GEN_FILE),
			   mglru_gen_total(lrugen, 1, LRU_GEN_ANON),
			   mglru_gen_total(lrugen, 1, LRU_GEN_FILE),
			   mglru_gen_total(lrugen, 2, LRU_GEN_ANON),
			   mglru_gen_total(lrugen, 2, LRU_GEN_FILE),
			   mglru_gen_total(lrugen, 3, LRU_GEN_ANON),
			   mglru_gen_total(lrugen, 3, LRU_GEN_FILE));
		spin_unlock_irqrestore(&pgdat->lru_lock, flags);
	}

	return 0;
}

static ssize_t mglru_stats_write(struct file *file, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	char cmd[64];
	struct pglist_data *pgdat;
	unsigned int val;

	if (count >= sizeof(cmd))
		return -EINVAL;

	if (copy_from_user(cmd, buf, count))
		return -EFAULT;

	cmd[count] = '\0';
	strim(cmd);

	if (!strcmp(cmd, "age")) {
		for_each_online_pgdat(pgdat) {
			struct lruvec *lruvec = node_lruvec(pgdat);
			unsigned long flags;

			spin_lock_irqsave(&pgdat->lru_lock, flags);
			lru_gen_advance_seq(lruvec);
			spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		}
		return count;
	}

	if (sscanf(cmd, "age=%u", &val) == 1) {
		unsigned int i, steps = min_t(unsigned int, val, MAX_NR_GENS);

		if (!steps)
			return -EINVAL;

		for_each_online_pgdat(pgdat) {
			struct lruvec *lruvec = node_lruvec(pgdat);
			unsigned long flags;

			spin_lock_irqsave(&pgdat->lru_lock, flags);
			for (i = 0; i < steps; i++)
				lru_gen_advance_seq(lruvec);
			spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		}
		return count;
	}

	if (!strcmp(cmd, "enable"))
		return lru_gen_set_state(true) ? : count;

	if (!strcmp(cmd, "disable"))
		return lru_gen_set_state(false) ? : count;

	if (!strcmp(cmd, "reset")) {
		for_each_online_pgdat(pgdat) {
			struct lruvec *lruvec = node_lruvec(pgdat);
			unsigned long flags;

			spin_lock_irqsave(&pgdat->lru_lock, flags);
			lru_gen_reset_lruvec(lruvec);
			spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		}
		return count;
	}

	if (sscanf(cmd, "min_ttl_ms=%u", &val) == 1)
		return lru_gen_set_min_ttl(val) ? : count;

	if (sscanf(cmd, "age_period_ms=%u", &val) == 1)
		return lru_gen_set_age_period(val) ? : count;

	if (sscanf(cmd, "weight_anon_pct=%u", &val) == 1)
		return lru_gen_set_weight_anon(val) ? : count;

	if (sscanf(cmd, "dedup_window_ms=%u", &val) == 1)
		return lru_gen_set_dedup_window(val) ? : count;

	if (sscanf(cmd, "normalize=%u", &val) == 1)
		return lru_gen_set_normalize(!!val) ? : count;

	if (sscanf(cmd, "ptwalk_pages=%u", &val) == 1)
		return lru_gen_set_ptwalk_pages(val) ? : count;

	if (sscanf(cmd, "reclaim_ptwalk=%u", &val) == 1)
		return lru_gen_set_reclaim_ptwalk(!!val) ? : count;

	if (sscanf(cmd, "reclaim_feedback=%u", &val) == 1)
		return lru_gen_set_reclaim_feedback(!!val) ? : count;

	if (sscanf(cmd, "reclaim_advance=%u", &val) == 1)
		return lru_gen_set_reclaim_advance(!!val) ? : count;

	if (sscanf(cmd, "ptwalk_clear_young=%u", &val) == 1)
		return lru_gen_set_ptwalk_clear_young(!!val) ? : count;

	if (!strcmp(cmd, "sample_mm")) {
		for_each_online_pgdat(pgdat)
			lru_gen_scan_current_mm(node_lruvec(pgdat), NULL);
		return count;
	}

	return -EINVAL;
}

static int mglru_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, mglru_stats_show, inode->i_private);
}

static const struct file_operations mglru_stats_fops = {
	.owner = THIS_MODULE,
	.open = mglru_stats_open,
	.read = seq_read,
	.write = mglru_stats_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init mglru_debugfs_init(void)
{
	if (!debugfs_initialized())
		return 0;

	mglru_debugfs_root = debugfs_create_dir("mglru", NULL);
	if (IS_ERR_OR_NULL(mglru_debugfs_root))
		return -ENOMEM;

	debugfs_create_file("stats", 0644, mglru_debugfs_root, NULL,
			    &mglru_stats_fops);
	return 0;
}
late_initcall(mglru_debugfs_init);
#endif

#ifdef CONFIG_PROC_FS
static int mglru_advance_all_nodes(unsigned int nr_to_advance)
{
	struct pglist_data *pgdat;
	unsigned int i;

	if (!nr_to_advance)
		return -EINVAL;

	for_each_online_pgdat(pgdat) {
		struct lruvec *lruvec = node_lruvec(pgdat);
		unsigned long flags;

		spin_lock_irqsave(&pgdat->lru_lock, flags);
		for (i = 0; i < nr_to_advance; i++)
			lru_gen_advance_seq(lruvec);
		spin_unlock_irqrestore(&pgdat->lru_lock, flags);
	}

	return 0;
}

static void mglru_sample_current_mm_all_nodes(void)
{
	struct pglist_data *pgdat;

	for_each_online_pgdat(pgdat)
		lru_gen_scan_current_mm(node_lruvec(pgdat), NULL);
}

static int mglru_proc_show(struct seq_file *m, void *v)
{
	struct pglist_data *pgdat;

	seq_printf(m,
		   "enabled=%d min_ttl_ms=%u age_period_ms=%u weight_anon_pct=%u dedup_window_ms=%u normalize=%d ptwalk_pages=%u reclaim_ptwalk=%d reclaim_feedback=%d reclaim_advance=%d ptwalk_clear_young=%d\n",
		   lru_gen_get_state(), lru_gen_get_min_ttl(),
		   lru_gen_get_age_period(), lru_gen_get_weight_anon(),
		   lru_gen_get_dedup_window(), lru_gen_get_normalize(),
		   lru_gen_get_ptwalk_pages(), lru_gen_get_reclaim_ptwalk(),
		   lru_gen_get_reclaim_feedback(),
		   lru_gen_get_reclaim_advance(),
		   lru_gen_get_ptwalk_clear_young());

	for_each_online_pgdat(pgdat) {
		struct lruvec *lruvec = node_lruvec(pgdat);
		struct lru_gen_struct *lrugen = &lruvec->lrugen;
		unsigned long flags;
		unsigned long anon_old, file_old;
		unsigned long global_mm_walk_age;

		spin_lock_irqsave(&pgdat->lru_lock, flags);
		anon_old = lru_gen_count_old(lrugen, LRU_GEN_ANON);
		file_old = lru_gen_count_old(lrugen, LRU_GEN_FILE);
		global_mm_walk_age = jiffies - lru_gen_mm_walk_seq[pgdat->node_id];
		seq_printf(m,
			   "node=%d max_seq=%lu min_seq=(anon:%lu file:%lu) old=(anon:%lu file:%lu) pressure=(anon:%lu file:%lu) tier=(anon:%u file:%u) mm_walk=(ok:%lu fail:%lu fallback:%lu sampled:%lu young_cleared:%lu global_age_jiffies:%lu) stall=%u last_cycle=(scanned:%lu reclaimed:%lu efficiency:%u%%)\n",
			   pgdat->node_id, lrugen->max_seq,
			   lrugen->min_seq[LRU_GEN_ANON],
			   lrugen->min_seq[LRU_GEN_FILE], anon_old, file_old,
			   lrugen->pressure[LRU_GEN_ANON],
			   lrugen->pressure[LRU_GEN_FILE],
			   lrugen->tiers[LRU_GEN_ANON],
			   lrugen->tiers[LRU_GEN_FILE],
			   lrugen->mm_walk_success, lrugen->mm_walk_failures,
			   lrugen->mm_walk_fallback, lrugen->mm_walk_sampled_ptes,
			   lrugen->mm_walk_young_cleared, global_mm_walk_age,
			   lrugen->reclaim_stall,
			   lrugen->last_scanned, lrugen->last_reclaimed,
			   lrugen->last_efficiency);
		spin_unlock_irqrestore(&pgdat->lru_lock, flags);
	}

	return 0;
}

static int mglru_apply_command(const char *cmd)
{
	unsigned int val, nr_to_advance = 1;

	if (!strcmp(cmd, "age"))
		return mglru_advance_all_nodes(1);

	if (sscanf(cmd, "age=%u", &nr_to_advance) == 1)
		return mglru_advance_all_nodes(min_t(unsigned int, nr_to_advance, MAX_NR_GENS));

	if (!strcmp(cmd, "reset")) {
		struct pglist_data *pgdat;

		for_each_online_pgdat(pgdat) {
			struct lruvec *lruvec = node_lruvec(pgdat);
			unsigned long flags;

			spin_lock_irqsave(&pgdat->lru_lock, flags);
			lru_gen_reset_lruvec(lruvec);
			spin_unlock_irqrestore(&pgdat->lru_lock, flags);
		}
		return 0;
	}

	if (!strcmp(cmd, "enable"))
		return lru_gen_set_state(true);

	if (!strcmp(cmd, "disable"))
		return lru_gen_set_state(false);

	if (sscanf(cmd, "min_ttl_ms=%u", &val) == 1)
		return lru_gen_set_min_ttl(val);

	if (sscanf(cmd, "age_period_ms=%u", &val) == 1)
		return lru_gen_set_age_period(val);

	if (sscanf(cmd, "weight_anon_pct=%u", &val) == 1)
		return lru_gen_set_weight_anon(val);

	if (sscanf(cmd, "dedup_window_ms=%u", &val) == 1)
		return lru_gen_set_dedup_window(val);

	if (sscanf(cmd, "normalize=%u", &val) == 1)
		return lru_gen_set_normalize(!!val);

	if (sscanf(cmd, "ptwalk_pages=%u", &val) == 1)
		return lru_gen_set_ptwalk_pages(val);

	if (sscanf(cmd, "reclaim_ptwalk=%u", &val) == 1)
		return lru_gen_set_reclaim_ptwalk(!!val);

	if (sscanf(cmd, "reclaim_feedback=%u", &val) == 1)
		return lru_gen_set_reclaim_feedback(!!val);

	if (sscanf(cmd, "reclaim_advance=%u", &val) == 1)
		return lru_gen_set_reclaim_advance(!!val);

	if (sscanf(cmd, "ptwalk_clear_young=%u", &val) == 1)
		return lru_gen_set_ptwalk_clear_young(!!val);

	if (!strcmp(cmd, "sample_mm")) {
		mglru_sample_current_mm_all_nodes();
		return 0;
	}

	return -EINVAL;
}

static ssize_t mglru_proc_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	char cmd[64];
	int ret;

	if (count >= sizeof(cmd))
		return -EINVAL;

	if (copy_from_user(cmd, buf, count))
		return -EFAULT;

	cmd[count] = '\0';
	strim(cmd);
	ret = mglru_apply_command(cmd);

	return ret ? ret : count;
}

static int mglru_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, mglru_proc_show, inode->i_private);
}

static const struct file_operations mglru_proc_fops = {
	.owner = THIS_MODULE,
	.open = mglru_proc_open,
	.read = seq_read,
	.write = mglru_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init mglru_proc_init(void)
{
	mglru_proc_entry = proc_create("lru_gen", 0644, NULL, &mglru_proc_fops);
	return mglru_proc_entry ? 0 : -ENOMEM;
}
late_initcall(mglru_proc_init);
#endif

#ifdef CONFIG_SYSFS
static ssize_t enabled_show(struct kobject *kobj, struct kobj_attribute *attr,
			    char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_state());
}

static ssize_t enabled_store(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_state(val);
	return ret ? ret : count;
}

static ssize_t min_ttl_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
			       char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%u\n", lru_gen_get_min_ttl());
}

static ssize_t min_ttl_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_min_ttl(val);
	return ret ? ret : count;
}

static ssize_t age_period_ms_show(struct kobject *kobj, struct kobj_attribute *attr,
				  char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%u\n", lru_gen_get_age_period());
}

static ssize_t age_period_ms_store(struct kobject *kobj, struct kobj_attribute *attr,
				   const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_age_period(val);
	return ret ? ret : count;
}

static ssize_t weight_anon_pct_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%u\n", lru_gen_get_weight_anon());
}

static ssize_t weight_anon_pct_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_weight_anon(val);
	return ret ? ret : count;
}

static ssize_t dedup_window_ms_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%u\n", lru_gen_get_dedup_window());
}

static ssize_t dedup_window_ms_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_dedup_window(val);
	return ret ? ret : count;
}

static ssize_t pressure_normalize_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_normalize());
}

static ssize_t pressure_normalize_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_normalize(val);
	return ret ? ret : count;
}

static ssize_t ptwalk_pages_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%u\n", lru_gen_get_ptwalk_pages());
}

static ssize_t ptwalk_pages_store(struct kobject *kobj,
				  struct kobj_attribute *attr,
				  const char *buf, size_t count)
{
	unsigned int val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_ptwalk_pages(val);
	return ret ? ret : count;
}

static ssize_t ptwalk_clear_young_show(struct kobject *kobj,
				       struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_ptwalk_clear_young());
}

static ssize_t ptwalk_clear_young_store(struct kobject *kobj,
					struct kobj_attribute *attr,
					const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_ptwalk_clear_young(val);
	return ret ? ret : count;
}

static ssize_t reclaim_ptwalk_show(struct kobject *kobj,
				   struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_reclaim_ptwalk());
}

static ssize_t reclaim_ptwalk_store(struct kobject *kobj,
				    struct kobj_attribute *attr,
				    const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_reclaim_ptwalk(val);
	return ret ? ret : count;
}

static ssize_t reclaim_feedback_show(struct kobject *kobj,
				     struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_reclaim_feedback());
}

static ssize_t reclaim_feedback_store(struct kobject *kobj,
				      struct kobj_attribute *attr,
				      const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_reclaim_feedback(val);
	return ret ? ret : count;
}

static ssize_t reclaim_advance_show(struct kobject *kobj,
				    struct kobj_attribute *attr, char *buf)
{
	(void)kobj;
	(void)attr;

	return scnprintf(buf, PAGE_SIZE, "%d\n", lru_gen_get_reclaim_advance());
}

static ssize_t reclaim_advance_store(struct kobject *kobj,
				     struct kobj_attribute *attr,
				     const char *buf, size_t count)
{
	bool val;
	int ret;

	(void)kobj;
	(void)attr;

	ret = kstrtobool(buf, &val);
	if (ret)
		return ret;

	ret = lru_gen_set_reclaim_advance(val);
	return ret ? ret : count;
}

static struct kobj_attribute enabled_attr = __ATTR_RW(enabled);
static struct kobj_attribute min_ttl_ms_attr = __ATTR_RW(min_ttl_ms);
static struct kobj_attribute age_period_ms_attr = __ATTR_RW(age_period_ms);
static struct kobj_attribute weight_anon_pct_attr = __ATTR_RW(weight_anon_pct);
static struct kobj_attribute dedup_window_ms_attr = __ATTR_RW(dedup_window_ms);
static struct kobj_attribute pressure_normalize_attr =
	__ATTR_RW(pressure_normalize);
static struct kobj_attribute ptwalk_pages_attr = __ATTR_RW(ptwalk_pages);
static struct kobj_attribute reclaim_ptwalk_attr = __ATTR_RW(reclaim_ptwalk);
static struct kobj_attribute reclaim_feedback_attr = __ATTR_RW(reclaim_feedback);
static struct kobj_attribute reclaim_advance_attr = __ATTR_RW(reclaim_advance);
static struct kobj_attribute ptwalk_clear_young_attr =
	__ATTR_RW(ptwalk_clear_young);

static struct attribute *mglru_sysfs_attrs[] = {
	&enabled_attr.attr,
	&min_ttl_ms_attr.attr,
	&age_period_ms_attr.attr,
	&weight_anon_pct_attr.attr,
	&dedup_window_ms_attr.attr,
	&pressure_normalize_attr.attr,
	&ptwalk_pages_attr.attr,
	&reclaim_ptwalk_attr.attr,
	&reclaim_feedback_attr.attr,
	&reclaim_advance_attr.attr,
	&ptwalk_clear_young_attr.attr,
	NULL,
};

static struct attribute_group mglru_sysfs_attr_group = {
	.attrs = mglru_sysfs_attrs,
};

static int __init mglru_sysfs_init(void)
{
	int ret;

	if (!mm_kobj)
		return 0;

	mglru_kobj = kobject_create_and_add("lru_gen", mm_kobj);
	if (!mglru_kobj)
		return -ENOMEM;

	ret = sysfs_create_group(mglru_kobj, &mglru_sysfs_attr_group);
	if (ret) {
		kobject_put(mglru_kobj);
		mglru_kobj = NULL;
	}

	return ret;
}
late_initcall(mglru_sysfs_init);
#endif

#endif
