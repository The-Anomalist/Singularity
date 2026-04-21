// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-Gen LRU aging and reclaim machinery for the 4.19 backport.
 */

#include <linux/mm.h>
#include <linux/mm_inline.h>
#include <linux/mmzone.h>
#include <linux/jump_label.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/string.h>
#include <linux/swap.h>

#ifdef CONFIG_LRU_GEN
static DEFINE_STATIC_KEY_FALSE(lru_gen_caps);
static bool __read_mostly lru_gen_boot_enabled =
	IS_ENABLED(CONFIG_LRU_GEN_ENABLED);
static unsigned int __read_mostly lru_gen_min_ttl_ms;
static unsigned int __read_mostly lru_gen_age_period_ms = 1000;

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
	unsigned long seq = lrugen->max_seq + 1;
	unsigned int gen = seq % MAX_NR_GENS;
	unsigned int type;

	lrugen->max_seq = seq;
	lrugen->timestamps[gen] = jiffies;

	for (type = 0; type < ANON_AND_FILE; type++) {
		if (lrugen->min_seq[type] + MIN_NR_GENS <= seq)
			lrugen->min_seq[type] = seq - MIN_NR_GENS + 1;
	}
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
	lrugen->last_reclaim = jiffies;
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
	lrugen->pressure[type] += delta;
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
	lrugen->accessed[type]++;
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
	unsigned int type;
	unsigned long nr_pages;
	unsigned long threshold;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	type = page_is_file_cache(page) ? LRU_GEN_FILE : LRU_GEN_ANON;
	nr_pages = hpage_nr_pages(page);
	threshold = SWAP_CLUSTER_MAX * (from_reclaim ? 2 : 4);

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	lrugen->accessed[type] += nr_pages;
	/*
	 * Reclaim-driven references are collected from page table walks
	 * (via page_referenced()), so use them as a direct aging signal.
	 */
	if (from_reclaim)
		lrugen->pressure[type] += min_t(unsigned long, nr_pages, SWAP_CLUSTER_MAX);

	if (lrugen->accessed[type] >= threshold) {
		lrugen->accessed[type] = 0;
		lru_gen_advance_seq(lruvec);
	}
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
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
}

void lru_gen_enter_reclaim(struct lruvec *lruvec, struct scan_control *sc)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen;
	unsigned int type;

	(void)sc;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	gen = lrugen->max_seq % MAX_NR_GENS;
	if (lrugen->timestamps[gen] + HZ / 4 < jiffies)
		lrugen->timestamps[gen] = jiffies;
	if (lru_gen_should_age(lrugen)) {
		lrugen->evicted[LRU_GEN_ANON] = 0;
		lrugen->evicted[LRU_GEN_FILE] = 0;
		lru_gen_advance_seq(lruvec);
	}

	/*
	 * Reclaim path for the oldest generations:
	 * if an oldest generation is already drained, move its floor forward so
	 * reclaim naturally shifts to the next oldest generation.
	 */
	for (type = 0; type < ANON_AND_FILE; type++) {
		if (!lru_gen_count_old(lrugen, type) &&
		    lrugen->min_seq[type] < lrugen->max_seq)
			lrugen->min_seq[type]++;
	}
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_adjust_scan(struct lruvec *lruvec, struct scan_control *sc,
			 unsigned long *nr)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned long anon_scan;
	unsigned long file_scan;
	unsigned long total;
	unsigned long denom;
	unsigned long anon_weight;
	unsigned long file_weight;
	unsigned long anon_old = 0;
	unsigned long file_old = 0;
	unsigned long anon_old_age = 0;
	unsigned long file_old_age = 0;
	unsigned long min_ttl;
	unsigned long min_ttl_jiffies;
	unsigned int preferred;
	unsigned int zone;

	if (!lru_gen_enabled() || !lru_gen_get_state())
		return;

	anon_scan = nr[LRU_INACTIVE_ANON] + nr[LRU_ACTIVE_ANON];
	file_scan = nr[LRU_INACTIVE_FILE] + nr[LRU_ACTIVE_FILE];
	total = anon_scan + file_scan;
	if (!total)
		return;

	/*
	 * Eviction decision stage:
	 * steer scan pressure toward colder generations using pressure[].
	 * Keep at least one cluster of each type to avoid starvation.
	 */
	anon_weight = lrugen->pressure[LRU_GEN_ANON] + 1;
	file_weight = lrugen->pressure[LRU_GEN_FILE] + 1;

	for (zone = 0; zone < MAX_NR_ZONES; zone++) {
		anon_old += lrugen->nr_pages[lrugen->min_seq[LRU_GEN_ANON] % MAX_NR_GENS]
					  [LRU_GEN_ANON][zone];
		file_old += lrugen->nr_pages[lrugen->min_seq[LRU_GEN_FILE] % MAX_NR_GENS]
					  [LRU_GEN_FILE][zone];
	}
	anon_old_age = jiffies - lrugen->timestamps[lrugen->min_seq[LRU_GEN_ANON] % MAX_NR_GENS];
	file_old_age = jiffies - lrugen->timestamps[lrugen->min_seq[LRU_GEN_FILE] % MAX_NR_GENS];
	min_ttl = READ_ONCE(lru_gen_min_ttl_ms);
	min_ttl_jiffies = msecs_to_jiffies(min_ttl);

	anon_weight += anon_old / SWAP_CLUSTER_MAX;
	file_weight += file_old / SWAP_CLUSTER_MAX;

	/*
	 * Respect optional working-set protection: until the oldest generation
	 * reaches min_ttl, de-emphasize it instead of scanning it aggressively.
	 */
	if (min_ttl && anon_old_age < min_ttl_jiffies)
		anon_weight >>= 1;
	if (min_ttl && file_old_age < min_ttl_jiffies)
		file_weight >>= 1;

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
	unsigned int anon_tier, file_tier;
	unsigned long efficiency, split;
	unsigned long now = jiffies;
	unsigned long anon_eff = 0, file_eff = 0;

	if (!lru_gen_enabled() || !lru_gen_get_state() || !scanned)
		return;

	/*
	 * memcg/tier tuning stage:
	 * derive a coarse reclaim tier based on observed reclaim efficiency.
	 */
	efficiency = reclaimed * 100 / scanned;
	anon_tier = min_t(unsigned int, MAX_NR_GENS - 1,
			  lrugen->pressure[LRU_GEN_ANON] / (8 * SWAP_CLUSTER_MAX));
	file_tier = min_t(unsigned int, MAX_NR_GENS - 1,
			  lrugen->pressure[LRU_GEN_FILE] / (8 * SWAP_CLUSTER_MAX));

	if (efficiency < 10) {
		anon_tier = min_t(unsigned int, anon_tier + 1, MAX_NR_GENS - 1);
		file_tier = min_t(unsigned int, file_tier + 1, MAX_NR_GENS - 1);
	} else if (efficiency > 50) {
		anon_tier = anon_tier ? anon_tier - 1 : 0;
		file_tier = file_tier ? file_tier - 1 : 0;
	}

	lrugen->tiers[LRU_GEN_ANON] = anon_tier;
	lrugen->tiers[LRU_GEN_FILE] = file_tier;

	split = max_t(unsigned long, 1, scanned / 2);
	if (reclaimed)
		anon_eff = min_t(unsigned long, reclaimed, split) * 100 / split;
	if (scanned > split)
		file_eff = (reclaimed > split ? reclaimed - split : 0) * 100 /
			   (scanned - split);

	if (anon_eff + 5 < file_eff)
		lrugen->pressure[LRU_GEN_ANON] += SWAP_CLUSTER_MAX / 2;
	else if (file_eff + 5 < anon_eff)
		lrugen->pressure[LRU_GEN_FILE] += SWAP_CLUSTER_MAX / 2;

	lru_gen_decay_pressure(lrugen);

	/*
	 * Tier-aware aging: advance generations faster when reclaim is weak or
	 * when higher tiers are selected by pressure feedback.
	 */
	if (efficiency < 20 || anon_tier + file_tier >= MAX_NR_GENS ||
	    now - lrugen->last_reclaim >= HZ) {
		spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
		lru_gen_advance_seq(lruvec);
		spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	}
	lrugen->last_reclaim = now;
}

/*
 * Reclaim integration hook.
 *
 * This is intentionally conservative for the first stage of this backport:
 * keep classic reclaim behavior unless full MGLRU scan/evict wiring is
 * available. Returning false hands control back to legacy shrink_node().
 */
#endif
