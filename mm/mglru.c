// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-Gen LRU groundwork for 4.19 backporting.
 */

#include <linux/mm.h>
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

static int lru_gen_lru_type(enum lru_list lru)
{
	return is_file_lru(lru) ? LRU_GEN_FILE : LRU_GEN_ANON;
}

void lru_gen_update_size(struct lruvec *lruvec, enum lru_list lru,
			 enum zone_type zid, long delta)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned long seq;
	unsigned int type, gen;
	long *nr_pages;

	if (!lru_gen_enabled() || !lrugen->enabled || lru == LRU_UNEVICTABLE || !delta)
		return;

	type = lru_gen_lru_type(lru);

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	seq = is_active_lru(lru) ? lrugen->max_seq : lrugen->min_seq[type];
	gen = seq % MAX_NR_GENS;
	nr_pages = &lrugen->nr_pages[gen][type][zid];

	*nr_pages += delta;
	if (*nr_pages < 0)
		*nr_pages = 0;
	if (delta > 0)
		lrugen->timestamps[gen] = jiffies;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

static void lru_gen_decay_pressure(struct lru_gen_struct *lrugen)
{
	unsigned int type;

	for (type = 0; type < ANON_AND_FILE; type++)
		lrugen->pressure[type] -= lrugen->pressure[type] >> 3;
}

static bool lru_gen_should_age(struct lru_gen_struct *lrugen,
			       struct scan_control *sc)
{
	unsigned long gen = lrugen->max_seq % MAX_NR_GENS;
	unsigned long age = jiffies - lrugen->timestamps[gen];

	if (age >= HZ)
		return true;

	if (sc && sc->priority <= DEF_PRIORITY - 2)
		return true;

	return lrugen->pressure[LRU_GEN_ANON] + lrugen->pressure[LRU_GEN_FILE] >=
	       SWAP_CLUSTER_MAX * 16;
}

static unsigned int lru_gen_pick_scan_type(struct lru_gen_struct *lrugen,
					   struct scan_control *sc)
{
	unsigned long anon = lrugen->pressure[LRU_GEN_ANON] + 1;
	unsigned long file = lrugen->pressure[LRU_GEN_FILE] + 1;

	if (!sc->may_swap || !total_swap_pages)
		return LRU_GEN_FILE;

	anon += lrugen->tiers[LRU_GEN_ANON] * SWAP_CLUSTER_MAX;
	file += lrugen->tiers[LRU_GEN_FILE] * SWAP_CLUSTER_MAX;

	return anon >= file ? LRU_GEN_ANON : LRU_GEN_FILE;
}

bool lru_gen_enabled(void)
{
	return static_branch_likely(&lru_gen_caps);
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
	lrugen->enabled = lru_gen_boot_enabled;
}

void lru_gen_track_page_scan(struct lruvec *lruvec, enum lru_list lru,
			     unsigned long nr_scanned, unsigned long nr_taken,
			     unsigned long nr_reclaimed)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned long gen;
	unsigned long delta;
	int type;

	if (!lru_gen_enabled() || !lrugen->enabled || !nr_scanned)
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

	if (!lru_gen_enabled() || !lrugen->enabled)
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

void lru_gen_note_lru_move(struct lruvec *lruvec, enum lru_list old_lru,
			   enum lru_list new_lru, unsigned long nr_pages)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int old_type, new_type;
	unsigned long old_seq;

	if (!lru_gen_enabled() || !lrugen->enabled || !nr_pages)
		return;

	if (old_lru == LRU_UNEVICTABLE || new_lru == LRU_UNEVICTABLE)
		return;

	old_type = lru_gen_lru_type(old_lru);
	new_type = lru_gen_lru_type(new_lru);

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	old_seq = is_active_lru(old_lru) ? lrugen->max_seq : lrugen->min_seq[old_type];
	if (!is_active_lru(new_lru) && old_type == new_type &&
	    old_seq == lrugen->min_seq[new_type])
		lrugen->evicted[new_type] += nr_pages;
	spin_unlock_irq(&lruvec_pgdat(lruvec)->lru_lock);
}

void lru_gen_enter_reclaim(struct lruvec *lruvec, struct scan_control *sc)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen;

	if (!lru_gen_enabled() || !lrugen->enabled)
		return;

	spin_lock_irq(&lruvec_pgdat(lruvec)->lru_lock);
	gen = lrugen->max_seq % MAX_NR_GENS;
	if (lrugen->timestamps[gen] + HZ / 4 < jiffies)
		lrugen->timestamps[gen] = jiffies;
	if (lru_gen_should_age(lrugen, sc)) {
		lrugen->evicted[LRU_GEN_ANON] = 0;
		lrugen->evicted[LRU_GEN_FILE] = 0;
		lru_gen_advance_seq(lruvec);
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
	unsigned int preferred;
	unsigned int zone;

	if (!lru_gen_enabled() || !lrugen->enabled)
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
	anon_weight += anon_old / SWAP_CLUSTER_MAX;
	file_weight += file_old / SWAP_CLUSTER_MAX;

	if (!sc->may_swap || !total_swap_pages)
		anon_weight = 0;
	preferred = lru_gen_pick_scan_type(lrugen, sc);

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

	if (!lru_gen_enabled() || !lrugen->enabled || !scanned)
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
