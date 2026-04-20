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
#include <linux/string.h>

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
	lrugen->enabled = lru_gen_boot_enabled;
}

/*
 * Reclaim integration hook.
 *
 * This is intentionally conservative for the first stage of this backport:
 * keep classic reclaim behavior unless full MGLRU scan/evict wiring is
 * available. Returning false hands control back to legacy shrink_node().
 */
bool lru_gen_shrink_node(struct pglist_data *pgdat, struct scan_control *sc)
{
	struct lruvec *lruvec;
	struct lru_gen_struct *lrugen;
	unsigned int gen;

	(void)sc;

	if (!lru_gen_enabled())
		return false;

	lruvec = node_lruvec(pgdat);
	lrugen = &lruvec->lrugen;

	if (!lrugen->enabled)
		return false;

	/*
	 * Keep generation timestamps and sequence numbers moving while reclaim is
	 * active. Full MGLRU scan/evict wiring will consume this state.
	 */
	spin_lock_irq(&pgdat->lru_lock);
	gen = lrugen->max_seq % MAX_NR_GENS;
	if (time_is_before_eq_jiffies(lrugen->timestamps[gen] + HZ))
		lru_gen_advance_seq(lruvec);
	spin_unlock_irq(&pgdat->lru_lock);

	return false;
}
#endif
