// SPDX-License-Identifier: GPL-2.0
/*
 * Multi-Gen LRU groundwork for 4.19 backporting.
 */

#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/jump_label.h>

#ifdef CONFIG_LRU_GEN
static DEFINE_STATIC_KEY_TRUE(lru_gen_caps);

bool lru_gen_enabled(void)
{
	return static_branch_likely(&lru_gen_caps);
}

void lru_gen_init_lruvec(struct lruvec *lruvec)
{
	struct lru_gen_struct *lrugen = &lruvec->lrugen;
	unsigned int gen, type, zone;

	for (gen = 0; gen < MAX_NR_GENS; gen++) {
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
	lrugen->enabled = IS_ENABLED(CONFIG_LRU_GEN_ENABLED);
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
	if (!lru_gen_enabled())
		return false;

	return false;
}
#endif
