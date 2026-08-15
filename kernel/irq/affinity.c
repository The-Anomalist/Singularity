// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2016 Thomas Gleixner.
 * Copyright (C) 2016-2017 Christoph Hellwig.
 */
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/cpu.h>

static void irq_spread_init_one(struct cpumask *irqmsk, struct cpumask *nmsk,
				int cpus_per_vec)
{
	const struct cpumask *siblmsk;
	int cpu, sibl;

	for ( ; cpus_per_vec > 0; ) {
		cpu = cpumask_first(nmsk);

		/* Should not happen, but I'm too lazy to think about it */
		if (cpu >= nr_cpu_ids)
			return;

		cpumask_clear_cpu(cpu, nmsk);
		cpumask_set_cpu(cpu, irqmsk);
		cpus_per_vec--;

		/* If the cpu has siblings, use them first */
		siblmsk = topology_sibling_cpumask(cpu);
		for (sibl = -1; cpus_per_vec > 0; ) {
			sibl = cpumask_next(sibl, siblmsk);
			if (sibl >= nr_cpu_ids)
				break;
			if (!cpumask_test_and_clear_cpu(sibl, nmsk))
				continue;
			cpumask_set_cpu(sibl, irqmsk);
			cpus_per_vec--;
		}
	}
}

static cpumask_var_t *alloc_node_to_cpumask(void)
{
	cpumask_var_t *masks;
	int node;

	masks = kcalloc(nr_node_ids, sizeof(cpumask_var_t), GFP_KERNEL);
	if (!masks)
		return NULL;

	for (node = 0; node < nr_node_ids; node++) {
		if (!zalloc_cpumask_var(&masks[node], GFP_KERNEL))
			goto out_unwind;
	}

	return masks;

out_unwind:
	while (--node >= 0)
		free_cpumask_var(masks[node]);
	kfree(masks);
	return NULL;
}

static void free_node_to_cpumask(cpumask_var_t *masks)
{
	int node;

	for (node = 0; node < nr_node_ids; node++)
		free_cpumask_var(masks[node]);
	kfree(masks);
}

static void build_node_to_cpumask(cpumask_var_t *masks)
{
	int cpu;

	for_each_possible_cpu(cpu)
		cpumask_set_cpu(cpu, masks[cpu_to_node(cpu)]);
}

static int get_nodes_in_cpumask(cpumask_var_t *node_to_cpumask,
				const struct cpumask *mask, nodemask_t *nodemsk)
{
	int n, nodes = 0;

	/* Calculate the number of nodes in the supplied affinity mask */
	for_each_node(n) {
		if (cpumask_intersects(mask, node_to_cpumask[n])) {
			node_set(n, *nodemsk);
			nodes++;
		}
	}
	return nodes;
}

static int irq_build_affinity_masks(int startvec, int basevec, int numvecs,
				    cpumask_var_t *node_to_cpumask,
				    const struct cpumask *cpu_mask,
				    struct cpumask *nmsk,
				    struct cpumask *masks)
{
	int n, nodes, cpus_per_vec, extra_vecs, done = 0;
	int last_affv = basevec + numvecs;
	int curvec = startvec;
	nodemask_t nodemsk = NODE_MASK_NONE;

	if (!cpumask_weight(cpu_mask))
		return 0;

	nodes = get_nodes_in_cpumask(node_to_cpumask, cpu_mask, &nodemsk);

	/*
	 * If the number of nodes in the mask is greater than or equal the
	 * number of vectors we just spread the vectors across the nodes.
	 */
	if (numvecs <= nodes) {
		for_each_node_mask(n, nodemsk) {
			cpumask_or(masks + curvec, masks + curvec, node_to_cpumask[n]);
			if (++curvec == last_affv)
				curvec = basevec;
		}
		done = numvecs;
		goto out;
	}

	for_each_node_mask(n, nodemsk) {
		int ncpus, v, vecs_to_assign, vecs_per_node;

		/* Spread the vectors per node */
		vecs_per_node = (numvecs - (curvec - basevec)) / nodes;

		/* Get the cpus on this node which are in the mask */
		cpumask_and(nmsk, cpu_mask, node_to_cpumask[n]);

		/* Calculate the number of cpus per vector */
		ncpus = cpumask_weight(nmsk);
		vecs_to_assign = min(vecs_per_node, ncpus);

		/* Account for rounding errors */
		extra_vecs = ncpus - vecs_to_assign * (ncpus / vecs_to_assign);

		for (v = 0; curvec < last_affv && v < vecs_to_assign;
		     curvec++, v++) {
			cpus_per_vec = ncpus / vecs_to_assign;

			/* Account for extra vectors to compensate rounding errors */
			if (extra_vecs) {
				cpus_per_vec++;
				--extra_vecs;
			}
			irq_spread_init_one(masks + curvec, nmsk, cpus_per_vec);
		}

		done += v;
		if (done >= numvecs)
			break;
		if (curvec >= last_affv)
			curvec = basevec;
		--nodes;
	}

out:
	return done;
}

/**
 * irq_create_affinity_masks - Create affinity masks for multiqueue spreading
 * @nvecs:	The total number of vectors
 * @affd:	Description of the affinity requirements
 *
 * Returns the masks pointer or NULL if allocation failed.
 */
struct cpumask *
irq_create_affinity_masks(int nvecs, const struct irq_affinity *affd)
{
	int affvecs = nvecs - affd->pre_vectors - affd->post_vectors;
	struct irq_affinity affd_copy = *affd;
	int set_used[IRQ_AFFINITY_MAX_SETS] = { 0 };
	int curvec, usedvecs, i;
	cpumask_var_t nmsk, npresmsk, *node_to_cpumask;
	struct cpumask *masks = NULL;

	/*
	 * If there aren't any vectors left after applying the pre/post
	 * vectors don't bother with assigning affinity.
	 */
	if (nvecs == affd->pre_vectors + affd->post_vectors)
		return NULL;

	/*
	 * A device can have groups of queues with different purposes (for
	 * example, read and write queues).  Spread each such group over the
	 * CPUs independently instead of allowing the larger group to consume
	 * all useful affinity slots first.
	 *
	 * Work on a copy so callers may keep a const affinity descriptor and
	 * so a sizing callback cannot leave stale state behind between retries.
	 */
	if (affd_copy.calc_sets) {
		affd_copy.calc_sets(&affd_copy, affvecs);
	} else if (!affd_copy.nr_sets) {
		affd_copy.nr_sets = 1;
		affd_copy.set_size[0] = affvecs;
	}

	if (WARN_ON_ONCE(!affd_copy.nr_sets ||
			 affd_copy.nr_sets > IRQ_AFFINITY_MAX_SETS))
		return NULL;

	usedvecs = 0;
	for (i = 0; i < affd_copy.nr_sets; i++) {
		if (!affd_copy.set_size[i] ||
		    affd_copy.set_size[i] > affvecs - usedvecs)
			return NULL;
		usedvecs += affd_copy.set_size[i];
	}
	if (WARN_ON_ONCE(usedvecs != affvecs))
		return NULL;
	affd = &affd_copy;

	if (!zalloc_cpumask_var(&nmsk, GFP_KERNEL))
		return NULL;

	if (!zalloc_cpumask_var(&npresmsk, GFP_KERNEL))
		goto outcpumsk;

	node_to_cpumask = alloc_node_to_cpumask();
	if (!node_to_cpumask)
		goto outnpresmsk;

	masks = kcalloc(nvecs, sizeof(*masks), GFP_KERNEL);
	if (!masks)
		goto outnodemsk;

	/* Fill out vectors at the beginning that don't need affinity */
	for (curvec = 0; curvec < affd->pre_vectors; curvec++)
		cpumask_copy(masks + curvec, irq_default_affinity);

	/* Stabilize the cpumasks */
	get_online_cpus();
	build_node_to_cpumask(node_to_cpumask);

	/* Spread every affinity set independently over present CPUs. */
	usedvecs = 0;
	for (i = 0; i < affd->nr_sets; i++) {
		int size = affd->set_size[i];

		set_used[i] = irq_build_affinity_masks(curvec, curvec, size,
						       node_to_cpumask, cpu_present_mask,
						       nmsk, masks);
		usedvecs += set_used[i];
		curvec += size;
	}

	/*
	 * Spread on non present CPUs starting from the next vector to be
	 * handled. If the spreading of present CPUs already exhausted the
	 * vector space, assign the non present CPUs to the already spread
	 * out vectors.
	 */
	cpumask_andnot(npresmsk, cpu_possible_mask, cpu_present_mask);
	curvec = affd->pre_vectors;
	for (i = 0; i < affd->nr_sets; i++) {
		int startvec = curvec;
		int size = affd->set_size[i];

		if (set_used[i] < size)
			startvec += set_used[i];
		usedvecs += irq_build_affinity_masks(startvec, curvec, size,
				node_to_cpumask,
				npresmsk, nmsk, masks);
		curvec += size;
	}
	put_online_cpus();

	/* Fill out vectors at the end that don't need affinity */
	if (usedvecs >= affvecs)
		curvec = affd->pre_vectors + affvecs;
	else
		curvec = affd->pre_vectors + usedvecs;
	for (; curvec < nvecs; curvec++)
		cpumask_copy(masks + curvec, irq_default_affinity);

outnodemsk:
	free_node_to_cpumask(node_to_cpumask);
outnpresmsk:
	free_cpumask_var(npresmsk);
outcpumsk:
	free_cpumask_var(nmsk);
	return masks;
}

/**
 * irq_calc_affinity_vectors - Calculate the optimal number of vectors
 * @minvec:	The minimum number of vectors available
 * @maxvec:	The maximum number of vectors available
 * @affd:	Description of the affinity requirements
 */
int irq_calc_affinity_vectors(int minvec, int maxvec, const struct irq_affinity *affd)
{
	int resv = affd->pre_vectors + affd->post_vectors;
	int vecs = maxvec - resv;
	int ret;

	if (resv > minvec)
		return 0;

	get_online_cpus();
	ret = min_t(int, cpumask_weight(cpu_possible_mask), vecs) + resv;
	put_online_cpus();
	return ret;
}
