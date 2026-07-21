// SPDX-License-Identifier: GPL-2.0
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/cpumask.h>
#include <linux/export.h>
#include <linux/bootmem.h>

/**
 * cpumask_next - get the next cpu in a cpumask
 * @n: the cpu prior to the place to search (ie. return will be > @n)
 * @srcp: the cpumask pointer
 *
 * Returns >= nr_cpu_ids if no further cpus set.
 */
unsigned int cpumask_next(int n, const struct cpumask *srcp)
{
	/* -1 is a legal arg here. */
	if (n != -1)
		cpumask_check(n);
	return find_next_bit(cpumask_bits(srcp), nr_cpumask_bits, n + 1);
}
EXPORT_SYMBOL(cpumask_next);

/**
 * cpumask_next_and - get the next cpu in *src1p & *src2p
 * @n: the cpu prior to the place to search (ie. return will be > @n)
 * @src1p: the first cpumask pointer
 * @src2p: the second cpumask pointer
 *
 * Returns >= nr_cpu_ids if no further cpus set in both.
 */
int cpumask_next_and(int n, const struct cpumask *src1p,
		     const struct cpumask *src2p)
{
	/* -1 is a legal arg here. */
	if (n != -1)
		cpumask_check(n);
	return find_next_and_bit(cpumask_bits(src1p), cpumask_bits(src2p),
		nr_cpumask_bits, n + 1);
}
EXPORT_SYMBOL(cpumask_next_and);

/*
 * Return the n-th CPU in @src1p & @src2p, or nr_cpumask_bits when the
 * intersection contains fewer than n + 1 CPUs.  Selecting directly from
 * bitmap words avoids walking every preceding CPU in large affinity masks.
 */
unsigned int cpumask_nth_and(unsigned int n, const struct cpumask *src1p,
			     const struct cpumask *src2p)
{
	const unsigned long *src1 = cpumask_bits(src1p);
	const unsigned long *src2 = cpumask_bits(src2p);
	unsigned int word;

	for (word = 0; word < BITS_TO_LONGS(nr_cpumask_bits); word++) {
		unsigned long bits;
		unsigned int weight;

		bits = src1[word] & src2[word];

		if (word == BITS_TO_LONGS(nr_cpumask_bits) - 1)
			bits &= BITMAP_LAST_WORD_MASK(nr_cpumask_bits);
		weight = hweight_long(bits);

		if (n >= weight) {
			n -= weight;
			continue;
		}

		while (n) {
			bits &= bits - 1;
			n--;
		}
		return word * BITS_PER_LONG + __ffs(bits);
	}

	return nr_cpumask_bits;
}
EXPORT_SYMBOL(cpumask_nth_and);

/*
 * Return the n-th CPU in @src1p & ~@src2p, or nr_cpumask_bits when the
 * difference contains fewer than n + 1 CPUs.
 */
unsigned int cpumask_nth_andnot(unsigned int n, const struct cpumask *src1p,
				const struct cpumask *src2p)
{
	const unsigned long *src1 = cpumask_bits(src1p);
	const unsigned long *src2 = cpumask_bits(src2p);
	unsigned int word;

	for (word = 0; word < BITS_TO_LONGS(nr_cpumask_bits); word++) {
		unsigned long bits;
		unsigned int weight;

		bits = src1[word] & ~src2[word];

		if (word == BITS_TO_LONGS(nr_cpumask_bits) - 1)
			bits &= BITMAP_LAST_WORD_MASK(nr_cpumask_bits);
		weight = hweight_long(bits);

		if (n >= weight) {
			n -= weight;
			continue;
		}

		while (n) {
			bits &= bits - 1;
			n--;
		}
		return word * BITS_PER_LONG + __ffs(bits);
	}

	return nr_cpumask_bits;
}
EXPORT_SYMBOL(cpumask_nth_andnot);

static unsigned int cpumask_weight_and(const struct cpumask *src1p,
				       const struct cpumask *src2p)
{
	const unsigned long *src1 = cpumask_bits(src1p);
	const unsigned long *src2 = cpumask_bits(src2p);
	unsigned int weight = 0;
	unsigned int word;

	for (word = 0; word < BITS_TO_LONGS(nr_cpumask_bits); word++) {
		unsigned long bits = src1[word] & src2[word];

		if (word == BITS_TO_LONGS(nr_cpumask_bits) - 1)
			bits &= BITMAP_LAST_WORD_MASK(nr_cpumask_bits);
		weight += hweight_long(bits);
	}

	return weight;
}

/**
 * cpumask_any_but - return a "random" in a cpumask, but not this one.
 * @mask: the cpumask to search
 * @cpu: the cpu to ignore.
 *
 * Often used to find any cpu but smp_processor_id() in a mask.
 * Returns >= nr_cpu_ids if no cpus set.
 */
int cpumask_any_but(const struct cpumask *mask, unsigned int cpu)
{
	unsigned int i;

	cpumask_check(cpu);
	for_each_cpu(i, mask)
		if (i != cpu)
			break;
	return i;
}
EXPORT_SYMBOL(cpumask_any_but);

/**
 * cpumask_next_wrap - helper to implement for_each_cpu_wrap
 * @n: the cpu prior to the place to search
 * @mask: the cpumask pointer
 * @start: the start point of the iteration
 * @wrap: assume @n crossing @start terminates the iteration
 *
 * Returns >= nr_cpu_ids on completion
 *
 * Note: the @wrap argument is required for the start condition when
 * we cannot assume @start is set in @mask.
 */
int cpumask_next_wrap(int n, const struct cpumask *mask, int start, bool wrap)
{
	int next;

again:
	next = cpumask_next(n, mask);

	if (wrap && n < start && next >= start) {
		return nr_cpumask_bits;

	} else if (next >= nr_cpumask_bits) {
		wrap = true;
		n = -1;
		goto again;
	}

	return next;
}
EXPORT_SYMBOL(cpumask_next_wrap);

/* These are not inline because of header tangles. */
#ifdef CONFIG_CPUMASK_OFFSTACK
/**
 * alloc_cpumask_var_node - allocate a struct cpumask on a given node
 * @mask: pointer to cpumask_var_t where the cpumask is returned
 * @flags: GFP_ flags
 *
 * Only defined when CONFIG_CPUMASK_OFFSTACK=y, otherwise is
 * a nop returning a constant 1 (in <linux/cpumask.h>)
 * Returns TRUE if memory allocation succeeded, FALSE otherwise.
 *
 * In addition, mask will be NULL if this fails.  Note that gcc is
 * usually smart enough to know that mask can never be NULL if
 * CONFIG_CPUMASK_OFFSTACK=n, so does code elimination in that case
 * too.
 */
bool alloc_cpumask_var_node(cpumask_var_t *mask, gfp_t flags, int node)
{
	*mask = kmalloc_node(cpumask_size(), flags, node);

#ifdef CONFIG_DEBUG_PER_CPU_MAPS
	if (!*mask) {
		printk(KERN_ERR "=> alloc_cpumask_var: failed!\n");
		dump_stack();
	}
#endif

	return *mask != NULL;
}
EXPORT_SYMBOL(alloc_cpumask_var_node);

bool zalloc_cpumask_var_node(cpumask_var_t *mask, gfp_t flags, int node)
{
	return alloc_cpumask_var_node(mask, flags | __GFP_ZERO, node);
}
EXPORT_SYMBOL(zalloc_cpumask_var_node);

/**
 * alloc_cpumask_var - allocate a struct cpumask
 * @mask: pointer to cpumask_var_t where the cpumask is returned
 * @flags: GFP_ flags
 *
 * Only defined when CONFIG_CPUMASK_OFFSTACK=y, otherwise is
 * a nop returning a constant 1 (in <linux/cpumask.h>).
 *
 * See alloc_cpumask_var_node.
 */
bool alloc_cpumask_var(cpumask_var_t *mask, gfp_t flags)
{
	return alloc_cpumask_var_node(mask, flags, NUMA_NO_NODE);
}
EXPORT_SYMBOL(alloc_cpumask_var);

bool zalloc_cpumask_var(cpumask_var_t *mask, gfp_t flags)
{
	return alloc_cpumask_var(mask, flags | __GFP_ZERO);
}
EXPORT_SYMBOL(zalloc_cpumask_var);

/**
 * alloc_bootmem_cpumask_var - allocate a struct cpumask from the bootmem arena.
 * @mask: pointer to cpumask_var_t where the cpumask is returned
 *
 * Only defined when CONFIG_CPUMASK_OFFSTACK=y, otherwise is
 * a nop (in <linux/cpumask.h>).
 * Either returns an allocated (zero-filled) cpumask, or causes the
 * system to panic.
 */
void __init alloc_bootmem_cpumask_var(cpumask_var_t *mask)
{
	*mask = memblock_virt_alloc(cpumask_size(), 0);
}

/**
 * free_cpumask_var - frees memory allocated for a struct cpumask.
 * @mask: cpumask to free
 *
 * This is safe on a NULL mask.
 */
void free_cpumask_var(cpumask_var_t mask)
{
	kfree(mask);
}
EXPORT_SYMBOL(free_cpumask_var);

/**
 * free_bootmem_cpumask_var - frees result of alloc_bootmem_cpumask_var
 * @mask: cpumask to free
 */
void __init free_bootmem_cpumask_var(cpumask_var_t mask)
{
	memblock_free_early(__pa(mask), cpumask_size());
}
#endif

/**
 * cpumask_local_spread - select the i'th cpu with local numa cpu's first
 * @i: index number
 * @node: local numa_node
 *
 * This function selects an online CPU according to a numa aware policy;
 * local cpus are returned first, followed by non-local ones, then it
 * wraps around.
 *
 * The selection is performed word-wise, which avoids repeatedly walking a
 * potentially large CPU mask while setting up IRQ affinity.
 */
unsigned int cpumask_local_spread(unsigned int i, int node)
{
	unsigned int local_cpus;

	/* Wrap: we always want a cpu. */
	i %= num_online_cpus();

	if (node == NUMA_NO_NODE)
		return cpumask_nth_and(i, cpu_online_mask, cpu_online_mask);

	/* Prefer local CPUs, then select from the remaining online CPUs. */
	local_cpus = cpumask_weight_and(cpumask_of_node(node), cpu_online_mask);
	if (i < local_cpus)
		return cpumask_nth_and(i, cpumask_of_node(node),
				       cpu_online_mask);

	return cpumask_nth_andnot(i - local_cpus, cpu_online_mask,
				    cpumask_of_node(node));
}
EXPORT_SYMBOL(cpumask_local_spread);
