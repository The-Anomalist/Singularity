// SPDX-License-Identifier: GPL-2.0
#include <linux/cache.h>
#include <linux/cpu.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/singularity_qos.h>
#include <linux/skbuff.h>
#include <linux/blkdev.h>

/*
 * These hooks sit in some of the hottest paths in the kernel.  A single set
 * of atomic counters makes every CPU take ownership of the same cache line on
 * each context switch, packet, and I/O.  Keep the write side CPU-local and do
 * the (rare) aggregation when userspace reads the proc file instead.
 */
struct sqh_cpu_stats {
	unsigned long sched_enqueue;
	unsigned long sched_dequeue;
	unsigned long mem_reclaim_pages;
	unsigned long mem_reclaim_events;
	unsigned long kswapd_reclaim_events;
	unsigned long net_rx_packets;
	unsigned long net_rx_bytes;
	unsigned long block_issue;
	unsigned long block_complete;
	unsigned long block_error;
} ____cacheline_aligned;

static DEFINE_PER_CPU(struct sqh_cpu_stats, sqh_stats);

void sqh_sched_enqueue(struct task_struct *p, int cpu, int flags)
{
	(void)p;
	(void)cpu;
	(void)flags;
	this_cpu_inc(sqh_stats.sched_enqueue);
}

void sqh_sched_dequeue(struct task_struct *p, int cpu, int flags)
{
	(void)p;
	(void)cpu;
	(void)flags;
	this_cpu_inc(sqh_stats.sched_dequeue);
}

void sqh_mem_reclaim(struct pglist_data *pgdat, unsigned long reclaimed, bool kswapd)
{
	(void)pgdat;
	this_cpu_inc(sqh_stats.mem_reclaim_events);
	this_cpu_add(sqh_stats.mem_reclaim_pages, reclaimed);
	if (kswapd)
		this_cpu_inc(sqh_stats.kswapd_reclaim_events);
}

void sqh_net_rx(struct sk_buff *skb)
{
	this_cpu_inc(sqh_stats.net_rx_packets);
	this_cpu_add(sqh_stats.net_rx_bytes, skb->len);
}

void sqh_block_issue(struct request *rq)
{
	(void)rq;
	this_cpu_inc(sqh_stats.block_issue);
}

void sqh_block_complete(struct request *rq, blk_status_t error)
{
	(void)rq;
	this_cpu_inc(sqh_stats.block_complete);
	if (error != BLK_STS_OK)
		this_cpu_inc(sqh_stats.block_error);
}

static int sqh_proc_show(struct seq_file *m, void *v)
{
	struct sqh_cpu_stats total = { };
	int cpu;

	for_each_possible_cpu(cpu) {
		const struct sqh_cpu_stats *stats = per_cpu_ptr(&sqh_stats, cpu);

		total.sched_enqueue += READ_ONCE(stats->sched_enqueue);
		total.sched_dequeue += READ_ONCE(stats->sched_dequeue);
		total.mem_reclaim_events += READ_ONCE(stats->mem_reclaim_events);
		total.mem_reclaim_pages += READ_ONCE(stats->mem_reclaim_pages);
		total.kswapd_reclaim_events += READ_ONCE(stats->kswapd_reclaim_events);
		total.net_rx_packets += READ_ONCE(stats->net_rx_packets);
		total.net_rx_bytes += READ_ONCE(stats->net_rx_bytes);
		total.block_issue += READ_ONCE(stats->block_issue);
		total.block_complete += READ_ONCE(stats->block_complete);
		total.block_error += READ_ONCE(stats->block_error);
	}

	seq_puts(m, "singularity_qos_hints\n");
	seq_printf(m, "sched_enqueue=%lu\n", total.sched_enqueue);
	seq_printf(m, "sched_dequeue=%lu\n", total.sched_dequeue);
	seq_printf(m, "mem_reclaim_events=%lu\n", total.mem_reclaim_events);
	seq_printf(m, "mem_reclaim_pages=%lu\n", total.mem_reclaim_pages);
	seq_printf(m, "kswapd_reclaim_events=%lu\n", total.kswapd_reclaim_events);
	seq_printf(m, "net_rx_packets=%lu\n", total.net_rx_packets);
	seq_printf(m, "net_rx_bytes=%lu\n", total.net_rx_bytes);
	seq_printf(m, "block_issue=%lu\n", total.block_issue);
	seq_printf(m, "block_complete=%lu\n", total.block_complete);
	seq_printf(m, "block_error=%lu\n", total.block_error);
	return 0;
}

static int sqh_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, sqh_proc_show, NULL);
}

static const struct file_operations sqh_proc_fops = {
	.owner = THIS_MODULE,
	.open = sqh_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init sqh_init(void)
{
	if (!proc_create("singularity_qos_hints", 0444, NULL, &sqh_proc_fops))
		pr_warn("singularity_qos_hints: failed to create /proc entry\n");

	return 0;
}
late_initcall(sqh_init);
