// SPDX-License-Identifier: GPL-2.0
#include <linux/atomic.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/sched.h>
#include <linux/singularity_qos.h>
#include <linux/skbuff.h>
#include <linux/blkdev.h>

static atomic_long_t sqh_sched_enqueue_cnt = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_sched_dequeue_cnt = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_mem_reclaim_pages = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_mem_reclaim_events = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_kswapd_reclaim_events = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_net_rx_packets = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_net_rx_bytes = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_block_issue_cnt = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_block_complete_cnt = ATOMIC_LONG_INIT(0);
static atomic_long_t sqh_block_error_cnt = ATOMIC_LONG_INIT(0);

void sqh_sched_enqueue(struct task_struct *p, int cpu, int flags)
{
	(void)p;
	(void)cpu;
	(void)flags;
	atomic_long_inc(&sqh_sched_enqueue_cnt);
}

void sqh_sched_dequeue(struct task_struct *p, int cpu, int flags)
{
	(void)p;
	(void)cpu;
	(void)flags;
	atomic_long_inc(&sqh_sched_dequeue_cnt);
}

void sqh_mem_reclaim(struct pglist_data *pgdat, unsigned long reclaimed, bool kswapd)
{
	(void)pgdat;
	atomic_long_inc(&sqh_mem_reclaim_events);
	atomic_long_add(reclaimed, &sqh_mem_reclaim_pages);
	if (kswapd)
		atomic_long_inc(&sqh_kswapd_reclaim_events);
}

void sqh_net_rx(struct sk_buff *skb)
{
	atomic_long_inc(&sqh_net_rx_packets);
	atomic_long_add(skb->len, &sqh_net_rx_bytes);
}

void sqh_block_issue(struct request *rq)
{
	(void)rq;
	atomic_long_inc(&sqh_block_issue_cnt);
}

void sqh_block_complete(struct request *rq, blk_status_t error)
{
	atomic_long_inc(&sqh_block_complete_cnt);
	if (error != BLK_STS_OK)
		atomic_long_inc(&sqh_block_error_cnt);
}

static int sqh_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "orion_atlas_qos_hints\n");
	seq_printf(m, "sched_enqueue=%ld\n", atomic_long_read(&sqh_sched_enqueue_cnt));
	seq_printf(m, "sched_dequeue=%ld\n", atomic_long_read(&sqh_sched_dequeue_cnt));
	seq_printf(m, "mem_reclaim_events=%ld\n", atomic_long_read(&sqh_mem_reclaim_events));
	seq_printf(m, "mem_reclaim_pages=%ld\n", atomic_long_read(&sqh_mem_reclaim_pages));
	seq_printf(m, "kswapd_reclaim_events=%ld\n", atomic_long_read(&sqh_kswapd_reclaim_events));
	seq_printf(m, "net_rx_packets=%ld\n", atomic_long_read(&sqh_net_rx_packets));
	seq_printf(m, "net_rx_bytes=%ld\n", atomic_long_read(&sqh_net_rx_bytes));
	seq_printf(m, "block_issue=%ld\n", atomic_long_read(&sqh_block_issue_cnt));
	seq_printf(m, "block_complete=%ld\n", atomic_long_read(&sqh_block_complete_cnt));
	seq_printf(m, "block_error=%ld\n", atomic_long_read(&sqh_block_error_cnt));
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
	if (!proc_create("orion_atlas_qos_hints", 0444, NULL, &sqh_proc_fops))
		pr_warn("orion_atlas_qos_hints: failed to create /proc entry\n");

	return 0;
}
late_initcall(sqh_init);
