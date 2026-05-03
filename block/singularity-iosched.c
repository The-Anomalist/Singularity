// SPDX-License-Identifier: GPL-2.0
/*
 * Singularity cgroup-aware latency I/O scheduler.
 */
#include <linux/bio.h>
#include <linux/blk-cgroup.h>
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/hash.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/jiffies.h>

#define SING_NR_CLASSES          2
#define SING_SYNC_CLASS          0
#define SING_BG_CLASS            1
#define SING_MAX_BUCKETS         16
#define SING_DYN_FG_BATCH_MIN    2
#define SING_DYN_FG_BATCH_MAX    12
#define SING_BG_STARVE_MS        12

static unsigned int sing_fg_batch = 6;
module_param_named(fg_batch, sing_fg_batch, uint, 0644);
MODULE_PARM_DESC(fg_batch, "Consecutive sync-class dispatch budget");

static bool sing_adaptive_batch = true;
module_param_named(adaptive_batch, sing_adaptive_batch, bool, 0644);
MODULE_PARM_DESC(adaptive_batch,
		 "Adapt sync dispatch budget to queue pressure and write ratio");

struct sing_data {
	struct list_head q[SING_NR_CLASSES][SING_MAX_BUCKETS];
	unsigned long bg_head_jiffies[SING_MAX_BUCKETS];
	unsigned int rr[SING_NR_CLASSES];
	unsigned int fg_budget;
	unsigned int queued[SING_NR_CLASSES];
};

static inline unsigned int sing_class(struct request *rq)
{
	return rq_is_sync(rq) ? SING_SYNC_CLASS : SING_BG_CLASS;
}

static inline unsigned int sing_bucket(struct request *rq)
{
	struct blkcg *blkcg;
	u64 serial = 0;

	if (rq->bio) {
		blkcg = bio_blkcg(rq->bio);
		if (blkcg)
			serial = blkcg->css.serial_nr;
	}

	return hash_64(serial, ilog2(SING_MAX_BUCKETS));
}

static inline struct list_head *sing_list(struct sing_data *sd,
					  struct request *rq)
{
	return &sd->q[sing_class(rq)][sing_bucket(rq)];
}

static inline bool sing_bg_starving(struct sing_data *sd)
{
	unsigned int i;
	unsigned long now = jiffies;
	unsigned long starve = msecs_to_jiffies(SING_BG_STARVE_MS);

	for (i = 0; i < SING_MAX_BUCKETS; i++) {
		if (!sd->bg_head_jiffies[i])
			continue;
		if (time_after_eq(now, sd->bg_head_jiffies[i] + starve))
			return true;
	}

	return false;
}

static inline unsigned int sing_dynamic_fg_batch(struct sing_data *sd)
{
	unsigned int sync_q = sd->queued[SING_SYNC_CLASS];
	unsigned int bg_q = sd->queued[SING_BG_CLASS];
	unsigned int total_q = sync_q + bg_q;
	unsigned int batch;

	if (!sing_adaptive_batch)
		return max_t(unsigned int, 1, sing_fg_batch);

	batch = clamp_t(unsigned int, sing_fg_batch,
			SING_DYN_FG_BATCH_MIN, SING_DYN_FG_BATCH_MAX);

	/*
	 * Preserve foreground latency under heavy queue pressure. If the queue
	 * is mostly background writes, ease off to create larger contiguous
	 * write windows and reduce wakeups.
	 */
	if (total_q > 24 && sync_q > bg_q)
		batch = min_t(unsigned int, batch + 2, SING_DYN_FG_BATCH_MAX);
	else if (bg_q > (sync_q << 1))
		batch = max_t(unsigned int, batch - 2, SING_DYN_FG_BATCH_MIN);
	else if (total_q < 8)
		batch = max_t(unsigned int, batch - 1, SING_DYN_FG_BATCH_MIN);

	return batch;
}

static void sing_merged_requests(struct request_queue *q, struct request *rq,
				 struct request *next)
{
	list_del_init(&next->queuelist);
}

static int sing_dispatch_class(struct request_queue *q, struct sing_data *sd,
			       unsigned int class)
{
	unsigned int i;

	for (i = 0; i < SING_MAX_BUCKETS; i++) {
		unsigned int b = (sd->rr[class] + i) & (SING_MAX_BUCKETS - 1);
		struct request *rq;

		rq = list_first_entry_or_null(&sd->q[class][b], struct request,
					     queuelist);
		if (!rq)
			continue;

		list_del_init(&rq->queuelist);
		if (class == SING_BG_CLASS && list_empty(&sd->q[class][b]))
			sd->bg_head_jiffies[b] = 0;
		sd->queued[class]--;
		sd->rr[class] = (b + 1) & (SING_MAX_BUCKETS - 1);
		elv_dispatch_sort(q, rq);
		return 1;
	}

	return 0;
}

static int sing_dispatch(struct request_queue *q, int force)
{
	struct sing_data *sd = q->elevator->elevator_data;
	unsigned int fg_batch = sing_dynamic_fg_batch(sd);

	/* Bound BG latency to avoid stalling streaming writes. */
	if (sing_bg_starving(sd) && sing_dispatch_class(q, sd, SING_BG_CLASS)) {
		sd->fg_budget = 0;
		return 1;
	}

	if (sd->fg_budget < fg_batch && sing_dispatch_class(q, sd, SING_SYNC_CLASS)) {
		sd->fg_budget++;
		return 1;
	}

	if (sing_dispatch_class(q, sd, SING_BG_CLASS)) {
		sd->fg_budget = 0;
		return 1;
	}

	if (sing_dispatch_class(q, sd, SING_SYNC_CLASS)) {
		sd->fg_budget = min(sd->fg_budget + 1, fg_batch);
		return 1;
	}

	return 0;
}

static void sing_add_request(struct request_queue *q, struct request *rq)
{
	struct sing_data *sd = q->elevator->elevator_data;
	unsigned int class = sing_class(rq);
	unsigned int bucket = sing_bucket(rq);
	struct list_head *head = &sd->q[class][bucket];
	bool was_empty = list_empty(head);

	list_add_tail(&rq->queuelist, head);
	sd->queued[class]++;
	if (class == SING_BG_CLASS && was_empty)
		sd->bg_head_jiffies[bucket] = jiffies;
}

static struct request *sing_former_request(struct request_queue *q,
					   struct request *rq)
{
	struct sing_data *sd = q->elevator->elevator_data;
	struct list_head *head = sing_list(sd, rq);

	if (rq->queuelist.prev == head)
		return NULL;
	return list_prev_entry(rq, queuelist);
}

static struct request *sing_latter_request(struct request_queue *q,
					   struct request *rq)
{
	struct sing_data *sd = q->elevator->elevator_data;
	struct list_head *head = sing_list(sd, rq);

	if (rq->queuelist.next == head)
		return NULL;
	return list_next_entry(rq, queuelist);
}

static int sing_init_queue(struct request_queue *q, struct elevator_type *e)
{
	struct sing_data *sd;
	struct elevator_queue *eq;
	unsigned int c, b;

	eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	sd = kzalloc_node(sizeof(*sd), GFP_KERNEL, q->node);
	if (!sd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}

	for (c = 0; c < SING_NR_CLASSES; c++)
		for (b = 0; b < SING_MAX_BUCKETS; b++)
			INIT_LIST_HEAD(&sd->q[c][b]);

	eq->elevator_data = sd;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static void sing_exit_queue(struct elevator_queue *e)
{
	struct sing_data *sd = e->elevator_data;
	unsigned int c, b;

	for (c = 0; c < SING_NR_CLASSES; c++)
		for (b = 0; b < SING_MAX_BUCKETS; b++)
			BUG_ON(!list_empty(&sd->q[c][b]));

	kfree(sd);
}

static struct elevator_type elevator_singularity = {
	.ops.sq = {
		.elevator_merge_req_fn          = sing_merged_requests,
		.elevator_dispatch_fn           = sing_dispatch,
		.elevator_add_req_fn            = sing_add_request,
		.elevator_former_req_fn         = sing_former_request,
		.elevator_latter_req_fn         = sing_latter_request,
		.elevator_init_fn               = sing_init_queue,
		.elevator_exit_fn               = sing_exit_queue,
	},
	.elevator_name = "singularity",
	.elevator_owner = THIS_MODULE,
};

static int __init sing_init(void)
{
	return elv_register(&elevator_singularity);
}

static void __exit sing_exit(void)
{
	elv_unregister(&elevator_singularity);
}

module_init(sing_init);
module_exit(sing_exit);

MODULE_AUTHOR("Singularity Performance Team");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cgroup-aware latency-first I/O scheduler");
