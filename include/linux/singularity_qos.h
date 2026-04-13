#ifndef _LINUX_SINGULARITY_QOS_H
#define _LINUX_SINGULARITY_QOS_H

#include <linux/types.h>
#include <linux/blk_types.h>

struct task_struct;
struct request;
struct sk_buff;
struct pglist_data;

#ifdef CONFIG_SINGULARITY_QOS_HINTS
void sqh_sched_enqueue(struct task_struct *p, int cpu, int flags);
void sqh_sched_dequeue(struct task_struct *p, int cpu, int flags);
void sqh_mem_reclaim(struct pglist_data *pgdat, unsigned long reclaimed, bool kswapd);
void sqh_net_rx(struct sk_buff *skb);
void sqh_block_issue(struct request *rq);
void sqh_block_complete(struct request *rq, blk_status_t error);
#else
static inline void sqh_sched_enqueue(struct task_struct *p, int cpu, int flags) {}
static inline void sqh_sched_dequeue(struct task_struct *p, int cpu, int flags) {}
static inline void sqh_mem_reclaim(struct pglist_data *pgdat, unsigned long reclaimed, bool kswapd) {}
static inline void sqh_net_rx(struct sk_buff *skb) {}
static inline void sqh_block_issue(struct request *rq) {}
static inline void sqh_block_complete(struct request *rq, blk_status_t error) {}
#endif

#endif /* _LINUX_SINGULARITY_QOS_H */
