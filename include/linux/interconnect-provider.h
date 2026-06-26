/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_INTERCONNECT_PROVIDER_H__
#define __LINUX_INTERCONNECT_PROVIDER_H__

#include <linux/list.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/of.h>

struct class;
struct device;
struct device_node;
struct of_phandle_args;

struct icc_path;

struct icc_provider {
    struct device *dev;
    struct device_node *of_node;
    struct list_head node;
    struct hlist_node hlist;
    struct icc_path *(*xlate)(struct icc_provider *,
                              const struct of_phandle_args *spec);
    int (*set)(struct icc_path *path, u32 avg_bw, u32 peak_bw);
    void (*release)(struct icc_path *path);
};

struct icc_path {
        struct icc_provider *provider;
        u32 id;
        u32 tag;
        u32 avg_bw;
        u32 peak_bw;
        struct mutex lock;
        /*
         * Tracks the last system resume generation this path synced
         * against. When the system resumes from suspend we bump a
         * global generation counter so callers can reapply bandwidth
         * votes even if the requested values did not change.
         */
        unsigned long resume_seq;
        void *data;
};

int icc_provider_register(struct icc_provider *provider);
void icc_provider_unregister(struct icc_provider *provider);
struct icc_path *icc_of_xlate_onecell(struct icc_provider *provider,
                                      const struct of_phandle_args *spec);

#if IS_ENABLED(CONFIG_INTERCONNECT)
struct class *icc_class_get(void);
#else
static inline struct class *icc_class_get(void)
{
	return NULL;
}
#endif

#endif

