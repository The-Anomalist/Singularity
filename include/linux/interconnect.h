/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __LINUX_INTERCONNECT_H__
#define __LINUX_INTERCONNECT_H__

#include <linux/err.h>
#include <linux/types.h>

struct device;
struct icc_path;

#if IS_ENABLED(CONFIG_INTERCONNECT)
struct icc_path *of_icc_get(struct device *dev, const char *name);
struct icc_path *devm_of_icc_get(struct device *dev, const char *name);
void icc_put(struct icc_path *path);
int icc_set_bw(struct icc_path *path, u32 avg_bw, u32 peak_bw);
int icc_set_tag(struct icc_path *path, u32 tag);
#else
static inline struct icc_path *of_icc_get(struct device *dev, const char *name)
{
return ERR_PTR(-ENODEV);
}

static inline struct icc_path *devm_of_icc_get(struct device *dev,
      const char *name)
{
return ERR_PTR(-ENODEV);
}

static inline void icc_put(struct icc_path *path) { }

static inline int icc_set_bw(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
return -ENODEV;
}

static inline int icc_set_tag(struct icc_path *path, u32 tag)
{
return -ENODEV;
}
#endif

#endif
