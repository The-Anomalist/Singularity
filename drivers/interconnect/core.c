// SPDX-License-Identifier: GPL-2.0-only
/*
 * Simple interconnect framework backport.
 */

#include <linux/device.h>
#include <linux/hashtable.h>
#include <linux/init.h>
#include <linux/interconnect-provider.h>
#include <linux/interconnect.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <linux/property.h>
#include <linux/rcupdate.h>
#include <linux/slab.h>

static LIST_HEAD(icc_providers);
static DEFINE_HASHTABLE(icc_providers_ht, 6);
static DEFINE_MUTEX(icc_lock);
static struct class *icc_class;
static atomic_long_t icc_resume_seq = ATOMIC_LONG_INIT(0);

static int icc_pm_notifier(struct notifier_block *nb, unsigned long action,
                           void *data)
{
        switch (action) {
        case PM_POST_SUSPEND:
        case PM_POST_HIBERNATION:
        case PM_POST_RESTORE:
                atomic_long_inc(&icc_resume_seq);
                break;
        default:
                break;
        }

        return NOTIFY_DONE;
}

static struct notifier_block icc_pm_nb = {
        .notifier_call = icc_pm_notifier,
};

static void icc_class_dev_release(struct device *dev)
{
	/* All class devices only carry dynamically allocated drvdata */
	kfree(dev_get_drvdata(dev));
}

static int __init icc_init_sysfs(void)
{
        icc_class = class_create(THIS_MODULE, "interconnect");
        if (IS_ERR(icc_class)) {
                long ret = PTR_ERR(icc_class);

		pr_err("interconnect: failed to create sysfs class (%ld)\n", ret);
		icc_class = NULL;

		return ret;
	}

        icc_class->dev_release = icc_class_dev_release;

        register_pm_notifier(&icc_pm_nb);

        return 0;
}
postcore_initcall(icc_init_sysfs);

struct class *icc_class_get(void)
{
	return icc_class;
}
EXPORT_SYMBOL_GPL(icc_class_get);

static struct icc_provider *icc_find_provider_rcu(struct device_node *np)
{
	struct icc_provider *provider;

	hash_for_each_possible_rcu(icc_providers_ht, provider, hlist,
				   (unsigned long)np) {
		if (provider->of_node == np)
			return provider;
	}

	return NULL;
}

int icc_provider_register(struct icc_provider *provider)
{
	struct icc_provider *tmp;
	int ret = 0;

	if (!provider || !provider->dev || !provider->xlate)
		return -EINVAL;

	if (!provider->of_node)
		provider->of_node = provider->dev->of_node;

	if (!provider->of_node)
		return -EINVAL;

	mutex_lock(&icc_lock);
	hash_for_each_possible(icc_providers_ht, tmp, hlist,
			       (unsigned long)provider->of_node) {
		if (tmp->of_node == provider->of_node) {
			ret = -EEXIST;
			goto out_unlock;
		}
	}

	INIT_HLIST_NODE(&provider->hlist);
	hash_add_rcu(icc_providers_ht, &provider->hlist,
		     (unsigned long)provider->of_node);
	list_add_tail(&provider->node, &icc_providers);

out_unlock:
	mutex_unlock(&icc_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(icc_provider_register);

void icc_provider_unregister(struct icc_provider *provider)
{
	if (!provider)
		return;

	mutex_lock(&icc_lock);
	hash_del_rcu(&provider->hlist);
	list_del(&provider->node);
	mutex_unlock(&icc_lock);

	synchronize_rcu();
}
EXPORT_SYMBOL_GPL(icc_provider_unregister);

struct icc_path *icc_of_xlate_onecell(struct icc_provider *provider,
      const struct of_phandle_args *spec)
{
struct icc_path *path;

if (!provider || !spec || spec->args_count != 1)
return ERR_PTR(-EINVAL);

path = kzalloc(sizeof(*path), GFP_KERNEL);
if (!path)
return ERR_PTR(-ENOMEM);

path->provider = provider;
path->id = spec->args[0];
mutex_init(&path->lock);

return path;
}
EXPORT_SYMBOL_GPL(icc_of_xlate_onecell);

static struct icc_provider *icc_get_provider(struct device_node *np)
{
	struct icc_provider *provider;

	rcu_read_lock();
	provider = icc_find_provider_rcu(np);
	rcu_read_unlock();

	return provider;
}

static struct icc_path *__of_icc_get(struct device *dev, int index)
{
struct of_phandle_args spec;
struct icc_provider *provider;
struct icc_path *path;
int ret;

if (!dev || !dev->of_node)
return ERR_PTR(-EINVAL);

ret = of_parse_phandle_with_args(dev->of_node, "interconnects",
"#interconnect-cells", index, &spec);
if (ret)
return ERR_PTR(ret);

provider = icc_get_provider(spec.np);
if (!provider) {
of_node_put(spec.np);
return ERR_PTR(-EPROBE_DEFER);
}

path = provider->xlate(provider, &spec);
of_node_put(spec.np);

return path;
}

struct icc_path *of_icc_get(struct device *dev, const char *name)
{
int index = 0;

if (!dev)
return ERR_PTR(-EINVAL);

if (name) {
index = of_property_match_string(dev->of_node,
"interconnect-names", name);
if (index < 0)
return ERR_PTR(index);
}

return __of_icc_get(dev, index);
}
EXPORT_SYMBOL_GPL(of_icc_get);

static void devm_icc_put(struct device *dev, void *res)
{
struct icc_path *path = *(struct icc_path **)res;

icc_put(path);
}

struct icc_path *devm_of_icc_get(struct device *dev, const char *name)
{
struct icc_path **ptr, *path;

ptr = devres_alloc(devm_icc_put, sizeof(*ptr), GFP_KERNEL);
if (!ptr)
return ERR_PTR(-ENOMEM);

path = of_icc_get(dev, name);
if (IS_ERR(path)) {
devres_free(ptr);
return path;
}

*ptr = path;
devres_add(dev, ptr);

return path;
}
EXPORT_SYMBOL_GPL(devm_of_icc_get);

void icc_put(struct icc_path *path)
{
if (IS_ERR_OR_NULL(path))
return;

if (path->provider && path->provider->release)
path->provider->release(path);
else
kfree(path);
}
EXPORT_SYMBOL_GPL(icc_put);

int icc_set_tag(struct icc_path *path, u32 tag)
{
	if (IS_ERR_OR_NULL(path))
		return -EINVAL;

	mutex_lock(&path->lock);
	path->tag = tag;
	mutex_unlock(&path->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(icc_set_tag);

int icc_set_bw(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	unsigned long seq;
	u32 prev_avg, prev_peak;
	unsigned long prev_seq;
	bool need_reapply;
	int ret;

	if (IS_ERR_OR_NULL(path))
		return -EINVAL;

	/* Keep the cache check and provider transaction atomic per path. */
	mutex_lock(&path->lock);

	seq = atomic_long_read(&icc_resume_seq);
	prev_avg = path->avg_bw;
	prev_peak = path->peak_bw;
	prev_seq = path->resume_seq;
	need_reapply = path->resume_seq != seq ||
		       path->applied_tag != path->tag;

	if (!need_reapply && prev_avg == avg_bw && prev_peak == peak_bw) {
		ret = 0;
		goto out_unlock;
	}

	/*
	 * Keep zero-bandwidth paths in the fast path after resume. There is
	 * no vote to restore, so avoid an unnecessary provider callback.
	 */
	if (need_reapply && !avg_bw && !peak_bw && !prev_avg && !prev_peak) {
		path->applied_tag = path->tag;
		path->resume_seq = seq;
		ret = 0;
		goto out_unlock;
	}

	if (!path->provider || !path->provider->set) {
		path->avg_bw = avg_bw;
		path->peak_bw = peak_bw;
		path->applied_tag = path->tag;
		path->resume_seq = seq;
		ret = 0;
		goto out_unlock;
	}

	ret = path->provider->set(path, avg_bw, peak_bw);
	if (ret) {
		/*
		 * Do not cache a vote that the provider rejected or deferred. Kona can
		 * return -EAGAIN while cmd-db/RPMh is not ready; the next caller must
		 * still reprogram the vote instead of being skipped as unchanged.
		 */
		path->avg_bw = prev_avg;
		path->peak_bw = prev_peak;
		path->resume_seq = prev_seq;
		goto out_unlock;
	}

	path->avg_bw = avg_bw;
	path->peak_bw = peak_bw;
	path->applied_tag = path->tag;
	path->resume_seq = seq;

	ret = 0;
out_unlock:
	mutex_unlock(&path->lock);
	return ret;
}

EXPORT_SYMBOL_GPL(icc_set_bw);
