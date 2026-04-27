// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2013-2014, 2018, 2019, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "devbw: " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/devfreq.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/interconnect.h>
#include <linux/math64.h>
#include <linux/freezer.h>
#include <linux/pm.h>
#include <linux/suspend.h>
#include <trace/events/power.h>
#include <linux/msm-bus.h>
#include <linux/msm-bus-board.h>

/* Has to be ULL to prevent overflow where this macro is used. */
#define MBYTE (1ULL << 20)
#define MAX_PATHS	2
#define DBL_BUF		2

struct dev_data {
	struct icc_path *icc_paths[MAX_PATHS];
	struct msm_bus_vectors vectors[MAX_PATHS * DBL_BUF];
	struct msm_bus_paths bw_levels[DBL_BUF];
	struct msm_bus_scale_pdata bw_data;
	int num_paths;
	int num_icc_paths;
	u32 bus_client;
	int cur_idx;
	int cur_ab;
	int cur_ib;
	bool icc_supported;
	bool use_icc;
	bool icc_share_ab;
	bool icc_share_peak;
	bool icc_strict;
	u32 icc_boost_percent;
	u32 icc_min_avg_kbps;
	u32 icc_min_peak_kbps;
	u32 icc_upscale_percent;
	u32 polling_ms;
	long gov_ab;
	bool freeze_bw_blocked;
	struct devfreq *df;
	struct devfreq_dev_profile dp;
};

static bool devbw_suspend_in_progress(void)
{
	/*
	 * Defer bandwidth churn only while tasks/devices are being frozen for
	 * system suspend. During resume, pm_suspend_target_state may still reflect
	 * the previous sleep state while display bring-up is already running; do not
	 * block resume votes in that window.
	 */
	return READ_ONCE(pm_freezing);
}

static void devbw_log_icc_state(struct device *dev, struct dev_data *d)
{
	int i;

	dev_info_ratelimited(dev,
		"ICC state: supported=%d use_icc=%d requested_paths=%d attached_paths=%d\n",
		d->icc_supported, d->use_icc, d->num_paths, d->num_icc_paths);

	for (i = 0; i < d->num_paths; i++)
		dev_info_ratelimited(dev, "ICC path[%d]=%p\n", i,
				     d->icc_paths[i]);
}

static int set_bw(struct device *dev, int new_ib, int new_ab)
{
	struct dev_data *d = dev_get_drvdata(dev);
	u64 kbps;
	int i, ret;
	bool deferred = false;

	if (d->cur_ib == new_ib && d->cur_ab == new_ab)
		return 0;

	/*
	 * During suspend prepare/freezer, keep the last programmed vote and block
	 * vote churn from active governors/workqueues until PM has quiesced.
	 */
	if (unlikely(devbw_suspend_in_progress())) {
		if (!d->freeze_bw_blocked)
			dev_info_ratelimited(dev,
				"suspend-prep active, deferring devbw vote updates\n");
		d->freeze_bw_blocked = true;
		return 0;
	}

	d->freeze_bw_blocked = false;

	if (d->use_icc) {
		u32 avg_bw, peak_bw;

		/*
		 * msm-bus programmed per-path peak votes at full requested IB while
		 * average votes were split across paths. Keep AB behavior for legacy
		 * tuning, but do not split IB unless DT explicitly asks for it.
		 */
		kbps = (u64)new_ib * MBYTE;
		if (d->icc_share_peak)
			kbps = div_u64(kbps, d->num_icc_paths);
		peak_bw = div_u64(kbps, 1000ULL);

		kbps = (u64)new_ab * MBYTE;
		if (d->icc_share_ab)
			kbps = div_u64(kbps, d->num_icc_paths);
		avg_bw = div_u64(kbps, 1000ULL);

		if (new_ib && !peak_bw)
			peak_bw = 1;
		if (new_ab && !avg_bw)
			avg_bw = 1;

		/*
		 * Ramp-up assist: on upward transitions apply an optional temporary
		 * uplift so short bursts can reach sustainable DDR corners faster.
		 */
		if (d->icc_upscale_percent > 100 && new_ib > d->cur_ib) {
			u64 boosted;

			if (avg_bw) {
				boosted = mult_frac((u64)avg_bw,
						d->icc_upscale_percent, 100);
				avg_bw = min_t(u64, boosted, U32_MAX);
			}

			if (peak_bw) {
				boosted = mult_frac((u64)peak_bw,
						d->icc_upscale_percent, 100);
				peak_bw = min_t(u64, boosted, U32_MAX);
			}
		}

		/*
		 * Optional DT controlled headroom for benchmark-heavy or latency
		 * sensitive SKUs. Keep disabled by default and only apply on
		 * non-zero votes.
		 */
		if (d->icc_boost_percent > 100) {
			u64 boosted;

			if (avg_bw) {
				boosted = mult_frac((u64)avg_bw,
						d->icc_boost_percent, 100);
				avg_bw = min_t(u64, boosted, U32_MAX);
			}

			if (peak_bw) {
				boosted = mult_frac((u64)peak_bw,
						d->icc_boost_percent, 100);
				peak_bw = min_t(u64, boosted, U32_MAX);
			}
		}

		/*
		 * Keep non-zero ICC requests above optional DT floors so critical
		 * clients (CPU/GPU/NPU) don't fall into very low perf corners.
		 */
		if (avg_bw && d->icc_min_avg_kbps &&
		    avg_bw < d->icc_min_avg_kbps)
			avg_bw = d->icc_min_avg_kbps;

		if (peak_bw && d->icc_min_peak_kbps &&
		    peak_bw < d->icc_min_peak_kbps)
			peak_bw = d->icc_min_peak_kbps;

		dev_info_ratelimited(dev,
			"ICC vote: dev=%s freq=%d avg=%u peak=%u paths=%d split_ab=%d split_peak=%d\n",
			dev_name(dev), new_ib, avg_bw, peak_bw, d->num_icc_paths,
			d->icc_share_ab, d->icc_share_peak);

		for (i = 0; i < d->num_icc_paths; i++) {
			ret = icc_set_bw(d->icc_paths[i], avg_bw, peak_bw);
			if (ret == -EAGAIN) {
				dev_dbg_ratelimited(dev,
					"ICC bandwidth request deferred (-EAGAIN), continuing\n");
				deferred = true;
				continue;
			}
			if (ret) {
				dev_err(dev, "ICC bandwidth request failed (%d)\n",
					ret);
				return ret;
			}
		}

		/*
		 * Keep cur_* unchanged when a vote was deferred so the next
		 * governor tick retries the exact same request instead of
		 * treating it as already applied.
		 */
		if (!deferred) {
			d->cur_ib = new_ib;
			d->cur_ab = new_ab;
		}

		return 0;
	}

	i = (d->cur_idx + 1) % DBL_BUF;

	d->bw_levels[i].vectors[0].ib = new_ib * MBYTE;
	d->bw_levels[i].vectors[0].ab = new_ab / d->num_paths * MBYTE;
	d->bw_levels[i].vectors[1].ib = new_ib * MBYTE;
	d->bw_levels[i].vectors[1].ab = new_ab / d->num_paths * MBYTE;

	dev_dbg(dev, "BW MBps: AB: %d IB: %d\n", new_ab, new_ib);

	ret = msm_bus_scale_client_update_request(d->bus_client, i);
	if (ret) {
		dev_err(dev, "bandwidth request failed (%d)\n", ret);
	} else {
		d->cur_idx = i;
		d->cur_ib = new_ib;
		d->cur_ab = new_ab;
	}

	return ret;
}

static int devbw_target(struct device *dev, unsigned long *freq, u32 flags)
{
	struct dev_data *d = dev_get_drvdata(dev);
	struct dev_pm_opp *opp;
	long gov_ab;

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);

	/*
	 * Performance-style governors generally drive only IB (freq) while AB
	 * stays at zero. For ICC this leads to weak/zero aggregate visibility and
	 * can starve paths where AB is treated as the sustaining vote.
	 *
	 * When no governor-provided AB is available, start from the requested IB
	 * and add modest headroom on non-decreasing transitions so short GPU bursts
	 * do not spend their whole frame waiting for sustained bandwidth to catch up.
	 */
	gov_ab = d->gov_ab;
	if (!gov_ab) {
		gov_ab = *freq;
		if (*freq >= d->cur_ib)
			gov_ab = mult_frac(*freq, 5, 4);
	}

	return set_bw(dev, *freq, gov_ab);
}

static int devbw_get_dev_status(struct device *dev,
				struct devfreq_dev_status *stat)
{
	struct dev_data *d = dev_get_drvdata(dev);

	stat->private_data = &d->gov_ab;
	return 0;
}

#define PROP_PORTS "qcom,src-dst-ports"
#define PROP_ACTIVE "qcom,active-only"

static void devbw_log_icc_dt(struct device *dev, int num_icc_paths)
{
	struct property *prop;

	dev_info_ratelimited(dev, "DT node: %s\n",
			     dev->of_node ? dev->of_node->full_name : "<none>");

	prop = of_find_property(dev->of_node, "interconnects", NULL);
	if (prop)
		dev_info_ratelimited(dev,
			"interconnects present (len=%d bytes, parsed_paths=%d)\n",
			prop->length, num_icc_paths);
	else
		dev_info_ratelimited(dev, "interconnects property missing\n");

	prop = of_find_property(dev->of_node, "interconnect-names", NULL);
	if (prop)
		dev_info_ratelimited(dev, "interconnect-names present (len=%d bytes)\n",
				     prop->length);
	else
		dev_info_ratelimited(dev, "interconnect-names property missing\n");
}

int devfreq_add_devbw(struct device *dev)
{
	struct dev_data *d;
	struct devfreq_dev_profile *p;
	u32 ports[MAX_PATHS * 2];
	bool have_ports;
	const char *icc_name;
	const char *gov_name;
	int ret, len, i, num_paths = 0, num_icc_paths;
	struct opp_table *opp_table;
	u32 version;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	dev_set_drvdata(dev, d);
	have_ports = of_find_property(dev->of_node, PROP_PORTS, &len);

	num_icc_paths = of_count_phandle_with_args(dev->of_node,
					  "interconnects",
					  "#interconnect-cells");
	d->icc_supported = of_find_property(dev->of_node, "interconnects", NULL);
	d->icc_share_ab = true;
	d->icc_share_peak = of_property_read_bool(dev->of_node,
						 "qcom,icc-split-peak-kbps");
	d->icc_strict = of_property_read_bool(dev->of_node, "qcom,icc-strict");
	if (of_find_property(dev->of_node, "qcom,icc-no-split-ab-kbps", NULL))
		d->icc_share_ab = false;
	of_property_read_u32(dev->of_node, "qcom,icc-min-avg-kbps",
			     &d->icc_min_avg_kbps);
	of_property_read_u32(dev->of_node, "qcom,icc-min-peak-kbps",
			     &d->icc_min_peak_kbps);
	of_property_read_u32(dev->of_node, "qcom,icc-boost-percent",
			     &d->icc_boost_percent);
	of_property_read_u32(dev->of_node, "qcom,icc-upscale-percent",
			     &d->icc_upscale_percent);
	of_property_read_u32(dev->of_node, "qcom,polling-ms", &d->polling_ms);
	if (d->icc_boost_percent && d->icc_boost_percent < 100)
		d->icc_boost_percent = 100;
	if (d->icc_boost_percent > 400)
		d->icc_boost_percent = 400;
	if (d->icc_upscale_percent && d->icc_upscale_percent < 100)
		d->icc_upscale_percent = 100;
	if (d->icc_upscale_percent > 300)
		d->icc_upscale_percent = 300;
	if (!d->polling_ms)
		d->polling_ms = 50;
	if (d->polling_ms < 5)
		d->polling_ms = 5;
	if (d->polling_ms > 100)
		d->polling_ms = 100;
	ret = 0;
	if (num_icc_paths < 0) {
		devbw_log_icc_dt(dev, num_icc_paths);

		if (!of_find_property(dev->of_node, "interconnects", NULL))
			num_icc_paths = 0;
		else if (have_ports && !d->icc_strict) {
			dev_warn(dev,
				 "Invalid ICC description (%d), falling back to msm-bus\n",
				 num_icc_paths);
			num_icc_paths = 0;
		} else {
			return num_icc_paths;
		}
	}

	if (num_icc_paths > 0) {
		devbw_log_icc_dt(dev, num_icc_paths);

		if (num_icc_paths > MAX_PATHS) {
			if (have_ports && !d->icc_strict) {
				dev_warn(dev,
					 "Unexpected number of ICC paths, falling back to msm-bus\n");
				num_icc_paths = 0;
				goto use_msm_bus;
			}

			dev_err(dev, "Unexpected number of ICC paths\n");
			return -EINVAL;
		}

		if (of_find_property(dev->of_node, "interconnect-names", NULL)) {
			for (i = 0; i < num_icc_paths; i++) {
				ret = of_property_read_string_index(dev->of_node,
								    "interconnect-names",
								    i, &icc_name);
				if (ret) {
					dev_warn_ratelimited(dev,
						"interconnect-names[%d] read failed (%d)\n",
						i, ret);
					break;
				}

				dev_dbg(dev, "Requesting ICC path[%d] by name '%s'\n",
					i, icc_name);

				d->icc_paths[i] = devm_of_icc_get(dev, icc_name);
				if (IS_ERR(d->icc_paths[i])) {
					ret = PTR_ERR(d->icc_paths[i]);
					d->icc_paths[i] = NULL;
					dev_warn_ratelimited(dev,
						"ICC attach failed at index %d (name '%s', err=%d)\n",
						i, icc_name, ret);
					break;
				}

				dev_dbg(dev,
					"Attached ICC path[%d] using name '%s'\n",
					i, icc_name);
			}
		} else if (num_icc_paths == 1) {
			d->icc_paths[0] = devm_of_icc_get(dev, NULL);
			if (IS_ERR(d->icc_paths[0])) {
				ret = PTR_ERR(d->icc_paths[0]);
				d->icc_paths[0] = NULL;
				dev_warn_ratelimited(dev,
					"ICC attach failed for unnamed single path (err=%d)\n",
					ret);
			} else {
				dev_dbg(dev, "Attached ICC unnamed single path\n");
			}
		} else {
			if (have_ports && !d->icc_strict) {
				dev_err_ratelimited(dev,
					"ICC attach failed: missing interconnect-names for %d paths, falling back to msm-bus\n",
					num_icc_paths);
				ret = -EINVAL;
				goto use_msm_bus;
			}

			dev_err(dev,
				"ICC attach failed: missing interconnect-names for %d paths\n",
				num_icc_paths);
			return -EINVAL;
		}

		if (!ret) {
			for (i = 0; i < num_icc_paths; i++) {
				if (!d->icc_paths[i]) {
					ret = -ENODEV;
					dev_err_ratelimited(dev,
						"ICC attach failed: NULL path at index %d\n",
						i);
					break;
				}
			}
		}

		if (!ret) {
			d->use_icc = true;
			d->num_paths = num_icc_paths;
			d->num_icc_paths = num_icc_paths;
		}
	}

	if (ret) {
		if (!have_ports || d->icc_strict)
			return ret;

		dev_err_ratelimited(dev,
			"ICC unavailable (err=%d), falling back to msm-bus via %s\n",
			ret, PROP_PORTS);
	}

use_msm_bus:
	if (!d->use_icc && have_ports) {
		len /= sizeof(ports[0]);
		if (len % 2 || len > ARRAY_SIZE(ports)) {
			dev_err(dev, "Unexpected number of ports\n");
			return -EINVAL;
		}

		ret = of_property_read_u32_array(dev->of_node, PROP_PORTS,
						 ports, len);
		if (ret)
			return ret;

		num_paths = len / 2;
	} else {
		if (!d->use_icc)
			return -EINVAL;
	}

	if (!d->use_icc) {
		d->bw_levels[0].vectors = &d->vectors[0];
		d->bw_levels[1].vectors = &d->vectors[MAX_PATHS];
		d->bw_data.usecase = d->bw_levels;
		d->bw_data.num_usecases = ARRAY_SIZE(d->bw_levels);
		d->bw_data.name = dev_name(dev);
		d->bw_data.active_only = of_property_read_bool(dev->of_node,
							PROP_ACTIVE);

		for (i = 0; i < num_paths; i++) {
			d->bw_levels[0].vectors[i].src = ports[2 * i];
			d->bw_levels[0].vectors[i].dst = ports[2 * i + 1];
			d->bw_levels[1].vectors[i].src = ports[2 * i];
			d->bw_levels[1].vectors[i].dst = ports[2 * i + 1];
		}
		d->bw_levels[0].num_paths = num_paths;
		d->bw_levels[1].num_paths = num_paths;
		d->num_paths = num_paths;
	}

	devbw_log_icc_state(dev, d);

	p = &d->dp;
	p->polling_ms = d->polling_ms;
	p->target = devbw_target;
	p->get_dev_status = devbw_get_dev_status;

	if (of_device_is_compatible(dev->of_node, "qcom,devbw-ddr")) {
		version = (1 << of_fdt_get_ddrtype());
		opp_table = dev_pm_opp_set_supported_hw(dev, &version, 1);
		if (IS_ERR(opp_table)) {
			dev_err(dev, "Failed to set supported hardware\n");
			return PTR_ERR(opp_table);
		}
	}

	ret = dev_pm_opp_of_add_table(dev);
	if (ret)
		dev_err(dev, "Couldn't parse OPP table:%d\n", ret);

	if (!d->use_icc) {
		d->bus_client = msm_bus_scale_register_client(&d->bw_data);
		if (!d->bus_client) {
			dev_err(dev, "Unable to register bus client\n");
			return -ENODEV;
		}
	}

	if (of_property_read_string(dev->of_node, "governor", &gov_name))
		gov_name = "performance";

	d->df = devfreq_add_device(dev, p, gov_name, NULL);
	if (IS_ERR(d->df)) {
		if (!d->use_icc)
			msm_bus_scale_unregister_client(d->bus_client);
		return PTR_ERR(d->df);
	}

	return 0;
}

int devfreq_remove_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);
	int i;

	if (d->use_icc) {
		for (i = 0; i < d->num_icc_paths; i++)
			icc_set_bw(d->icc_paths[i], 0, 0);
	} else {
		msm_bus_scale_unregister_client(d->bus_client);
	}

	devfreq_remove_device(d->df);
	return 0;
}

int devfreq_suspend_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);

	return devfreq_suspend_device(d->df);
}

int devfreq_resume_devbw(struct device *dev)
{
	struct dev_data *d = dev_get_drvdata(dev);

	return devfreq_resume_device(d->df);
}

static int devfreq_devbw_probe(struct platform_device *pdev)
{
	return devfreq_add_devbw(&pdev->dev);
}

static int devfreq_devbw_remove(struct platform_device *pdev)
{
	return devfreq_remove_devbw(&pdev->dev);
}

static int __maybe_unused devfreq_devbw_suspend(struct device *dev)
{
	return devfreq_suspend_devbw(dev);
}

static int __maybe_unused devfreq_devbw_resume(struct device *dev)
{
	return devfreq_resume_devbw(dev);
}

static SIMPLE_DEV_PM_OPS(devfreq_devbw_pm_ops,
			 devfreq_devbw_suspend, devfreq_devbw_resume);

static const struct of_device_id devbw_match_table[] = {
	{ .compatible = "qcom,devbw-llcc" },
	{ .compatible = "qcom,devbw-ddr" },
	{ .compatible = "qcom,devbw" },
	{}
};

static struct platform_driver devbw_driver = {
	.probe = devfreq_devbw_probe,
	.remove = devfreq_devbw_remove,
	.driver = {
		.name = "devbw",
		.of_match_table = devbw_match_table,
		.pm = &devfreq_devbw_pm_ops,
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(devbw_driver);
MODULE_DESCRIPTION("Device DDR bandwidth voting driver MSM SoCs");
MODULE_LICENSE("GPL v2");
