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
	bool use_icc;
	long gov_ab;
	struct devfreq *df;
	struct devfreq_dev_profile dp;
};

static int set_bw(struct device *dev, int new_ib, int new_ab)
{
	struct dev_data *d = dev_get_drvdata(dev);
	int i, ret;

	if (d->cur_ib == new_ib && d->cur_ab == new_ab)
		return 0;

	if (d->use_icc) {
		u32 avg_bw, peak_bw;

		peak_bw = div_u64((u64)new_ib * MBYTE,
				 d->num_icc_paths * 1000ULL);
		avg_bw = div_u64((u64)new_ab * MBYTE,
				d->num_icc_paths * 1000ULL);
		dev_dbg(dev, "ICC BW KBps: AB: %u IB: %u\n", avg_bw, peak_bw);

		for (i = 0; i < d->num_icc_paths; i++) {
			ret = icc_set_bw(d->icc_paths[i], avg_bw, peak_bw);
			if (ret) {
				dev_err(dev, "ICC bandwidth request failed (%d)\n",
					ret);
				return ret;
			}
		}

		d->cur_ib = new_ib;
		d->cur_ab = new_ab;

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

	opp = devfreq_recommended_opp(dev, freq, flags);
	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);

	return set_bw(dev, *freq, d->gov_ab);
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
	ret = 0;
	if (num_icc_paths > 0) {
		if (num_icc_paths > MAX_PATHS) {
			dev_err(dev, "Unexpected number of ICC paths\n");
			return -EINVAL;
		}

		if (of_find_property(dev->of_node, "interconnect-names", NULL)) {
			for (i = 0; i < num_icc_paths; i++) {
				ret = of_property_read_string_index(dev->of_node,
								    "interconnect-names",
								    i, &icc_name);
				if (ret)
					break;

				d->icc_paths[i] = devm_of_icc_get(dev, icc_name);
				if (IS_ERR(d->icc_paths[i]))
					break;
			}

			if (i < num_icc_paths)
				ret = IS_ERR(d->icc_paths[i]) ?
					PTR_ERR(d->icc_paths[i]) : ret;
		} else if (num_icc_paths == 1) {
			d->icc_paths[0] = devm_of_icc_get(dev, NULL);
			if (IS_ERR(d->icc_paths[0]))
				ret = PTR_ERR(d->icc_paths[0]);
		} else {
			dev_err(dev,
				"Multiple ICC paths require interconnect-names\n");
			return -EINVAL;
		}

		if (!ret) {
			d->use_icc = true;
			d->num_paths = num_icc_paths;
			d->num_icc_paths = num_icc_paths;
		}
	} else if (num_icc_paths < 0 &&
		   of_find_property(dev->of_node, "interconnects", NULL)) {
		return num_icc_paths;
	}

	if (ret) {
		if (ret == -EPROBE_DEFER || !have_ports)
			return ret;

		dev_warn(dev, "ICC unavailable (%d), falling back to msm-bus\n", ret);
	}

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

	p = &d->dp;
	p->polling_ms = 50;
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
		.suppress_bind_attrs = true,
	},
};

module_platform_driver(devbw_driver);
MODULE_DESCRIPTION("Device DDR bandwidth voting driver MSM SoCs");
MODULE_LICENSE("GPL v2");
