// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2018-2019, The Linux Foundation. All rights reserved.
 */

#include <linux/interconnect.h>
#include <linux/msm-bus.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct proxy_client {
	struct msm_bus_scale_pdata *pdata;
	unsigned int client_handle;
	struct icc_path *icc_paths[2];
	int num_icc_paths;
	bool use_icc;
};

static struct proxy_client proxy_client_info;

static int msm_bus_device_proxy_client_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const char *icc_name;
	int ret;
	int num_icc_paths;
	int i;

	num_icc_paths = of_count_phandle_with_args(np, "interconnects",
						   "#interconnect-cells");
	if (num_icc_paths > ARRAY_SIZE(proxy_client_info.icc_paths)) {
		dev_warn(dev,
			 "Unexpected number of ICC paths (%d), using msm_bus fallback\n",
			 num_icc_paths);
		num_icc_paths = 0;
	}

	if (num_icc_paths > 0) {
		for (i = 0; i < num_icc_paths; i++) {
			ret = of_property_read_string_index(np,
							    "interconnect-names",
							    i, &icc_name);
			if (ret) {
				dev_warn(dev,
					 "Missing interconnect-names[%d], using msm_bus fallback\n",
					 i);
				break;
			}

			proxy_client_info.icc_paths[i] = devm_of_icc_get(dev,
								 icc_name);
			if (IS_ERR(proxy_client_info.icc_paths[i])) {
				ret = PTR_ERR(proxy_client_info.icc_paths[i]);
				proxy_client_info.icc_paths[i] = NULL;
				dev_warn(dev,
					 "Failed to get ICC path '%s' (%d), using msm_bus fallback\n",
					 icc_name, ret);
				break;
			}
		}

		if (i == num_icc_paths) {
			proxy_client_info.num_icc_paths = num_icc_paths;
			proxy_client_info.use_icc = true;

			for (i = 0; i < num_icc_paths; i++) {
				ret = icc_set_bw(proxy_client_info.icc_paths[i],
						 0, 1500000);
				if (ret) {
					dev_warn(dev,
						 "Failed to vote ICC path %d (%d), using msm_bus fallback\n",
						 i, ret);
					while (--i >= 0)
						icc_set_bw(proxy_client_info.icc_paths[i],
							   0, 0);
					proxy_client_info.use_icc = false;
					proxy_client_info.num_icc_paths = 0;
					break;
				}
			}
		}
	}

	proxy_client_info.pdata = msm_bus_cl_get_pdata(pdev);

	if (!proxy_client_info.pdata)
		return proxy_client_info.use_icc ? 0 : -ENODATA;

	if (proxy_client_info.use_icc)
		return 0;

	proxy_client_info.client_handle =
		msm_bus_scale_register_client(proxy_client_info.pdata);

	if (!proxy_client_info.client_handle) {
		dev_err(&pdev->dev, "Unable to register bus client\n");
		return -ENODEV;
	}

	ret = msm_bus_scale_client_update_request(
					proxy_client_info.client_handle, 1);
	if (ret)
		dev_err(&pdev->dev, "Bandwidth update failed (%d)\n", ret);

	return ret;
}

static const struct of_device_id proxy_client_match[] = {
	{.compatible = "qcom,bus-proxy-client"},
	{}
};

static struct platform_driver msm_bus_proxy_client_driver = {
	.probe = msm_bus_device_proxy_client_probe,
	.driver = {
		.name = "msm_bus_proxy_client_device",
		.of_match_table = proxy_client_match,
	},
};

static int __init msm_bus_proxy_client_init_driver(void)
{
	int rc;

	rc =  platform_driver_register(&msm_bus_proxy_client_driver);
	if (rc) {
		pr_err("Failed to register proxy client device driver\n");
		return rc;
	}

	return rc;
}

static int __init msm_bus_proxy_client_unvote(void)
{
	int ret;
	int i;

	if (proxy_client_info.use_icc) {
		for (i = 0; i < proxy_client_info.num_icc_paths; i++) {
			ret = icc_set_bw(proxy_client_info.icc_paths[i], 0, 0);
			if (ret)
				pr_err("%s: ICC unvote failed for path %d (%d)\n",
				       __func__, i, ret);
		}
	}

	if (!proxy_client_info.pdata || !proxy_client_info.client_handle)
		return 0;

	ret = msm_bus_scale_client_update_request(
					proxy_client_info.client_handle, 0);
	if (ret)
		pr_err("%s: bandwidth update request failed (%d)\n",
			__func__, ret);

	msm_bus_scale_unregister_client(proxy_client_info.client_handle);

	return 0;
}

subsys_initcall_sync(msm_bus_proxy_client_init_driver);
late_initcall_sync(msm_bus_proxy_client_unvote);
