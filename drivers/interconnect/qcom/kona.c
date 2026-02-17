// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm Kona virtual interconnect provider for OnePlus 8T (SM8250).
 *
 * This version adds simple bandwidth floors for CPU and GPU paths to
 * prevent ICC from voting extremely low bandwidth values that can
 * stall QoS and tank CPU/GPU performance under load.
 */

#include <linux/interconnect-provider.h>
#include <linux/interconnect.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of_device.h>
#include <linux/kdev_t.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

#include <dt-bindings/interconnect/qcom,kona.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/rpmh.h>

/*
 * Description of each logical Kona ICC path.
 *
 * id  - logical ICC path ID (from dt-bindings/interconnect/qcom,kona.h)
 */
enum kona_icc_role {
        KONA_ROLE_CPU,
        KONA_ROLE_GPU,
        KONA_ROLE_NPU,
        KONA_ROLE_DISPLAY,
        KONA_ROLE_GENERIC,
};

struct kona_icc_node_desc {
        u32 id;
        const char *name;
        const char *ab;
        const char *ib;
        enum kona_icc_role role;
};

struct kona_icc_provider {
        struct icc_provider provider;
        const struct kona_icc_node_desc *nodes;
        size_t num_nodes;
        bool boot_floor_vote;
        u64 *last_ab;
        u64 *last_ib;
        struct device **sysfs_nodes;
        struct class *icc_class;
#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
        bool gpu_llcc_turbo;
#endif
};

struct kona_icc_data {
        const struct kona_icc_node_desc *nodes;
        size_t num_nodes;
        bool boot_floor_vote;
};

struct kona_icc_node_sysfs {
        struct kona_icc_provider *qp;
        unsigned int index;
};

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
/*
 * Aggressive but safe bandwidth floors (in KB/s) tuned for OnePlus 8T (kona v2)
 * to guarantee fast ramp on short bursts (Geekbench) while keeping long-running
 * workloads (Antutu/gaming) fed. These are intentionally biased a bit high to
 * avoid under-voting critical CPU/GPU/NPU traffic.
 */
#define KONA_CPU_DDR_AB_FLOOR_KB	(16000000ULL) /* ~16 GB/s */
#define KONA_CPU_DDR_IB_FLOOR_KB	(26000000ULL) /* ~26 GB/s */
#define KONA_CPU_LLCC_AB_FLOOR_KB	(10000000ULL) /* ~10 GB/s */
#define KONA_CPU_LLCC_IB_FLOOR_KB	(17000000ULL) /* ~17 GB/s */
#define KONA_GPU_DDR_AB_FLOOR_KB	(12000000ULL) /* ~12 GB/s */
#define KONA_GPU_DDR_IB_FLOOR_KB	(22000000ULL) /* ~22 GB/s */
#define KONA_GPU_LLCC_AB_FLOOR_KB	(8000000ULL)  /* ~8 GB/s */
#define KONA_GPU_LLCC_IB_FLOOR_KB	(15000000ULL) /* ~15 GB/s */
#define KONA_NPU_DDR_AB_FLOOR_KB	(10000000ULL) /* ~10 GB/s */
#define KONA_NPU_DDR_IB_FLOOR_KB	(18000000ULL) /* ~18 GB/s */
#define KONA_NPU_LLCC_AB_FLOOR_KB	(7000000ULL)  /* ~7 GB/s */
#define KONA_NPU_LLCC_IB_FLOOR_KB	(13000000ULL) /* ~13 GB/s */

/*
 * Global minimum floors for any non-zero bandwidth vote. This protects
 * against bw_hwmon / memlat (or other clients) voting extremely small
 * values that cause QoS collapse and starve CPU/GPU.
 */
#define KONA_ICC_MIN_AB_FLOOR_KB	(1000000ULL)  /* 1 GB/s */
#define KONA_ICC_MIN_IB_FLOOR_KB	(2000000ULL)  /* 2 GB/s */

/*
 * Downscale hysteresis: ignore tiny AB/IB drops that only create RPMh churn.
 * Larger drops and all increases are honored immediately. Values are KB/s.
 */
#define KONA_HYST_PERCENT			5      /* tolerate small 5% dips */
#define KONA_HYST_AB_STEP_KB		 100000 /* or 100 MB/s, whichever is smaller */
#define KONA_HYST_IB_STEP_KB		 150000 /* or 150 MB/s, whichever is smaller */

/*
 * Performance bias lets us intentionally over-vote for critical paths so CPU
 * and DDR/LLCC interconnects stay out of the lowest performance corners when
 * the system is busy. Expressed in percent (e.g. 125 = +25% headroom).
 * A lighter headroom value and thresholds make the behavior adaptive:
 *
 * - kona_perf_bias_light: light-load headroom that preserves battery.
 * - kona_perf_bias:       default bias for normal requests.
 * - kona_perf_bias_turbo: extra bias for very large votes (race-to-performance).
 * - kona_perf_light_kb / kona_perf_turbo_kb: thresholds for selecting a profile.
 */
static unsigned int kona_perf_bias = 130;
static unsigned int kona_perf_bias_light = 112;
static unsigned int kona_perf_bias_turbo = 150;
static unsigned long kona_perf_light_kb = 1500000;   /* 1.5 GB/s */
static unsigned long kona_perf_turbo_kb = 12000000;  /* 12 GB/s */
module_param(kona_perf_bias, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias,
        "Percent headroom added on CPU/DDR/LLCC/GPU/NPU paths (default: 130)");
module_param(kona_perf_bias_light, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias_light,
        "Percent headroom added for light requests to save power (default: 112)");
module_param(kona_perf_bias_turbo, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias_turbo,
        "Percent headroom added for very large votes (default: 150)");
module_param(kona_perf_light_kb, ulong, 0644);
MODULE_PARM_DESC(kona_perf_light_kb,
        "Threshold KB/s for light-load bias selection (default: 1500000)");
module_param(kona_perf_turbo_kb, ulong, 0644);
MODULE_PARM_DESC(kona_perf_turbo_kb,
        "Threshold KB/s for turbo bias selection (default: 12000000)");

/*
 * GPU keep-alive floor and IB prioritization for gpu-ddr path.
 *
 * keepalive_*: keep a minimum vote between short frame gaps so BCMs stay in
 * a responsive corner instead of repeatedly collapsing to idle.
 * ib_*:        bias and floor IB over AB so command bursts hit DDR quickly.
 */
static bool kona_gpu_keepalive_enable = true;
static unsigned long kona_gpu_keepalive_ab_kb = 800000;   /* 800 MB/s */
static unsigned long kona_gpu_keepalive_ib_kb = 1800000;  /* 1.8 GB/s */
static bool kona_cpu_keepalive_enable = true;
static unsigned long kona_cpu_keepalive_ab_kb = 1400000;  /* 1.4 GB/s */
static unsigned long kona_cpu_keepalive_ib_kb = 2600000;  /* 2.6 GB/s */
static bool kona_npu_keepalive_enable = true;
static unsigned long kona_npu_keepalive_ab_kb = 1000000;  /* 1.0 GB/s */
static unsigned long kona_npu_keepalive_ib_kb = 2000000;  /* 2.0 GB/s */
static unsigned int kona_gpu_ib_boost_percent = 145;
static unsigned int kona_gpu_ib_min_ratio_percent = 180;
static bool kona_gpu_llcc_turbo_enable = true;
static unsigned long kona_gpu_llcc_turbo_enter_ib_kb = 12000000; /* 12 GB/s */
static unsigned long kona_gpu_llcc_turbo_exit_ib_kb = 9000000;   /* 9 GB/s */
static unsigned long kona_gpu_llcc_turbo_ab_kb = KONA_GPU_LLCC_IB_FLOOR_KB;
static unsigned long kona_gpu_llcc_turbo_ib_kb = KONA_GPU_LLCC_IB_FLOOR_KB;
module_param(kona_gpu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_enable,
        "Keep non-zero floor for gpu-ddr AB/IB between short idle gaps");
module_param(kona_gpu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_ab_kb,
        "gpu-ddr keepalive AB floor in KB/s (default: 800000)");
module_param(kona_gpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_ib_kb,
        "gpu-ddr keepalive IB floor in KB/s (default: 1800000)");
module_param(kona_cpu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_enable,
        "Keep non-zero floor for cpu-ddr/cpu-llcc AB/IB between short idle gaps");
module_param(kona_cpu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ab_kb,
        "cpu keepalive AB floor in KB/s (default: 1400000)");
module_param(kona_cpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ib_kb,
        "cpu keepalive IB floor in KB/s (default: 2600000)");
module_param(kona_npu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_enable,
        "Keep non-zero floor for npu-ddr/npu-llcc AB/IB between short idle gaps");
module_param(kona_npu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ab_kb,
        "npu keepalive AB floor in KB/s (default: 1000000)");
module_param(kona_npu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ib_kb,
        "npu keepalive IB floor in KB/s (default: 2000000)");
module_param(kona_gpu_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_ib_boost_percent,
        "Percent boost applied to gpu-ddr IB after floors (default: 145)");
module_param(kona_gpu_ib_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_ib_min_ratio_percent,
        "Minimum gpu-ddr IB as percent of AB (default: 180)");
module_param(kona_gpu_llcc_turbo_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_enable,
        "Force high gpu-llcc vote while gpu-ddr IB remains above threshold");
module_param(kona_gpu_llcc_turbo_enter_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_enter_ib_kb,
        "gpu-ddr IB KB/s threshold to enter LLCC turbo pinning");
module_param(kona_gpu_llcc_turbo_exit_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_exit_ib_kb,
        "gpu-ddr IB KB/s threshold to exit LLCC turbo pinning");
module_param(kona_gpu_llcc_turbo_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_ab_kb,
        "Forced gpu-llcc AB vote in KB/s while turbo pinning is active");
module_param(kona_gpu_llcc_turbo_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_ib_kb,
        "Forced gpu-llcc IB vote in KB/s while turbo pinning is active");

static void kona_icc_update_gpu_llcc_turbo(struct kona_icc_provider *qp, u64 ib)
{
        if (!kona_gpu_llcc_turbo_enable)
                return;

        if (!qp->gpu_llcc_turbo && ib >= kona_gpu_llcc_turbo_enter_ib_kb)
                qp->gpu_llcc_turbo = true;
        else if (qp->gpu_llcc_turbo && ib <= kona_gpu_llcc_turbo_exit_ib_kb)
                qp->gpu_llcc_turbo = false;
}

static void kona_icc_apply_gpu_llcc_turbo(struct kona_icc_provider *qp,
                                          const struct kona_icc_node_desc *desc,
                                          u64 *ab, u64 *ib)
{
        if (!kona_gpu_llcc_turbo_enable)
                return;

        if (desc->id != KONA_ICC_GPU_TO_LLCC)
                return;

        if (!qp->gpu_llcc_turbo)
                return;

        if (*ab < kona_gpu_llcc_turbo_ab_kb)
                *ab = kona_gpu_llcc_turbo_ab_kb;
        if (*ib < kona_gpu_llcc_turbo_ib_kb)
                *ib = kona_gpu_llcc_turbo_ib_kb;
}

static u64 kona_icc_add_headroom(u64 value, unsigned int bias)
{
        /* Avoid overflow when adding headroom; values are already in KBps. */
        return mul_u64_u32_div(value, bias, 100);
}

static bool kona_icc_apply_keepalive_vote(struct kona_icc_provider *qp,
					 unsigned int index, u64 *ab, u64 *ib)
{
	bool keepalive = false;
	const struct kona_icc_node_desc *desc;
	u64 keepalive_ab = 0, keepalive_ib = 0;

	if (!qp->last_ab || !qp->last_ib || *ab || *ib)
		return false;

	if (!qp->last_ab[index] && !qp->last_ib[index])
		return false;

	desc = &qp->nodes[index];

	switch (desc->role) {
	case KONA_ROLE_CPU:
		if (!kona_cpu_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_cpu_keepalive_ab_kb;
		keepalive_ib = kona_cpu_keepalive_ib_kb;
		break;
	case KONA_ROLE_GPU:
		if (!kona_gpu_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_gpu_keepalive_ab_kb;
		keepalive_ib = kona_gpu_keepalive_ib_kb;
		break;
	case KONA_ROLE_NPU:
		if (!kona_npu_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_npu_keepalive_ab_kb;
		keepalive_ib = kona_npu_keepalive_ib_kb;
		break;
	case KONA_ROLE_DISPLAY:
	case KONA_ROLE_GENERIC:
	default:
		break;
	}

	if (!keepalive)
		return false;

	*ab = max_t(u64, keepalive_ab, qp->last_ab[index]);
	*ib = max_t(u64, keepalive_ib, qp->last_ib[index]);

	return true;
}

static unsigned int kona_icc_pick_bias(const struct kona_icc_node_desc *desc,
                                      u64 ab, u64 ib)
{
        unsigned long vote = max(ab, ib);

        switch (desc->role) {
        case KONA_ROLE_CPU:
        case KONA_ROLE_GPU:
        case KONA_ROLE_NPU:
        case KONA_ROLE_GENERIC:
                if (vote >= kona_perf_turbo_kb)
                        return kona_perf_bias_turbo;
                if (vote <= kona_perf_light_kb)
                        return kona_perf_bias_light;
                return kona_perf_bias;
        case KONA_ROLE_DISPLAY:
        default:
                return 100;
        }
}

static void kona_icc_apply_hysteresis(struct kona_icc_provider *qp,
			     const struct kona_icc_node_desc *desc,
			     unsigned int index, u64 *ab, u64 *ib)
{
	u64 prev_ab, prev_ib, ab_drop, ib_drop, ab_win, ib_win;

	if (!qp->last_ab || !qp->last_ib)
		return;

	prev_ab = qp->last_ab[index];
	prev_ib = qp->last_ib[index];

	/* Only suppress very small downvotes; keep upscales and big drops. */
	if (prev_ab && *ab && *ab < prev_ab) {
		ab_drop = prev_ab - *ab;
		ab_win = min_t(u64, prev_ab * KONA_HYST_PERCENT / 100,
			       KONA_HYST_AB_STEP_KB);
		if (ab_drop < ab_win)
			*ab = prev_ab;
	}

	if (prev_ib && *ib && *ib < prev_ib) {
		ib_drop = prev_ib - *ib;
		ib_win = min_t(u64, prev_ib * KONA_HYST_PERCENT / 100,
			       KONA_HYST_IB_STEP_KB);
		if (ib_drop < ib_win)
			*ib = prev_ib;
	}

	pr_debug("kona-icc: hysteresis %s prev ab/ib=%llu/%llu new=%llu/%llu\n",
		 desc->name, prev_ab, prev_ib, *ab, *ib);
}

static void __maybe_unused
kona_icc_apply_floor(const struct kona_icc_node_desc *desc,
		     u64 *ab, u64 *ib)
{
	switch (desc->id) {
	case KONA_ICC_CPU_TO_MEM:
		if (*ab && *ab < KONA_CPU_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU_DDR_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_CPU_TO_LLCC:
		if (*ab && *ab < KONA_CPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU_LLCC_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU_LLCC_IB_FLOOR_KB;
		break;
	case KONA_ICC_NPU_TO_MEM:
	case KONA_ICC_NPUDSP_TO_MEM:
		if (*ab && *ab < KONA_NPU_DDR_AB_FLOOR_KB)
			*ab = KONA_NPU_DDR_AB_FLOOR_KB;
		if (*ib && *ib < KONA_NPU_DDR_IB_FLOOR_KB)
			*ib = KONA_NPU_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_NPU_TO_LLCC:
		if (*ab && *ab < KONA_NPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_NPU_LLCC_AB_FLOOR_KB;
		if (*ib && *ib < KONA_NPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_NPU_LLCC_IB_FLOOR_KB;
		break;
	case KONA_ICC_GPU_TO_MEM:
		if (*ab && *ab < KONA_GPU_DDR_AB_FLOOR_KB)
			*ab = KONA_GPU_DDR_AB_FLOOR_KB;
		if (*ib && *ib < KONA_GPU_DDR_IB_FLOOR_KB)
			*ib = KONA_GPU_DDR_IB_FLOOR_KB;

		/* Prioritize burst bandwidth for GPU->DDR traffic. */
		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_gpu_ib_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_gpu_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_ib_min_ratio_percent, 100);
		break;
	case KONA_ICC_GPU_TO_LLCC:
		if (*ab && *ab < KONA_GPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_GPU_LLCC_AB_FLOOR_KB;
		if (*ib && *ib < KONA_GPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_GPU_LLCC_IB_FLOOR_KB;
		break;
	default:
		break;
	}

	/* Final safety net: clamp any very small non-zero votes. */
        if (*ab && *ab < KONA_ICC_MIN_AB_FLOOR_KB)
                *ab = KONA_ICC_MIN_AB_FLOOR_KB;

        if (*ib && *ib < KONA_ICC_MIN_IB_FLOOR_KB)
                *ib = KONA_ICC_MIN_IB_FLOOR_KB;

        /*
         * Add configurable headroom to CPU/GPU/NPU/DDR/LLCC paths to avoid collapsing
         * performance when the SoC is under heavy load. Display paths stay closer
         * to the requested vote to keep power sane.
         */
        if (desc->role != KONA_ROLE_DISPLAY) {
                unsigned int bias = kona_icc_pick_bias(desc, *ab, *ib);

                if (*ab)
                        *ab = kona_icc_add_headroom(*ab, bias);
                if (*ib)
                        *ib = kona_icc_add_headroom(*ib, bias);
        }

        pr_debug("kona-icc: vote for %s after floor/bias (ab=%llu KBps, ib=%llu KBps)\n",
                 desc->name, *ab, *ib);
}
#endif /* CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR */

/*
 * Node table.
 */
static const struct kona_icc_node_desc kona_nodes[] = {
	{
		.id = KONA_ICC_GPU_TO_LLCC,
		.name = "gpu-llcc",
		.ab = "GPU_LLCC_AB",
		.ib = "GPU_LLCC_IB",
		.role = KONA_ROLE_GPU,
	},
	{
		.id = KONA_ICC_GPU_TO_MEM,
		.name = "gpu-ddr",
		.ab = "GPU_MEM_AB",
		.ib = "GPU_MEM_IB",
		.role = KONA_ROLE_GPU,
	},
	{
		.id = KONA_ICC_NPU_TO_LLCC,
		.name = "npu-llcc",
		.ab = "NPU_LLCC_AB",
		.ib = "NPU_LLCC_IB",
		.role = KONA_ROLE_NPU,
	},
	{
		.id = KONA_ICC_NPU_TO_MEM,
		.name = "npu-ddr",
		.ab = "NPU_MEM_AB",
		.ib = "NPU_MEM_IB",
		.role = KONA_ROLE_NPU,
	},
	{
		.id = KONA_ICC_CPU_TO_LLCC,
		.name = "cpu-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU_TO_MEM,
		.name = "cpu-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_DISP0_TO_MEM,
		.name = "disp0-ddr",
		.ab = "DISP0_MEM_AB",
		.ib = "DISP0_MEM_IB",
		.role = KONA_ROLE_DISPLAY,
	},
	{
		.id = KONA_ICC_DISP1_TO_MEM,
		.name = "disp1-ddr",
		.ab = "DISP1_MEM_AB",
		.ib = "DISP1_MEM_IB",
		.role = KONA_ROLE_DISPLAY,
	},
	{
		.id = KONA_ICC_NPUDSP_TO_MEM,
		.name = "npudsp-ddr",
		.ab = "NPU_MEM_AB",
		.ib = "NPU_MEM_IB",
		.role = KONA_ROLE_NPU,
	},
};

static const struct kona_icc_data kona_data = {
	.nodes = kona_nodes,
	.num_nodes = ARRAY_SIZE(kona_nodes),
	.boot_floor_vote = true,
};

static const struct kona_icc_node_desc *
kona_find_desc(struct kona_icc_provider *qp, u32 id, unsigned int *index)
{
	unsigned int i;

	for (i = 0; i < qp->num_nodes; i++) {
		if (qp->nodes[i].id == id) {
			if (index)
				*index = i;
			return &qp->nodes[i];
		}
	}

	return NULL;
}

static struct icc_path *kona_icc_xlate(struct icc_provider *provider,
				       const struct of_phandle_args *spec)
{
	struct kona_icc_provider *qp = dev_get_drvdata(provider->dev);
	const struct kona_icc_node_desc *desc;
	struct icc_path *path;
	unsigned int index = 0;

	if (!qp)
		return ERR_PTR(-EINVAL);

	desc = kona_find_desc(qp, spec->args[0], &index);
	if (!desc)
		return ERR_PTR(-EINVAL);

	path = icc_of_xlate_onecell(provider, spec);
	if (IS_ERR(path))
		return path;

	path->data = (void *)(uintptr_t)index;

	return path;
}

static int kona_icc_send_bw(struct device *dev, const char *res, u32 kbps)
{
	struct tcs_cmd cmd = {};
	u32 addr;
	int ret;

	if (!res)
		return 0;

	/*
	 * Boot-safe handling: cmd-db and RPMh address translation may not be
	 * ready during early boot. Never propagate -EPROBE_DEFER to ICC consumers.
	 * Use -EAGAIN internally to signal "try again later".
	 */
	if (!cmd_db_ready())
		return -EAGAIN;

	addr = cmd_db_read_addr(res);
	if (!addr)
		return -EAGAIN;

	/* cmd.data is KB/s; callers must already scale to KB/s. */
	cmd.addr = addr;
	cmd.data = kbps;
	cmd.wait = false;

	ret = rpmh_write(dev, RPMH_ACTIVE_ONLY_STATE, &cmd, 1);
	if (ret == -EPROBE_DEFER)
		return -EAGAIN;

	return ret;
}

static int kona_icc_set(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	struct kona_icc_provider *qp;
	u64 ab, ib;
	unsigned int index;
	int ret;

	if (IS_ERR_OR_NULL(path) || !path->provider)
		return -EINVAL;

	qp = dev_get_drvdata(path->provider->dev);
	if (!qp)
		return -EINVAL;

	index = (unsigned int)(uintptr_t)path->data;
	if (index >= qp->num_nodes)
		return -EINVAL;

	/*
	 * Kona v2 RPMh BCMs expect interconnect votes in KB/s (decimal 1000)
	 * packed into cmd.data. Keep ICC consumer units aligned to KB/s to
	 * avoid u32 saturation and unit skew across clients.
	 */
	ab = avg_bw;
	ib = peak_bw;

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
	/*
	 * Apply per-path and global floors for non-zero votes. 0/0 votes
	 * are treated as truly idle and are allowed to collapse.
	 */
	if (ab || ib) {
		const struct kona_icc_node_desc *desc = &qp->nodes[index];

		kona_icc_apply_floor(desc, &ab, &ib);
		kona_icc_apply_hysteresis(qp, desc, index, &ab, &ib);
	}

	/*
	 * Keep-alive vote for CPU/GPU/NPU paths when clients briefly request 0/0
	 * between bursts; this avoids repeated collapses into deep bus idle states.
	 */
	kona_icc_apply_keepalive_vote(qp, index, &ab, &ib);

	if (qp->nodes[index].id == KONA_ICC_GPU_TO_MEM) {
		kona_icc_update_gpu_llcc_turbo(qp, ib);
	} else if (qp->last_ib) {
		unsigned int gpu_mem_index = 0;

		if (kona_find_desc(qp, KONA_ICC_GPU_TO_MEM, &gpu_mem_index))
			kona_icc_update_gpu_llcc_turbo(qp, qp->last_ib[gpu_mem_index]);
	}

	kona_icc_apply_gpu_llcc_turbo(qp, &qp->nodes[index], &ab, &ib);
#endif

#ifdef DEBUG
	if (qp->nodes[index].role == KONA_ROLE_CPU)
		pr_info("kona-icc: %s avg=%uKB/s peak=%uKB/s -> ab=%lluKB/s ib=%lluKB/s prev ab/ib=%llu/%llu\n",
			qp->nodes[index].name, avg_bw, peak_bw,
			ab, ib,
			qp->last_ab ? qp->last_ab[index] : 0,
			qp->last_ib ? qp->last_ib[index] : 0);
#endif

	if (qp->last_ab && qp->last_ib &&
	    qp->last_ab[index] == ab && qp->last_ib[index] == ib)
		return 0;

	/*
	 * Apply AB/IB votes. If cmd-db/RPMh translation isn't ready yet,
	 * kona_icc_send_bw() returns -EAGAIN. Treat that as a soft success
	 * but DO NOT cache the vote so a later caller can retry and apply it.
	 */
	/*
	 * Reduce BCM dirty-bit churn: only send changed values. For gpu-ddr path,
	 * send IB first so latency-sensitive bursts are serviced earlier.
	 */
	if (qp->nodes[index].id == KONA_ICC_GPU_TO_MEM) {
		if (!qp->last_ib || qp->last_ib[index] != ib) {
			ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ib, ib);
			if (ret == -EAGAIN)
				return 0;
			if (ret)
				return ret;
		}

		if (!qp->last_ab || qp->last_ab[index] != ab) {
			ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ab, ab);
			if (ret == -EAGAIN)
				return 0;
			if (ret)
				return ret;
		}
	} else {
		if (!qp->last_ab || qp->last_ab[index] != ab) {
			ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ab, ab);
			if (ret == -EAGAIN)
				return 0;
			if (ret)
				return ret;
		}

		if (!qp->last_ib || qp->last_ib[index] != ib) {
			ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ib, ib);
			if (ret == -EAGAIN)
				return 0;
			if (ret)
				return ret;
		}
	}

	if (qp->last_ab)
		qp->last_ab[index] = ab;
	if (qp->last_ib)
		qp->last_ib[index] = ib;

	return 0;
}

static ssize_t ab_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct kona_icc_node_sysfs *node = dev_get_drvdata(dev);

	if (!node || !node->qp || !node->qp->last_ab)
		return -EINVAL;

	return sysfs_emit(buf, "%llu\n", node->qp->last_ab[node->index]);
}

static ssize_t ib_show(struct device *dev,
		       struct device_attribute *attr, char *buf)
{
	struct kona_icc_node_sysfs *node = dev_get_drvdata(dev);

	if (!node || !node->qp || !node->qp->last_ib)
		return -EINVAL;

	return sysfs_emit(buf, "%llu\n", node->qp->last_ib[node->index]);
}

static ssize_t res_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct kona_icc_node_sysfs *node = dev_get_drvdata(dev);
	const struct kona_icc_node_desc *desc;

	if (!node || !node->qp)
		return -EINVAL;

	desc = &node->qp->nodes[node->index];

	return sysfs_emit(buf, "ab=%s ib=%s\n", desc->ab ?: "", desc->ib ?: "");
}

static DEVICE_ATTR_RO(ab);
static DEVICE_ATTR_RO(ib);
static DEVICE_ATTR_RO(res);

static struct attribute *kona_icc_attrs[] = {
	&dev_attr_ab.attr,
	&dev_attr_ib.attr,
	&dev_attr_res.attr,
	NULL,
};

static const struct attribute_group kona_icc_group = {
	.attrs = kona_icc_attrs,
};

static const struct attribute_group *kona_icc_groups[] = {
	&kona_icc_group,
	NULL,
};

static void kona_icc_unregister_sysfs(struct kona_icc_provider *qp)
{
	size_t i;

	if (!qp->icc_class || !qp->sysfs_nodes)
		return;

	for (i = 0; i < qp->num_nodes; i++) {
		if (qp->sysfs_nodes[i])
			device_unregister(qp->sysfs_nodes[i]);
	}
}

static int kona_icc_register_sysfs(struct platform_device *pdev,
				   struct kona_icc_provider *qp)
{
	struct kona_icc_node_sysfs *node_data;
	const struct kona_icc_node_desc *desc;
	struct device *icc_dev;
	size_t i;
	int ret = 0;

	qp->icc_class = icc_class_get();
	if (!qp->icc_class)
		return 0;

	qp->sysfs_nodes = devm_kcalloc(&pdev->dev, qp->num_nodes,
				       sizeof(*qp->sysfs_nodes), GFP_KERNEL);
	if (!qp->sysfs_nodes)
		return -ENOMEM;

	for (i = 0; i < qp->num_nodes; i++) {
		desc = &qp->nodes[i];

		node_data = kzalloc(sizeof(*node_data), GFP_KERNEL);
		if (!node_data) {
			ret = -ENOMEM;
			goto err_unregister;
		}

		node_data->qp = qp;
		node_data->index = i;

		icc_dev = device_create_with_groups(qp->icc_class,
						    qp->provider.dev,
						    MKDEV(0, 0),
						    node_data, kona_icc_groups,
						    "kona-%s", desc->name);
		if (IS_ERR(icc_dev)) {
			ret = PTR_ERR(icc_dev);
			kfree(node_data);
			goto err_unregister;
		}

		dev_set_drvdata(icc_dev, node_data);
		qp->sysfs_nodes[i] = icc_dev;
	}

	return 0;

err_unregister:
	while (i--) {
		if (qp->sysfs_nodes[i])
			device_unregister(qp->sysfs_nodes[i]);
	}

	return ret;
}

static void kona_icc_release(struct icc_path *path)
{
	kfree(path);
}

static void kona_icc_invalidate_cache(struct kona_icc_provider *qp)
{
	int i;

	if (!qp)
		return;

	for (i = 0; i < qp->num_nodes; i++) {
		if (qp->last_ab)
			qp->last_ab[i] = U64_MAX;
		if (qp->last_ib)
			qp->last_ib[i] = U64_MAX;
	}

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
	qp->gpu_llcc_turbo = false;
#endif
}

static int kona_icc_resume(struct device *dev)
{
	struct kona_icc_provider *qp = dev_get_drvdata(dev);

	/*
	 * RPMh ACTIVE_ONLY votes are not retained across suspend. Invalidate the
	 * software cache so the first post-resume icc_set_bw() always re-sends.
	 */
	kona_icc_invalidate_cache(qp);

	return 0;
}

static const struct dev_pm_ops kona_icc_pm_ops = {
	.resume = kona_icc_resume,
	.restore = kona_icc_resume,
	.thaw = kona_icc_resume,
};

static int kona_icc_probe(struct platform_device *pdev)
{
	const struct kona_icc_data *data =
		of_device_get_match_data(&pdev->dev);
	struct kona_icc_provider *qp;
	u64 ab, ib;
	int ret, i;

	qp = devm_kzalloc(&pdev->dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;

	if (data) {
		qp->nodes = data->nodes;
		qp->num_nodes = data->num_nodes;
		qp->boot_floor_vote = data->boot_floor_vote;
	} else {
		qp->nodes = kona_nodes;
		qp->num_nodes = ARRAY_SIZE(kona_nodes);
		qp->boot_floor_vote = true;
	}

	qp->last_ab = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
					   GFP_KERNEL);
	if (!qp->last_ab)
		return -ENOMEM;

	qp->last_ib = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
					   GFP_KERNEL);
	if (!qp->last_ib)
		return -ENOMEM;

        qp->provider.dev = &pdev->dev;
        qp->provider.of_node = pdev->dev.of_node;
        qp->provider.xlate = kona_icc_xlate;
        qp->provider.set = kona_icc_set;
        qp->provider.release = kona_icc_release;

	platform_set_drvdata(pdev, qp);

	ret = icc_provider_register(&qp->provider);
        if (ret)
                return ret;

	ret = kona_icc_register_sysfs(pdev, qp);
	if (ret) {
                kona_icc_unregister_sysfs(qp);
                icc_provider_unregister(&qp->provider);
                return ret;
        }

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
	if (qp->boot_floor_vote) {
		for (i = 0; i < qp->num_nodes; i++) {
			int r_ab, r_ib;

			ab = KONA_ICC_MIN_AB_FLOOR_KB;
			ib = KONA_ICC_MIN_IB_FLOOR_KB;
			kona_icc_apply_floor(&qp->nodes[i], &ab, &ib);

			r_ab = kona_icc_send_bw(qp->provider.dev,
					     qp->nodes[i].ab, ab);
			r_ib = kona_icc_send_bw(qp->provider.dev,
					     qp->nodes[i].ib, ib);

			/*
			 * Only cache if both votes were applied. If we got -EAGAIN,
			 * leave cache untouched so later consumers can retry once
			 * cmd-db/RPMh is ready.
			 */
			if (!r_ab && !r_ib) {
				qp->last_ab[i] = ab;
				qp->last_ib[i] = ib;
			}
		}
	}
#endif

        dev_info(&pdev->dev,
                 "Kona interconnect provider registered (%zu nodes)\n",
                 qp->num_nodes);

        return 0;
}

static int kona_icc_remove(struct platform_device *pdev)
{
	struct kona_icc_provider *qp = platform_get_drvdata(pdev);

        kona_icc_unregister_sysfs(qp);
        icc_provider_unregister(&qp->provider);

        return 0;
}

static const struct of_device_id kona_icc_of_match[] = {
	{ .compatible = "qcom,kona-interconnect", .data = &kona_data },
	{ .compatible = "qcom,kona-v2-interconnect", .data = &kona_data },
	{ }
};
MODULE_DEVICE_TABLE(of, kona_icc_of_match);

static struct platform_driver kona_icc_driver = {
	.probe = kona_icc_probe,
	.remove = kona_icc_remove,
	.driver = {
		.name = "kona-icc",
		.of_match_table = kona_icc_of_match,
		.pm = &kona_icc_pm_ops,
	},
};
module_platform_driver(kona_icc_driver);

MODULE_DESCRIPTION("Qualcomm Kona interconnect driver with BW floors");
MODULE_LICENSE("GPL v2");
