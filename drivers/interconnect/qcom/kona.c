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
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/suspend.h>
#include <linux/atomic.h>

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
        KONA_ROLE_CPU_PRIME,
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
	bool system_suspended;
	atomic_t votes_paused;
	u64 *last_ab;
	u64 *last_ib;
	u64 *req_ab;
	u64 *req_ib;
	u64 *saved_ab;
	u64 *saved_ib;
	unsigned long resume_jiffies;
	u8 resume_phase;
	struct delayed_work retry_work;
	atomic_t deferred_votes;
	atomic_t replay_runs;
	atomic_t display_replay_skips;
	unsigned long *last_active_jiffies;
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

#define KONA_RETRY_DELAY_MS	20

#define KONA_RESUME_PHASE0_DELAY_MS	0
#define KONA_RESUME_PHASE1_DELAY_MS	25
#define KONA_RESUME_PHASE2_DELAY_MS	120


static bool kona_resume_debug;
module_param_named(kona_resume_debug, kona_resume_debug, bool, 0644);
MODULE_PARM_DESC(kona_resume_debug, "Enable Kona ICC suspend/resume deferral debug");

static bool kona_perf_floor_enable = true;
module_param(kona_perf_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_perf_floor_enable,
	"Enable aggressive hard bandwidth floors (default: on)");

static bool kona_display_resume_floor_enable = true;
module_param_named(kona_display_resume_floor_enable, kona_display_resume_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_resume_floor_enable,
	"Enable a minimal DISPLAY resume bandwidth floor to avoid black-screen resumes");

static unsigned int kona_display_resume_floor_ab_kBps = 100000; /* 100 MB/s */
module_param_named(kona_display_resume_floor_ab_kBps, kona_display_resume_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_resume_floor_ab_kBps, "DISPLAY resume floor average BW (kB/s)");

static unsigned int kona_display_resume_floor_ib_kBps = 1000000; /* 1 GB/s */
module_param_named(kona_display_resume_floor_ib_kBps, kona_display_resume_floor_ib_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_resume_floor_ib_kBps, "DISPLAY resume floor peak BW (kB/s)");


static unsigned int kona_display_resume_hold_ms = 15000;
module_param_named(kona_display_resume_hold_ms, kona_display_resume_hold_ms, uint, 0644);
MODULE_PARM_DESC(kona_display_resume_hold_ms,
	"Hold a small DISPLAY non-zero vote for N ms after resume to avoid early collapse");

static bool kona_display_nonzero_floor_enable = true;
module_param_named(kona_display_nonzero_floor_enable, kona_display_nonzero_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_nonzero_floor_enable,
	"Force non-zero fallback floor for DISPLAY paths when clients request 0/0");

static unsigned int kona_display_nonzero_floor_ab_kBps = 80000; /* 80 MB/s */
module_param_named(kona_display_nonzero_floor_ab_kBps, kona_display_nonzero_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_nonzero_floor_ab_kBps,
	"Fallback DISPLAY floor average BW (kB/s) when a 0/0 vote is requested");

static unsigned int kona_display_nonzero_floor_ib_kBps = 160000; /* 160 MB/s */
module_param_named(kona_display_nonzero_floor_ib_kBps, kona_display_nonzero_floor_ib_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_nonzero_floor_ib_kBps,
	"Fallback DISPLAY floor peak BW (kB/s) when a 0/0 vote is requested");

static unsigned int kona_display_cfg_nonzero_floor_ab_kBps = 120000; /* 120 MB/s */
module_param_named(kona_display_cfg_nonzero_floor_ab_kBps, kona_display_cfg_nonzero_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_cfg_nonzero_floor_ab_kBps,
	"Fallback DISPLAY config-path floor average BW (kB/s) for 0/0 votes");

static unsigned int kona_display_cfg_nonzero_floor_ib_kBps = 240000; /* 240 MB/s */
module_param_named(kona_display_cfg_nonzero_floor_ib_kBps, kona_display_cfg_nonzero_floor_ib_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_cfg_nonzero_floor_ib_kBps,
	"Fallback DISPLAY config-path floor peak BW (kB/s) for 0/0 votes");

static bool kona_display_bootstrap_floor_enable = true;
module_param_named(kona_display_bootstrap_floor_enable, kona_display_bootstrap_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_bootstrap_floor_enable,
	"Allow one-shot DISPLAY floor during phase-0 replay when no saved/requested vote exists");

static bool kona_display_topology_strict;
module_param_named(kona_display_topology_strict, kona_display_topology_strict, bool, 0644);
MODULE_PARM_DESC(kona_display_topology_strict,
	"Fail probe if DISPLAY-critical ICC nodes are missing or not tagged DISPLAY");


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
#define KONA_CPU_PRIME_DDR_AB_FLOOR_KB	(18000000ULL) /* ~18 GB/s */
#define KONA_CPU_PRIME_DDR_IB_FLOOR_KB	(29000000ULL) /* ~29 GB/s */
#define KONA_CPU_PRIME_LLCC_AB_FLOOR_KB	(11000000ULL) /* ~11 GB/s */
#define KONA_CPU_PRIME_LLCC_IB_FLOOR_KB	(19000000ULL) /* ~19 GB/s */
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
#define KONA_PRIME_EXTRA_BIAS_PERCENT	10
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
static bool kona_gpu_keepalive_enable;
static unsigned long kona_gpu_keepalive_ab_kb = 200000;   /* 200 MB/s */
static unsigned long kona_gpu_keepalive_ib_kb = 500000;   /* 500 MB/s */
static bool kona_cpu_keepalive_enable;
static unsigned long kona_cpu_keepalive_ab_kb = 300000;   /* 300 MB/s */
static unsigned long kona_cpu_keepalive_ib_kb = 600000;   /* 600 MB/s */
static bool kona_npu_keepalive_enable;
static unsigned long kona_npu_keepalive_ab_kb = 250000;   /* 250 MB/s */
static unsigned long kona_npu_keepalive_ib_kb = 500000;   /* 500 MB/s */
static bool kona_disp_keepalive_enable = true;
static unsigned long kona_disp_keepalive_ab_kb = 200000;   /* 200 MB/s */
static unsigned long kona_disp_keepalive_ib_kb = 400000;   /* 400 MB/s */
static bool kona_keepalive_decay_enable = true;
static unsigned int kona_keepalive_decay_window_ms = 300;
static unsigned int kona_keepalive_decay_min_percent = 20;
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
        "gpu-ddr keepalive AB floor in KB/s (default: 200000)");
module_param(kona_gpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_ib_kb,
        "gpu-ddr keepalive IB floor in KB/s (default: 500000)");
module_param(kona_cpu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_enable,
        "Keep non-zero floor for cpu-ddr/cpu-llcc AB/IB between short idle gaps");
module_param(kona_cpu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ab_kb,
        "cpu keepalive AB floor in KB/s (default: 300000)");
module_param(kona_cpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ib_kb,
        "cpu keepalive IB floor in KB/s (default: 600000)");
module_param(kona_npu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_enable,
        "Keep non-zero floor for npu-ddr/npu-llcc AB/IB between short idle gaps");
module_param(kona_npu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ab_kb,
        "npu keepalive AB floor in KB/s (default: 250000)");
module_param(kona_npu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ib_kb,
        "npu keepalive IB floor in KB/s (default: 500000)");
module_param(kona_disp_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_enable,
	"Keep non-zero floor for disp0/disp1 DDR AB/IB between idle/off transitions");
module_param(kona_disp_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_ab_kb,
	"display keepalive AB floor in KB/s (default: 200000)");
module_param(kona_disp_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_ib_kb,
	"display keepalive IB floor in KB/s (default: 400000)");
module_param(kona_keepalive_decay_enable, bool, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_enable,
	"linearly decay keepalive votes after the last active request (default: on)");
module_param(kona_keepalive_decay_window_ms, uint, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_window_ms,
	"keepalive decay window in ms before allowing full collapse (default: 300)");
module_param(kona_keepalive_decay_min_percent, uint, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_min_percent,
	"minimum keepalive strength percent while inside decay window (default: 20)");

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
	unsigned int decay_percent = 100;

	if (!qp->last_ab || !qp->last_ib || *ab || *ib)
		return false;

	if (qp->last_ab[index] == U64_MAX || qp->last_ib[index] == U64_MAX)
		return false;

	if (!qp->last_ab[index] && !qp->last_ib[index])
		return false;

	desc = &qp->nodes[index];

	switch (desc->role) {
	case KONA_ROLE_CPU:
	case KONA_ROLE_CPU_PRIME:
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
		if (!kona_disp_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_disp_keepalive_ab_kb;
		keepalive_ib = kona_disp_keepalive_ib_kb;
		break;
	case KONA_ROLE_GENERIC:
	default:
		break;
	}

	if (!keepalive)
		return false;

	if (kona_keepalive_decay_enable && qp->last_active_jiffies) {
		unsigned long active_j = READ_ONCE(qp->last_active_jiffies[index]);
		u32 elapsed;
		unsigned int floor_percent;

		if (!active_j || !kona_keepalive_decay_window_ms)
			return false;

		/* Unsigned jiffies delta is wrap-safe for elapsed-time checks. */
		elapsed = jiffies_to_msecs(jiffies - active_j);
		if (elapsed >= kona_keepalive_decay_window_ms)
			return false;

		decay_percent = 100 -
			mul_u64_u32_div((u64)elapsed, 100,
					kona_keepalive_decay_window_ms);
		floor_percent = min_t(unsigned int, kona_keepalive_decay_min_percent,
				     100);
		decay_percent = max(decay_percent, floor_percent);

		keepalive_ab = mul_u64_u32_div(keepalive_ab, decay_percent, 100);
		keepalive_ib = mul_u64_u32_div(keepalive_ib, decay_percent, 100);

		if (!keepalive_ab && !keepalive_ib)
			return false;
	}

	/*
	 * Keepalive should be a bounded floor, not sticky reuse of the previous
	 * peak vote. Reusing last_ab/last_ib can pin large AB/IB indefinitely and
	 * prevent the interconnect from collapsing when clients request 0/0.
	 */
	*ab = keepalive_ab;
	*ib = keepalive_ib;

	return true;
}

static unsigned int kona_icc_pick_bias(const struct kona_icc_node_desc *desc,
                                      u64 ab, u64 ib)
{
        unsigned long vote = max(ab, ib);
        unsigned int bias;

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
        case KONA_ROLE_CPU_PRIME:
                if (vote >= kona_perf_turbo_kb)
                        bias = kona_perf_bias_turbo;
                else if (vote <= kona_perf_light_kb)
                        bias = kona_perf_bias_light;
                else
                        bias = kona_perf_bias;
                return min_t(unsigned int, bias + KONA_PRIME_EXTRA_BIAS_PERCENT, 200);
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
	if (!kona_perf_floor_enable)
		return;

	switch (desc->id) {
	case KONA_ICC_CPU_TO_MEM:
	case KONA_ICC_CPU_TO_GPU_CFG:
	case KONA_ICC_CPU0_TO_MEM:
	case KONA_ICC_CPU1_TO_MEM:
	case KONA_ICC_CPU2_TO_MEM:
	case KONA_ICC_CPU3_TO_MEM:
	case KONA_ICC_CPU4_TO_MEM:
	case KONA_ICC_CPU5_TO_MEM:
	case KONA_ICC_CPU6_TO_MEM:
		if (*ab && *ab < KONA_CPU_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU_DDR_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_CPU_TO_LLCC:
	case KONA_ICC_CPU0_TO_LLCC:
	case KONA_ICC_CPU1_TO_LLCC:
	case KONA_ICC_CPU2_TO_LLCC:
	case KONA_ICC_CPU3_TO_LLCC:
	case KONA_ICC_CPU4_TO_LLCC:
	case KONA_ICC_CPU5_TO_LLCC:
	case KONA_ICC_CPU6_TO_LLCC:
		if (*ab && *ab < KONA_CPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU_LLCC_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU_LLCC_IB_FLOOR_KB;
		break;
	case KONA_ICC_CPU7_TO_MEM:
		if (*ab && *ab < KONA_CPU_PRIME_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU_PRIME_DDR_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_PRIME_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU_PRIME_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_CPU7_TO_LLCC:
		if (*ab && *ab < KONA_CPU_PRIME_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU_PRIME_LLCC_AB_FLOOR_KB;
		if (*ib && *ib < KONA_CPU_PRIME_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU_PRIME_LLCC_IB_FLOOR_KB;
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
	case KONA_ICC_GMU_TO_MEM:
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
	case KONA_ICC_GMU_TO_LLCC:
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
	 * Keep low-bandwidth peripheral paths (e.g. RNG) unmodified so
	 * small functional votes do not get inflated by performance floors.
	 */
	if (desc->id == KONA_ICC_CPU_TO_PRNG)
		return;

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
		.id = KONA_ICC_GMU_TO_LLCC,
		.name = "gmu-llcc",
		.ab = "GPU_LLCC_AB",
		.ib = "GPU_LLCC_IB",
		.role = KONA_ROLE_GPU,
	},
	{
		.id = KONA_ICC_GMU_TO_MEM,
		.name = "gmu-ddr",
		.ab = "GPU_MEM_AB",
		.ib = "GPU_MEM_IB",
		.role = KONA_ROLE_GPU,
	},
	{
		.id = KONA_ICC_CPU_TO_GPU_CFG,
		.name = "cpu-gpu-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
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
		.id = KONA_ICC_CPU0_TO_LLCC,
		.name = "cpu0-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU0_TO_MEM,
		.name = "cpu0-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU1_TO_LLCC,
		.name = "cpu1-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU1_TO_MEM,
		.name = "cpu1-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU2_TO_LLCC,
		.name = "cpu2-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU2_TO_MEM,
		.name = "cpu2-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU3_TO_LLCC,
		.name = "cpu3-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU3_TO_MEM,
		.name = "cpu3-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU4_TO_LLCC,
		.name = "cpu4-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU4_TO_MEM,
		.name = "cpu4-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU5_TO_LLCC,
		.name = "cpu5-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU5_TO_MEM,
		.name = "cpu5-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU6_TO_LLCC,
		.name = "cpu6-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU6_TO_MEM,
		.name = "cpu6-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU,
	},
	{
		.id = KONA_ICC_CPU7_TO_LLCC,
		.name = "cpu7-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_CPU_PRIME,
	},
	{
		.id = KONA_ICC_CPU7_TO_MEM,
		.name = "cpu7-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CPU_PRIME,
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
	{
		.id = KONA_ICC_CPU_TO_PRNG,
		.name = "cpu-prng",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_USB0_TO_MEM,
		.name = "usb0-ddr",
		.ab = "USB0_MEM_AB",
		.ib = "USB0_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_USB1_TO_MEM,
		.name = "usb1-ddr",
		.ab = "USB1_MEM_AB",
		.ib = "USB1_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_QUP_TO_MEM,
		.name = "qup-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_UFS_TO_LLCC,
		.name = "ufs-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_UFS_TO_MEM,
		.name = "ufs-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_CAM_CFG,
		.name = "cam-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_GENERIC,
	},
	{
		.id = KONA_ICC_DISP_CFG,
		.name = "disp-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		/* Display register/config bus: treat as DISPLAY-critical. */
		.role = KONA_ROLE_DISPLAY,
	},
	{
		.id = KONA_ICC_VIDEO_CFG,
		.name = "video-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		/* Video/display clock/config bus: keep warm across resume. */
		.role = KONA_ROLE_DISPLAY,
	},
};

static inline void kona_icc_validate_node_count(void)
{
	BUILD_BUG_ON(ARRAY_SIZE(kona_nodes) != KONA_ICC_NUM_NODES);
}


static const struct kona_icc_data kona_data = {
	.nodes = kona_nodes,
	.num_nodes = ARRAY_SIZE(kona_nodes),
	.boot_floor_vote = true,
};

static const struct kona_icc_node_desc *
kona_find_desc(struct kona_icc_provider *qp, u32 id, unsigned int *index);

static bool kona_icc_is_display_critical_id(u32 id)
{
	switch (id) {
	case KONA_ICC_DISP0_TO_MEM:
	case KONA_ICC_DISP1_TO_MEM:
	case KONA_ICC_DISP_CFG:
	case KONA_ICC_VIDEO_CFG:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_display_cfg_id(u32 id)
{
	switch (id) {
	case KONA_ICC_DISP_CFG:
	case KONA_ICC_VIDEO_CFG:
		return true;
	default:
		return false;
	}
}

static void kona_icc_get_display_nonzero_floor(u32 id, u64 *ab, u64 *ib)
{
	u64 floor_ab, floor_ib;

	if (kona_icc_is_display_cfg_id(id)) {
		floor_ab = (u64)kona_display_cfg_nonzero_floor_ab_kBps;
		floor_ib = (u64)kona_display_cfg_nonzero_floor_ib_kBps;
	} else {
		floor_ab = (u64)kona_display_nonzero_floor_ab_kBps;
		floor_ib = (u64)kona_display_nonzero_floor_ib_kBps;
	}

	/* Keep IB at least 2x AB to avoid cnoc/config-path bring-up stalls. */
	if (floor_ab && !floor_ib)
		floor_ib = floor_ab * 2;
	else if (floor_ab && floor_ib < floor_ab * 2)
		floor_ib = floor_ab * 2;

	*ab = floor_ab;
	*ib = floor_ib;
}

static int kona_icc_validate_display_nodes(struct kona_icc_provider *qp)
{
	static const u32 required_ids[] = {
		KONA_ICC_DISP0_TO_MEM,
		KONA_ICC_DISP1_TO_MEM,
		KONA_ICC_DISP_CFG,
		KONA_ICC_VIDEO_CFG,
	};
	int i;

	for (i = 0; i < ARRAY_SIZE(required_ids); i++) {
		const struct kona_icc_node_desc *desc = kona_find_desc(qp, required_ids[i], NULL);

		if (!desc) {
			dev_warn(qp->provider.dev,
				 "kona-icc: missing DISPLAY-critical node id=%u from provider table\n",
				 required_ids[i]);
			if (kona_display_topology_strict)
				return -EINVAL;
			continue;
		}

		if (desc->role != KONA_ROLE_DISPLAY) {
			dev_warn(qp->provider.dev,
				 "kona-icc: node %s(id=%u) must be DISPLAY role (got %u)\n",
				 desc->name, required_ids[i], desc->role);
			if (kona_display_topology_strict)
				return -EINVAL;
		}
	}

	return 0;
}

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

static int kona_icc_send_bw(struct device *dev, const char *res, u32 kbps, bool wait)
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
	if (!cmd_db_ready()) {
		dev_dbg_ratelimited(dev, "kona-icc: cmd-db not ready for %s\n", res ?: "?");
		return -EAGAIN;
	}

	addr = cmd_db_read_addr(res);
	if (!addr) {
		dev_dbg_ratelimited(dev, "kona-icc: missing cmd-db addr for %s\n", res ?: "?");
		return -EAGAIN;
	}

	/* cmd.data is KB/s; callers must already scale to KB/s. */
	cmd.addr = addr;
	cmd.data = kbps;
	cmd.wait = wait;

	ret = rpmh_write(dev, RPMH_ACTIVE_ONLY_STATE, &cmd, 1);
	if (ret == -EBUSY || ret == -ETIMEDOUT || ret == -EPROBE_DEFER) {
		dev_dbg_ratelimited(dev,
			"kona-icc: rpmh deferring %s=%uKB/s ret=%d\n",
			res ?: "?", kbps, ret);
		return -EAGAIN;
	}

	return ret;
}

static int kona_icc_send_node_votes(struct kona_icc_provider *qp,
				    unsigned int index, u64 ab, u64 ib,
				    bool *retry)
{
	int ret;
	bool wait = false;

	if (retry)
		*retry = false;

	if (qp->nodes[index].id == KONA_ICC_GPU_TO_MEM) {
		ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ib, ib, wait);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			return ret;

		ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ab, ab, wait);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			return ret;
	} else {
		ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ab, ab, wait);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			return ret;

		ret = kona_icc_send_bw(qp->provider.dev, qp->nodes[index].ib, ib, wait);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			return ret;
	}

	if (qp->last_ab)
		qp->last_ab[index] = ab;
	if (qp->last_ib)
		qp->last_ib[index] = ib;

	return 0;

out_retry:
	if (retry)
		*retry = true;

	return -EAGAIN;
}

static bool kona_icc_can_program(struct kona_icc_provider *qp, const char **reason)
{
	if (unlikely(READ_ONCE(qp->system_suspended))) {
		if (reason)
			*reason = "system-suspended";
		return false;
	}

	if (unlikely(atomic_read(&qp->votes_paused))) {
		if (reason)
			*reason = "votes-paused";
		return false;
	}

	if (reason)
		*reason = "ready";

	return true;
}

static void kona_icc_queue_replay(struct kona_icc_provider *qp, unsigned int delay_ms,
				 const char *why)
{
	mod_delayed_work(system_wq, &qp->retry_work, msecs_to_jiffies(delay_ms));

	if (kona_resume_debug)
		dev_info_ratelimited(qp->provider.dev,
			"kona-icc: replay queued (%s, %ums) deferred=%d replay=%d skips=%d\n",
			why ?: "unknown", delay_ms,
			atomic_read(&qp->deferred_votes),
			atomic_read(&qp->replay_runs),
			atomic_read(&qp->display_replay_skips));
}


static bool kona_icc_replay_req_votes(struct kona_icc_provider *qp)
{
	bool need_retry = false;
	int i;

	if (!qp || !qp->req_ab || !qp->req_ib)
		return false;

	for (i = 0; i < qp->num_nodes; i++) {
		u64 ab = qp->req_ab[i];
		u64 ib = qp->req_ib[i];
		bool retry = false;

		if (ab == U64_MAX || ib == U64_MAX)
			continue;

		if (qp->last_ab && qp->last_ib &&
		    qp->last_ab[i] == ab && qp->last_ib[i] == ib)
			continue;

		kona_icc_send_node_votes(qp, i, ab, ib, &retry);
		if (retry)
			need_retry = true;
	}

	return need_retry;
}


static bool kona_icc_replay_req_votes_role(struct kona_icc_provider *qp,
					  enum kona_icc_role role,
					  bool apply_display_floor)
{
	bool need_retry = false;
	int i;

	for (i = 0; i < qp->num_nodes; i++) {
		bool retry = false;
		int ret;
		u64 ab = qp->req_ab[i];
		u64 ib = qp->req_ib[i];
		bool req_unset = (ab == U64_MAX && ib == U64_MAX);
		bool req_zero = (!req_unset && !ab && !ib);

		if (qp->nodes[i].role != role)
			continue;

		/*
		 * Never synthesize votes for nodes that have never received a request.
		 * For DISPLAY resume, prefer the last known non-zero vote if we have one.
		 */
		if (req_unset || req_zero) {
			bool used_fallback_floor = false;

			if (role != KONA_ROLE_DISPLAY || !apply_display_floor ||
			    !qp->saved_ab || !qp->saved_ib ||
			    qp->saved_ab[i] == U64_MAX || qp->saved_ib[i] == U64_MAX) {
				if (role == KONA_ROLE_DISPLAY && apply_display_floor &&
				    kona_display_nonzero_floor_enable) {
					kona_icc_get_display_nonzero_floor(qp->nodes[i].id,
								   &ab, &ib);
					used_fallback_floor = true;
					if (kona_resume_debug)
						dev_info_ratelimited(qp->provider.dev,
							"kona-icc: replaying fallback DISPLAY floor for %s (req=%s) ab=%llu ib=%llu\n",
							qp->nodes[i].name,
							req_unset ? "unset" : "0/0",
							ab, ib);
				} else {
					if (role == KONA_ROLE_DISPLAY)
						atomic_inc(&qp->display_replay_skips);
					continue;
				}
			}

			if (!used_fallback_floor && qp->saved_ab && qp->saved_ib &&
			    qp->saved_ab[i] != U64_MAX && qp->saved_ib[i] != U64_MAX) {
				if (kona_resume_debug && req_zero)
					dev_info_ratelimited(qp->provider.dev,
						"kona-icc: replaying saved DISPLAY vote for %s during resume (req=0/0)\n",
						qp->nodes[i].name);

				ab = qp->saved_ab[i];
				ib = qp->saved_ib[i];
			} else if (!used_fallback_floor && kona_display_bootstrap_floor_enable &&
				   qp->resume_phase == 0 &&
				   kona_icc_is_display_critical_id(qp->nodes[i].id)) {
				ab = (u64)kona_display_resume_floor_ab_kBps;
				ib = (u64)kona_display_resume_floor_ib_kBps;
				if (kona_resume_debug)
					dev_info_ratelimited(qp->provider.dev,
						"kona-icc: bootstrap DISPLAY floor for %s (missing saved/requested vote) ab=%llu ib=%llu\n",
						qp->nodes[i].name, ab, ib);
			} else if (!used_fallback_floor) {
				atomic_inc(&qp->display_replay_skips);
				continue;
			}
		} else {
			if (ab == U64_MAX)
				ab = 0;
			if (ib == U64_MAX)
				ib = 0;
		}

		if ((ab || ib) && apply_display_floor && kona_display_resume_floor_enable &&
		    role == KONA_ROLE_DISPLAY) {
			u64 floor_ab = (u64)kona_display_resume_floor_ab_kBps;
			u64 floor_ib = (u64)kona_display_resume_floor_ib_kBps;
			/*
			 * DISPLAY resume floor needs peak headroom for the initial modeset burst.
			 * If only AB is specified, derive IB = 2x AB. If both are specified,
			 * enforce IB >= 2x AB to avoid black-screen resumes on battery.
			 */
			if (floor_ab && !floor_ib)
				floor_ib = floor_ab * 2;
			else if (floor_ab && floor_ib < floor_ab * 2)
				floor_ib = floor_ab * 2;

			if (floor_ab && ab < floor_ab)
				ab = floor_ab;
			if (floor_ib && ib < floor_ib)
				ib = floor_ib;
		}

		if (!ab && !ib)
			continue;

		ret = kona_icc_send_node_votes(qp, i, ab, ib, &retry);
		if (ret == -EAGAIN || retry)
			need_retry = true;
	}

	return need_retry;
}

static bool kona_icc_replay_req_votes_phased(struct kona_icc_provider *qp)
{
	bool need_retry = false;

	switch (qp->resume_phase) {
	case 0:
		/*
		 * Display bring-up can start immediately after resume unpauses voting.
		 * Prioritize DISPLAY paths first so panel/SDE sees fabric/DDR votes early,
		 * especially on battery where CX can fully collapse.
		 *
		 * Keep this phase small to avoid an RPMh/apps_rsc replay storm.
		 */
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_DISPLAY, true);
		qp->resume_phase = 1;
		schedule_delayed_work(&qp->retry_work,
				      msecs_to_jiffies(KONA_RESUME_PHASE1_DELAY_MS));
		return need_retry;
	case 1:
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_CPU, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_CPU_PRIME, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_NPU, false);
		qp->resume_phase = 2;
		schedule_delayed_work(&qp->retry_work,
				      msecs_to_jiffies(KONA_RESUME_PHASE2_DELAY_MS));
		return need_retry;
	case 2:
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_GPU, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_GENERIC, false);
		qp->resume_phase = 3; /* done */
		return need_retry;
	default:
		break;
	}

	/* Normal steady-state replay path. */
	return kona_icc_replay_req_votes(qp);
}

static void kona_icc_retry_workfn(struct work_struct *work)
{
	struct kona_icc_provider *qp = container_of(to_delayed_work(work),
						    struct kona_icc_provider,
						    retry_work);
	bool need_retry = false;
	const char *reason;

	if (!qp->req_ab || !qp->req_ib)
		return;

	/*
	 * Do not replay votes while the system is in a suspended/paused window.
	 * Some wake/idle paths are not RPMh-safe and can wedge apps_rsc.
	 *
	 * If we were scheduled during resume_noirq (votes_paused=1), don't
	 * permanently drop the replay. Reschedule and try again once resume()
	 * unpauses voting.
	 */
	if (!kona_icc_can_program(qp, &reason)) {
		if (qp->req_ab && qp->req_ib)
			kona_icc_queue_replay(qp, 50, reason);
		return;
	}

	atomic_inc(&qp->replay_runs);
	need_retry = kona_icc_replay_req_votes_phased(qp);

	if (need_retry)
		kona_icc_queue_replay(qp, KONA_RETRY_DELAY_MS, "provider-not-ready");
}

static int kona_icc_set(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	struct kona_icc_provider *qp;
	u64 ab, ib;
	unsigned int index;

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
	 * Do not keep or boost votes while the system is entering/leaving sleep.
	 * During suspend we want consumers to be able to collapse to 0/0 so RPMh
	 * can park the fabric in a low-power corner for a clean wakeup path.
	 */
	if (READ_ONCE(qp->system_suspended))
		goto skip_perf_floor;

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

skip_perf_floor:
#endif

#ifdef DEBUG
	if (qp->nodes[index].role == KONA_ROLE_CPU ||
	    qp->nodes[index].role == KONA_ROLE_CPU_PRIME)
		pr_info("kona-icc: %s avg=%uKB/s peak=%uKB/s -> ab=%lluKB/s ib=%lluKB/s prev ab/ib=%llu/%llu\n",
			qp->nodes[index].name, avg_bw, peak_bw,
			ab, ib,
			qp->last_ab ? qp->last_ab[index] : 0,
			qp->last_ib ? qp->last_ib[index] : 0);
#endif
	
	/*
	 * Short post-resume anti-collapse window for DISPLAY: some clients
	 * transiently vote 0/0 during panel re-enable sequencing. On battery
	 * this can collapse interconnect too early and wedge panel bring-up.
	 */
	if (qp->nodes[index].role == KONA_ROLE_DISPLAY && !ab && !ib &&
	    !READ_ONCE(qp->system_suspended) && !atomic_read(&qp->votes_paused) &&
	    kona_display_resume_hold_ms &&
	    time_before(jiffies, qp->resume_jiffies +
			msecs_to_jiffies(kona_display_resume_hold_ms)) &&
	    qp->saved_ab && qp->saved_ib &&
	    qp->saved_ab[index] != U64_MAX && qp->saved_ib[index] != U64_MAX) {
		u64 hold_ab = max_t(u64, qp->saved_ab[index],
				   (u64)kona_display_resume_floor_ab_kBps);
		u64 hold_ib = max_t(u64, qp->saved_ib[index],
				   (u64)kona_display_resume_floor_ib_kBps);

		ab = hold_ab;
		ib = hold_ib;
		if (kona_resume_debug)
			dev_info_ratelimited(qp->provider.dev,
				"kona-icc: hold DISPLAY vote for %s during resume grace: ab=%llu ib=%llu\n",
				qp->nodes[index].name, ab, ib);
	}

	/*
	 * Hard non-zero fallback for DISPLAY paths: avoid 0/0 collapse on ddr and
	 * config-path links where panel/SDE/dispcc sequences can stall.
	 */
	if (qp->nodes[index].role == KONA_ROLE_DISPLAY && !ab && !ib &&
	    kona_display_nonzero_floor_enable && !READ_ONCE(qp->system_suspended)) {
		kona_icc_get_display_nonzero_floor(qp->nodes[index].id, &ab, &ib);
		if (kona_resume_debug)
			dev_info_ratelimited(qp->provider.dev,
				"kona-icc: fallback non-zero DISPLAY floor for %s: ab=%llu ib=%llu\n",
				qp->nodes[index].name, ab, ib);
	}

	/*
	 * Cache the client-requested AB/IB (pre-floor/keepalive adjustments).
	 * Resume replay logic relies on detecting explicit 0/0 requests.
	 */
	if (qp->req_ab)
		qp->req_ab[index] = (u64)avg_bw;
	if (qp->req_ib)
		qp->req_ib[index] = (u64)peak_bw;
	if (qp->last_active_jiffies && (avg_bw || peak_bw))
		qp->last_active_jiffies[index] = jiffies;

	/*
	 * Track last-known non-zero DISPLAY votes so resume can re-assert
	 * fabric bandwidth before dispcc/panel bring-up sequences.
	 *
	 * Preserve the remembered vote when a client requests 0/0 and we
	 * synthesize a temporary fallback floor; otherwise a transient 0/0
	 * would overwrite the remembered active vote with the tiny fallback.
	 */
	if (qp->nodes[index].role == KONA_ROLE_DISPLAY && (ab || ib) &&
	    (avg_bw || peak_bw)) {
		qp->saved_ab[index] = ab;
		qp->saved_ib[index] = ib;
	}


	/*
	 * Best-effort synchronous programming:
	 * - Apply the vote immediately when we're in a normal runtime window.
	 * - If RPMh/apps_rsc is busy, defer to retry_workfn() to replay later.
	 *
	 * Some clients (display / clock enable sequences) require the bandwidth
	 * vote to be active before continuing; fully deferring votes can cause
	 * black-screen / stuck-resume symptoms when the fabric stays at 0/0.
	 */
	{
		const char *reason;

		if (!kona_icc_can_program(qp, &reason)) {
			/*
			 * During suspend/resume windows RPMh ACTIVE_ONLY writes may be unsafe or
			 * transiently blocked. Keep cached requests updated and replay later, but
			 * do not fail consumers (e.g. devbw/SDE) on transient -EAGAIN windows.
			 */
			if (!READ_ONCE(qp->system_suspended) && (ab || ib))
				kona_icc_queue_replay(qp, 0, reason);

			atomic_inc(&qp->deferred_votes);

			if (kona_resume_debug && (ab || ib))
				dev_info_ratelimited(qp->provider.dev,
					"kona-icc: deferred vote node=%s reason=%s ab=%llu ib=%llu\n",
					qp->nodes[index].name, reason, ab, ib);

			return 0;
		}
	}

	{
		bool retry = false;
		int ret = kona_icc_send_node_votes(qp, index, ab, ib, &retry);

		if (ret == -EAGAIN || retry)
			kona_icc_queue_replay(qp, KONA_RETRY_DELAY_MS, "send-eagain");
		else if (ret)
			return ret;
	}

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

        static int kona_icc_suspend_noirq(struct device *dev)
{
	struct kona_icc_provider *qp = dev_get_drvdata(dev);

	if (qp)
		atomic_set(&qp->votes_paused, 1);
	/* Ensure no stale vote replays fire during suspend_noirq. */
	if (qp)
		cancel_delayed_work(&qp->retry_work);

	if (kona_resume_debug && qp)
		dev_info(dev, "kona-icc: votes paused (suspend_noirq)\n");

	return 0;
}

static int kona_icc_resume_noirq(struct device *dev)
{
	struct kona_icc_provider *qp = dev_get_drvdata(dev);

	/*
	 * Keep votes paused through resume_noirq. Unpause in normal resume() once
	 * RPMh/apps_rsc is expected to be ready for ACTIVE_ONLY writes.
	 */
	if (qp)
		atomic_set(&qp->votes_paused, 1);
	if (qp)
		cancel_delayed_work(&qp->retry_work);

	if (kona_resume_debug && qp)
		dev_info(dev, "kona-icc: resume_noirq (still paused)\n");

	return 0;
}



static int kona_icc_suspend(struct device *dev)
{
	struct kona_icc_provider *qp = dev_get_drvdata(dev);

	if (!qp)
		return 0;

	WRITE_ONCE(qp->system_suspended, true);
	/* Ensure no stale vote replays fire during suspend/idle windows. */
	cancel_delayed_work_sync(&qp->retry_work);

	return 0;
}


static int kona_icc_resume(struct device *dev)
{
	struct kona_icc_provider *qp = dev_get_drvdata(dev);

	if (qp)
		WRITE_ONCE(qp->system_suspended, false);

	if (qp)
		atomic_set(&qp->votes_paused, 0);

	if (kona_resume_debug && qp)
		dev_info(dev, "kona-icc: votes unpaused (resume)\n");

	/*
	 * RPMh ACTIVE_ONLY votes are not retained across suspend. Invalidate the
	 * software cache so the first post-resume icc_set_bw() always re-sends.
	 */
	kona_icc_invalidate_cache(qp);

	/*
	 * Resume hardening (no-idle-drain):
	 *   - Avoid an early resume replay storm that can congest RPMh/apps_rsc
	 *     and race the DSI/SDE bring-up window on battery.
	 *   - Replay last requested votes in phases (DISPLAY -> CPU/NPU -> GPU/GENERIC).
	 */
	if (qp) {
		qp->resume_jiffies = jiffies;
		qp->resume_phase = 0;
		kona_icc_queue_replay(qp, KONA_RESUME_PHASE0_DELAY_MS, "resume-phase0");
	}

	return 0;
}



#ifdef CONFIG_PM_SLEEP
static const struct dev_pm_ops kona_icc_pm_ops = {
	.suspend		= kona_icc_suspend,
	.resume			= kona_icc_resume,
	.suspend_noirq	= kona_icc_suspend_noirq,
	.resume_noirq	= kona_icc_resume_noirq,
	.freeze_noirq	= kona_icc_suspend_noirq,
	.thaw_noirq	= kona_icc_resume_noirq,
	.poweroff_noirq	= kona_icc_suspend_noirq,
	.restore_noirq	= kona_icc_resume_noirq,
};
#endif


static int kona_icc_probe(struct platform_device *pdev)
{
	const struct kona_icc_data *data =
		of_device_get_match_data(&pdev->dev);
	struct kona_icc_provider *qp;
	u64 __maybe_unused ab, ib;
	int ret, i;

	kona_icc_validate_node_count();

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

	qp->req_ab = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->req_ab)
		return -ENOMEM;

	qp->req_ib = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->req_ib)
		return -ENOMEM;

	qp->saved_ab = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->saved_ab)
		return -ENOMEM;

	qp->saved_ib = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->saved_ib)
		return -ENOMEM;

	qp->last_active_jiffies = devm_kcalloc(&pdev->dev, qp->num_nodes,
					      sizeof(unsigned long), GFP_KERNEL);
	if (!qp->last_active_jiffies)
		return -ENOMEM;


	INIT_DELAYED_WORK(&qp->retry_work, kona_icc_retry_workfn);
	atomic_set(&qp->deferred_votes, 0);
	atomic_set(&qp->replay_runs, 0);
	atomic_set(&qp->display_replay_skips, 0);

	for (i = 0; i < qp->num_nodes; i++) {
		qp->last_ab[i] = U64_MAX;
		qp->last_ib[i] = U64_MAX;
		qp->req_ab[i] = U64_MAX;
		qp->req_ib[i] = U64_MAX;
		qp->saved_ab[i] = U64_MAX;
		qp->saved_ib[i] = U64_MAX;
	}

        qp->provider.dev = &pdev->dev;
        qp->provider.of_node = pdev->dev.of_node;
        qp->provider.xlate = kona_icc_xlate;
        qp->provider.set = kona_icc_set;
        qp->provider.release = kona_icc_release;

	platform_set_drvdata(pdev, qp);

	ret = kona_icc_validate_display_nodes(qp);
	if (ret)
		return ret;

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

			if (qp->nodes[i].role == KONA_ROLE_DISPLAY)
				continue;

			ab = KONA_ICC_MIN_AB_FLOOR_KB;
			ib = KONA_ICC_MIN_IB_FLOOR_KB;
			kona_icc_apply_floor(&qp->nodes[i], &ab, &ib);

			r_ab = kona_icc_send_bw(qp->provider.dev,
					     qp->nodes[i].ab, ab, false);
			r_ib = kona_icc_send_bw(qp->provider.dev,
					     qp->nodes[i].ib, ib, false);

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

	cancel_delayed_work_sync(&qp->retry_work);

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
