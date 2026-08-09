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
#include <linux/bitmap.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/msm-bus.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/kdev_t.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/msm_drm_notify.h>
#include <linux/suspend.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>

#include <dt-bindings/interconnect/qcom,kona.h>
#include <dt-bindings/msm/msm-bus-ids.h>
#include <soc/qcom/cmd-db.h>
#include <soc/qcom/rpmh.h>

#include "kona-rpmh.h"

/*
 * Description of each logical Kona ICC path.
 *
 * id  - logical ICC path ID (from dt-bindings/interconnect/qcom,kona.h)
 */
enum kona_icc_role {
        KONA_ROLE_CPU,
        KONA_ROLE_CPU_PRIME,
        KONA_ROLE_GPU,
	KONA_ROLE_GMU,
        KONA_ROLE_NPU,
        KONA_ROLE_DSP,
        KONA_ROLE_MEDIA,
	KONA_ROLE_STORAGE,
	KONA_ROLE_IPA,
	KONA_ROLE_PERIPHERAL,
	KONA_ROLE_CONFIG,
	KONA_ROLE_RAW,
        KONA_ROLE_DISPLAY,
};

struct kona_icc_node_desc {
        u32 id;
        const char *name;
        const char *ab;
        const char *ib;
        enum kona_icc_role role;
};

struct kona_packed_inputs {
	u64 llcc_avg;
	u64 llcc_peak;
	u64 mem_avg;
	u64 mem_peak;
	unsigned int group_mask;
	bool dry_run;
	u64 config_generation;
};

struct kona_icc_provider {
	struct icc_provider provider;
	struct device *rpmh_dev;
	const struct kona_icc_node_desc *nodes;
	size_t num_nodes;
	bool boot_floor_vote;
	bool first_cpu_request_seen;
	u64 invalid_vote_count;
	u64 packed_submission_count;
	u64 packed_dry_run_build_count;
	u64 packed_dry_run_skip_count;
	u64 packed_aggregate_build_count;
	u64 packed_aggregate_unchanged_skip_count;
	u64 packed_config_generation;
	u64 packed_observed_generation;
	int last_packed_dry_run_error;
	struct kona_packed_inputs packed_current_inputs;
	struct kona_packed_inputs packed_last_dry_run_inputs;
	struct kona_packed_inputs packed_last_real_inputs;
	atomic64_t packed_update_generation;
	u64 packed_processed_generation;
	bool packed_fallback_active;
	bool packed_real_write_consumed;
	u64 packed_fallback_count;
	int last_failed_vcd;
	int last_packed_error;
	int last_legacy_fallback_error;
	u64 last_invalid_raw_ab;
	u64 last_invalid_raw_ib;
	u64 last_invalid_eff_ab;
	u64 last_invalid_eff_ib;
	u32 last_invalid_node_id;
	const char *last_invalid_node_name;
	struct kona_bcm_state cpu_bcms[KONA_CPU_BCM_COUNT];
	bool system_suspended;
	atomic_t votes_paused;
	/* Serialize logical-path updates that share a physical RPMh BCM. */
	struct mutex vote_lock;
	u64 *last_ab;
	u64 *last_ib;
	u64 *req_ab;
	u64 *req_ib;
	u64 *eff_ab;
	u64 *eff_ib;
	u64 *saved_ab;
	u64 *saved_ib;
	unsigned long resume_jiffies;
	u8 resume_phase;
	struct delayed_work retry_work;
	atomic_t deferred_votes;
	atomic_t replay_runs;
	atomic_t display_replay_skips;
	atomic_t replay_queue_skips;
	unsigned long *last_active_jiffies;
	unsigned long *dirty_nodes;
	unsigned long *replay_scan_nodes;
	spinlock_t dirty_lock;
	bool display_active;
	bool display_hints_available;
	bool display_protection;
	unsigned long display_off_jiffies;
	struct notifier_block display_nb;
	bool display_nb_registered;
	struct device **sysfs_nodes;
	struct class *icc_class;
	unsigned long gpu_oc_last_jiffies;
	unsigned long npu_oc_last_jiffies;
	unsigned long cpu_prime_oc_last_jiffies;
	unsigned long ux_turbo_last_jiffies;
#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
        bool gpu_llcc_turbo;
#endif
};

struct kona_icc_data {
        const struct kona_icc_node_desc *nodes;
        size_t num_nodes;
        bool boot_floor_vote;
};

static DEFINE_MUTEX(kona_packed_param_lock);
static struct kona_icc_provider *kona_packed_provider;

static void kona_icc_packed_parameter_changed(struct kona_icc_provider *qp,
					       bool enabled);
static bool kona_icc_is_cpu_memory_path(const struct kona_icc_node_desc *desc);
static void kona_icc_mark_dirty(struct kona_icc_provider *qp,
				unsigned int index);
static void kona_icc_packed_force_generation(struct kona_icc_provider *qp);
static void kona_icc_queue_replay(struct kona_icc_provider *qp,
				 unsigned int delay_ms, const char *why);

static bool kona_packed_runtime_enable;
static bool kona_packed_dry_run = true;
static bool kona_packed_real_write_enable;
static bool kona_packed_real_write_once = true;
static bool kona_packed_real_write_rearm;
static bool kona_packed_force_dirty;

static int kona_param_set_packed_real_write_enable(const char *val,
						    const struct kernel_param *kp)
{
	bool enabled;
	int ret;

	ret = kstrtobool(val, &enabled);
	if (ret)
		return ret;

	mutex_lock(&kona_packed_param_lock);
	WRITE_ONCE(kona_packed_real_write_enable, enabled);
	mutex_unlock(&kona_packed_param_lock);

	return 0;
}

static const struct kernel_param_ops kona_packed_real_write_enable_ops = {
	.set = kona_param_set_packed_real_write_enable,
	.get = param_get_bool,
};
module_param_cb(packed_real_write_enable,
		&kona_packed_real_write_enable_ops,
		&kona_packed_real_write_enable, 0644);
MODULE_PARM_DESC(packed_real_write_enable,
		 "Permit Stage 4 packed BCM hardware submissions");
module_param_named(packed_real_write_once,
		   kona_packed_real_write_once, bool, 0644);
MODULE_PARM_DESC(packed_real_write_once,
		 "Limit Stage 4 packed BCM hardware submission to one transaction");

static int kona_param_set_packed_real_write_rearm(const char *val,
						   const struct kernel_param *kp)
{
	struct kona_icc_provider *qp;
	bool rearm;
	int ret;

	ret = kstrtobool(val, &rearm);
	if (ret || !rearm)
		return ret;

	mutex_lock(&kona_packed_param_lock);
	if (READ_ONCE(kona_packed_real_write_enable)) {
		ret = -EBUSY;
		goto unlock;
	}

	qp = kona_packed_provider;
	if (qp) {
		mutex_lock(&qp->vote_lock);
		qp->packed_real_write_consumed = false;
		mutex_unlock(&qp->vote_lock);
	}
unlock:
	mutex_unlock(&kona_packed_param_lock);

	return ret;
}

static const struct kernel_param_ops kona_packed_real_write_rearm_ops = {
	.set = kona_param_set_packed_real_write_rearm,
	.get = param_get_bool,
};
module_param_cb(packed_real_write_rearm,
		&kona_packed_real_write_rearm_ops,
		&kona_packed_real_write_rearm, 0644);
MODULE_PARM_DESC(packed_real_write_rearm,
		 "Write 1 to clear the packed real-write one-shot latch while writes are disabled");

static int kona_param_set_packed_dry_run(const char *val,
					 const struct kernel_param *kp)
{
	bool dry_run;
	int ret;

	ret = kstrtobool(val, &dry_run);
	if (ret)
		return ret;

	mutex_lock(&kona_packed_param_lock);
	if (dry_run != kona_packed_dry_run) {
		WRITE_ONCE(kona_packed_dry_run, dry_run);
		if (kona_packed_provider && READ_ONCE(kona_packed_runtime_enable)) {
			struct kona_icc_provider *qp = kona_packed_provider;
			unsigned int i;

			mutex_lock(&qp->vote_lock);
			kona_icc_packed_force_generation(qp);
			for (i = 0; i < qp->num_nodes; i++)
				if (kona_icc_is_cpu_memory_path(&qp->nodes[i]))
					kona_icc_mark_dirty(qp, i);
			mutex_unlock(&qp->vote_lock);
			kona_icc_queue_replay(qp, 0, "packed-dry-run");
		}
	}
	mutex_unlock(&kona_packed_param_lock);

	return 0;
}

static const struct kernel_param_ops kona_packed_dry_run_ops = {
	.set = kona_param_set_packed_dry_run,
	.get = param_get_bool,
};
module_param_cb(packed_dry_run, &kona_packed_dry_run_ops,
		&kona_packed_dry_run, 0644);
MODULE_PARM_DESC(packed_dry_run,
		 "Build and log Stage 4 packed BCM commands without submitting them");

static int kona_param_set_packed_runtime_enable(const char *val,
						 const struct kernel_param *kp)
{
	bool enabled;
	int ret;

	ret = kstrtobool(val, &enabled);
	if (ret)
		return ret;

	mutex_lock(&kona_packed_param_lock);
	if (enabled != kona_packed_runtime_enable) {
		if (kona_packed_provider) {
			kona_icc_packed_parameter_changed(kona_packed_provider,
							 enabled);
		} else {
			WRITE_ONCE(kona_packed_runtime_enable, enabled);
		}
	}
	mutex_unlock(&kona_packed_param_lock);

	return 0;
}

static const struct kernel_param_ops kona_packed_runtime_enable_ops = {
	.set = kona_param_set_packed_runtime_enable,
	.get = param_get_bool,
};
module_param_cb(packed_runtime_enable, &kona_packed_runtime_enable_ops,
		&kona_packed_runtime_enable, 0644);
MODULE_PARM_DESC(packed_runtime_enable,
		 "Enable Stage 4 packed CPU BCM submissions at runtime");

#define KONA_PACKED_GROUP_MC0	BIT(0)
#define KONA_PACKED_GROUP_SH	BIT(1)
#define KONA_PACKED_GROUP_ALL	(KONA_PACKED_GROUP_MC0 | KONA_PACKED_GROUP_SH)
#define KONA_PACKED_REAL_WRITE_BLOCKED	1

static unsigned int kona_packed_group_mask = KONA_PACKED_GROUP_ALL;

static int kona_param_set_packed_group_mask(const char *val,
					     const struct kernel_param *kp)
{
	unsigned int mask;
	int ret;

	ret = kstrtouint(val, 0, &mask);
	if (ret)
		return ret;
	if (mask & ~KONA_PACKED_GROUP_ALL)
		return -EINVAL;

	mutex_lock(&kona_packed_param_lock);
	if (mask != kona_packed_group_mask) {
		if (kona_packed_provider && READ_ONCE(kona_packed_runtime_enable)) {
			struct kona_icc_provider *qp = kona_packed_provider;
			unsigned int i;

			mutex_lock(&qp->vote_lock);
			WRITE_ONCE(kona_packed_group_mask, mask);
			kona_icc_packed_force_generation(qp);
			for (i = 0; i < qp->num_nodes; i++)
				if (kona_icc_is_cpu_memory_path(&qp->nodes[i])) {
					qp->last_ab[i] = U64_MAX;
					qp->last_ib[i] = U64_MAX;
					kona_icc_mark_dirty(qp, i);
				}
			mutex_unlock(&qp->vote_lock);
			dev_err(qp->provider.dev,
				"kona-rpmh: packed runtime gate enabled mask=%u\n", mask);
			kona_icc_queue_replay(qp, 0, "packed-group-mask");
		} else {
			WRITE_ONCE(kona_packed_group_mask, mask);
		}
	}
	mutex_unlock(&kona_packed_param_lock);

	return 0;
}

static const struct kernel_param_ops kona_packed_group_mask_ops = {
	.set = kona_param_set_packed_group_mask,
	.get = param_get_uint,
};
module_param_cb(packed_group_mask, &kona_packed_group_mask_ops,
		&kona_packed_group_mask, 0644);
MODULE_PARM_DESC(packed_group_mask,
		 "Stage 4 packed groups: bit 0=MC0, bit 1=SH4/SH0");

static int kona_param_set_packed_force_dirty(const char *val,
					      const struct kernel_param *kp)
{
	struct kona_icc_provider *qp;
	bool force_dirty;
	int ret;

	ret = kstrtobool(val, &force_dirty);
	if (ret)
		return ret;

	mutex_lock(&kona_packed_param_lock);
	WRITE_ONCE(kona_packed_force_dirty, force_dirty);
	qp = kona_packed_provider;
	if (force_dirty && qp && READ_ONCE(kona_packed_runtime_enable)) {
		mutex_lock(&qp->vote_lock);
		kona_icc_packed_force_generation(qp);
		mutex_unlock(&qp->vote_lock);
		kona_icc_queue_replay(qp, 0, "packed-force-dirty");
	}
	mutex_unlock(&kona_packed_param_lock);

	return 0;
}

static const struct kernel_param_ops kona_packed_force_dirty_ops = {
	.set = kona_param_set_packed_force_dirty,
	.get = param_get_bool,
};
module_param_cb(packed_force_dirty, &kona_packed_force_dirty_ops,
		&kona_packed_force_dirty, 0644);
MODULE_PARM_DESC(packed_force_dirty,
		 "Force selected Stage 4 packed BCMs dirty for one build");

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

static bool kona_vote_debug;
module_param_named(vote_debug, kona_vote_debug, bool, 0644);
MODULE_PARM_DESC(kona_vote_debug, "Log Kona CPU vote transformation boundaries");

static bool kona_vote_debug_submit;
module_param_named(vote_debug_submit, kona_vote_debug_submit, bool, 0644);
MODULE_PARM_DESC(vote_debug_submit,
		 "Trace Kona shared aggregation and legacy submission");

static int kona_vote_debug_node = -1;
module_param_named(vote_debug_node, kona_vote_debug_node, int, 0644);
MODULE_PARM_DESC(vote_debug_node,
		 "Trace only this Kona logical node ID; -1 traces all nodes");

#define KONA_VOTE_TRACE(qp, desc, point, ab, ib) \
	do { \
		if (kona_vote_debug && \
		    (kona_vote_debug_node < 0 || \
		     (desc)->id == kona_vote_debug_node)) \
			dev_info((qp)->provider.dev, \
				 "kona-vote: id=%u name=%s point=%s ab=%llu ib=%llu\n", \
				 (desc)->id, (desc)->name, (point), \
				 (unsigned long long)(ab), \
				 (unsigned long long)(ib)); \
	} while (0)

/*
 * Both the ICC callback and the legacy RPMh sender have u32 vote interfaces.
 * Keeping this named limit next to the policy code makes that contract explicit
 * and prevents an uncached U64_MAX value from being mistaken for bandwidth.
 */
#define KONA_ICC_MAX_LOGICAL_VOTE	((u64)U32_MAX)

static bool kona_perf_floor_enable = true;
module_param(kona_perf_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_perf_floor_enable,
	"Enable adaptive bandwidth floors for latency-sensitive Kona paths (default: on)");

/*
 * CPU memory requests back the devbw and memlat governors.  Those requests
 * are performance critical, but RPMh normally accepts them asynchronously.
 * A benchmark can therefore begin its memory phase before a newly requested
 * DDR or LLCC corner has reached the resource controller.  Commit CPU memory
 * votes synchronously so the caller does not continue until the new corner is
 * visible to hardware.  Other traffic remains asynchronous to avoid adding
 * completion latency to display, storage, and peripheral clients.
 */
static bool kona_cpu_memory_sync_votes = true;
module_param_named(kona_cpu_memory_sync_votes, kona_cpu_memory_sync_votes,
		   bool, 0644);
MODULE_PARM_DESC(kona_cpu_memory_sync_votes, "Commit CPU memory votes synchronously");

static bool kona_rpmh_cpu_model;
module_param_named(rpmh_cpu_model, kona_rpmh_cpu_model, bool, 0444);
MODULE_PARM_DESC(rpmh_cpu_model,
		 "Deprecated: discover packed CPU BCM metadata without programming it");

static unsigned int kona_rpmh_model;
module_param_named(rpmh_model, kona_rpmh_model, uint, 0444);
MODULE_PARM_DESC(rpmh_model,
	"Packed BCM migration: 0=legacy, 1=telemetry, 2=SH4, 3=SH4+SH0, 4=CPU, 5=validated clients");

static unsigned int kona_cpu_model_stage(void)
{
	/* The old boolean is deliberately made a boot-safe discovery stage. */
	return kona_rpmh_model ? min(kona_rpmh_model, 5U) :
		(kona_rpmh_cpu_model ? 1 : 0);
}

static bool kona_display_resume_floor_enable = true;
module_param_named(kona_display_resume_floor_enable, kona_display_resume_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_resume_floor_enable,
	"Enable a minimal DISPLAY resume bandwidth floor to avoid black-screen resumes");

static unsigned int kona_display_resume_floor_ab_kBps = 256000; /* 256 MB/s */
module_param_named(kona_display_resume_floor_ab_kBps, kona_display_resume_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_resume_floor_ab_kBps, "DISPLAY resume floor average BW (kB/s)");

static unsigned int kona_display_resume_floor_ib_kBps = 2000000; /* 2 GB/s */
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

/* Match the active SDE data-bus floor; 80 MB/s cannot cover a modeset burst. */
static unsigned int kona_display_nonzero_floor_ab_kBps = 384000; /* 384 MB/s */
module_param_named(kona_display_nonzero_floor_ab_kBps, kona_display_nonzero_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_nonzero_floor_ab_kBps,
	"Fallback DISPLAY floor average BW (kB/s) when a 0/0 vote is requested");

static unsigned int kona_display_nonzero_floor_ib_kBps = 9600000; /* 9.6 GB/s */
module_param_named(kona_display_nonzero_floor_ib_kBps, kona_display_nonzero_floor_ib_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_nonzero_floor_ib_kBps,
	"Fallback DISPLAY floor peak BW (kB/s) when a 0/0 vote is requested");

static unsigned int kona_display_cfg_nonzero_floor_ab_kBps = 256000; /* 256 MB/s */
module_param_named(kona_display_cfg_nonzero_floor_ab_kBps, kona_display_cfg_nonzero_floor_ab_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_cfg_nonzero_floor_ab_kBps,
	"Fallback DISPLAY config-path floor average BW (kB/s) for 0/0 votes");

static unsigned int kona_display_cfg_nonzero_floor_ib_kBps = 512000; /* 512 MB/s */
module_param_named(kona_display_cfg_nonzero_floor_ib_kBps, kona_display_cfg_nonzero_floor_ib_kBps, uint, 0644);
MODULE_PARM_DESC(kona_display_cfg_nonzero_floor_ib_kBps,
	"Fallback DISPLAY config-path floor peak BW (kB/s) for 0/0 votes");

static bool kona_display_bootstrap_floor_enable = true;
module_param_named(kona_display_bootstrap_floor_enable, kona_display_bootstrap_floor_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_bootstrap_floor_enable,
	"Allow one-shot DISPLAY floor during phase-0 replay when no saved/requested vote exists");

static bool kona_display_notifier_enable;
module_param_named(kona_display_notifier_enable, kona_display_notifier_enable, bool, 0644);
MODULE_PARM_DESC(kona_display_notifier_enable,
	"Enable Kona ICC display notifier policy hints (default: off during staged bring-up)");

static bool kona_display_topology_strict;
module_param_named(kona_display_topology_strict, kona_display_topology_strict, bool, 0644);
MODULE_PARM_DESC(kona_display_topology_strict,
	"Fail probe if DISPLAY-critical ICC nodes are missing or not tagged DISPLAY");


/*
 * Staged Kona ICC bring-up.  Stage 0 intentionally leaves the provider
 * unregistered so legacy msm_bus fallback remains the rescue path.  Higher
 * stages expose only the IDs that have been selected for that stage; all other
 * IDs fail xlate with -ENODEV so hybrid consumers can keep using msm_bus
 * instead of switching every path to RPMh ICC at once.
 */
static unsigned int kona_icc_stage = 7;
module_param_named(kona_icc_stage, kona_icc_stage, uint, 0644);
MODULE_PARM_DESC(kona_icc_stage,
	"Kona ICC bring-up stage: 0=provider off, 1=GPU/GMU, 2=CPU/devbw, 3=storage/peripheral, 4=display, 5=NPU/media/camera/video, 6=IPA, 7=all ICC with CRYPTO/storage raw programming disabled by default");

#define KONA_ICC_STAGE_MAX	7

static bool kona_crypto_icc_enable = true;
module_param_named(kona_crypto_icc_enable, kona_crypto_icc_enable, bool, 0644);
MODULE_PARM_DESC(kona_crypto_icc_enable,
	"Expose CRYPTO ICC path while qcedev/qcrypto remain on legacy msm_bus");

static bool kona_crypto_raw_icc_enable;
module_param_named(kona_crypto_raw_icc_enable, kona_crypto_raw_icc_enable, bool, 0644);
MODULE_PARM_DESC(kona_crypto_raw_icc_enable,
	"Program CRYPTO RPMh votes instead of accepting the ICC path as a no-op during bring-up");

static bool kona_storage_raw_icc_enable;
module_param_named(kona_storage_raw_icc_enable,
		   kona_storage_raw_icc_enable, bool, 0644);
MODULE_PARM_DESC(kona_storage_raw_icc_enable,
	"Program UFS/SDHC RPMh votes instead of accepting storage ICC paths as cached no-ops during bring-up");

static bool kona_gpu_raw_icc_enable;
module_param_named(kona_gpu_raw_icc_enable,
		   kona_gpu_raw_icc_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_raw_icc_enable,
		 "Program GPU RPMh votes instead of accepting GPU ICC paths as cached no-ops");

static bool kona_gmu_raw_icc_enable;
module_param_named(kona_gmu_raw_icc_enable,
		   kona_gmu_raw_icc_enable, bool, 0644);
MODULE_PARM_DESC(kona_gmu_raw_icc_enable,
		 "Program GMU RPMh votes instead of accepting GMU ICC paths as cached no-ops");

/*
 * Real Kona CRYPTO bandwidth uses the legacy msm-bus CE0 BCM path:
 *   MSM_BUS_MASTER_CRYPTO_CORE_0 (125) -> MSM_BUS_SLAVE_EBI_CH0 (512)
 *   qcom,msm-bus vectors-KBps: 0/0 and 393600/393600
 *
 * Do not program CE0 through raw cmd-db aliases from this virtual ICC
 * provider. Instead, bridge the CRYPTO ICC path to the exported msm-bus
 * client API so the existing Qualcomm RPMh/BCM backend encodes CE0 safely.
 */
#define KONA_CRYPTO_CE0_MASTER          125
#define KONA_CRYPTO_CE0_SLAVE           512
#define KONA_CRYPTO_CE0_BW_KBPS         393600ULL

static bool kona_crypto_ce0_msm_bus_enable = true;
module_param_named(kona_crypto_ce0_msm_bus_enable,
		   kona_crypto_ce0_msm_bus_enable, bool, 0644);
MODULE_PARM_DESC(kona_crypto_ce0_msm_bus_enable,
	"Bridge KONA_ICC_CRYPTO_TO_MEM votes to the legacy CE0 msm_bus BCM client");

static bool kona_crypto_ce0_msm_bus_debug;
module_param_named(kona_crypto_ce0_msm_bus_debug,
		   kona_crypto_ce0_msm_bus_debug, bool, 0644);
MODULE_PARM_DESC(kona_crypto_ce0_msm_bus_debug,
	"Log CRYPTO ICC to CE0 msm_bus bridge votes");

static struct msm_bus_vectors kona_crypto_ce0_vectors[] = {
	{
		.src = KONA_CRYPTO_CE0_MASTER,
		.dst = KONA_CRYPTO_CE0_SLAVE,
		.ab = 0,
		.ib = 0,
	},
	{
		.src = KONA_CRYPTO_CE0_MASTER,
		.dst = KONA_CRYPTO_CE0_SLAVE,
		.ab = KONA_CRYPTO_CE0_BW_KBPS,
		.ib = KONA_CRYPTO_CE0_BW_KBPS,
	},
};

static struct msm_bus_paths kona_crypto_ce0_paths[] = {
	{
		.num_paths = 1,
		.vectors = &kona_crypto_ce0_vectors[0],
	},
	{
		.num_paths = 1,
		.vectors = &kona_crypto_ce0_vectors[1],
	},
};

static struct msm_bus_scale_pdata kona_crypto_ce0_pdata = {
	.usecase = kona_crypto_ce0_paths,
	.num_usecases = ARRAY_SIZE(kona_crypto_ce0_paths),
	.name = "kona-icc-crypto-ce0",
	.active_only = 1,
};

static DEFINE_MUTEX(kona_crypto_ce0_lock);
static u32 kona_crypto_ce0_client;
static bool kona_crypto_ce0_last_valid;
static unsigned int kona_crypto_ce0_last_idx;

static bool kona_npu_raw_safe_mode = true;
module_param_named(kona_npu_raw_safe_mode, kona_npu_raw_safe_mode, bool, 0644);
MODULE_PARM_DESC(kona_npu_raw_safe_mode,
	"Keep NPU/NPUDSP ICC votes raw: no floors, keepalive, telemetry, or resume replay");

static bool kona_gpu_policy_bypass_enable = true;
module_param_named(kona_gpu_policy_bypass_enable,
		   kona_gpu_policy_bypass_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_policy_bypass_enable,
		 "Skip Kona floors/boosts/hysteresis for GPU ICC votes before optional RPMh programming");

static bool kona_gmu_policy_bypass_enable = true;
module_param_named(kona_gmu_policy_bypass_enable,
		   kona_gmu_policy_bypass_enable, bool, 0644);
MODULE_PARM_DESC(kona_gmu_policy_bypass_enable,
	"Keep GMU ICC votes raw so GMU firmware/TCS bandwidth tables remain authoritative");

static bool kona_icc_is_crypto_path(const struct kona_icc_node_desc *desc)
{
	return desc->id == KONA_ICC_CRYPTO_TO_MEM;
}

static bool kona_icc_is_raw_npu_path(const struct kona_icc_node_desc *desc)
{
	switch (desc->id) {
	case KONA_ICC_NPU_TO_MEM:
	case KONA_ICC_NPU_TO_LLCC:
	case KONA_ICC_NPUDSP_TO_MEM:
		return kona_npu_raw_safe_mode;
	default:
		return false;
	}
}

static bool kona_icc_is_gpu_path(const struct kona_icc_node_desc *desc)
{
	switch (desc->id) {
	case KONA_ICC_GPU_TO_MEM:
	case KONA_ICC_GPU_TO_LLCC:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_gmu_path(const struct kona_icc_node_desc *desc)
{
	switch (desc->id) {
	case KONA_ICC_GMU_TO_MEM:
	case KONA_ICC_GMU_TO_LLCC:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_storage_path(const struct kona_icc_node_desc *desc)
{
	switch (desc->id) {
	case KONA_ICC_UFS_TO_MEM:
	case KONA_ICC_UFS_TO_LLCC:
	case KONA_ICC_SDHC2_TO_MEM:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_raw_role(const struct kona_icc_node_desc *desc)
{
	switch (desc->role) {
	case KONA_ROLE_IPA:
	case KONA_ROLE_PERIPHERAL:
	case KONA_ROLE_CONFIG:
	case KONA_ROLE_RAW:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_replay_suppressed_path(const struct kona_icc_node_desc *desc)
{
	return kona_icc_is_crypto_path(desc) || kona_icc_is_raw_npu_path(desc) ||
	       (!kona_storage_raw_icc_enable && kona_icc_is_storage_path(desc)) ||
	       (!kona_gpu_raw_icc_enable && kona_icc_is_gpu_path(desc)) ||
	       (!kona_gmu_raw_icc_enable && kona_icc_is_gmu_path(desc)) ||
	       kona_icc_is_raw_role(desc);
}

static bool kona_icc_is_policy_suppressed_path(const struct kona_icc_node_desc *desc)
{
	/*
	 * Keep the paths that recent safety fixes intentionally made raw/no-op
	 * suppressed. CONFIG/PERIPHERAL/IPA nodes are CPU_MEM/CPU_LLCC-backed on
	 * this virtual provider, so they may use the bounded floor policy; true RAW
	 * nodes remain excluded unless they get a dedicated, validated bridge.
	 */
	return kona_icc_is_crypto_path(desc) ||
	       kona_icc_is_raw_npu_path(desc) ||
	       desc->role == KONA_ROLE_RAW ||
	       ((!kona_gpu_raw_icc_enable || kona_gpu_policy_bypass_enable) &&
		kona_icc_is_gpu_path(desc)) ||
	       ((!kona_gmu_raw_icc_enable || kona_gmu_policy_bypass_enable) &&
		kona_icc_is_gmu_path(desc));
}

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
/*
 * Adaptive bandwidth floors (in KB/s) tuned for OnePlus 8/8T/8 Pro (kona v2).
 * Keep average votes modest, then reserve high instantaneous DDR/LLCC
 * headroom only while a path carries a meaningful workload.
 */
#define KONA_CPU_DDR_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_CPU_DDR_IB_FLOOR_KB	(40000000ULL) /* 40 GB/s */
#define KONA_CPU_LLCC_AB_FLOOR_KB	(3000000ULL)  /* 3 GB/s */
#define KONA_CPU_LLCC_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
/*
 * CPU0 commonly carries foreground/scheduler work and can oscillate around
 * 1.804 GHz in short bursts. Keep its baseline lower than generic CPU floors
 * to avoid over-voting memory, then apply a targeted uplift near that corner.
 */
#define KONA_CPU0_DDR_AB_FLOOR_KB	(2500000ULL)  /* 2.5 GB/s */
#define KONA_CPU0_DDR_IB_FLOOR_KB	(36000000ULL) /* 36 GB/s */
#define KONA_CPU0_LLCC_AB_FLOOR_KB	(2000000ULL)  /* 2 GB/s */
#define KONA_CPU0_LLCC_IB_FLOOR_KB	(44000000ULL) /* 44 GB/s */
#define KONA_CPU_PRIME_DDR_AB_FLOOR_KB	(6000000ULL)  /* 6 GB/s */
#define KONA_CPU_PRIME_DDR_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
#define KONA_CPU_PRIME_LLCC_AB_FLOOR_KB	(4500000ULL)  /* 4.5 GB/s */
#define KONA_CPU_PRIME_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
#define KONA_GPU_DDR_AB_FLOOR_KB	(5000000ULL)  /* 5 GB/s */
#define KONA_GPU_DDR_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
#define KONA_GPU_LLCC_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_GPU_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
/*
 * Keep GMU floors at least as high as GPU by default.
 *
 * GMU traffic can be bursty around perf-level transitions, so keep it
 * slightly above regular GPU floors while staying around 75% of the older
 * aggressive GMU defaults. Optional BIMC pinning below can still raise this
 * for boards that prove they need it.
 */
#define KONA_GMU_DDR_AB_FLOOR_KB	(5000000ULL)  /* 5 GB/s */
#define KONA_GMU_DDR_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
#define KONA_GMU_LLCC_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_GMU_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
#define KONA_NPU_DDR_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_NPU_DDR_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
#define KONA_NPU_LLCC_AB_FLOOR_KB	(3000000ULL)  /* 3 GB/s */
#define KONA_NPU_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
#define KONA_MEDIA_DDR_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_MEDIA_DDR_IB_FLOOR_KB	(48000000ULL) /* 48 GB/s */
#define KONA_MEDIA_LLCC_AB_FLOOR_KB	(3000000ULL)  /* 3 GB/s */
#define KONA_MEDIA_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
#define KONA_UX_DDR_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_UX_DDR_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */
/*
 * Storage paths need sustained AB for sequential transfers and enough IB to
 * prevent command/data bursts from waiting on a low DDR or LLCC corner.
 */
#define KONA_STORAGE_DDR_AB_FLOOR_KB	(4000000ULL)  /* 4 GB/s */
#define KONA_STORAGE_DDR_IB_FLOOR_KB	(52000000ULL) /* 52 GB/s */
#define KONA_STORAGE_LLCC_AB_FLOOR_KB	(3000000ULL)  /* 3 GB/s */
#define KONA_STORAGE_LLCC_IB_FLOOR_KB	(56000000ULL) /* 56 GB/s */

/*
 * CPU_MEM and CPU_LLCC are shared backing BCMs for a number of auxiliary
 * clients.  Give those clients a deliberately small baseline instead of
 * treating their requests as CPU workload traffic.  This preserves quick
 * peripheral/configuration bursts without retaining a multi-GB/s average
 * vote while idle.
 */
#define KONA_AUX_DDR_AB_FLOOR_KB	(512000ULL)  /* 512 MB/s */
#define KONA_AUX_DDR_IB_FLOOR_KB	(8192000ULL) /* 8 GB/s */
#define KONA_AUX_LLCC_AB_FLOOR_KB	(384000ULL)  /* 384 MB/s */
#define KONA_AUX_LLCC_IB_FLOOR_KB	(8192000ULL) /* 8 GB/s */

/*
 * Global minimum floors for any non-zero bandwidth vote. This protects
 * against bw_hwmon / memlat (or other clients) voting extremely small
 * values that cause QoS collapse and starve CPU/GPU.
 */
#define KONA_ICC_MIN_AB_FLOOR_KB	(128000ULL)   /* 128 MB/s */
#define KONA_ICC_MIN_IB_FLOOR_KB	(256000ULL)   /* 256 MB/s */

/*
 * Downscale hysteresis: ignore tiny AB/IB drops that only create RPMh churn.
 * Larger drops and all increases are honored immediately. Values are KB/s.
 */
#define KONA_HYST_PERCENT			5      /* tolerate small 5% dips */
#define KONA_HYST_AB_STEP_KB		 100000 /* or 100 MB/s, whichever is smaller */
#define KONA_HYST_IB_STEP_KB		 150000 /* or 150 MB/s, whichever is smaller */
static unsigned int kona_max_downscale_percent = 15;
module_param(kona_max_downscale_percent, uint, 0644);
MODULE_PARM_DESC(kona_max_downscale_percent,
	"Maximum AB/IB percent drop allowed per vote for non-zero downvotes (default: 15)");

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
static unsigned int kona_perf_bias = 120;
static unsigned int kona_perf_bias_light = 112;
static unsigned int kona_perf_bias_turbo = 145;
#define KONA_PRIME_EXTRA_BIAS_PERCENT	15
static unsigned long kona_perf_light_kb = 750000;   /* 750 MB/s */
static unsigned long kona_perf_turbo_kb = 6000000;  /* 6 GB/s */
static bool kona_perf_sustain_boost_enable = true;
static unsigned long kona_perf_sustain_kb = 2000000; /* 2 GB/s */
static unsigned int kona_perf_sustain_extra_bias = 15;
module_param(kona_perf_bias, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias,
        "Percent headroom added on CPU/DDR/LLCC/GPU/NPU paths (default: 120)");
module_param(kona_perf_bias_light, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias_light,
        "Percent headroom added for light requests to save power (default: 112)");
module_param(kona_perf_bias_turbo, uint, 0644);
MODULE_PARM_DESC(kona_perf_bias_turbo,
        "Percent headroom added for very large votes (default: 145)");
module_param(kona_perf_light_kb, ulong, 0644);
MODULE_PARM_DESC(kona_perf_light_kb,
        "Threshold KB/s for light-load bias selection (default: 750000)");
module_param(kona_perf_turbo_kb, ulong, 0644);
MODULE_PARM_DESC(kona_perf_turbo_kb,
        "Threshold KB/s for turbo bias selection (default: 6000000)");
module_param(kona_perf_sustain_boost_enable, bool, 0644);
MODULE_PARM_DESC(kona_perf_sustain_boost_enable,
	"Apply extra sustained-performance headroom on heavy CPU/GPU/NPU traffic");
module_param(kona_perf_sustain_kb, ulong, 0644);
MODULE_PARM_DESC(kona_perf_sustain_kb,
	"Threshold KB/s where sustained-performance extra headroom begins");
module_param(kona_perf_sustain_extra_bias, uint, 0644);
MODULE_PARM_DESC(kona_perf_sustain_extra_bias,
	"Additional headroom percent added above sustained-performance threshold");

/*
 * GPU keep-alive floor and IB prioritization for gpu-ddr path.
 *
 * keepalive_*: keep a minimum vote between short frame gaps so BCMs stay in
 * a responsive corner instead of repeatedly collapsing to idle.
 * ib_*:        bias and floor IB over AB so command bursts hit DDR quickly.
 */
static bool kona_gpu_keepalive_enable = false;
static unsigned long kona_gpu_keepalive_ab_kb = 1000000;   /* 1 GB/s */
static unsigned long kona_gpu_keepalive_ib_kb = 2400000;  /* 2.4 GB/s */
static bool kona_cpu_keepalive_enable = false;
static unsigned long kona_cpu_keepalive_ab_kb = 512000;   /* 512 MB/s */
static unsigned long kona_cpu_keepalive_ib_kb = 1200000;   /* 1.2 GB/s */
static bool kona_npu_keepalive_enable = false;
static unsigned long kona_npu_keepalive_ab_kb = 768000;   /* 768 MB/s */
static unsigned long kona_npu_keepalive_ib_kb = 1800000;  /* 1.8 GB/s */
static bool kona_dsp_keepalive_enable = false;
static unsigned long kona_dsp_keepalive_ab_kb = 640000;   /* 640 MB/s */
static unsigned long kona_dsp_keepalive_ib_kb = 1800000;  /* 1.8 GB/s */
static bool kona_media_keepalive_enable = false;
static unsigned long kona_media_keepalive_ab_kb = 1500000;  /* 1.5 GB/s */
static unsigned long kona_media_keepalive_ib_kb = 3200000; /* 3.2 GB/s */
static bool kona_disp_keepalive_enable = true;
static unsigned long kona_disp_keepalive_ab_kb = 256000;   /* 256 MB/s */
static unsigned long kona_disp_keepalive_ib_kb = 512000;   /* 512 MB/s */
static bool kona_keepalive_decay_enable = true;
static unsigned int kona_keepalive_decay_window_ms = 300;
static unsigned int kona_keepalive_decay_min_percent = 25;
static unsigned int kona_sleep_keepalive_percent = 18;
static unsigned int kona_sleep_perf_floor_percent = 35;
static unsigned long kona_sleep_perf_floor_trigger_kb = 2500000; /* 2.5 GB/s */
static bool kona_sleep_floor_decay_enable = true;
static unsigned int kona_sleep_floor_decay_delay_ms = 30000;
static unsigned int kona_sleep_floor_decay_percent = 15;
module_param_named(kona_sleep_perf_floor_percent, kona_sleep_perf_floor_percent, uint, 0644);
MODULE_PARM_DESC(kona_sleep_perf_floor_percent,
	"Percent of performance floors kept for substantial display-inactive requests (default: 35)");
module_param_named(kona_sleep_perf_floor_trigger_kb, kona_sleep_perf_floor_trigger_kb, ulong, 0644);
MODULE_PARM_DESC(kona_sleep_perf_floor_trigger_kb,
	"Minimum display-inactive request KB/s before applying scaled performance floors");
module_param_named(kona_sleep_floor_decay_enable, kona_sleep_floor_decay_enable, bool, 0644);
MODULE_PARM_DESC(kona_sleep_floor_decay_enable,
		 "Enable deeper screen-off Kona ICC floor decay after the grace window (default: on)");
module_param_named(kona_sleep_floor_decay_delay_ms, kona_sleep_floor_decay_delay_ms, uint, 0644);
MODULE_PARM_DESC(kona_sleep_floor_decay_delay_ms,
		 "Screen-off grace period in ms before using the decayed ICC floor percent");
module_param_named(kona_sleep_floor_decay_percent, kona_sleep_floor_decay_percent, uint, 0644);
MODULE_PARM_DESC(kona_sleep_floor_decay_percent,
		 "Percent of non-display floors kept after screen-off decay (default: 15)");

/*
 * Keep active scaling enabled, but require a modest request before applying the
 * large per-path floors. Sub-trigger votes still get the tiny global minimum
 * clamp inside kona_icc_apply_floor(), avoiding both 0-corner QoS collapse and
 * multi-GB/s DDR votes for housekeeping traffic.
 */
static bool kona_active_floor_scaling_enable = true;
static unsigned long kona_active_floor_trigger_kb = 256000; /* 256 MB/s */
/*
 * Keep full floors for only genuinely bandwidth-heavy work.  The Kona and
 * Kona-v2 compatibles share this provider, so these thresholds deliberately
 * describe a conservative common denominator rather than one board's peak
 * benchmark workload.
 */
static unsigned long kona_active_floor_low_kb = 1000000;  /* 1 GB/s */
static unsigned long kona_active_floor_high_kb = 4000000; /* 4 GB/s */
static unsigned int kona_active_floor_low_percent = 60;
static unsigned int kona_active_floor_mid_percent = 85;
module_param_named(kona_active_floor_scaling_enable, kona_active_floor_scaling_enable, bool, 0644);
MODULE_PARM_DESC(kona_active_floor_scaling_enable,
	"Enable display-on workload-aware downscaling of non-display performance floors");
module_param_named(kona_active_floor_trigger_kb, kona_active_floor_trigger_kb, ulong, 0644);
MODULE_PARM_DESC(kona_active_floor_trigger_kb, "Minimum non-display request for large active floors (default: 256000 KB/s)");
module_param_named(kona_active_floor_low_kb, kona_active_floor_low_kb, ulong, 0644);
MODULE_PARM_DESC(kona_active_floor_low_kb,
	"Display-on request threshold KB/s for low floor scaling bucket");
module_param_named(kona_active_floor_high_kb, kona_active_floor_high_kb, ulong, 0644);
MODULE_PARM_DESC(kona_active_floor_high_kb,
	"Display-on request threshold KB/s for high floor scaling bucket");
module_param_named(kona_active_floor_low_percent, kona_active_floor_low_percent, uint, 0644);
MODULE_PARM_DESC(kona_active_floor_low_percent,
	"Percent of post-path floor kept for low display-on non-display workload votes (default: 60)");
module_param_named(kona_active_floor_mid_percent, kona_active_floor_mid_percent, uint, 0644);
MODULE_PARM_DESC(kona_active_floor_mid_percent,
	"Percent of post-path floor kept for medium display-on non-display workload votes (default: 85)");

static unsigned int kona_gpu_ib_boost_percent = 150;
static unsigned int kona_gpu_ib_min_ratio_percent = 240;
static unsigned int kona_gpu_llcc_boost_percent = 135;
static unsigned int kona_gpu_llcc_min_ratio_percent = 210;
static unsigned int kona_cpu_ib_boost_percent = 125;
static unsigned int kona_cpu_ddr_min_ratio_percent = 200;
static unsigned int kona_cpu_llcc_min_ratio_percent = 180;
static unsigned int kona_cpu_prime_ib_boost_percent = 135;
static unsigned int kona_cpu_prime_ddr_min_ratio_percent = 220;
static unsigned int kona_cpu_prime_llcc_min_ratio_percent = 195;
static unsigned int kona_npu_ib_boost_percent = 190;
static unsigned int kona_npu_ib_min_ratio_percent = 250;
static unsigned int kona_storage_ab_boost_percent = 150;
static unsigned int kona_storage_ib_boost_percent = 175;
static unsigned int kona_storage_ib_min_ratio_percent = 220;
static bool kona_npu_oc_mem_pinning_enable = false;
static unsigned long kona_npu_oc_pin_threshold_kb = 4000000; /* 4 GB/s */
static unsigned int kona_npu_oc_pin_exit_percent = 75;
static unsigned int kona_npu_oc_pin_hold_ms = 200;
static unsigned long kona_npu_oc_floor_ab_kb = 20000000;      /* 20 GB/s */
static unsigned long kona_npu_oc_floor_ib_kb = 38000000;      /* 38 GB/s */
static unsigned long kona_npu_oc_llcc_floor_ab_kb = 16000000; /* 16 GB/s */
static unsigned long kona_npu_oc_llcc_floor_ib_kb = 28000000; /* 28 GB/s */
static bool kona_gpu_bimc_pinning_enable = false;
static bool kona_gpu_bimc_no_hyst_enable = false;
static unsigned long kona_gpu_bimc_floor_ab_kb = 16000000; /* 16 GB/s */
static unsigned long kona_gpu_bimc_floor_ib_kb = 32000000; /* 32 GB/s */
static unsigned long kona_gpu_bimc_pin_threshold_kb = 4000000; /* 4 GB/s */
static unsigned int kona_gpu_bimc_pin_exit_percent = 70;
static unsigned int kona_gpu_bimc_pin_hold_ms = 180;
static unsigned int kona_gpu_bimc_min_ratio_percent = 220;
static bool kona_gpu_llcc_turbo_enable = false;
static unsigned long kona_gpu_llcc_turbo_enter_ib_kb = 6000000; /* 6 GB/s */
static unsigned long kona_gpu_llcc_turbo_exit_ib_kb = 4000000;  /* 4 GB/s */
static unsigned long kona_gpu_llcc_turbo_ab_kb = 12000000;       /* 12 GB/s */
static unsigned long kona_gpu_llcc_turbo_ib_kb = 24000000;       /* 24 GB/s */
static bool kona_cpu_prime_oc_mem_pinning_enable = false;
static unsigned long kona_cpu_prime_oc_pin_threshold_kb = 6000000; /* 6 GB/s */
static unsigned int kona_cpu_prime_oc_pin_exit_percent = 75;
static unsigned int kona_cpu_prime_oc_pin_hold_ms = 240;
static unsigned long kona_cpu_prime_oc_floor_ab_kb = 26000000;      /* 26 GB/s */
static unsigned long kona_cpu_prime_oc_floor_ib_kb = 43000000;      /* 43 GB/s */
static unsigned long kona_cpu_prime_oc_llcc_floor_ab_kb = 18000000; /* 18 GB/s */
static unsigned long kona_cpu_prime_oc_llcc_floor_ib_kb = 28000000; /* 28 GB/s */
static unsigned int kona_cpu_prime_oc_min_ratio_percent = 180;
static bool kona_cpu0_1804_boost_enable = true;
static unsigned long kona_cpu0_1804_trigger_kb = 1305600; /* 1.804 GHz corner */
static unsigned int kona_cpu0_1804_boost_percent = 130;
static unsigned int kona_cpu0_1804_min_ratio_percent = 190;
static bool kona_ux_turbo_enable = true;
static unsigned long kona_ux_turbo_threshold_kb = 384000;      /* 384 MB/s */
static unsigned int kona_ux_turbo_exit_percent = 45;
static unsigned int kona_ux_turbo_hold_ms = 180;
static unsigned long kona_ux_turbo_ab_kb = KONA_UX_DDR_AB_FLOOR_KB;
static unsigned long kona_ux_turbo_ib_kb = KONA_UX_DDR_IB_FLOOR_KB;
module_param(kona_gpu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_enable,
        "Keep non-zero floor for gpu-ddr AB/IB between short idle gaps");
module_param(kona_gpu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_ab_kb,
        "gpu-ddr keepalive AB floor in KB/s (default: 1000000)");
module_param(kona_gpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_keepalive_ib_kb,
        "gpu-ddr keepalive IB floor in KB/s (default: 2400000)");
module_param(kona_cpu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_enable,
        "Keep non-zero floor for cpu-ddr/cpu-llcc AB/IB between short idle gaps");
module_param(kona_cpu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ab_kb,
        "cpu keepalive AB floor in KB/s (default: 512000)");
module_param(kona_cpu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_keepalive_ib_kb,
        "cpu keepalive IB floor in KB/s (default: 1200000)");
module_param(kona_npu_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_enable,
        "Keep non-zero floor for npu-ddr/npu-llcc AB/IB between short idle gaps");
module_param(kona_npu_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ab_kb,
        "npu keepalive AB floor in KB/s (default: 768000)");
module_param(kona_npu_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_keepalive_ib_kb,
        "npu keepalive IB floor in KB/s (default: 1800000)");
module_param(kona_dsp_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_dsp_keepalive_enable,
        "Keep non-zero floor for cam/video/cdsp config AB/IB between short idle gaps");
module_param(kona_dsp_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_dsp_keepalive_ab_kb,
        "dsp keepalive AB floor in KB/s (default: 640000)");
module_param(kona_dsp_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_dsp_keepalive_ib_kb,
        "dsp keepalive IB floor in KB/s (default: 1800000)");
module_param(kona_media_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_media_keepalive_enable,
	"Keep non-zero floor for video/cvp/camera data paths between short idle gaps");
module_param(kona_media_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_media_keepalive_ab_kb,
	"media keepalive AB floor in KB/s (default: 1500000)");
module_param(kona_media_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_media_keepalive_ib_kb,
	"media keepalive IB floor in KB/s (default: 3200000)");
module_param(kona_disp_keepalive_enable, bool, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_enable,
	"Keep non-zero floor for disp0/disp1 DDR AB/IB between idle/off transitions");
module_param(kona_disp_keepalive_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_ab_kb,
	"display keepalive AB floor in KB/s (default: 256000)");
module_param(kona_disp_keepalive_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_disp_keepalive_ib_kb,
	"display keepalive IB floor in KB/s (default: 512000)");
module_param(kona_keepalive_decay_enable, bool, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_enable,
	"linearly decay keepalive votes after the last active request (default: on)");
module_param(kona_keepalive_decay_window_ms, uint, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_window_ms,
	"keepalive decay window in ms before allowing full collapse (default: 300)");
module_param(kona_keepalive_decay_min_percent, uint, 0644);
MODULE_PARM_DESC(kona_keepalive_decay_min_percent,
	"minimum keepalive strength percent while inside decay window (default: 25)");
module_param(kona_sleep_keepalive_percent, uint, 0644);
MODULE_PARM_DESC(kona_sleep_keepalive_percent,
	"Percent of keepalive floor retained while display is inactive (default: 18)");

module_param(kona_gpu_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_ib_boost_percent,
        "Percent boost applied to gpu-ddr IB after floors (default: 150)");
module_param(kona_gpu_ib_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_ib_min_ratio_percent,
        "Minimum gpu-ddr IB as percent of AB (default: 240)");
module_param(kona_gpu_llcc_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_boost_percent,
        "Percent boost applied to gpu-llcc IB after floors (default: 135)");
module_param(kona_gpu_llcc_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_min_ratio_percent,
        "Minimum gpu-llcc IB as percent of AB (default: 210)");
module_param(kona_cpu_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_ib_boost_percent,
	"Percent boost applied to CPU DDR/LLCC IB after floors (default: 125)");
module_param(kona_cpu_ddr_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_ddr_min_ratio_percent,
	"Minimum CPU DDR IB as percent of AB (default: 200)");
module_param(kona_cpu_llcc_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_llcc_min_ratio_percent,
	"Minimum CPU LLCC IB as percent of AB (default: 180)");
module_param(kona_cpu_prime_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_ib_boost_percent,
	"Percent boost applied to prime CPU DDR/LLCC IB after floors (default: 135)");
module_param(kona_cpu_prime_ddr_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_ddr_min_ratio_percent,
	"Minimum prime CPU DDR IB as percent of AB (default: 220)");
module_param(kona_cpu_prime_llcc_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_llcc_min_ratio_percent,
	"Minimum prime CPU LLCC IB as percent of AB (default: 195)");
module_param(kona_npu_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_npu_ib_boost_percent,
	"Percent boost applied to npu-ddr/npu-llcc IB after floors (default: 190)");
module_param(kona_npu_ib_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_npu_ib_min_ratio_percent,
	"Minimum npu-ddr/npu-llcc IB as percent of AB (default: 250)");
module_param(kona_storage_ab_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_storage_ab_boost_percent,
		 "Percent boost applied to storage AB for sustained sequential throughput (default: 150)");
module_param(kona_storage_ib_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_storage_ib_boost_percent,
		 "Percent boost applied to storage IB for read/write bursts (default: 175)");
module_param(kona_storage_ib_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_storage_ib_min_ratio_percent,
		 "Minimum storage IB as percent of AB (default: 220)");
module_param(kona_npu_oc_mem_pinning_enable, bool, 0644);
MODULE_PARM_DESC(kona_npu_oc_mem_pinning_enable,
	"Pin elevated NPU AB/IB floors when sustained high NPU bandwidth suggests overclocked operation (default: off)");
module_param(kona_npu_oc_pin_threshold_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_oc_pin_threshold_kb,
	"Enable NPU OC pinning above this max(AB,IB) threshold in KB/s");
module_param(kona_npu_oc_pin_exit_percent, uint, 0644);
MODULE_PARM_DESC(kona_npu_oc_pin_exit_percent,
	"Exit threshold as percent of enter threshold for NPU OC pinning (default: 75)");
module_param(kona_npu_oc_pin_hold_ms, uint, 0644);
MODULE_PARM_DESC(kona_npu_oc_pin_hold_ms,
	"Hold NPU OC pinning for N ms after threshold crossing (default: 200)");
module_param(kona_npu_oc_floor_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_oc_floor_ab_kb,
	"Pinned npu-ddr AB floor in KB/s while NPU OC pinning is active");
module_param(kona_npu_oc_floor_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_oc_floor_ib_kb,
	"Pinned npu-ddr IB floor in KB/s while NPU OC pinning is active");
module_param(kona_npu_oc_llcc_floor_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_oc_llcc_floor_ab_kb,
	"Pinned npu-llcc AB floor in KB/s while NPU OC pinning is active");
module_param(kona_npu_oc_llcc_floor_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_npu_oc_llcc_floor_ib_kb,
	"Pinned npu-llcc IB floor in KB/s while NPU OC pinning is active");
module_param(kona_gpu_bimc_pinning_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_pinning_enable,
        "Force aggressive gpu-ddr BIMC AB/IB floor to avoid starvation at high GPU clocks (default: off)");
module_param(kona_gpu_bimc_no_hyst_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_no_hyst_enable,
        "Disable downvote hysteresis for gpu-ddr votes to remove ramp-down/ramp-up lag");
module_param(kona_gpu_bimc_floor_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_floor_ab_kb,
        "Pinned gpu-ddr BIMC AB floor in KB/s (default: 16000000)");
module_param(kona_gpu_bimc_floor_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_floor_ib_kb,
        "Pinned gpu-ddr BIMC IB floor in KB/s (default: 32000000)");
module_param(kona_gpu_bimc_pin_threshold_kb, ulong, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_pin_threshold_kb,
        "Only enable gpu-ddr BIMC pinning above this KB/s request threshold");
module_param(kona_gpu_bimc_pin_exit_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_pin_exit_percent,
	"Exit threshold as percent of enter threshold for GPU BIMC pinning (default: 70)");
module_param(kona_gpu_bimc_pin_hold_ms, uint, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_pin_hold_ms,
	"Hold GPU BIMC pinning for N ms after threshold crossing (default: 180)");
module_param(kona_gpu_bimc_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_gpu_bimc_min_ratio_percent,
        "Minimum gpu-ddr BIMC IB as percent of AB when pinning is active (default: 220)");
module_param(kona_gpu_llcc_turbo_enable, bool, 0644);
MODULE_PARM_DESC(kona_gpu_llcc_turbo_enable,
        "Optionally force a high gpu-llcc vote while gpu-ddr IB remains above threshold (default: off)");
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
module_param(kona_cpu_prime_oc_mem_pinning_enable, bool, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_mem_pinning_enable,
	"Pin higher CPU-prime DDR/LLCC floors when prime-core bandwidth indicates overclocked operation (default: off)");
module_param(kona_cpu_prime_oc_pin_threshold_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_pin_threshold_kb,
	"Enable CPU-prime OC pinning above this max(AB,IB) threshold in KB/s");
module_param(kona_cpu_prime_oc_pin_exit_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_pin_exit_percent,
	"Exit threshold as percent of enter threshold for CPU-prime OC pinning (default: 75)");
module_param(kona_cpu_prime_oc_pin_hold_ms, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_pin_hold_ms,
	"Hold CPU-prime OC pinning for N ms after threshold crossing (default: 240)");
module_param(kona_cpu_prime_oc_floor_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_floor_ab_kb,
	"Pinned cpu7-ddr AB floor in KB/s while CPU-prime OC pinning is active");
module_param(kona_cpu_prime_oc_floor_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_floor_ib_kb,
	"Pinned cpu7-ddr IB floor in KB/s while CPU-prime OC pinning is active");
module_param(kona_cpu_prime_oc_llcc_floor_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_llcc_floor_ab_kb,
	"Pinned cpu7-llcc AB floor in KB/s while CPU-prime OC pinning is active");
module_param(kona_cpu_prime_oc_llcc_floor_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_llcc_floor_ib_kb,
	"Pinned cpu7-llcc IB floor in KB/s while CPU-prime OC pinning is active");
module_param(kona_cpu_prime_oc_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu_prime_oc_min_ratio_percent,
	"Minimum CPU-prime IB as percent of AB while CPU-prime OC pinning is active");
module_param(kona_cpu0_1804_boost_enable, bool, 0644);
MODULE_PARM_DESC(kona_cpu0_1804_boost_enable,
	"Enable targeted CPU0 memory uplift around the 1.804 GHz BW corner");
module_param(kona_cpu0_1804_trigger_kb, ulong, 0644);
MODULE_PARM_DESC(kona_cpu0_1804_trigger_kb,
	"CPU0 BW trigger in KB/s that represents the 1.804 GHz operating corner");
module_param(kona_cpu0_1804_boost_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu0_1804_boost_percent,
	"Percent boost applied to CPU0 IB once the 1.804 GHz trigger is reached");
module_param(kona_cpu0_1804_min_ratio_percent, uint, 0644);
MODULE_PARM_DESC(kona_cpu0_1804_min_ratio_percent,
	"Minimum CPU0 IB as percent of AB once the 1.804 GHz trigger is reached");
module_param(kona_ux_turbo_enable, bool, 0644);
MODULE_PARM_DESC(kona_ux_turbo_enable,
	"Enable automatic UX bandwidth floor for app/screen transitions");
module_param(kona_ux_turbo_threshold_kb, ulong, 0644);
MODULE_PARM_DESC(kona_ux_turbo_threshold_kb,
	"UX request threshold in KB/s that enables the transition turbo floor");
module_param(kona_ux_turbo_exit_percent, uint, 0644);
MODULE_PARM_DESC(kona_ux_turbo_exit_percent,
	"Exit threshold as percent of enter threshold for UX turbo pinning");
module_param(kona_ux_turbo_hold_ms, uint, 0644);
MODULE_PARM_DESC(kona_ux_turbo_hold_ms,
	"Hold UX turbo floor for N ms after a qualifying transition request");
module_param(kona_ux_turbo_ab_kb, ulong, 0644);
MODULE_PARM_DESC(kona_ux_turbo_ab_kb,
	"Pinned UX AB floor in KB/s while transition turbo is active (default: 4000000)");
module_param(kona_ux_turbo_ib_kb, ulong, 0644);
MODULE_PARM_DESC(kona_ux_turbo_ib_kb,
	"Pinned UX IB floor in KB/s while transition turbo is active (default: 56000000)");

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

static inline bool kona_icc_display_runtime_active(struct kona_icc_provider *qp)
{
	/*
	 * Display-derived floors are only safe when the DRM notifier is live.
	 * If notifier registration failed, keep the ICC provider available for
	 * CPU/GPU/NPU clients but avoid applying stale display-state hints.
	 */
	return READ_ONCE(qp->display_hints_available) &&
		READ_ONCE(qp->display_active) &&
		!READ_ONCE(qp->system_suspended);
}

static u64 kona_icc_add_headroom(u64 value, unsigned int bias)
{
        /* Avoid overflow when adding headroom; values are already in KBps. */
        return mul_u64_u32_div(value, bias, 100);
}

static void kona_icc_apply_cpu_ib_headroom(bool prime, bool llcc,
						 u64 *ab, u64 *ib)
{
	unsigned int boost = prime ? kona_cpu_prime_ib_boost_percent :
					 kona_cpu_ib_boost_percent;
	unsigned int min_ratio = prime ?
		(llcc ? kona_cpu_prime_llcc_min_ratio_percent :
			kona_cpu_prime_ddr_min_ratio_percent) :
		(llcc ? kona_cpu_llcc_min_ratio_percent :
			kona_cpu_ddr_min_ratio_percent);

	if (*ib)
		*ib = kona_icc_add_headroom(*ib, boost);
	if (*ab && *ib < mul_u64_u32_div(*ab, min_ratio, 100))
		*ib = mul_u64_u32_div(*ab, min_ratio, 100);
}

static inline bool kona_icc_sleep_mode_active(struct kona_icc_provider *qp,
				      const struct kona_icc_node_desc *desc)
{
	if (desc->role == KONA_ROLE_DISPLAY)
		return false;

	/*
	 * Without a registered display notifier we cannot trust display_active,
	 * so do not enter display-inactive floor/decay policy from stale state.
	 */
	if (!READ_ONCE(qp->display_hints_available))
		return false;

	return !READ_ONCE(qp->display_active);
}

static unsigned int kona_icc_sleep_floor_percent(struct kona_icc_provider *qp)
{
	unsigned int floor_percent;

	floor_percent = min_t(unsigned int, kona_sleep_perf_floor_percent, 100);
	if (!kona_sleep_floor_decay_enable ||
	    !kona_sleep_floor_decay_delay_ms ||
	    !READ_ONCE(qp->display_off_jiffies))
		return floor_percent;

	if (time_before(jiffies, READ_ONCE(qp->display_off_jiffies) +
			msecs_to_jiffies(kona_sleep_floor_decay_delay_ms)))
		return floor_percent;

	return min(floor_percent,
		   min_t(unsigned int, kona_sleep_floor_decay_percent, 100));
}

static bool kona_icc_apply_keepalive_vote(struct kona_icc_provider *qp,
					 unsigned int index, u64 *ab, u64 *ib)
{
	bool keepalive = false;
	bool display_inactive;
	const struct kona_icc_node_desc *desc;
	u64 keepalive_ab = 0, keepalive_ib = 0;
	unsigned int decay_percent = 100;

	if (!qp->last_ab || !qp->last_ib || *ab || *ib)
		return false;

	desc = &qp->nodes[index];

	if (desc->role == KONA_ROLE_DISPLAY &&
	    !kona_icc_display_runtime_active(qp))
		return false;

	if (qp->last_ab[index] == U64_MAX || qp->last_ib[index] == U64_MAX)
		return false;

	if (!qp->last_ab[index] && !qp->last_ib[index])
		return false;

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
	case KONA_ROLE_DSP:
		if (!kona_dsp_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_dsp_keepalive_ab_kb;
		keepalive_ib = kona_dsp_keepalive_ib_kb;
		break;
	case KONA_ROLE_MEDIA:
		if (!kona_media_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_media_keepalive_ab_kb;
		keepalive_ib = kona_media_keepalive_ib_kb;
		break;
	case KONA_ROLE_DISPLAY:
		if (!kona_disp_keepalive_enable)
			break;
		keepalive = true;
		keepalive_ab = kona_disp_keepalive_ab_kb;
		keepalive_ib = kona_disp_keepalive_ib_kb;
		break;
	case KONA_ROLE_GMU:
	case KONA_ROLE_STORAGE:
	case KONA_ROLE_IPA:
	case KONA_ROLE_PERIPHERAL:
	case KONA_ROLE_CONFIG:
	case KONA_ROLE_RAW:
	default:
		break;
	}

	if (!keepalive)
		return false;

	display_inactive = !READ_ONCE(qp->display_active);
	if (display_inactive && desc->role != KONA_ROLE_DISPLAY) {
		unsigned int sleep_percent;

		sleep_percent = min_t(unsigned int, kona_sleep_keepalive_percent, 100);
		if (!sleep_percent)
			return false;

		keepalive_ab = mul_u64_u32_div(keepalive_ab, sleep_percent, 100);
		keepalive_ib = mul_u64_u32_div(keepalive_ib, sleep_percent, 100);

		if (!keepalive_ab && !keepalive_ib)
			return false;
	}

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
        case KONA_ROLE_DSP:
        case KONA_ROLE_MEDIA:
                if (vote >= kona_perf_turbo_kb)
                        bias = kona_perf_bias_turbo;
                else if (vote <= kona_perf_light_kb)
                        bias = kona_perf_bias_light;
                else
                        bias = kona_perf_bias;
                break;
        case KONA_ROLE_CPU_PRIME:
                if (vote >= kona_perf_turbo_kb)
                        bias = kona_perf_bias_turbo;
                else if (vote <= kona_perf_light_kb)
                        bias = kona_perf_bias_light;
                else
                        bias = kona_perf_bias;
                bias = min_t(unsigned int, bias + KONA_PRIME_EXTRA_BIAS_PERCENT, 200);
                break;
        case KONA_ROLE_DISPLAY:
        default:
                return 100;
        }

	if (kona_perf_sustain_boost_enable && vote >= kona_perf_sustain_kb &&
	    desc->role != KONA_ROLE_DISPLAY)
		bias = min_t(unsigned int, bias + kona_perf_sustain_extra_bias, 200);

	return bias;
}

static void kona_icc_apply_hysteresis(struct kona_icc_provider *qp,
			     const struct kona_icc_node_desc *desc,
			     unsigned int index, u64 *ab, u64 *ib)
{
	u64 prev_ab, prev_ib, ab_drop, ib_drop, ab_win, ib_win;
	u64 min_next_ab, min_next_ib;
	unsigned int max_drop;

	if (!qp->last_ab || !qp->last_ib)
		return;

	prev_ab = qp->last_ab[index];
	prev_ib = qp->last_ib[index];
	/*
	 * last_* starts at U64_MAX so the first real vote is always sent.  It is
	 * a cache sentinel, not bandwidth: feeding it to the maximum-downscale
	 * calculation produces 85% of U64_MAX (0xd999999999999998 with the
	 * default 15% limit) and overwrites both otherwise valid votes.
	 */
	if (prev_ab == U64_MAX || prev_ib == U64_MAX)
		return;

	if (kona_gpu_bimc_no_hyst_enable &&
	    (desc->id == KONA_ICC_GPU_TO_MEM || desc->id == KONA_ICC_GMU_TO_MEM))
		return;

	/*
	 * When the display is off, prioritize fast collapse over smoothness so
	 * idle drain stays low during screen-off standby.
	 */
	if (kona_icc_sleep_mode_active(qp, desc))
		return;

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

	/*
	 * Multi-client paths can emit very large non-zero downvotes in a single
	 * update, then bounce back on the next frame/tick. Limit per-update
	 * downscale to smooth node behavior and reduce RPMh corner thrash.
	 *
	 * 0/0 votes remain untouched so idle collapse still works.
	 */
	max_drop = min_t(unsigned int, kona_max_downscale_percent, 95);
	if (max_drop) {
		if (prev_ab && *ab && *ab < prev_ab) {
			min_next_ab = mul_u64_u32_div(prev_ab, 100 - max_drop, 100);
			if (*ab < min_next_ab)
				*ab = min_next_ab;
		}

		if (prev_ib && *ib && *ib < prev_ib) {
			min_next_ib = mul_u64_u32_div(prev_ib, 100 - max_drop, 100);
			if (*ib < min_next_ib)
				*ib = min_next_ib;
		}
	}

	pr_debug("kona-icc: hysteresis %s prev ab/ib=%llu/%llu new=%llu/%llu\n",
		 desc->name, prev_ab, prev_ib, *ab, *ib);
}

static bool kona_icc_pin_latched(bool enabled, u64 req_max_kb,
				 unsigned long enter_kb,
				 unsigned int exit_percent,
				 unsigned int hold_ms,
				 unsigned long *last_jiffies)
{
	unsigned long hold_jiffies, exit_kb;

	if (!enabled || !enter_kb)
		return false;

	exit_percent = clamp_val(exit_percent, 1, 100);
	exit_kb = mul_u64_u32_div(enter_kb, exit_percent, 100);
	hold_jiffies = msecs_to_jiffies(hold_ms);

	if (req_max_kb >= enter_kb) {
		*last_jiffies = jiffies;
		return true;
	}

	if (*last_jiffies &&
	    req_max_kb >= exit_kb &&
	    time_before(jiffies, *last_jiffies + hold_jiffies)) {
		*last_jiffies = jiffies;
		return true;
	}

	if (*last_jiffies &&
	    time_before(jiffies, *last_jiffies + hold_jiffies))
		return true;

	return false;
}

static bool kona_icc_is_ux_path(const struct kona_icc_node_desc *desc)
{
	/*
	 * Limit UX turbo to display/SDE register traffic. DISP0/1 memory paths
	 * already carry real bandwidth requests and should not receive synthetic
	 * multi-GB/s boosts while GPU or storage clients are active.
	 */
	return desc->id == KONA_ICC_DISP_CFG;
}

static void kona_icc_apply_ux_turbo(struct kona_icc_provider *qp,
				   const struct kona_icc_node_desc *desc,
				   u64 req_max, u64 *ab, u64 *ib)
{
	if (!kona_icc_is_ux_path(desc))
		return;

	if (!kona_icc_pin_latched(kona_ux_turbo_enable, req_max,
				 kona_ux_turbo_threshold_kb,
				 kona_ux_turbo_exit_percent,
				 kona_ux_turbo_hold_ms,
				 &qp->ux_turbo_last_jiffies))
		return;

	if (*ab < kona_ux_turbo_ab_kb)
		*ab = kona_ux_turbo_ab_kb;
	if (*ib < kona_ux_turbo_ib_kb)
		*ib = kona_ux_turbo_ib_kb;
}

static bool kona_icc_is_aux_llcc_path(const struct kona_icc_node_desc *desc)
{
	return desc->id == KONA_ICC_IPA_TO_LLCC;
}

static bool kona_icc_has_aux_cpu_backing(const struct kona_icc_node_desc *desc)
{
	switch (desc->role) {
	case KONA_ROLE_CONFIG:
	case KONA_ROLE_PERIPHERAL:
	case KONA_ROLE_IPA:
		return true;
	case KONA_ROLE_MEDIA:
		/* Video paths have dedicated floors; camera/config paths use CPU BCMs. */
		return desc->id != KONA_ICC_VIDEO_TO_MEM &&
		       desc->id != KONA_ICC_VIDEO_TO_LLCC;
	default:
		return false;
	}
}

static void kona_icc_apply_aux_baseline(const struct kona_icc_node_desc *desc,
					u64 *ab, u64 *ib)
{
	if (kona_icc_is_aux_llcc_path(desc)) {
		*ab = max_t(u64, *ab, KONA_AUX_LLCC_AB_FLOOR_KB);
		*ib = max_t(u64, *ib, KONA_AUX_LLCC_IB_FLOOR_KB);
	} else {
		*ab = max_t(u64, *ab, KONA_AUX_DDR_AB_FLOOR_KB);
		*ib = max_t(u64, *ib, KONA_AUX_DDR_IB_FLOOR_KB);
	}
}

static void __maybe_unused
kona_icc_calculate_floor(struct kona_icc_provider *qp,
			 const struct kona_icc_node_desc *desc,
			 u64 req_ab, u64 req_ib, u64 *ab, u64 *ib)
{
	bool active;
	bool sleep_mode;
	bool sleep_floor_active;
	bool active_floor_active;
	bool gpu_pin_active;
	bool npu_pin_active;
	bool cpu_prime_pin_active;
	u64 req_max;
	unsigned int active_scale_percent = 100;

	if (!kona_perf_floor_enable)
		return;

	/*
	 * Keep low-bandwidth peripheral paths (e.g. RNG) unmodified so
	 * small functional votes do not get inflated by performance floors.
	 */
	if (desc->id == KONA_ICC_CPU_TO_PRNG)
		return;

	/*
	 * Treat votes with either AB or IB as active.
	 *
	 * Many legacy clients still send IB-only requests (AB=0, IB>0). If we
	 * only clamp non-zero AB, those paths can stay effectively AB-starved
	 * and DDR corners do not rise enough under load.
	 */
	req_max = max(req_ab, req_ib);
	sleep_mode = kona_icc_sleep_mode_active(qp, desc);
	sleep_floor_active = sleep_mode && kona_sleep_perf_floor_trigger_kb &&
			     req_max >= kona_sleep_perf_floor_trigger_kb;
	active_floor_active = !kona_active_floor_trigger_kb ||
			      desc->role == KONA_ROLE_DISPLAY ||
			      req_max >= kona_active_floor_trigger_kb;
	active = (*ab || *ib) && (!sleep_mode || sleep_floor_active) &&
		 active_floor_active;

	/*
	 * Only pin GPU/GMU BIMC floors for heavy GPU load. This avoids paying
	 * peak floor cost for every non-zero vote.
	 */
	gpu_pin_active = kona_icc_pin_latched(kona_gpu_bimc_pinning_enable,
		req_max, kona_gpu_bimc_pin_threshold_kb,
		kona_gpu_bimc_pin_exit_percent, kona_gpu_bimc_pin_hold_ms,
		&qp->gpu_oc_last_jiffies);
	npu_pin_active = kona_icc_pin_latched(kona_npu_oc_mem_pinning_enable,
		req_max, kona_npu_oc_pin_threshold_kb,
		kona_npu_oc_pin_exit_percent, kona_npu_oc_pin_hold_ms,
		&qp->npu_oc_last_jiffies);
	cpu_prime_pin_active = kona_icc_pin_latched(kona_cpu_prime_oc_mem_pinning_enable,
		req_max, kona_cpu_prime_oc_pin_threshold_kb,
		kona_cpu_prime_oc_pin_exit_percent, kona_cpu_prime_oc_pin_hold_ms,
		&qp->cpu_prime_oc_last_jiffies);

	switch (desc->id) {
	case KONA_ICC_CPU0_TO_MEM:
		if (active && *ab < KONA_CPU0_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU0_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU0_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU0_DDR_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(false, false, ab, ib);

		if (active && kona_cpu0_1804_boost_enable &&
		    req_max >= kona_cpu0_1804_trigger_kb) {
			if (*ib)
				*ib = kona_icc_add_headroom(*ib, kona_cpu0_1804_boost_percent);
			if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_cpu0_1804_min_ratio_percent, 100))
				*ib = mul_u64_u32_div(*ab,
						kona_cpu0_1804_min_ratio_percent, 100);
		}
		break;
	case KONA_ICC_CPU0_TO_LLCC:
		if (active && *ab < KONA_CPU0_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU0_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU0_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU0_LLCC_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(false, true, ab, ib);

		if (active && kona_cpu0_1804_boost_enable &&
		    req_max >= kona_cpu0_1804_trigger_kb) {
			if (*ib)
				*ib = kona_icc_add_headroom(*ib, kona_cpu0_1804_boost_percent);
			if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_cpu0_1804_min_ratio_percent, 100))
				*ib = mul_u64_u32_div(*ab,
						kona_cpu0_1804_min_ratio_percent, 100);
		}
		break;
	case KONA_ICC_CPU_TO_MEM:
	case KONA_ICC_CPU1_TO_MEM:
	case KONA_ICC_CPU2_TO_MEM:
	case KONA_ICC_CPU3_TO_MEM:
	case KONA_ICC_CPU4_TO_MEM:
	case KONA_ICC_CPU5_TO_MEM:
	case KONA_ICC_CPU6_TO_MEM:
		if (active && *ab < KONA_CPU_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU_DDR_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(false, false, ab, ib);
		break;
	case KONA_ICC_VIDEO_TO_MEM:
		if (active && *ab < KONA_MEDIA_DDR_AB_FLOOR_KB)
			*ab = KONA_MEDIA_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_MEDIA_DDR_IB_FLOOR_KB)
			*ib = KONA_MEDIA_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_UFS_TO_MEM:
	case KONA_ICC_SDHC2_TO_MEM:
	case KONA_ICC_UFS_TO_LLCC:
		if (desc->id == KONA_ICC_UFS_TO_LLCC) {
			if (active && *ab < KONA_STORAGE_LLCC_AB_FLOOR_KB)
				*ab = KONA_STORAGE_LLCC_AB_FLOOR_KB;
			if (active && *ib < KONA_STORAGE_LLCC_IB_FLOOR_KB)
				*ib = KONA_STORAGE_LLCC_IB_FLOOR_KB;
		} else {
			if (active && *ab < KONA_STORAGE_DDR_AB_FLOOR_KB)
				*ab = KONA_STORAGE_DDR_AB_FLOOR_KB;
			if (active && *ib < KONA_STORAGE_DDR_IB_FLOOR_KB)
				*ib = KONA_STORAGE_DDR_IB_FLOOR_KB;
		}
		if (active && *ab)
			*ab = kona_icc_add_headroom(*ab, kona_storage_ab_boost_percent);
		if (active && *ib)
			*ib = kona_icc_add_headroom(*ib, kona_storage_ib_boost_percent);
		if (active && *ab && *ib < mul_u64_u32_div(*ab,
							kona_storage_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab, kona_storage_ib_min_ratio_percent, 100);
		break;
	case KONA_ICC_VIDEO_CFG:
		if (active && *ab < KONA_MEDIA_DDR_AB_FLOOR_KB)
			*ab = KONA_MEDIA_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_MEDIA_DDR_IB_FLOOR_KB)
			*ib = KONA_MEDIA_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_DISP_CFG:
		/*
		 * Do not turn small display register votes into UX-turbo DDR votes.
		 * A modest non-zero cfg floor is enough to avoid SDE/dispcc collapse
		 * without stacking multi-GB/s display traffic on top of GPU votes.
		 */
		if (active && *ab < kona_display_cfg_nonzero_floor_ab_kBps)
			*ab = kona_display_cfg_nonzero_floor_ab_kBps;
		if (active && *ib < kona_display_cfg_nonzero_floor_ib_kBps)
			*ib = kona_display_cfg_nonzero_floor_ib_kBps;
		break;
	case KONA_ICC_CPU_TO_LLCC:
	case KONA_ICC_CPU1_TO_LLCC:
	case KONA_ICC_CPU2_TO_LLCC:
	case KONA_ICC_CPU3_TO_LLCC:
	case KONA_ICC_CPU4_TO_LLCC:
	case KONA_ICC_CPU5_TO_LLCC:
	case KONA_ICC_CPU6_TO_LLCC:
		if (active && *ab < KONA_CPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU_LLCC_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(false, true, ab, ib);
		break;
	case KONA_ICC_DISP0_TO_MEM:
	case KONA_ICC_DISP1_TO_MEM:
		/*
		 * Preserve real display bandwidth requests. The dedicated 0/0 fallback
		 * and resume hold below cover panel bring-up; applying UX DDR floors here
		 * can stack badly with concurrent GPU workloads such as Antutu.
		 */
		break;
	case KONA_ICC_CPU7_TO_MEM:
		if (cpu_prime_pin_active) {
			if (active && *ab < kona_cpu_prime_oc_floor_ab_kb)
				*ab = kona_cpu_prime_oc_floor_ab_kb;
			if (active && *ib < kona_cpu_prime_oc_floor_ib_kb)
				*ib = kona_cpu_prime_oc_floor_ib_kb;
		}
		if (active && *ab < KONA_CPU_PRIME_DDR_AB_FLOOR_KB)
			*ab = KONA_CPU_PRIME_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU_PRIME_DDR_IB_FLOOR_KB)
			*ib = KONA_CPU_PRIME_DDR_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(true, false, ab, ib);
		if (cpu_prime_pin_active && *ab && *ib <
		    mul_u64_u32_div(*ab, kona_cpu_prime_oc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_cpu_prime_oc_min_ratio_percent, 100);
		break;
	case KONA_ICC_CPU7_TO_LLCC:
		if (cpu_prime_pin_active) {
			if (active && *ab < kona_cpu_prime_oc_llcc_floor_ab_kb)
				*ab = kona_cpu_prime_oc_llcc_floor_ab_kb;
			if (active && *ib < kona_cpu_prime_oc_llcc_floor_ib_kb)
				*ib = kona_cpu_prime_oc_llcc_floor_ib_kb;
		}
		if (active && *ab < KONA_CPU_PRIME_LLCC_AB_FLOOR_KB)
			*ab = KONA_CPU_PRIME_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_CPU_PRIME_LLCC_IB_FLOOR_KB)
			*ib = KONA_CPU_PRIME_LLCC_IB_FLOOR_KB;
		if (active)
			kona_icc_apply_cpu_ib_headroom(true, true, ab, ib);
		if (cpu_prime_pin_active && *ab && *ib <
		    mul_u64_u32_div(*ab, kona_cpu_prime_oc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_cpu_prime_oc_min_ratio_percent, 100);
		break;
	case KONA_ICC_NPU_TO_MEM:
	case KONA_ICC_NPUDSP_TO_MEM:
	case KONA_ICC_CVP_TO_MEM:
		if (npu_pin_active) {
			if (active && *ab < kona_npu_oc_floor_ab_kb)
				*ab = kona_npu_oc_floor_ab_kb;
			if (active && *ib < kona_npu_oc_floor_ib_kb)
				*ib = kona_npu_oc_floor_ib_kb;
		}

		if (active && *ab < KONA_NPU_DDR_AB_FLOOR_KB)
			*ab = KONA_NPU_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_NPU_DDR_IB_FLOOR_KB)
			*ib = KONA_NPU_DDR_IB_FLOOR_KB;

		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_npu_ib_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_npu_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_npu_ib_min_ratio_percent, 100);
		break;
	case KONA_ICC_PAS_TO_MEM:
		/* PAS/CDSP bring-up must not start from an effective 0/0 vote. */
		if (*ab < KONA_NPU_DDR_AB_FLOOR_KB)
			*ab = KONA_NPU_DDR_AB_FLOOR_KB;
		if (*ib < KONA_NPU_DDR_IB_FLOOR_KB)
			*ib = KONA_NPU_DDR_IB_FLOOR_KB;
		break;
	case KONA_ICC_NPU_TO_LLCC:
		if (npu_pin_active) {
			if (active && *ab < kona_npu_oc_llcc_floor_ab_kb)
				*ab = kona_npu_oc_llcc_floor_ab_kb;
			if (active && *ib < kona_npu_oc_llcc_floor_ib_kb)
				*ib = kona_npu_oc_llcc_floor_ib_kb;
		}

		if (active && *ab < KONA_NPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_NPU_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_NPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_NPU_LLCC_IB_FLOOR_KB;

		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_npu_ib_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_npu_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_npu_ib_min_ratio_percent, 100);
		break;
	case KONA_ICC_VIDEO_TO_LLCC:
		if (active && *ab < KONA_MEDIA_LLCC_AB_FLOOR_KB)
			*ab = KONA_MEDIA_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_MEDIA_LLCC_IB_FLOOR_KB)
			*ib = KONA_MEDIA_LLCC_IB_FLOOR_KB;
		break;
	case KONA_ICC_GPU_TO_MEM:
		if (gpu_pin_active) {
			if (active && *ab < kona_gpu_bimc_floor_ab_kb)
				*ab = kona_gpu_bimc_floor_ab_kb;
			if (active && *ib < kona_gpu_bimc_floor_ib_kb)
				*ib = kona_gpu_bimc_floor_ib_kb;
		}

		if (active && *ab < KONA_GPU_DDR_AB_FLOOR_KB)
			*ab = KONA_GPU_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_GPU_DDR_IB_FLOOR_KB)
			*ib = KONA_GPU_DDR_IB_FLOOR_KB;

		/* Prioritize burst bandwidth for GPU->DDR traffic. */
		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_gpu_ib_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_gpu_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_ib_min_ratio_percent, 100);

		if (gpu_pin_active && *ab && *ib <
		    mul_u64_u32_div(*ab, kona_gpu_bimc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_bimc_min_ratio_percent, 100);
		break;
	case KONA_ICC_GMU_TO_MEM:
		/*
		 * GMU follows GPU QoS policy; keep the same or higher floor so GMU
		 * transitions never under-vote compared to regular GPU traffic.
		 */
		if (gpu_pin_active) {
			if (active && *ab < kona_gpu_bimc_floor_ab_kb)
				*ab = kona_gpu_bimc_floor_ab_kb;
			if (active && *ib < kona_gpu_bimc_floor_ib_kb)
				*ib = kona_gpu_bimc_floor_ib_kb;
		}

		if (active && *ab < KONA_GMU_DDR_AB_FLOOR_KB)
			*ab = KONA_GMU_DDR_AB_FLOOR_KB;
		if (active && *ib < KONA_GMU_DDR_IB_FLOOR_KB)
			*ib = KONA_GMU_DDR_IB_FLOOR_KB;

		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_gpu_ib_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_gpu_ib_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_ib_min_ratio_percent, 100);

		if (gpu_pin_active && *ab && *ib <
		    mul_u64_u32_div(*ab, kona_gpu_bimc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_bimc_min_ratio_percent, 100);
		break;
	case KONA_ICC_GPU_TO_LLCC:
		if (active && *ab < KONA_GPU_LLCC_AB_FLOOR_KB)
			*ab = KONA_GPU_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_GPU_LLCC_IB_FLOOR_KB)
			*ib = KONA_GPU_LLCC_IB_FLOOR_KB;

		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_gpu_llcc_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_gpu_llcc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_llcc_min_ratio_percent, 100);
		break;
	case KONA_ICC_GMU_TO_LLCC:
		if (active && *ab < KONA_GMU_LLCC_AB_FLOOR_KB)
			*ab = KONA_GMU_LLCC_AB_FLOOR_KB;
		if (active && *ib < KONA_GMU_LLCC_IB_FLOOR_KB)
			*ib = KONA_GMU_LLCC_IB_FLOOR_KB;

		if (*ib)
			*ib = kona_icc_add_headroom(*ib, kona_gpu_llcc_boost_percent);
		if (*ab && *ib < mul_u64_u32_div(*ab,
						kona_gpu_llcc_min_ratio_percent, 100))
			*ib = mul_u64_u32_div(*ab,
				     kona_gpu_llcc_min_ratio_percent, 100);
		break;
	default:
		if (active && kona_icc_has_aux_cpu_backing(desc))
			kona_icc_apply_aux_baseline(desc, ab, ib);
		break;
	}

	kona_icc_apply_ux_turbo(qp, desc, req_max, ab, ib);

	/*
	 * Display-on active scaling for non-display paths:
	 * - sub-trigger request: skip large performance floors entirely
	 * - low request: retain a responsive floor (default 35%)
	 * - medium request: retain most of the floor (default 65%)
	 * - high request: keep full floors (100%)
	 *
	 * GPU pinning is exempt from this scaling because the pin only latches
	 * once a real GPU request is already large enough to risk top-clock
	 * starvation; shrinking that vote would defeat the anti-starvation path.
	 *
	 * Classify on raw client request max(req_ab, req_ib), not post-floor vote.
	 * Apply after path-specific floors and before minimum clamps/bias. Scale
	 * only synthetic headroom: never reduce an explicit consumer request.
	 */
	if (kona_active_floor_scaling_enable && active_floor_active &&
	    !kona_icc_sleep_mode_active(qp, desc) &&
	    desc->role != KONA_ROLE_DISPLAY &&
	    !(desc->role == KONA_ROLE_GPU && gpu_pin_active)) {
		unsigned long low_kb = kona_active_floor_low_kb;
		unsigned long high_kb = max(kona_active_floor_high_kb, low_kb);
		unsigned int low_pct = clamp_val(kona_active_floor_low_percent, 0, 100);
		unsigned int mid_pct = clamp_val(kona_active_floor_mid_percent, 0, 100);

		if (req_max <= low_kb)
			active_scale_percent = low_pct;
		else if (req_max < high_kb)
			active_scale_percent = mid_pct;

		if (active_scale_percent < 100) {
			*ab = mul_u64_u32_div(*ab, active_scale_percent, 100);
			*ib = mul_u64_u32_div(*ib, active_scale_percent, 100);
			*ab = max(*ab, req_ab);
			*ib = max(*ib, req_ib);
		}
	}

	if (sleep_mode && sleep_floor_active) {
		unsigned int sleep_floor_percent;

		sleep_floor_percent = kona_icc_sleep_floor_percent(qp);
		if (!sleep_floor_percent) {
			*ab = req_ab;
			*ib = req_ib;
		} else {
			*ab = mul_u64_u32_div(*ab, sleep_floor_percent, 100);
			*ib = mul_u64_u32_div(*ib, sleep_floor_percent, 100);
		}
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
	if (desc->role != KONA_ROLE_DISPLAY && (!sleep_mode || sleep_floor_active) &&
	    active_floor_active) {
		unsigned int bias = kona_icc_pick_bias(desc, *ab, *ib);

		if (*ab)
			*ab = kona_icc_add_headroom(*ab, bias);
		if (*ib)
			*ib = kona_icc_add_headroom(*ib, bias);
	}

	pr_debug("kona-icc: vote for %s after floor/bias (ab=%llu KBps, ib=%llu KBps)\n",
		 desc->name, *ab, *ib);
}

static void __maybe_unused
kona_icc_apply_floor(struct kona_icc_provider *qp,
		     const struct kona_icc_node_desc *desc,
		     u64 req_ab, u64 req_ib, u64 *ab, u64 *ib)
{
	u64 new_ab = *ab;
	u64 new_ib = *ib;

	kona_icc_calculate_floor(qp, desc, req_ab, req_ib,
				 &new_ab, &new_ib);

	if (new_ab > KONA_ICC_MAX_LOGICAL_VOTE ||
	    new_ib > KONA_ICC_MAX_LOGICAL_VOTE)
		return;

	*ab = new_ab;
	*ib = new_ib;
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
		.role = KONA_ROLE_GMU,
	},
	{
		.id = KONA_ICC_GMU_TO_MEM,
		.name = "gmu-ddr",
		.ab = "GPU_MEM_AB",
		.ib = "GPU_MEM_IB",
		.role = KONA_ROLE_GMU,
	},
	{
		.id = KONA_ICC_CPU_TO_GPU_CFG,
		.name = "cpu-gpu-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_CONFIG,
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
		.id = KONA_ICC_VIDEO_TO_LLCC,
		.name = "video-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_VIDEO_TO_MEM,
		.name = "video-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_CVP_TO_MEM,
		.name = "cvp-ddr",
		.ab = "NPU_MEM_AB",
		.ib = "NPU_MEM_IB",
		.role = KONA_ROLE_NPU,
	},
	{
		.id = KONA_ICC_PAS_TO_MEM,
		.name = "pas-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_DSP,
	},
	{
		.id = KONA_ICC_CPU_TO_PRNG,
		.name = "cpu-prng",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_USB0_TO_MEM,
		.name = "usb0-ddr",
		.ab = "USB0_MEM_AB",
		.ib = "USB0_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_USB1_TO_MEM,
		.name = "usb1-ddr",
		.ab = "USB1_MEM_AB",
		.ib = "USB1_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_QUP_TO_MEM,
		.name = "qup-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_SDHC2_TO_MEM,
		.name = "sdhc2-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_STORAGE,
	},
	{
		.id = KONA_ICC_UFS_TO_LLCC,
		.name = "ufs-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_STORAGE,
	},
	{
		.id = KONA_ICC_UFS_TO_MEM,
		.name = "ufs-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_STORAGE,
	},
	{
		.id = KONA_ICC_CAM_CFG,
		.name = "cam-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_MEDIA,
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
		/* Video/camera path is latency sensitive and bandwidth hungry. */
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_CAM_HF0_TO_MEM,
		.name = "cam-hf0-ddr",
		/*
		 * Camera high-frequency path: keep the conservative CPU_MEM BCM
		 * mapping until board-specific camera cmd-db names are proven safe.
		 */
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_CAM_SF0_TO_MEM,
		.name = "cam-sf0-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_CAM_SF_ICP_TO_MEM,
		.name = "cam-sf-icp-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_MEDIA,
	},
	{
		.id = KONA_ICC_PCIE0_TO_MEM,
		.name = "pcie0-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_PCIE1_TO_MEM,
		.name = "pcie1-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_PCIE2_TO_MEM,
		.name = "pcie2-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_CRYPTO_TO_MEM,
		.name = "crypto-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_RAW,
	},
	{
		.id = KONA_ICC_TSIF_TO_MEM,
		.name = "tsif-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_PERIPHERAL,
	},
	{
		.id = KONA_ICC_IPA_TO_LLCC,
		.name = "ipa-llcc",
		.ab = "CPU_LLCC_AB",
		.ib = "CPU_LLCC_IB",
		.role = KONA_ROLE_IPA,
	},
	{
		.id = KONA_ICC_IPA_TO_MEM,
		.name = "ipa-ddr",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_IPA,
	},
	{
		.id = KONA_ICC_IPA_TO_IMEM,
		.name = "ipa-imem",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_IPA,
	},
	{
		.id = KONA_ICC_IPA_CFG,
		.name = "ipa-cfg",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_IPA,
	},
	{
		.id = KONA_ICC_IPA_CORE,
		.name = "ipa-core",
		.ab = "CPU_MEM_AB",
		.ib = "CPU_MEM_IB",
		.role = KONA_ROLE_IPA,
	},
};

static bool kona_icc_stage_allows_id(u32 id)
{
	unsigned int stage = min_t(unsigned int, kona_icc_stage, KONA_ICC_STAGE_MAX);

	/*
	 * qcedev/qcrypto are forced to legacy msm_bus by default, so this
	 * can be exposed for DT/ICC compatibility without programming RPMh
	 * unless kona_crypto_raw_icc_enable is explicitly enabled.
	 */
	if (id == KONA_ICC_CRYPTO_TO_MEM && !kona_crypto_icc_enable)
		return false;

	if (stage >= KONA_ICC_STAGE_MAX)
		return true;

	switch (id) {
	case KONA_ICC_GPU_TO_LLCC:
	case KONA_ICC_GPU_TO_MEM:
	case KONA_ICC_GMU_TO_LLCC:
	case KONA_ICC_GMU_TO_MEM:
		return stage >= 1;
	case KONA_ICC_CPU_TO_LLCC:
	case KONA_ICC_CPU_TO_MEM:
	case KONA_ICC_CPU0_TO_LLCC:
	case KONA_ICC_CPU0_TO_MEM:
	case KONA_ICC_CPU1_TO_LLCC:
	case KONA_ICC_CPU1_TO_MEM:
	case KONA_ICC_CPU2_TO_LLCC:
	case KONA_ICC_CPU2_TO_MEM:
	case KONA_ICC_CPU3_TO_LLCC:
	case KONA_ICC_CPU3_TO_MEM:
	case KONA_ICC_CPU4_TO_LLCC:
	case KONA_ICC_CPU4_TO_MEM:
	case KONA_ICC_CPU5_TO_LLCC:
	case KONA_ICC_CPU5_TO_MEM:
	case KONA_ICC_CPU6_TO_LLCC:
	case KONA_ICC_CPU6_TO_MEM:
	case KONA_ICC_CPU7_TO_LLCC:
	case KONA_ICC_CPU7_TO_MEM:
	case KONA_ICC_CPU_TO_GPU_CFG:
	case KONA_ICC_CPU_TO_PRNG:
		return stage >= 2;
	case KONA_ICC_NPU_TO_LLCC:
	case KONA_ICC_NPU_TO_MEM:
	case KONA_ICC_NPUDSP_TO_MEM:
		return stage >= 5;
	case KONA_ICC_UFS_TO_LLCC:
	case KONA_ICC_UFS_TO_MEM:
	case KONA_ICC_SDHC2_TO_MEM:
	case KONA_ICC_TSIF_TO_MEM:
	case KONA_ICC_QUP_TO_MEM:
	case KONA_ICC_USB0_TO_MEM:
	case KONA_ICC_USB1_TO_MEM:
	case KONA_ICC_PCIE0_TO_MEM:
	case KONA_ICC_PCIE1_TO_MEM:
	case KONA_ICC_PCIE2_TO_MEM:
	case KONA_ICC_CRYPTO_TO_MEM:
		return stage >= 3;
	case KONA_ICC_DISP0_TO_MEM:
	case KONA_ICC_DISP1_TO_MEM:
	case KONA_ICC_DISP_CFG:
		return stage >= 4;
	case KONA_ICC_CAM_CFG:
	case KONA_ICC_VIDEO_CFG:
	case KONA_ICC_VIDEO_TO_LLCC:
	case KONA_ICC_VIDEO_TO_MEM:
	case KONA_ICC_CVP_TO_MEM:
	case KONA_ICC_PAS_TO_MEM:
	case KONA_ICC_CAM_HF0_TO_MEM:
	case KONA_ICC_CAM_SF0_TO_MEM:
	case KONA_ICC_CAM_SF_ICP_TO_MEM:
		return stage >= 5;
	case KONA_ICC_IPA_TO_LLCC:
	case KONA_ICC_IPA_TO_MEM:
	case KONA_ICC_IPA_TO_IMEM:
	case KONA_ICC_IPA_CFG:
	case KONA_ICC_IPA_CORE:
		return stage >= 6;
	default:
		return false;
	}
}

static size_t kona_icc_enabled_node_count(struct kona_icc_provider *qp)
{
	size_t count = 0;
	int i;

	for (i = 0; i < qp->num_nodes; i++)
		if (kona_icc_stage_allows_id(qp->nodes[i].id))
			count++;

	return count;
}

static int kona_icc_validate_nodes(struct device *dev,
				   const struct kona_icc_node_desc *nodes,
				   size_t num_nodes)
{
	DECLARE_BITMAP(seen, KONA_ICC_NUM_NODES);
	static const u32 stage1_ids[] = {
		KONA_ICC_GPU_TO_LLCC, KONA_ICC_GPU_TO_MEM,
		KONA_ICC_GMU_TO_LLCC, KONA_ICC_GMU_TO_MEM,
	};
	int i, j;

	bitmap_zero(seen, KONA_ICC_NUM_NODES);

	for (i = 0; i < num_nodes; i++) {
		if (nodes[i].id >= KONA_ICC_NUM_NODES) {
			dev_err(dev, "kona-icc: node %s has invalid id=%u (max=%u)\n",
				nodes[i].name ?: "?", nodes[i].id, KONA_ICC_NUM_NODES - 1);
			return -EINVAL;
		}
		if (test_and_set_bit(nodes[i].id, seen)) {
			dev_err(dev, "kona-icc: duplicate node id=%u (%s)\n",
				nodes[i].id, nodes[i].name ?: "?");
			return -EINVAL;
		}
		if (!nodes[i].name || !nodes[i].ab || !nodes[i].ib) {
			dev_err(dev, "kona-icc: node id=%u has missing name/ab/ib resource\n",
				nodes[i].id);
			return -EINVAL;
		}
	}

	if (kona_icc_stage >= 1) {
		for (j = 0; j < ARRAY_SIZE(stage1_ids); j++) {
			if (!test_bit(stage1_ids[j], seen)) {
				dev_err(dev, "kona-icc: missing required stage-1 id=%u\n",
					stage1_ids[j]);
				return -EINVAL;
			}
		}
	}

	return 0;
}



static const struct kona_icc_data kona_data = {
	.nodes = kona_nodes,
	.num_nodes = ARRAY_SIZE(kona_nodes),
	.boot_floor_vote = false,
};

static const struct kona_icc_node_desc *
kona_find_desc(struct kona_icc_provider *qp, u32 id, unsigned int *index);

static bool kona_icc_is_display_critical_id(u32 id)
{
	switch (id) {
	case KONA_ICC_DISP0_TO_MEM:
	case KONA_ICC_DISP1_TO_MEM:
	case KONA_ICC_DISP_CFG:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_is_display_cfg_id(u32 id)
{
	switch (id) {
	case KONA_ICC_DISP_CFG:
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
	if (!spec || spec->args_count < 1)
		return ERR_PTR(-EINVAL);

	desc = kona_find_desc(qp, spec->args[0], &index);
	if (!desc)
		return ERR_PTR(-EINVAL);
	if (!kona_icc_stage_allows_id(desc->id)) {
		dev_info_ratelimited(provider->dev,
			"kona-icc: stage %u rejects disabled id=%u (%s); msm_bus fallback expected for hybrid clients\n",
			min_t(unsigned int, kona_icc_stage, KONA_ICC_STAGE_MAX),
			desc->id, desc->name);
		return ERR_PTR(-ENODEV);
	}

	path = icc_of_xlate_onecell(provider, spec);
	if (IS_ERR(path))
		return path;

	path->data = (void *)(uintptr_t)index;

	return path;
}

struct kona_icc_legacy_submit_result {
	u32 addr;
	int raw_ret;
	int final_ret;
	const char *cause;
	bool cmd_db_addr_missing;
};

static int kona_icc_send_bw_result(struct device *dev, const char *res, u32 kbps,
				   bool wait,
				   struct kona_icc_legacy_submit_result *result)
{
	struct tcs_cmd cmd = {};
	u32 addr;
	int ret;

	if (result) {
		result->addr = 0;
		result->raw_ret = 0;
		result->final_ret = 0;
		result->cause = "none";
		result->cmd_db_addr_missing = false;
	}

	if (!res)
		return 0;

	/*
	 * Boot-safe handling: cmd-db and RPMh address translation may not be
	 * ready during early boot. Never propagate -EPROBE_DEFER to ICC consumers.
	 * Use -EAGAIN internally to signal "try again later".
	 */
	ret = cmd_db_ready();
	if (ret) {
		dev_dbg_ratelimited(dev, "kona-icc: cmd-db not ready for %s\n", res ?: "?");
		if (result) {
			result->cause = "cmd-db-not-ready";
			result->raw_ret = ret;
			result->final_ret = ret == -EPROBE_DEFER ? -EAGAIN : ret;
		}
		return ret == -EPROBE_DEFER ? -EAGAIN : ret;
	}

	addr = cmd_db_read_addr(res);
	if (!addr) {
		dev_dbg_ratelimited(dev, "kona-icc: missing cmd-db addr for %s\n", res ?: "?");
		if (result) {
			result->cause = "cmd-db-address-missing";
			result->raw_ret = -ENODEV;
			result->final_ret = -ENODEV;
			result->cmd_db_addr_missing = true;
		}
		return -ENODEV;
	}
	if (result)
		result->addr = addr;

	/* cmd.data is KB/s; callers must already scale to KB/s. */
	cmd.addr = addr;
	cmd.data = kbps;
	cmd.wait = wait;

	ret = rpmh_write(dev, RPMH_ACTIVE_ONLY_STATE, &cmd, 1);
	if (result)
		result->raw_ret = ret;
	if (ret == -EBUSY || ret == -ETIMEDOUT || ret == -EPROBE_DEFER) {
		dev_dbg_ratelimited(dev,
			"kona-icc: rpmh deferring %s=%uKB/s ret=%d\n",
			res ?: "?", kbps, ret);
		if (result) {
			result->cause = ret == -EBUSY ? "rpmh-busy" :
				ret == -ETIMEDOUT ? "rpmh-timeout" :
				"rpmh-probe-defer";
			result->final_ret = -EAGAIN;
		}
		return -EAGAIN;
	}
	if (result) {
		result->cause = ret == -EAGAIN ? "rpmh-eagain" :
			ret ? "rpmh-hard-error" : "none";
		result->final_ret = ret;
	}

	return ret;
}

static int kona_icc_send_bw(struct device *dev, const char *res, u32 kbps, bool wait)
{
	return kona_icc_send_bw_result(dev, res, kbps, wait, NULL);
}

static int kona_icc_register_crypto_ce0_client(struct device *dev)
{
	if (kona_crypto_ce0_client)
		return 0;

	kona_crypto_ce0_client =
		msm_bus_scale_register_client(&kona_crypto_ce0_pdata);
	if (!kona_crypto_ce0_client) {
		dev_warn(dev,
			 "kona-icc: failed to register CRYPTO CE0 msm_bus client\n");
		return -EPROBE_DEFER;
	}

	dev_info(dev,
		 "kona-icc: registered CRYPTO CE0 msm_bus bridge client\n");
	return 0;
}

static int kona_icc_send_crypto_ce0_vote(struct kona_icc_provider *qp,
					 unsigned int index, u64 ab, u64 ib)
{
	struct device *dev = qp->provider.dev;
	unsigned int vote_idx = (ab || ib) ? 1 : 0;
	int ret;

	if (!kona_crypto_ce0_msm_bus_enable)
		return 0;

	mutex_lock(&kona_crypto_ce0_lock);

	ret = kona_icc_register_crypto_ce0_client(dev);
	if (ret)
		goto out_unlock;

	if (kona_crypto_ce0_last_valid && kona_crypto_ce0_last_idx == vote_idx) {
		ret = 0;
		goto out_cache;
	}

	ret = msm_bus_scale_client_update_request(kona_crypto_ce0_client,
						    vote_idx);
	if (ret) {
		dev_warn_ratelimited(dev,
			"kona-icc: CRYPTO CE0 msm_bus vote idx=%u failed ret=%d\n",
			vote_idx, ret);
		goto out_unlock;
	}

	kona_crypto_ce0_last_idx = vote_idx;
	kona_crypto_ce0_last_valid = true;

	if (kona_crypto_ce0_msm_bus_debug)
		dev_info_ratelimited(dev,
			"kona-icc: CRYPTO ICC -> CE0 msm_bus idx=%u ab=%llu ib=%llu\n",
			vote_idx, ab, ib);

out_cache:
	if (qp->last_ab)
		qp->last_ab[index] = ab;
	if (qp->last_ib)
		qp->last_ib[index] = ib;
out_unlock:
	mutex_unlock(&kona_crypto_ce0_lock);
	return ret;
}

static bool kona_icc_vote_component_unchanged(u64 *last, unsigned int index, u64 vote)
{
	return last && last[index] != U64_MAX && last[index] == vote;
}

/*
 * Several logical Kona paths terminate at the same RPMh BCM (most notably
 * CPU_MEM and CPU_LLCC).  The virtual provider sends BCM commands directly,
 * so treating the per-node cache as the hardware cache lets the last, smaller
 * client vote overwrite an already active devbw or memlat vote.  That is
 * particularly visible in memory microbenchmarks, where latency-monitor and
 * devbw updates interleave frequently.
 *
 * Retain the largest effective vote for a shared resource.  AB is normally
 * additive in the generic ICC framework, but summing here would add each
 * synthetic performance floor as well as the real requests and permanently
 * over-vote DDR.  A max aggregation preserves the existing floor policy while
 * preventing a low-rate sibling from pulling a shared channel below the vote
 * required by the active high-rate sibling.
 */
static u64 kona_icc_shared_resource_vote(struct kona_icc_provider *qp,
					 const char *resource, bool average)
{
	u64 vote = 0;
	unsigned int i;

	if (!resource || !qp->eff_ab || !qp->eff_ib)
		return vote;

	for (i = 0; i < qp->num_nodes; i++) {
		const struct kona_icc_node_desc *node = &qp->nodes[i];
		const char *node_resource = average ? node->ab : node->ib;
		u64 node_vote = average ? qp->eff_ab[i] : qp->eff_ib[i];

		if (!node_resource || strcmp(resource, node_resource) ||
		    node_vote == U64_MAX ||
		    kona_icc_is_policy_suppressed_path(node))
			continue;

		vote = max(vote, node_vote);
	}

	return vote;
}

static void kona_icc_cache_shared_resource_vote(struct kona_icc_provider *qp,
						 const char *resource, bool average,
						 u64 vote)
{
	unsigned int i;

	if (!resource)
		return;

	for (i = 0; i < qp->num_nodes; i++) {
		const char *node_resource = average ? qp->nodes[i].ab : qp->nodes[i].ib;
		u64 *last = average ? qp->last_ab : qp->last_ib;

		if (last && node_resource && !strcmp(resource, node_resource))
			last[i] = vote;
	}
}

static bool kona_icc_is_cpu_memory_path(const struct kona_icc_node_desc *desc)
{
	switch (desc->id) {
	case KONA_ICC_CPU_TO_LLCC:
	case KONA_ICC_CPU_TO_MEM:
	case KONA_ICC_CPU0_TO_LLCC:
	case KONA_ICC_CPU0_TO_MEM:
	case KONA_ICC_CPU1_TO_LLCC:
	case KONA_ICC_CPU1_TO_MEM:
	case KONA_ICC_CPU2_TO_LLCC:
	case KONA_ICC_CPU2_TO_MEM:
	case KONA_ICC_CPU3_TO_LLCC:
	case KONA_ICC_CPU3_TO_MEM:
	case KONA_ICC_CPU4_TO_LLCC:
	case KONA_ICC_CPU4_TO_MEM:
	case KONA_ICC_CPU5_TO_LLCC:
	case KONA_ICC_CPU5_TO_MEM:
	case KONA_ICC_CPU6_TO_LLCC:
	case KONA_ICC_CPU6_TO_MEM:
	case KONA_ICC_CPU7_TO_LLCC:
	case KONA_ICC_CPU7_TO_MEM:
		return true;
	default:
		return false;
	}
}

static bool kona_icc_cpu_to_memory(u32 id)
{
	return id == KONA_ICC_CPU_TO_MEM ||
		(id >= KONA_ICC_CPU0_TO_MEM && id <= KONA_ICC_CPU7_TO_MEM &&
		 !(id & 1));
}

static bool kona_icc_cpu_generic(u32 id)
{
	return id == KONA_ICC_CPU_TO_LLCC || id == KONA_ICC_CPU_TO_MEM;
}

static void kona_icc_cpu_endpoint_vote(struct kona_icc_provider *qp,
				       bool memory, u64 *avg, u64 *peak)
{
	u64 generic_avg = 0, per_cpu_avg = 0, policy_avg = 0, policy_peak = 0;
	u64 raw_peak = 0;
	unsigned int i;

	for (i = 0; i < qp->num_nodes; i++) {
		const struct kona_icc_node_desc *node = &qp->nodes[i];
		u64 raw_avg, raw_ib;

		if (!kona_icc_is_cpu_memory_path(node) ||
		    kona_icc_cpu_to_memory(node->id) != memory)
			continue;
		if (qp->req_ab[i] == U64_MAX || qp->req_ib[i] == U64_MAX ||
		    qp->eff_ab[i] == U64_MAX || qp->eff_ib[i] == U64_MAX)
			continue;

		raw_avg = qp->req_ab[i];
		raw_ib = qp->req_ib[i];
		if (kona_icc_cpu_generic(node->id))
			generic_avg = max(generic_avg, raw_avg);
		else if (U64_MAX - per_cpu_avg < raw_avg)
			per_cpu_avg = U64_MAX;
		else
			per_cpu_avg += raw_avg;
		raw_peak = max(raw_peak, raw_ib);
		policy_avg = max(policy_avg, qp->eff_ab[i]);
		policy_peak = max(policy_peak, qp->eff_ib[i]);
	}

	/* Generic and per-CPU paths overlap; never blindly add both views. */
	*avg = max(max(generic_avg, per_cpu_avg), policy_avg);
	*peak = max(raw_peak, policy_peak);
}

static int kona_icc_cpu_bcm_metadata(struct kona_icc_provider *qp,
				     struct kona_bcm_state *bcm)
{
	struct kona_bcm_db aux;
	size_t aux_len;
	int ret;

	if (bcm->metadata_valid)
		return 0;
	ret = cmd_db_ready();
	if (ret)
		return ret == -EPROBE_DEFER ? -EAGAIN : ret;
	if (cmd_db_read_slave_id(bcm->name) != CMD_DB_HW_BCM)
		return -ENODEV;
	aux_len = cmd_db_read_aux_data_len(bcm->name);
	if (!aux_len)
		return -ENOENT;
	if (aux_len != sizeof(aux))
		return -EPROTO;
	ret = cmd_db_read_aux_data(bcm->name, (u8 *)&aux, sizeof(aux));
	if (ret < 0)
		return ret;
	if (ret != sizeof(aux))
		return -EINVAL;
	bcm->addr = cmd_db_read_addr(bcm->name);
	bcm->unit = le32_to_cpu(aux.unit);
	bcm->width = le16_to_cpu(aux.width);
	bcm->vcd = aux.vcd;
	if (!bcm->addr || !bcm->unit || !bcm->width)
		return -EINVAL;
	bcm->metadata_valid = true;
	return 0;
}

static u64 kona_div_round_up_u64(u64 numerator, u64 denominator)
{
	u64 quotient;

	if (!denominator)
		return U64_MAX;
	if (!numerator)
		return 0;

	quotient = div64_u64(numerator, denominator);
	if (numerator % denominator)
		quotient++;

	return quotient;
}

static u64 kona_icc_cpu_normalize(u64 kbps, struct kona_bcm_state *bcm,
				  const struct kona_qnode_vote *qnode,
				  bool average, u64 *raw_vote)
{
	u64 bytes_per_sec = 0, numerator = 0, denominator, raw;
	bool overflowed;

	/*
	 * Match the in-tree RPMh BCM voter:
	 *   lnode_ib = max_ib * bcm_width / qnode_buswidth
	 *   lnode_ab = sum_ab * bcm_width / (qnode_buswidth * channels)
	 *   vec_{a,b} = max_lnode_{ab,ib} / bcm_unit_size
	 * ICC clients provide kB/s, so convert to B/s before applying the BCM
	 * width, qnode bus width/channel scaling, and command-db unit size.
	 */
	overflowed = check_mul_overflow(kbps, 1000ULL, &bytes_per_sec) ||
		     check_mul_overflow(bytes_per_sec, (u64)bcm->width,
					&numerator);
	denominator = qnode->buswidth;
	if (average)
		denominator *= qnode->channels;
	denominator *= bcm->unit;

	if (overflowed)
		raw = U64_MAX;
	else
		raw = kona_div_round_up_u64(numerator, denominator);
	if (raw_vote)
		*raw_vote = raw;

	return min_t(u64, raw, KONA_BCM_VOTE_MASK);
}

static int kona_icc_build_cpu_bcm_group(struct kona_icc_provider *qp,
					const unsigned int *indices,
					unsigned int num_bcms,
					struct tcs_cmd *cmds,
					unsigned int *cmd_indices,
					unsigned int *num_cmds, u32 *batch_n,
					unsigned int *num_messages)
{
	struct kona_bcm_state *bcms = qp->cpu_bcms;
	unsigned int base = *num_cmds;
	unsigned int i, n = 0;
	bool wait = false;
	bool dry_run = READ_ONCE(kona_packed_dry_run);

	for (i = 0; i < num_bcms; i++) {
		struct kona_bcm_state *bcm = &bcms[indices[i]];
		u64 threshold;

		if (!bcm->dirty)
			continue;
		threshold = mul_u64_u32_div(bcm->committed_x, 125, 100);
		wait |= (!bcm->committed_x && bcm->requested_x) ||
			bcm->requested_x > threshold;
		threshold = mul_u64_u32_div(bcm->committed_y, 125, 100);
		wait |= (!bcm->committed_y && bcm->requested_y) ||
			bcm->requested_y > threshold;
	}

	for (i = 0; i < num_bcms; i++) {
		struct kona_bcm_state *bcm = &bcms[indices[i]];
		bool commit;
		bool valid;

		if (!bcm->dirty)
			continue;
		commit = i == num_bcms - 1;
		if (!commit) {
			unsigned int j;

			commit = true;
			for (j = i + 1; j < num_bcms; j++)
				if (bcms[indices[j]].dirty) {
					commit = false;
					break;
				}
		}
		valid = bcm->requested_x || bcm->requested_y;
		cmds[base + n].addr = bcm->addr;
		cmds[base + n].data = KONA_BCM_TCS_CMD(commit, valid,
							bcm->requested_x,
							bcm->requested_y);
		cmds[base + n].wait = commit && wait;
		bcm->requested_data = cmds[base + n].data;
		cmd_indices[base + n] = indices[i];
		n++;
	}
	if (!n)
		return 0;
	if (dry_run) {
		for (i = 0; i < n; i++) {
			struct kona_bcm_state *bcm = &bcms[cmd_indices[base + i]];

			bcm->dry_run_data = bcm->requested_data;
			bcm->dry_run_generation = bcm->requested_generation;
		}
		qp->packed_dry_run_build_count++;
	}

	batch_n[*num_messages] = n;
	(*num_messages)++;
	*num_cmds += n;
	return 0;
}

static void kona_packed_snapshot_inputs(struct kona_icc_provider *qp,
					struct kona_packed_inputs *inputs)
{
	kona_icc_cpu_endpoint_vote(qp, false, &inputs->llcc_avg,
				   &inputs->llcc_peak);
	kona_icc_cpu_endpoint_vote(qp, true, &inputs->mem_avg,
				   &inputs->mem_peak);
	inputs->group_mask = READ_ONCE(kona_packed_group_mask);
	inputs->dry_run = READ_ONCE(kona_packed_dry_run);
	inputs->config_generation = qp->packed_config_generation;
}

static int kona_icc_commit_cpu_bcms(struct kona_icc_provider *qp,
				    const struct kona_packed_inputs *inputs,
				    int *failed_vcd)
{
	struct kona_qnode_vote llcc = { .buswidth = 16, .channels = 4 };
	struct kona_qnode_vote apps = { .buswidth = 32, .channels = 2 };
	struct kona_qnode_vote ebi = { .buswidth = 4, .channels = 4 };
	struct kona_bcm_state *bcms = qp->cpu_bcms;
	static const unsigned int vcd0[] = { KONA_CPU_BCM_MC0 };
	static const unsigned int vcd1[] = {
		KONA_CPU_BCM_SH4, KONA_CPU_BCM_SH0,
	};
	struct tcs_cmd cmds[KONA_CPU_BCM_COUNT] = {};
	unsigned int cmd_indices[KONA_CPU_BCM_COUNT];
	u32 batch_n[KONA_CPU_BCM_COUNT + 1] = {};
	unsigned int num_cmds = 0, num_messages = 0;
	unsigned int i;
	unsigned int mask = inputs->group_mask;
	bool force_dirty = READ_ONCE(kona_packed_force_dirty);
	int ret;

	apps.avg = max(inputs->llcc_avg, inputs->mem_avg);
	llcc.avg = apps.avg;
	apps.peak = max(inputs->llcc_peak, inputs->mem_peak);
	llcc.peak = apps.peak;
	ebi.avg = inputs->mem_avg;
	ebi.peak = inputs->mem_peak;

	for (i = 0; i < KONA_CPU_BCM_COUNT; i++) {
		struct kona_bcm_state *bcm = &bcms[i];
		unsigned int group = i == KONA_CPU_BCM_MC0 ?
			KONA_PACKED_GROUP_MC0 : KONA_PACKED_GROUP_SH;
		const struct kona_qnode_vote *qnode = i == KONA_CPU_BCM_SH4 ?
			&apps : i == KONA_CPU_BCM_SH0 ? &llcc : &ebi;
		u64 requested_x, requested_y, raw_x, raw_y;

		/* A masked group is neither attempted nor failed. */
		if (!(mask & group))
			continue;
		ret = kona_icc_cpu_bcm_metadata(qp, bcm);
		if (ret) {
			dev_err(qp->provider.dev,
				"kona-rpmh: invalid metadata for %s: %d\n",
				bcm->name, ret);
			return ret;
		}
		requested_x = kona_icc_cpu_normalize(qnode->avg, bcm, qnode, true, &raw_x);
		requested_y = kona_icc_cpu_normalize(qnode->peak, bcm, qnode, false, &raw_y);
		if (requested_x != bcm->requested_x || requested_y != bcm->requested_y ||
		    raw_x != bcm->raw_x || raw_y != bcm->raw_y)
			bcm->requested_generation++;
		bcm->requested_x = requested_x;
		bcm->requested_y = requested_y;
		bcm->raw_x = raw_x;
		bcm->raw_y = raw_y;
		bcm->saturated_x = raw_x > KONA_BCM_VOTE_MASK;
		bcm->saturated_y = raw_y > KONA_BCM_VOTE_MASK;
		if ((bcm->saturated_x && raw_x != bcm->last_diagnosed_x) ||
		    (bcm->saturated_y && raw_y != bcm->last_diagnosed_y))
			bcm->saturation_count++;
		bcm->last_diagnosed_x = raw_x;
		bcm->last_diagnosed_y = raw_y;
		if (force_dirty) {
			bcm->dirty = true;
		} else if (READ_ONCE(kona_packed_dry_run)) {
			bcm->dirty = bcm->dry_run_generation != bcm->requested_generation;
			if (!bcm->dirty)
				qp->packed_dry_run_skip_count++;
		} else {
			bcm->dirty = requested_x != bcm->committed_x ||
				requested_y != bcm->committed_y;
		}
	}

	/* Preserve ascending VCD order while constructing one native RPMh batch. */
	if (mask & KONA_PACKED_GROUP_MC0) {
		ret = kona_icc_build_cpu_bcm_group(qp, vcd0, ARRAY_SIZE(vcd0),
						   cmds, cmd_indices, &num_cmds,
						   batch_n, &num_messages);
		if (ret)
			return ret;
	}
	if (mask & KONA_PACKED_GROUP_SH) {
		ret = kona_icc_build_cpu_bcm_group(qp, vcd1, ARRAY_SIZE(vcd1),
						   cmds, cmd_indices, &num_cmds,
						   batch_n, &num_messages);
		if (ret)
			return ret;
	}
	if (force_dirty) {
		WRITE_ONCE(kona_packed_force_dirty, false);
		pr_emerg("kona-rpmh: breadcrumb packed-force-dirty-consumed mask=%u\n",
			 mask);
	}

	/* Dry-run construction and telemetry never cross an RPMh boundary. */
	if (READ_ONCE(kona_packed_dry_run) || !num_cmds)
		goto update_legacy_cache;

	if (kona_cpu_model_stage() < 4 ||
	    !READ_ONCE(kona_packed_runtime_enable) ||
	    !READ_ONCE(kona_packed_group_mask) || qp->packed_fallback_active)
		return -EPERM;
	if (!READ_ONCE(kona_packed_real_write_enable)) {
		return KONA_PACKED_REAL_WRITE_BLOCKED;
	}
	if (READ_ONCE(kona_packed_real_write_once) &&
	    qp->packed_real_write_consumed) {
		dev_warn_ratelimited(qp->provider.dev,
				     "kona-rpmh: packed real-write one-shot blocked\n");
		return KONA_PACKED_REAL_WRITE_BLOCKED;
	}

	/* Consume before submission so the complete hardware attempt is one-shot. */
	if (READ_ONCE(kona_packed_real_write_once))
		qp->packed_real_write_consumed = true;
	qp->packed_submission_count++;
	/*
	 * Do not call rpmh_invalidate() here.  It invalidates every cached active
	 * and sleep set owned by the APPS RSC, while this transaction can rebuild
	 * only the selected CPU BCM messages.
	 */
	ret = rpmh_write_batch(qp->rpmh_dev, RPMH_ACTIVE_ONLY_STATE, cmds,
			       batch_n);
	if (ret)
		goto batch_failed;

	for (i = 0; i < num_cmds; i++) {
		struct kona_bcm_state *bcm = &bcms[cmd_indices[i]];

		bcm->committed_x = bcm->requested_x;
		bcm->committed_y = bcm->requested_y;
		bcm->committed_data = bcm->requested_data;
		bcm->committed_generation = bcm->requested_generation;
		bcm->dirty = false;
		bcm->last_error = 0;
	}
	goto update_legacy_cache;

batch_failed:
	if (num_messages == 1)
		*failed_vcd = bcms[cmd_indices[0]].vcd;
	else
		*failed_vcd = -1;
	for (i = 0; i < num_cmds; i++) {
		struct kona_bcm_state *bcm = &bcms[cmd_indices[i]];

		bcm->last_error = ret;
		bcm->failure_count++;
		if (ret == -EAGAIN || ret == -EBUSY || ret == -ETIMEDOUT ||
		    ret == -EPROBE_DEFER)
			bcm->retry_count++;
	}
	if (ret == -EAGAIN || ret == -EBUSY || ret == -ETIMEDOUT ||
	    ret == -EPROBE_DEFER)
		return -EAGAIN;
	return ret;
update_legacy_cache:
	/* Only the complete packed model owns all legacy CPU resources. */
	if (!READ_ONCE(kona_packed_dry_run) && mask == KONA_PACKED_GROUP_ALL)
		for (i = 0; i < qp->num_nodes; i++)
			if (kona_icc_is_cpu_memory_path(&qp->nodes[i])) {
				qp->last_ab[i] = qp->eff_ab[i];
				qp->last_ib[i] = qp->eff_ib[i];
			}
	return 0;
}


static int kona_icc_send_vote_component(struct kona_icc_provider *qp,
					unsigned int index, const char *res,
					u64 vote, u64 *last,
					bool wait, bool average,
					bool *missing_alias)
{
	struct kona_icc_legacy_submit_result result;
	int ret;

	if (kona_icc_vote_component_unchanged(last, index, vote))
		return 0;

	ret = kona_icc_send_bw_result(qp->provider.dev, res,
				      min_t(u64, vote, U32_MAX), wait, &result);
	if (missing_alias && result.cmd_db_addr_missing)
		*missing_alias = true;
	if (ret)
		return ret;

	kona_icc_cache_shared_resource_vote(qp, res, average, vote);

	return 0;
}

static bool kona_icc_vote_valid(u64 ab, u64 ib);
static void kona_icc_validate_vote(struct kona_icc_provider *qp,
				   const struct kona_icc_node_desc *desc,
				   u64 raw_ab, u64 raw_ib, u64 *ab, u64 *ib);

static int kona_icc_process_packed_aggregate(struct kona_icc_provider *qp,
						 bool *skip_legacy)
{
	struct kona_packed_inputs inputs;
	struct kona_packed_inputs *observed;
	bool dry_run;
	bool aggregate_changed;
	bool config_changed;
	unsigned int i;
	int failed_vcd = -2;
	int ret;

	if (skip_legacy)
		*skip_legacy = false;

	if (kona_cpu_model_stage() < 4 ||
	    !READ_ONCE(kona_packed_runtime_enable) ||
	    !READ_ONCE(kona_packed_group_mask) ||
	    qp->packed_fallback_active)
		return 0;

	dry_run = READ_ONCE(kona_packed_dry_run);
	kona_packed_snapshot_inputs(qp, &inputs);
	qp->packed_current_inputs = inputs;
	observed = dry_run ? &qp->packed_last_dry_run_inputs :
		&qp->packed_last_real_inputs;

	aggregate_changed = inputs.llcc_avg != observed->llcc_avg ||
		inputs.llcc_peak != observed->llcc_peak ||
		inputs.mem_avg != observed->mem_avg ||
		inputs.mem_peak != observed->mem_peak;
	config_changed = inputs.group_mask != observed->group_mask ||
		inputs.dry_run != observed->dry_run ||
		inputs.config_generation != observed->config_generation;

	if (!aggregate_changed && !config_changed) {
		qp->packed_aggregate_unchanged_skip_count++;
		qp->packed_processed_generation = qp->packed_config_generation;
		if (!dry_run && READ_ONCE(kona_packed_real_write_enable) &&
		    inputs.group_mask == KONA_PACKED_GROUP_ALL && skip_legacy)
			*skip_legacy = true;
		return 0;
	}

	ret = kona_icc_commit_cpu_bcms(qp, &inputs, &failed_vcd);
	if (ret == KONA_PACKED_REAL_WRITE_BLOCKED)
		return 0;
	if (!ret) {
		qp->packed_aggregate_build_count++;
		qp->packed_processed_generation = qp->packed_config_generation;
		qp->packed_observed_generation = qp->packed_config_generation;
		*observed = inputs;
		if (dry_run)
			qp->last_packed_dry_run_error = 0;
		else if (inputs.group_mask == KONA_PACKED_GROUP_ALL && skip_legacy)
			*skip_legacy = true;
		return 0;
	}

	if (dry_run) {
		qp->last_packed_dry_run_error = ret;
		return 0;
	}

	qp->packed_fallback_active = true;
	qp->packed_processed_generation = qp->packed_config_generation;
	qp->packed_fallback_count++;
	qp->last_failed_vcd = failed_vcd;
	qp->last_packed_error = ret;
	for (i = 0; i < qp->num_nodes; i++)
		if (kona_icc_is_cpu_memory_path(&qp->nodes[i])) {
			qp->last_ab[i] = U64_MAX;
			qp->last_ib[i] = U64_MAX;
		}
	dev_err(qp->provider.dev,
		"kona-rpmh: packed commit failed vcd=%d error=%d; using legacy CPU path for this boot\n",
		failed_vcd, ret);
	return 0;
}

static int kona_icc_send_node_votes(struct kona_icc_provider *qp,
				    unsigned int index, u64 ab, u64 ib,
				    bool *retry, bool *missing_alias)
{
	const struct kona_icc_node_desc *desc;
	int ret;
	bool wait;

	if (WARN_ON_ONCE(!qp || index >= qp->num_nodes))
		return -EINVAL;

	desc = &qp->nodes[index];
	wait = kona_cpu_memory_sync_votes && kona_icc_is_cpu_memory_path(desc);
	if (unlikely(!kona_icc_vote_valid(ab, ib))) {
		u64 raw_ab = qp->req_ab && qp->req_ab[index] != U64_MAX ?
			qp->req_ab[index] : 0;
		u64 raw_ib = qp->req_ib && qp->req_ib[index] != U64_MAX ?
			qp->req_ib[index] : 0;

		kona_icc_validate_vote(qp, desc, raw_ab, raw_ib, &ab, &ib);
		if (qp->eff_ab)
			qp->eff_ab[index] = ab;
		if (qp->eff_ib)
			qp->eff_ib[index] = ib;
	}

	if (retry)
		*retry = false;
	if (missing_alias)
		*missing_alias = false;

	/* Program the aggregate for a BCM, rather than this node's last vote. */
	if (kona_vote_debug_submit)
		KONA_VOTE_TRACE(qp, desc, "before-shared-aggregation", ab, ib);
	ab = kona_icc_shared_resource_vote(qp, desc->ab, true);
	ib = kona_icc_shared_resource_vote(qp, desc->ib, false);
	if (kona_vote_debug_submit)
		KONA_VOTE_TRACE(qp, desc, "after-shared-aggregation", ab, ib);
	if (unlikely(!kona_icc_vote_valid(ab, ib))) {
		dev_err_ratelimited(qp->provider.dev,
				"kona-icc: invalid shared legacy vote id=%u name=%s vote=%llu/%llu\n",
				desc->id, desc->name, (unsigned long long)ab,
				(unsigned long long)ib);
		ret = -ERANGE;
		goto out_legacy;
	}
	if (kona_vote_debug && kona_icc_is_cpu_memory_path(desc))
		dev_info(qp->provider.dev,
			 "kona-vote: id=%u before-legacy=%llu/%llu stage=%u\n",
			 desc->id, (unsigned long long)ab,
			 (unsigned long long)ib, kona_cpu_model_stage());
	if (kona_vote_debug_submit)
		KONA_VOTE_TRACE(qp, desc, "before-legacy-send", ab, ib);

	/*
	 * Only touch RPMh resources whose component changed. Many consumers
	 * adjust AB and IB independently, and replay can revisit nodes after one
	 * side was already accepted. Skipping identical components cuts redundant
	 * ACTIVE_ONLY writes while preserving the GPU-before-AB ordering quirk.
	 */
	if (desc->id == KONA_ICC_GPU_TO_MEM) {
		ret = kona_icc_send_vote_component(qp, index, desc->ib, ib,
						   qp->last_ib, wait,
						   false, missing_alias);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			goto out_legacy;

		ret = kona_icc_send_vote_component(qp, index, desc->ab, ab,
						   qp->last_ab, wait,
						   true, missing_alias);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			goto out_legacy;
	} else {
		ret = kona_icc_send_vote_component(qp, index, desc->ab, ab,
						   qp->last_ab, wait,
						   true, missing_alias);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			goto out_legacy;

		ret = kona_icc_send_vote_component(qp, index, desc->ib, ib,
						   qp->last_ib, wait,
						   false, missing_alias);
		if (ret == -EAGAIN)
			goto out_retry;
		if (ret)
			goto out_legacy;
	}

	ret = 0;
out_legacy:
	if (kona_cpu_model_stage() >= 4 && qp->packed_fallback_active &&
	    kona_icc_is_cpu_memory_path(desc))
		qp->last_legacy_fallback_error = ret;
	return ret;

out_retry:
	if (retry)
		*retry = true;
	ret = -EAGAIN;
	goto out_legacy;
}

static void kona_icc_packed_force_generation(struct kona_icc_provider *qp)
{
	qp->packed_config_generation++;
	atomic64_inc(&qp->packed_update_generation);
}

static void kona_icc_mark_dirty(struct kona_icc_provider *qp, unsigned int index)
{
	unsigned long flags;

	if (qp->dirty_nodes)
		spin_lock_irqsave(&qp->dirty_lock, flags);
	if (qp->dirty_nodes) {
		__set_bit(index, qp->dirty_nodes);
		spin_unlock_irqrestore(&qp->dirty_lock, flags);
	}
}

static void kona_icc_clear_dirty(struct kona_icc_provider *qp, unsigned int index)
{
	unsigned long flags;

	if (qp->dirty_nodes)
		spin_lock_irqsave(&qp->dirty_lock, flags);
	if (qp->dirty_nodes) {
		__clear_bit(index, qp->dirty_nodes);
		spin_unlock_irqrestore(&qp->dirty_lock, flags);
	}
}

static bool kona_icc_is_dirty(struct kona_icc_provider *qp, unsigned int index)
{
	unsigned long flags;
	bool dirty = false;

	if (qp->dirty_nodes)
		spin_lock_irqsave(&qp->dirty_lock, flags);
	if (qp->dirty_nodes) {
		dirty = test_bit(index, qp->dirty_nodes);
		spin_unlock_irqrestore(&qp->dirty_lock, flags);
	}

	return dirty;
}

static void kona_icc_mark_all_dirty(struct kona_icc_provider *qp)
{
	unsigned long flags;

	if (qp->dirty_nodes)
		spin_lock_irqsave(&qp->dirty_lock, flags);
	if (qp->dirty_nodes) {
		bitmap_fill(qp->dirty_nodes, qp->num_nodes);
		spin_unlock_irqrestore(&qp->dirty_lock, flags);
	}
}

static bool kona_icc_snapshot_dirty(struct kona_icc_provider *qp)
{
	unsigned long flags;

	if (!qp->dirty_nodes || !qp->replay_scan_nodes)
		return false;

	spin_lock_irqsave(&qp->dirty_lock, flags);
	bitmap_copy(qp->replay_scan_nodes, qp->dirty_nodes, qp->num_nodes);
	spin_unlock_irqrestore(&qp->dirty_lock, flags);

	return !bitmap_empty(qp->replay_scan_nodes, qp->num_nodes);
}

static bool kona_icc_vote_is_unchanged(struct kona_icc_provider *qp,
				      unsigned int index, u64 ab, u64 ib)
{
	const struct kona_icc_node_desc *desc;

	if (!qp->last_ab || !qp->last_ib)
		return false;
	if (kona_cpu_model_stage() >= 4 &&
	    kona_icc_is_cpu_memory_path(&qp->nodes[index]))
		return false;

	if (qp->last_ab[index] == U64_MAX || qp->last_ib[index] == U64_MAX)
		return false;

	/* last_* is shared per physical BCM; compare it with the BCM aggregate. */
	desc = &qp->nodes[index];
	ab = kona_icc_shared_resource_vote(qp, desc->ab, true);
	ib = kona_icc_shared_resource_vote(qp, desc->ib, false);

	return qp->last_ab[index] == ab && qp->last_ib[index] == ib;
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
	unsigned long delay = msecs_to_jiffies(delay_ms);
	unsigned long target = jiffies + delay;

	if (delayed_work_pending(&qp->retry_work)) {
		unsigned long cur_target = READ_ONCE(qp->retry_work.timer.expires);

		if (time_before_eq(cur_target, target)) {
			atomic_inc(&qp->replay_queue_skips);
			return;
		}
	}

	mod_delayed_work(system_wq, &qp->retry_work, msecs_to_jiffies(delay_ms));

	if (kona_resume_debug)
		dev_info_ratelimited(qp->provider.dev,
			"kona-icc: replay queued (%s, %ums) deferred=%d replay=%d skips=%d queue-skips=%d\n",
			why ?: "unknown", delay_ms,
			atomic_read(&qp->deferred_votes),
			atomic_read(&qp->replay_runs),
			atomic_read(&qp->display_replay_skips),
			atomic_read(&qp->replay_queue_skips));
}

/*
 * Parameter writes only change ownership and queue normal vote work.  The
 * worker takes vote_lock and is the sole place that may reach packed RPMh I/O.
 * Packed committed values intentionally remain historical after disarming.
 */
static void kona_icc_packed_parameter_changed(struct kona_icc_provider *qp,
					       bool enabled)
{
	unsigned int i;

	mutex_lock(&qp->vote_lock);
	WRITE_ONCE(kona_packed_runtime_enable, enabled);
	kona_icc_packed_force_generation(qp);
	for (i = 0; i < qp->num_nodes; i++) {
		if (!kona_icc_is_cpu_memory_path(&qp->nodes[i]))
			continue;
		/* Force either complete packed ownership or safe hybrid legacy votes. */
		qp->last_ab[i] = U64_MAX;
		qp->last_ib[i] = U64_MAX;
		kona_icc_mark_dirty(qp, i);
	}
	mutex_unlock(&qp->vote_lock);

	if (enabled)
		dev_err(qp->provider.dev,
			"kona-rpmh: packed runtime gate enabled mask=%u\n",
			READ_ONCE(kona_packed_group_mask));
	else
		dev_err(qp->provider.dev,
			"kona-rpmh: packed runtime gate disabled; restoring legacy CPU votes\n");

	kona_icc_queue_replay(qp, 0, enabled ? "packed-runtime-enable" :
			      "packed-runtime-disable");
}


static bool kona_icc_replay_req_votes(struct kona_icc_provider *qp)
{
	bool need_retry = false;
	bool packed_skip_legacy = false;
	unsigned long i;

	if (!qp || !qp->eff_ab || !qp->eff_ib || !kona_icc_snapshot_dirty(qp))
		return false;

	if (kona_cpu_model_stage() >= 4 && READ_ONCE(kona_packed_runtime_enable) &&
	    READ_ONCE(kona_packed_group_mask) && !qp->packed_fallback_active) {
		bool any_cpu_memory_activity = false;

		for_each_set_bit(i, qp->replay_scan_nodes, qp->num_nodes)
			if (kona_icc_is_cpu_memory_path(&qp->nodes[i])) {
				any_cpu_memory_activity = true;
				break;
			}

		if (any_cpu_memory_activity)
			kona_icc_process_packed_aggregate(qp, &packed_skip_legacy);
	}

	for_each_set_bit(i, qp->replay_scan_nodes, qp->num_nodes) {
		u64 ab = qp->eff_ab[i];
		u64 ib = qp->eff_ib[i];
		bool retry = false;
		int ret;

		if (ab == U64_MAX || ib == U64_MAX) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

		if (kona_icc_is_replay_suppressed_path(&qp->nodes[i])) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

		if (packed_skip_legacy && kona_icc_is_cpu_memory_path(&qp->nodes[i])) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

		if (kona_icc_vote_is_unchanged(qp, i, ab, ib)) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

		ret = kona_icc_send_node_votes(qp, i, ab, ib, &retry, NULL);
		if (ret == 0) {
			kona_icc_clear_dirty(qp, i);
		} else if (retry) {
			need_retry = true;
		} else {
			/*
			 * Preserve dirty state on hard errors so votes are not
			 * lost forever; retry_work will run again on the next
			 * queued replay or icc_set_bw() update.
			 */
			dev_warn_ratelimited(qp->provider.dev,
				"kona-icc: replay failed for %s (ab=%llu ib=%llu)\n",
				qp->nodes[i].name, ab, ib);
		}
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
		u64 req_ab = qp->req_ab[i];
		u64 req_ib = qp->req_ib[i];
		u64 ab = qp->eff_ab ? qp->eff_ab[i] : req_ab;
		u64 ib = qp->eff_ib ? qp->eff_ib[i] : req_ib;
		bool req_unset = (req_ab == U64_MAX && req_ib == U64_MAX);
		bool req_zero = (!req_unset && !req_ab && !req_ib);

		if (qp->nodes[i].role != role)
			continue;
		if (!kona_icc_is_dirty(qp, i))
			continue;
		if (kona_icc_is_replay_suppressed_path(&qp->nodes[i])) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

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
					kona_icc_clear_dirty(qp, i);
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
				kona_icc_clear_dirty(qp, i);
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

		if (!ab && !ib) {
			kona_icc_clear_dirty(qp, i);
			continue;
		}

		ret = kona_icc_send_node_votes(qp, i, ab, ib, &retry, NULL);
		if (ret == -EAGAIN || retry)
			need_retry = true;
		else if (!ret)
			kona_icc_clear_dirty(qp, i);
		else
			dev_warn_ratelimited(qp->provider.dev,
				"kona-icc: replay failed for %s (ab=%llu ib=%llu ret=%d)\n",
				qp->nodes[i].name, ab, ib, ret);
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
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_DSP, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_MEDIA, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_STORAGE, false);
		qp->resume_phase = 2;
		schedule_delayed_work(&qp->retry_work,
				      msecs_to_jiffies(KONA_RESUME_PHASE2_DELAY_MS));
		return need_retry;
	case 2:
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_GPU, false);
		need_retry |= kona_icc_replay_req_votes_role(qp, KONA_ROLE_GMU, false);
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

	if (!qp->eff_ab || !qp->eff_ib)
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
	mutex_lock(&qp->vote_lock);
	need_retry = kona_icc_replay_req_votes_phased(qp);
	mutex_unlock(&qp->vote_lock);

	if (need_retry)
		kona_icc_queue_replay(qp, KONA_RETRY_DELAY_MS, "provider-not-ready");
}

static bool kona_icc_vote_valid(u64 ab, u64 ib)
{
	return ab != U64_MAX && ib != U64_MAX &&
	       ab <= KONA_ICC_MAX_LOGICAL_VOTE &&
	       ib <= KONA_ICC_MAX_LOGICAL_VOTE;
}

/* vote_lock serializes the telemetry and all effective-vote cache updates. */
static void kona_icc_validate_vote(struct kona_icc_provider *qp,
				   const struct kona_icc_node_desc *desc,
				   u64 raw_ab, u64 raw_ib, u64 *ab, u64 *ib)
{
	u64 bad_ab = *ab;
	u64 bad_ib = *ib;

	if (likely(kona_icc_vote_valid(bad_ab, bad_ib)))
		return;

	qp->invalid_vote_count++;
	qp->last_invalid_raw_ab = raw_ab;
	qp->last_invalid_raw_ib = raw_ib;
	qp->last_invalid_eff_ab = bad_ab;
	qp->last_invalid_eff_ib = bad_ib;
	qp->last_invalid_node_id = desc->id;
	qp->last_invalid_node_name = desc->name;

	/* Raw callback votes are u32 and are therefore always sender-safe. */
	*ab = min_t(u64, raw_ab, KONA_ICC_MAX_LOGICAL_VOTE);
	*ib = min_t(u64, raw_ib, KONA_ICC_MAX_LOGICAL_VOTE);
	dev_err_ratelimited(qp->provider.dev,
			"kona-icc: invalid effective vote id=%u name=%s raw=%llu/%llu effective=%llu/%llu fallback=%llu/%llu count=%llu\n",
			desc->id, desc->name, (unsigned long long)raw_ab,
			(unsigned long long)raw_ib, (unsigned long long)bad_ab,
			(unsigned long long)bad_ib, (unsigned long long)*ab,
			(unsigned long long)*ib,
			(unsigned long long)qp->invalid_vote_count);
}

static int __kona_icc_set(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	struct kona_icc_provider *qp;
	const struct kona_icc_node_desc *desc;
	u64 prev_req_ab = U64_MAX, prev_req_ib = U64_MAX;
	u64 prev_eff_ab = U64_MAX, prev_eff_ib = U64_MAX;
	u64 ab = (u64)avg_bw;
	u64 ib = (u64)peak_bw;
	unsigned int index;

	if (IS_ERR_OR_NULL(path) || !path->provider)
		return -EINVAL;

	qp = dev_get_drvdata(path->provider->dev);
	if (!qp)
		return -EINVAL;

	index = (unsigned int)(uintptr_t)path->data;
	if (index >= qp->num_nodes)
		return -EINVAL;

	desc = &qp->nodes[index];
	KONA_VOTE_TRACE(qp, desc, "callback-entry", avg_bw, peak_bw);
	KONA_VOTE_TRACE(qp, desc, "after-initialization", ab, ib);

	/*
	 * Kona v2 RPMh BCMs expect interconnect votes in KB/s (decimal 1000)
	 * packed into cmd.data. Keep ICC consumer units aligned to KB/s to
	 * avoid u32 saturation and unit skew across clients.
	 */
	if (kona_vote_debug && kona_icc_is_cpu_memory_path(desc))
		dev_info(qp->provider.dev,
			 "kona-vote: id=%u raw=%u/%u initial=%llu/%llu base=%llu/%llu\n",
			 desc->id, avg_bw, peak_bw, (unsigned long long)ab,
			 (unsigned long long)ib, (unsigned long long)ab,
			 (unsigned long long)ib);

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
	KONA_VOTE_TRACE(qp, desc, "before-floor", ab, ib);
	if ((ab || ib) && kona_perf_floor_enable && kona_icc_stage >= 4 &&
	    !kona_icc_is_policy_suppressed_path(desc)) {
		kona_icc_apply_floor(qp, desc, avg_bw, peak_bw, &ab, &ib);
	}
	KONA_VOTE_TRACE(qp, desc, "after-floor", ab, ib);
	KONA_VOTE_TRACE(qp, desc, "before-hysteresis", ab, ib);
	if ((ab || ib) && kona_perf_floor_enable && kona_icc_stage >= 4 &&
	    !kona_icc_is_policy_suppressed_path(desc))
		kona_icc_apply_hysteresis(qp, desc, index, &ab, &ib);
	KONA_VOTE_TRACE(qp, desc, "after-hysteresis", ab, ib);

	/*
	 * Keep-alive vote for CPU/GPU/NPU paths when clients briefly request 0/0
	 * between bursts; this avoids repeated collapses into deep bus idle states.
	 *
	 * Raw-only clients must preserve exact 0/0 collapse semantics while their
	 * ICC exposure is validated.
	 */
	KONA_VOTE_TRACE(qp, desc, "before-keepalive", ab, ib);
	if (!kona_icc_is_policy_suppressed_path(desc))
		kona_icc_apply_keepalive_vote(qp, index, &ab, &ib);
	KONA_VOTE_TRACE(qp, desc, "after-keepalive", ab, ib);

	KONA_VOTE_TRACE(qp, desc, "before-gpu-turbo", ab, ib);
	if (desc->id == KONA_ICC_GPU_TO_MEM) {
		kona_icc_update_gpu_llcc_turbo(qp, ib);
	} else if (qp->last_ib) {
		unsigned int gpu_mem_index = 0;

		if (kona_find_desc(qp, KONA_ICC_GPU_TO_MEM, &gpu_mem_index))
			kona_icc_update_gpu_llcc_turbo(qp, qp->last_ib[gpu_mem_index]);
	}

	kona_icc_apply_gpu_llcc_turbo(qp, desc, &ab, &ib);
	KONA_VOTE_TRACE(qp, desc, "after-gpu-turbo", ab, ib);
	if (kona_vote_debug && kona_icc_is_cpu_memory_path(desc))
		dev_info(qp->provider.dev,
			 "kona-vote: id=%u after-floor-policy-resume-turbo=%llu/%llu\n",
			 desc->id, (unsigned long long)ab,
			 (unsigned long long)ib);


skip_perf_floor:
#endif

#ifdef DEBUG
	if (desc->role == KONA_ROLE_CPU ||
	    desc->role == KONA_ROLE_CPU_PRIME)
		pr_info("kona-icc: %s avg=%uKB/s peak=%uKB/s -> ab=%lluKB/s ib=%lluKB/s prev ab/ib=%llu/%llu\n",
			desc->name, avg_bw, peak_bw,
			(unsigned long long)ab, (unsigned long long)ib,
			(unsigned long long)(qp->last_ab ? qp->last_ab[index] : 0),
			(unsigned long long)(qp->last_ib ? qp->last_ib[index] : 0));
#endif
	
	/*
	 * Short post-resume anti-collapse window for DISPLAY: some clients
	 * transiently vote 0/0 during panel re-enable sequencing. On battery
	 * this can collapse interconnect too early and wedge panel bring-up.
	 */
	KONA_VOTE_TRACE(qp, desc, "before-display-resume", ab, ib);
	if (desc->role == KONA_ROLE_DISPLAY && !ab && !ib &&
	    kona_icc_display_runtime_active(qp) &&
	    !atomic_read(&qp->votes_paused) &&
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
				desc->name, ab, ib);
	}
	KONA_VOTE_TRACE(qp, desc, "after-display-resume", ab, ib);

	/*
	 * Hard non-zero fallback for DISPLAY paths: avoid 0/0 collapse on ddr and
	 * config-path links where panel/SDE/dispcc sequences can stall.
	 */
	KONA_VOTE_TRACE(qp, desc, "before-display-fallback", ab, ib);
	if (desc->role == KONA_ROLE_DISPLAY && !ab && !ib &&
	    kona_display_nonzero_floor_enable &&
	    kona_icc_display_runtime_active(qp)) {
		kona_icc_get_display_nonzero_floor(desc->id, &ab, &ib);
		if (kona_resume_debug)
			dev_info_ratelimited(qp->provider.dev,
				"kona-icc: fallback non-zero DISPLAY floor for %s: ab=%llu ib=%llu\n",
				desc->name, ab, ib);
	}
	KONA_VOTE_TRACE(qp, desc, "after-display-fallback", ab, ib);

	/* Validate the final transformed value before cache, replay, or submission. */
	KONA_VOTE_TRACE(qp, desc, "before-validator", ab, ib);
	kona_icc_validate_vote(qp, desc, (u64)avg_bw, (u64)peak_bw, &ab, &ib);
	KONA_VOTE_TRACE(qp, desc, "after-validator", ab, ib);
	KONA_VOTE_TRACE(qp, desc, "before-cache", ab, ib);
	if (kona_vote_debug && kona_icc_is_cpu_memory_path(desc))
		dev_info(qp->provider.dev,
			 "kona-vote: id=%u before-cache=%llu/%llu\n", desc->id,
			 (unsigned long long)ab, (unsigned long long)ib);

	/*
	 * Cache both the raw client request and the final effective vote. Resume
	 * policy still needs the raw 0/0 intent, but deferred/retry replay must use
	 * the fully transformed vote (floors, keepalive, display fallback, turbo) so
	 * transient RPMh busy windows do not replay under-sized bandwidth and cause
	 * app-switch/scroll jank.
	 */
	if (qp->req_ab)
		prev_req_ab = qp->req_ab[index];
	if (qp->req_ib)
		prev_req_ib = qp->req_ib[index];
	if (qp->eff_ab)
		prev_eff_ab = qp->eff_ab[index];
	if (qp->eff_ib)
		prev_eff_ib = qp->eff_ib[index];
	if (qp->req_ab)
		qp->req_ab[index] = (u64)avg_bw;
	if (qp->req_ib)
		qp->req_ib[index] = (u64)peak_bw;
	if (qp->eff_ab)
		qp->eff_ab[index] = ab;
	if (qp->eff_ib)
		qp->eff_ib[index] = ib;
	if (WARN_ON_ONCE(!qp->eff_ab || !qp->eff_ib ||
			 qp->eff_ab[index] != ab || qp->eff_ib[index] != ib))
		return -EIO;
	KONA_VOTE_TRACE(qp, desc, "after-cache", qp->eff_ab[index],
			qp->eff_ib[index]);
	if (kona_vote_debug && kona_icc_is_cpu_memory_path(desc))
		dev_info(qp->provider.dev,
			 "kona-vote: id=%u after-cache req=%llu/%llu eff=%llu/%llu\n",
			 desc->id, (unsigned long long)qp->req_ab[index],
			 (unsigned long long)qp->req_ib[index],
			 (unsigned long long)qp->eff_ab[index],
			 (unsigned long long)qp->eff_ib[index]);
	if (kona_icc_is_cpu_memory_path(desc) &&
	    !qp->first_cpu_request_seen) {
		qp->first_cpu_request_seen = true;
		dev_err(qp->provider.dev,
			"kona-rpmh: first CPU request id=%u name=%s raw=%u/%u policy=%llu/%llu stage=%u\n",
			desc->id, desc->name, avg_bw, peak_bw,
			(unsigned long long)ab, (unsigned long long)ib,
			kona_cpu_model_stage());
	}
	if (prev_req_ab != avg_bw || prev_req_ib != peak_bw ||
	    prev_eff_ab != ab || prev_eff_ib != ib)
		kona_icc_mark_dirty(qp, index);
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
	if (desc->role == KONA_ROLE_DISPLAY && (ab || ib) &&
	    (avg_bw || peak_bw)) {
		qp->saved_ab[index] = ab;
		qp->saved_ib[index] = ib;
	}

	if (kona_cpu_model_stage() >= 4 && kona_icc_is_cpu_memory_path(desc)) {
		bool skip_legacy = false;

		kona_icc_process_packed_aggregate(qp, &skip_legacy);
		if (skip_legacy) {
			kona_icc_clear_dirty(qp, index);
			return 0;
		}
	}

	/*
	 * Avoid redundant RPMh writes when the effective vote is unchanged.
	 * Keep req_* and saved_* updates above so resume replay still tracks
	 * the newest client intent even when programming can be skipped.
	 */
	if (kona_icc_vote_is_unchanged(qp, index, ab, ib)) {
		kona_icc_clear_dirty(qp, index);
		return 0;
	}

	/*
	 * CRYPTO is special on Kona: the real bandwidth path is the legacy CE0
	 * BCM, not CPU_MEM_AB/CPU_MEM_IB and not raw CE0 cmd-db programming from
	 * this virtual provider. When enabled, bridge ICC votes to a dedicated
	 * msm_bus CE0 client so Qualcomm's existing RPMh/BCM backend encodes the
	 * CE0 vote safely. If the bridge is disabled, keep the previous safe no-op.
	 */
	if (kona_icc_is_crypto_path(desc) && !kona_crypto_raw_icc_enable) {
		int ret;

		if (kona_crypto_ce0_msm_bus_enable) {
			ret = kona_icc_send_crypto_ce0_vote(qp, index, ab, ib);
			if (ret)
				return ret;
		}

		kona_icc_clear_dirty(qp, index);
		return 0;
	}

	/*
	 * GPU/GMU bandwidth is primarily controlled by KGSL/GMU HFI/TCS on Kona.
	 * Expose these ICC paths so clients can probe and vote normally, but do not
	 * also program the same GPU_MEM/GPU_LLCC RPMh resources from this virtual
	 * provider unless explicitly enabled for debug. This avoids duplicate or
	 * racing memory votes during heavy benchmark loads while still caching the
	 * accepted client vote and suppressing replay storms.
	 */
	if (kona_icc_is_gpu_path(desc) && !kona_gpu_raw_icc_enable) {
		if (qp->last_ab)
			qp->last_ab[index] = ab;
		if (qp->last_ib)
			qp->last_ib[index] = ib;
		kona_icc_clear_dirty(qp, index);
		return 0;
	}

	if (kona_icc_is_gmu_path(desc) && !kona_gmu_raw_icc_enable) {
		if (qp->last_ab)
			qp->last_ab[index] = ab;
		if (qp->last_ib)
			qp->last_ib[index] = ib;
		kona_icc_clear_dirty(qp, index);
		return 0;
	}

	/*
	 * UFS/SDHC consumers on Kona issue frequent 0/0 <-> high bandwidth ICC
	 * updates during sequential read/write bursts. These logical nodes are
	 * intentionally exposed for DT compatibility at stage 7, but their current
	 * cmd-db aliases reuse CPU_MEM/CPU_LLCC BCMs instead of a validated storage
	 * path. Programming those raw aliases concurrently with CPU, display and
	 * GPU/GMU votes can wedge apps_rsc/RPMh hard enough to leave no pstore.
	 *
	 * Mirror the CRYPTO safety model: accept and cache storage ICC requests, but
	 * do not send raw RPMh storage votes unless explicitly enabled for debug.
	 */
	if (kona_icc_is_storage_path(desc) && !kona_storage_raw_icc_enable) {
		if (qp->last_ab)
			qp->last_ab[index] = ab;
		if (qp->last_ib)
			qp->last_ib[index] = ib;
		kona_icc_clear_dirty(qp, index);
		return 0;
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
			kona_icc_mark_dirty(qp, index);

			if (kona_resume_debug && (ab || ib))
				dev_info_ratelimited(qp->provider.dev,
					"kona-icc: deferred vote node=%s reason=%s ab=%llu ib=%llu\n",
					desc->name, reason, ab, ib);

			return 0;
		}
	}

	{
		bool missing_alias = false;
		bool retry = false;
		int ret = kona_icc_send_node_votes(qp, index, ab, ib, &retry,
						   &missing_alias);

		if (ret == -EAGAIN || retry) {
			kona_icc_mark_dirty(qp, index);
			kona_icc_queue_replay(qp, KONA_RETRY_DELAY_MS, "send-eagain");
		} else if (ret == -ENODEV && missing_alias) {
			kona_icc_mark_dirty(qp, index);

			return 0;
		} else if (ret) {
			return ret;
		} else {
			kona_icc_clear_dirty(qp, index);
		}
	}

	return 0;
}

static int kona_icc_set(struct icc_path *path, u32 avg_bw, u32 peak_bw)
{
	struct kona_icc_provider *qp;
	int ret;

	if (IS_ERR_OR_NULL(path) || !path->provider)
		return -EINVAL;

	qp = dev_get_drvdata(path->provider->dev);
	if (!qp)
		return -EINVAL;

	/*
	 * icc_set_bw() only serializes callers of the same logical path.  Kona
	 * exposes several logical CPU paths backed by the same CPU_MEM and
	 * CPU_LLCC BCMs, so two devbw/memlat workers can otherwise both compute
	 * an aggregate and then send their votes out of order.  In that race the
	 * older, smaller vote can land last and leave DDR/LLCC under-voted until
	 * another governor update, severely hurting both single- and multi-core
	 * workloads.  Cover the effective-vote update, shared aggregation, RPMh
	 * transaction, and cache update with a provider-wide mutex.
	 */
	mutex_lock(&qp->vote_lock);
	ret = __kona_icc_set(path, avg_bw, peak_bw);
	mutex_unlock(&qp->vote_lock);

	return ret;
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

static ssize_t physical_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct kona_icc_node_sysfs *node = dev_get_drvdata(dev);
	struct kona_bcm_state *bcm;
	const char *packed_mode;
	ssize_t len = 0;
	unsigned int i;

	if (!node || !node->qp)
		return -EINVAL;
	if (!kona_icc_is_cpu_memory_path(&node->qp->nodes[node->index]))
		return sysfs_emit(buf, "legacy-resource\n");
	if (node->qp->packed_fallback_active)
		packed_mode = "sticky-legacy-fallback";
	else if (kona_cpu_model_stage() < 4 ||
		 !READ_ONCE(kona_packed_runtime_enable) ||
		 !READ_ONCE(kona_packed_group_mask))
		packed_mode = "stage4-unarmed";
	else if (READ_ONCE(kona_packed_dry_run))
		packed_mode = "stage4-armed-dry-run";
	else if (!READ_ONCE(kona_packed_real_write_enable))
		packed_mode = "stage4-real-blocked";
	else
		packed_mode = "stage4-armed-real";

	for (i = 0; i < KONA_CPU_BCM_COUNT; i++) {
		bcm = &node->qp->cpu_bcms[i];
		len += scnprintf(buf + len, PAGE_SIZE - len,
			"stage=%u %s addr=%#x vcd=%u width=%u unit=%u raw=%llu/%llu req=%llu/%llu "
			"saturated=%u/%u committed=%llu/%llu data=%#x dry_data=%#x generation=%llu/%llu dry_generation=%llu dirty=%u retries=%u failures=%u fallback=%u last_error=%d saturation=%u\n",
			kona_cpu_model_stage(), bcm->name, bcm->addr, bcm->vcd, bcm->width, bcm->unit,
			bcm->raw_x, bcm->raw_y, bcm->requested_x, bcm->requested_y,
			bcm->saturated_x, bcm->saturated_y,
			bcm->committed_x, bcm->committed_y,
			bcm->committed_data, bcm->dry_run_data, bcm->requested_generation,
			bcm->committed_generation, bcm->dry_run_generation, bcm->dirty, bcm->retry_count,
			bcm->failure_count, bcm->fallback, bcm->last_error,
			bcm->saturation_count);
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,
		"invalid_votes=%llu last_invalid_node=%u/%s raw=%llu/%llu effective=%llu/%llu packed_mode=%s packed_runtime_enable=%u packed_group_mask=%u packed_force_dirty=%u packed_dry_run=%u packed_real_write_enable=%u packed_real_write_once=%u packed_real_write_consumed=%u packed_submissions=%llu packed_dry_run_builds=%llu packed_dry_run_skips=%llu packed_update_generation=%llu packed_processed_generation=%llu packed_update_pending=%u replay_pending=%u packed_fallback_active=%u packed_fallback_count=%llu last_failed_vcd=%d last_packed_error=%d last_legacy_fallback_error=%d current_aggregate=%llu/%llu/%llu/%llu dry_aggregate=%llu/%llu/%llu/%llu real_aggregate=%llu/%llu/%llu/%llu packed_config_generation=%llu packed_observed_generation=%llu aggregate_builds=%llu aggregate_unchanged_skips=%llu last_dry_run_error=%d pending_cause=%s\n",
		(unsigned long long)node->qp->invalid_vote_count,
		node->qp->last_invalid_node_id,
		node->qp->last_invalid_node_name ?: "none",
		(unsigned long long)node->qp->last_invalid_raw_ab,
		(unsigned long long)node->qp->last_invalid_raw_ib,
		(unsigned long long)node->qp->last_invalid_eff_ab,
		(unsigned long long)node->qp->last_invalid_eff_ib,
		packed_mode,
		READ_ONCE(kona_packed_runtime_enable),
		READ_ONCE(kona_packed_group_mask),
		READ_ONCE(kona_packed_force_dirty),
		READ_ONCE(kona_packed_dry_run),
		READ_ONCE(kona_packed_real_write_enable),
		READ_ONCE(kona_packed_real_write_once),
		READ_ONCE(node->qp->packed_real_write_consumed),
		(unsigned long long)node->qp->packed_submission_count,
		(unsigned long long)node->qp->packed_dry_run_build_count,
		(unsigned long long)node->qp->packed_dry_run_skip_count,
		(unsigned long long)atomic64_read(&node->qp->packed_update_generation),
		(unsigned long long)node->qp->packed_processed_generation,
		node->qp->packed_processed_generation !=
			atomic64_read(&node->qp->packed_update_generation),
		delayed_work_pending(&node->qp->retry_work),
		node->qp->packed_fallback_active,
		(unsigned long long)node->qp->packed_fallback_count,
		node->qp->last_failed_vcd, node->qp->last_packed_error,
		node->qp->last_legacy_fallback_error,
		(unsigned long long)node->qp->packed_current_inputs.llcc_avg,
		(unsigned long long)node->qp->packed_current_inputs.llcc_peak,
		(unsigned long long)node->qp->packed_current_inputs.mem_avg,
		(unsigned long long)node->qp->packed_current_inputs.mem_peak,
		(unsigned long long)node->qp->packed_last_dry_run_inputs.llcc_avg,
		(unsigned long long)node->qp->packed_last_dry_run_inputs.llcc_peak,
		(unsigned long long)node->qp->packed_last_dry_run_inputs.mem_avg,
		(unsigned long long)node->qp->packed_last_dry_run_inputs.mem_peak,
		(unsigned long long)node->qp->packed_last_real_inputs.llcc_avg,
		(unsigned long long)node->qp->packed_last_real_inputs.llcc_peak,
		(unsigned long long)node->qp->packed_last_real_inputs.mem_avg,
		(unsigned long long)node->qp->packed_last_real_inputs.mem_peak,
		(unsigned long long)node->qp->packed_config_generation,
		(unsigned long long)node->qp->packed_observed_generation,
		(unsigned long long)node->qp->packed_aggregate_build_count,
		(unsigned long long)node->qp->packed_aggregate_unchanged_skip_count,
		node->qp->last_packed_dry_run_error,
		node->qp->packed_processed_generation !=
			atomic64_read(&node->qp->packed_update_generation) ?
			"config" : delayed_work_pending(&node->qp->retry_work) ?
			"generic-dirty-replay" : "none");
	return len;
}

static DEVICE_ATTR_RO(ab);
static DEVICE_ATTR_RO(ib);
static DEVICE_ATTR_RO(res);
static DEVICE_ATTR_RO(physical);

static struct attribute *kona_icc_attrs[] = {
	&dev_attr_ab.attr,
	&dev_attr_ib.attr,
	&dev_attr_res.attr,
	&dev_attr_physical.attr,
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
	kona_icc_mark_all_dirty(qp);

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
	qp->gpu_llcc_turbo = false;
#endif
}

static int kona_icc_display_notifier_cb(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct kona_icc_provider *qp =
		container_of(nb, struct kona_icc_provider, display_nb);
	struct msm_drm_notifier *evdata = data;
	int *blank;
	int i;

	if (event != MSM_DRM_EARLY_EVENT_BLANK && event != MSM_DRM_EVENT_BLANK)
		return NOTIFY_DONE;

	if (!evdata || evdata->id != MSM_DRM_PRIMARY_DISPLAY || !evdata->data)
		return NOTIFY_DONE;

	blank = evdata->data;

	switch (*blank) {
	case MSM_DRM_BLANK_UNBLANK:
		WRITE_ONCE(qp->display_active, true);
		WRITE_ONCE(qp->display_off_jiffies, 0);
		break;
	case MSM_DRM_BLANK_POWERDOWN:
		WRITE_ONCE(qp->display_active, false);
		WRITE_ONCE(qp->display_off_jiffies, jiffies);
		for (i = 0; i < qp->num_nodes; i++) {
			if (qp->nodes[i].role != KONA_ROLE_DISPLAY)
				continue;
			qp->saved_ab[i] = U64_MAX;
			qp->saved_ib[i] = U64_MAX;
		}
		break;
	default:
		break;
	}

	return NOTIFY_OK;
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
	struct device *rpmh_dev;
	struct kona_icc_provider *qp;
	u64 __maybe_unused ab, ib;
	int ret, i;

	if (!kona_icc_stage) {
		dev_warn(&pdev->dev,
			 "kona-icc: stage 0 rescue mode; provider disabled so consumers use msm_bus fallback\n");
		return -ENODEV;
	}

	if (kona_icc_stage > KONA_ICC_STAGE_MAX) {
		dev_warn(&pdev->dev, "kona-icc: clamping stage %u to %u (full)\n",
			 kona_icc_stage, KONA_ICC_STAGE_MAX);
		kona_icc_stage = KONA_ICC_STAGE_MAX;
	}

	ret = kona_icc_validate_nodes(&pdev->dev, kona_nodes, ARRAY_SIZE(kona_nodes));
	if (ret)
		return ret;

	rpmh_dev = msm_bus_rpmh_get_rsc_client(MSM_BUS_RSC_APPS);
	if (!rpmh_dev) {
		dev_info(&pdev->dev,
			 "kona-rpmh: APPS RSC client unavailable; deferring probe\n");
		return -EPROBE_DEFER;
	}
	dev_info(&pdev->dev, "kona-rpmh: resolved APPS RSC client %s\n",
		 dev_name(rpmh_dev));

	qp = devm_kzalloc(&pdev->dev, sizeof(*qp), GFP_KERNEL);
	if (!qp)
		return -ENOMEM;
	qp->rpmh_dev = rpmh_dev;
	qp->last_failed_vcd = -1;
	qp->cpu_bcms[KONA_CPU_BCM_SH4].name = "SH4";
	qp->cpu_bcms[KONA_CPU_BCM_SH0].name = "SH0";
	qp->cpu_bcms[KONA_CPU_BCM_MC0].name = "MC0";
	/* ACV is solver-owned on Kona; never synthesize a normal X/Y vote for it. */

	/*
	 * Some panels briefly drop their SDE vote to 0/0 while re-enabling.
	 * Boards that opt in need display blank/unblank notifications so the
	 * fallback floor is applied only while the panel is actually active.
	 */
	qp->display_protection = of_property_read_bool(pdev->dev.of_node,
					       "qcom,display-icc-protection");

	if (data) {
		qp->nodes = data->nodes;
		qp->num_nodes = data->num_nodes;
		qp->boot_floor_vote = data->boot_floor_vote;
	} else {
		qp->nodes = kona_nodes;
		qp->num_nodes = ARRAY_SIZE(kona_nodes);
		/* Never issue one-shot RPMh floor votes from provider probe by default. */
		qp->boot_floor_vote = false;
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

	qp->eff_ab = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->eff_ab)
		return -ENOMEM;

	qp->eff_ib = devm_kcalloc(&pdev->dev, qp->num_nodes, sizeof(u64),
				  GFP_KERNEL);
	if (!qp->eff_ib)
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

	qp->dirty_nodes = bitmap_zalloc(qp->num_nodes, GFP_KERNEL);
	if (!qp->dirty_nodes)
		return -ENOMEM;

	qp->replay_scan_nodes = bitmap_zalloc(qp->num_nodes, GFP_KERNEL);
	if (!qp->replay_scan_nodes) {
		bitmap_free(qp->dirty_nodes);
		return -ENOMEM;
	}

	spin_lock_init(&qp->dirty_lock);
	mutex_init(&qp->vote_lock);


	INIT_DELAYED_WORK(&qp->retry_work, kona_icc_retry_workfn);
	/* Use phased replay only after resume(); steady-state retries stay immediate. */
	qp->resume_phase = 3;
	atomic_set(&qp->deferred_votes, 0);
	atomic_set(&qp->replay_runs, 0);
	atomic_set(&qp->display_replay_skips, 0);
	atomic_set(&qp->replay_queue_skips, 0);
	qp->display_active = true;
	qp->display_hints_available = false;
	qp->display_off_jiffies = 0;
	qp->display_nb.notifier_call = kona_icc_display_notifier_cb;

	for (i = 0; i < qp->num_nodes; i++) {
		qp->last_ab[i] = U64_MAX;
		qp->last_ib[i] = U64_MAX;
		qp->req_ab[i] = U64_MAX;
		qp->req_ib[i] = U64_MAX;
		qp->eff_ab[i] = U64_MAX;
		qp->eff_ib[i] = U64_MAX;
		qp->saved_ab[i] = U64_MAX;
		qp->saved_ib[i] = U64_MAX;
	}
	kona_icc_mark_all_dirty(qp);

        qp->provider.dev = &pdev->dev;
        qp->provider.of_node = pdev->dev.of_node;
        qp->provider.xlate = kona_icc_xlate;
        qp->provider.set = kona_icc_set;
        qp->provider.release = kona_icc_release;

	platform_set_drvdata(pdev, qp);

	ret = kona_icc_validate_display_nodes(qp);
	if (ret)
		goto err_free_bitmaps;

	ret = icc_provider_register(&qp->provider);
        if (ret)
		goto err_free_bitmaps;

	ret = kona_icc_register_sysfs(pdev, qp);
	if (ret) {
                kona_icc_unregister_sysfs(qp);
                icc_provider_unregister(&qp->provider);
		goto err_free_bitmaps;
        }

	if ((!kona_display_notifier_enable && !qp->display_protection) ||
	    kona_icc_stage < 4) {
		dev_info(&pdev->dev,
			 "kona-icc: display notifier disabled (param=%d dt=%d stage=%u)\n",
			 kona_display_notifier_enable, qp->display_protection,
			 kona_icc_stage);
		ret = -ENODEV;
	} else {
		ret = msm_drm_register_client(&qp->display_nb);
	}

	if (ret) {
		/*
		 * Display notifications are policy hints, not a hard dependency for
		 * the ICC provider. Keep the provider registered so CPU/GPU/devbw
		 * clients can attach to ICC, but disable display-hint-only behavior
		 * and skip the one-shot boot floor votes for this staged bring-up.
		 */
		dev_warn(&pdev->dev,
			 "display notifier unavailable (%d); keeping ICC provider without display hints\n",
			 ret);
		qp->display_nb_registered = false;
		WRITE_ONCE(qp->display_hints_available, false);
		qp->boot_floor_vote = false;
	} else {
		qp->display_nb_registered = true;
		WRITE_ONCE(qp->display_hints_available, true);
	}

#ifdef CONFIG_INTERCONNECT_QCOM_KONA_PERF_FLOOR
	if (qp->boot_floor_vote && kona_icc_stage >= 4 &&
	    kona_cpu_model_stage() < 4) {
		for (i = 0; i < qp->num_nodes; i++) {
			int r_ab, r_ib;

			if (qp->nodes[i].role == KONA_ROLE_DISPLAY ||
			    kona_icc_is_policy_suppressed_path(&qp->nodes[i]))
				continue;

			ab = KONA_ICC_MIN_AB_FLOOR_KB;
			ib = KONA_ICC_MIN_IB_FLOOR_KB;
			kona_icc_apply_floor(qp, &qp->nodes[i], ab, ib, &ab, &ib);

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
		 "kona-icc: provider registered stage=%u implemented=%zu enabled=%zu gpu/gmu=%s display_notifier=%d boot_floor=%d perf_floor=%d\n",
		 kona_icc_stage, qp->num_nodes, kona_icc_enabled_node_count(qp),
		 (kona_icc_stage >= 1) ? "enabled" : "disabled",
		 qp->display_nb_registered, qp->boot_floor_vote,
		 kona_perf_floor_enable && kona_icc_stage >= 4);
	dev_err(&pdev->dev,
		"kona-rpmh: provider probe complete migration_stage=%u legacy_compat=%u\n",
		kona_cpu_model_stage(), kona_rpmh_cpu_model);
	if (kona_cpu_model_stage() >= 1) {
		for (i = 0; i < KONA_CPU_BCM_COUNT; i++) {
			ret = kona_icc_cpu_bcm_metadata(qp, &qp->cpu_bcms[i]);
			if (ret == -EAGAIN) {
				dev_err(&pdev->dev,
					"kona-rpmh: %s metadata temporarily unavailable; first request will retry\n",
					qp->cpu_bcms[i].name);
				continue;
			}
			if (ret) {
				qp->cpu_bcms[i].fallback = true;
				qp->cpu_bcms[i].last_error = ret;
				dev_err(&pdev->dev,
					"kona-rpmh: fallback %s permanent metadata error=%d\n",
					qp->cpu_bcms[i].name, ret);
				/* Keep ownership coherent: all CPU aliases remain legacy. */
				if (kona_cpu_model_stage() >= 4)
					kona_rpmh_model = 1;
			}
		}
	}
	if (kona_cpu_model_stage() == 2 || kona_cpu_model_stage() == 3)
		dev_err(&pdev->dev,
			"kona-rpmh: stage %u validates metadata only: local CPU aliases cannot split SH4/SH0 ownership; legacy programming retained\n",
			kona_cpu_model_stage());

	mutex_lock(&kona_packed_param_lock);
	kona_packed_provider = qp;
	mutex_unlock(&kona_packed_param_lock);

        return 0;

err_free_bitmaps:
	bitmap_free(qp->replay_scan_nodes);
	qp->replay_scan_nodes = NULL;
	bitmap_free(qp->dirty_nodes);
	qp->dirty_nodes = NULL;
	return ret;
}

static int kona_icc_remove(struct platform_device *pdev)
{
	struct kona_icc_provider *qp = platform_get_drvdata(pdev);

	mutex_lock(&kona_packed_param_lock);
	if (kona_packed_provider == qp)
		kona_packed_provider = NULL;
	mutex_unlock(&kona_packed_param_lock);

	cancel_delayed_work_sync(&qp->retry_work);

	if (qp->display_nb_registered) {
		msm_drm_unregister_client(&qp->display_nb);
		qp->display_nb_registered = false;
	}

	kona_icc_unregister_sysfs(qp);
	icc_provider_unregister(&qp->provider);

	mutex_lock(&kona_crypto_ce0_lock);
	if (kona_crypto_ce0_client) {
		msm_bus_scale_unregister_client(kona_crypto_ce0_client);
		kona_crypto_ce0_client = 0;
		kona_crypto_ce0_last_valid = false;
	}
	mutex_unlock(&kona_crypto_ce0_lock);

	bitmap_free(qp->replay_scan_nodes);
	bitmap_free(qp->dirty_nodes);

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
