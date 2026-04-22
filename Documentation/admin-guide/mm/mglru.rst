=========
Multi-Gen LRU (MGLRU)
=========

This kernel tree carries a backport-oriented Multi-Gen LRU implementation.
It keeps the classic active/inactive lists as the physical data structure,
while using generation metadata to bias aging and scan selection.

Runtime controls
================

The following sysctls are available under ``/proc/sys/vm``:

* ``lru_gen_enabled``: enable/disable MGLRU reclaim heuristics.
* ``lru_gen_min_ttl_ms``: protect oldest generations for a minimum age.
* ``lru_gen_age_period_ms``: target cadence for generation advancement.
* ``lru_gen_weight_anon_pct``: bias anon-vs-file reclaim weighting.
* ``lru_gen_dedup_window_ms``: deduplicate bursty access samples.
* ``lru_gen_pressure_normalize``: normalize pressure counters over time.
* ``lru_gen_ptwalk_pages``: cap PTE samples per reclaim-triggered page table walk.
* reclaim-context MM walks first sample ``current->mm`` and can
  opportunistically sample a fallback userspace mm during global reclaim
  when reclaim runs in kernel threads.

When ``CONFIG_SYSFS=y``, the same tunables are also available under
``/sys/kernel/mm/lru_gen/``:

* ``enabled``
* ``min_ttl_ms``
* ``age_period_ms``
* ``weight_anon_pct``
* ``dedup_window_ms``
* ``pressure_normalize``
* ``ptwalk_pages``

Debugfs interface
=================

When ``CONFIG_DEBUG_FS=y``, MGLRU exposes ``/sys/kernel/debug/mglru/stats``.

Reading this file shows per-node generation and pressure state.

Writing supported commands:

* ``age``: force one generation advance on each online node.
* ``age=<n>``: force up to ``n`` generation advances per online node
  (clamped to ``MAX_NR_GENS``).
* ``reset``: reset pressure/tier accounting without disabling MGLRU.
* ``enable``: turn MGLRU on (same effect as ``lru_gen_enabled=1``).
* ``disable``: turn MGLRU off (same effect as ``lru_gen_enabled=0``).
* ``sample_mm``: run an immediate bounded PTE-young sampling pass over the
  current task MM on every online node.

Procfs interface
================

MGLRU also exposes ``/proc/lru_gen`` for environments where debugfs is not
mounted in production.

Reading ``/proc/lru_gen`` returns a compact per-node snapshot similar to the
debugfs stats output.

Writing accepts the following commands:

* ``age`` / ``age=<n>`` / ``reset`` / ``enable`` / ``disable`` /
  ``sample_mm`` (same semantics as debugfs).
* ``min_ttl_ms=<n>``: set ``lru_gen_min_ttl_ms``.
* ``age_period_ms=<n>``: set ``lru_gen_age_period_ms``.
* ``weight_anon_pct=<n>``: set ``lru_gen_weight_anon_pct``.
* ``dedup_window_ms=<n>``: set ``lru_gen_dedup_window_ms``.
* ``normalize=<0|1>``: set ``lru_gen_pressure_normalize``.
* ``ptwalk_pages=<n>``: set ``lru_gen_ptwalk_pages``.

VM statistics
=============

MGLRU contributes the following counters to ``/proc/vmstat`` when enabled:

* ``mglru_aged``: number of generation-advance events.
* ``mglru_evicted``: number of pages observed leaving active aging paths.
* ``mglru_activated``: number of access samples fed into MGLRU feedback.
* ``mglru_deduped``: number of access samples filtered by deduplication.
* ``mglru_mm_walk_success``: number of reclaim-time MM walks that found young
  pages to age.
* ``mglru_mm_walk_fail``: number of reclaim-time MM walks that produced no
  aging signal.
* ``mglru_mm_walk_fallback``: number of MM walk attempts that used a fallback
  process context instead of ``current``.
* debugfs/proc snapshots include ``mm_walk=(ok/fail/fallback)`` and ``stall``
  to show MM walk coverage and reclaim-stall feedback state.

Tracepoints
===========

MGLRU also exports reclaim tracepoints under the ``vmscan`` tracepoint group:

* ``mm_vmscan_lru_gen_advance``: emitted when a node advances to a new
  generation sequence and reports oldest-generation state.
* ``mm_vmscan_lru_gen_feedback``: emitted when reclaim feedback retunes MGLRU
  pressure/tier heuristics.

Reclaim integration
===================

MGLRU hooks into reclaim through vmscan and now uses an extended multi-pass
loop per node:

* tries multiple generation-aware reclaim passes before giving up;
* retries with ``memcg_low_reclaim`` enabled if early passes stall;
* temporarily increases reclaim aggressiveness when scanning repeatedly
  produces no reclaimed pages;
* exits early when oldest generations are empty so classic reclaim logic can
  continue balancing without spinning in empty MGLRU generations.
* samples young PTEs from direct-reclaim callers (bounded by
  ``lru_gen_ptwalk_pages``) to provide page-table-walk aging feedback.
* in kernel-thread reclaim contexts, periodically samples a fallback userspace
  mm to keep aging signals active when ``current->mm`` is unavailable.
* tracks reclaim-stall streaks and accelerates aging/pressure feedback under
  persistent low reclaim efficiency.

Validation guidance (long-tail workloads)
=========================================

For correctness/performance validation across mixed workloads, exercise at
least these scenarios while collecting ``/proc/lru_gen`` snapshots before,
during, and after pressure:

* latency-sensitive anon-heavy reclaim (app switch + foreground service load);
* file-cache-heavy streaming + background writes (sustained dirty/writeback);
* memcg-protected hierarchies (``memory.low``/``memory.min`` stress);
* kswapd-only pressure windows (background reclaim without direct reclaim mm).

Track whether ``stall`` converges back down after pressure relief and whether
``mm_walk fallback`` remains bounded (indicating reclaim-context MM sampling
helps coverage without over-triggering).
