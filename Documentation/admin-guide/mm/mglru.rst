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

Debugfs interface
=================

When ``CONFIG_DEBUG_FS=y``, MGLRU exposes ``/sys/kernel/debug/mglru/stats``.

Reading this file shows per-node generation and pressure state.

Writing supported commands:

* ``age``: force one generation advance on each online node.
* ``reset``: reset pressure/tier accounting without disabling MGLRU.
* ``enable``: turn MGLRU on (same effect as ``lru_gen_enabled=1``).
* ``disable``: turn MGLRU off (same effect as ``lru_gen_enabled=0``).

Procfs interface
================

MGLRU also exposes ``/proc/lru_gen`` for environments where debugfs is not
mounted in production.

Reading ``/proc/lru_gen`` returns a compact per-node snapshot similar to the
debugfs stats output.

Writing accepts the following commands:

* ``age`` / ``reset`` / ``enable`` / ``disable`` (same semantics as debugfs).
* ``min_ttl_ms=<n>``: set ``lru_gen_min_ttl_ms``.
* ``age_period_ms=<n>``: set ``lru_gen_age_period_ms``.
* ``weight_anon_pct=<n>``: set ``lru_gen_weight_anon_pct``.
* ``dedup_window_ms=<n>``: set ``lru_gen_dedup_window_ms``.
* ``normalize=<0|1>``: set ``lru_gen_pressure_normalize``.

VM statistics
=============

MGLRU contributes the following counters to ``/proc/vmstat`` when enabled:

* ``mglru_aged``: number of generation-advance events.
* ``mglru_evicted``: number of pages observed leaving active aging paths.
* ``mglru_activated``: number of access samples fed into MGLRU feedback.
