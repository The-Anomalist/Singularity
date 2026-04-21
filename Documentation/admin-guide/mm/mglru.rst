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

Debugfs interface
=================

When ``CONFIG_DEBUG_FS=y``, MGLRU exposes ``/sys/kernel/debug/mglru/stats``.

Reading this file shows per-node generation and pressure state.

Writing supported commands:

* ``age``: force one generation advance on each online node.
* ``reset``: reset pressure/tier accounting without disabling MGLRU.
* ``enable``: turn MGLRU on (same effect as ``lru_gen_enabled=1``).
* ``disable``: turn MGLRU off (same effect as ``lru_gen_enabled=0``).

VM statistics
=============

MGLRU contributes the following counters to ``/proc/vmstat`` when enabled:

* ``mglru_aged``: number of generation-advance events.
* ``mglru_evicted``: number of pages observed leaving active aging paths.
* ``mglru_activated``: number of access samples fed into MGLRU feedback.

