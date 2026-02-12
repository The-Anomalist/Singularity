# UCLASS scheduler core

UCLASS (Unified Capacity and Latency Aware Scheduling System) is staged as a
layer on top of the existing 4.19 scheduler so devices can adopt it safely.

## Directory layout

- `core.c`: shared UCLASS feature gating helpers.
- `wakeup.c`: wakeup preemption helpers and wakeup granularity shaping.
- `placement.c`: task placement helpers for idle-first and prev-CPU bias.
- `uclass.h`: UCLASS API exposed to scheduler internals.
- `Makefile`: build rules for the UCLASS objects.

## Migration strategy toward a standalone scheduler

1. **Stage 1:** helper-based policy hooks only.
2. **Stage 2 (current):** split wakeup and placement logic into dedicated
   compilation units while retaining stock fair/EAS entry points.
3. **Stage 3:** add UCLASS-owned runqueue accounting helpers and tracing.
4. **Stage 4:** optional full scheduler-class style replacement with build-time
   fallback to stock CFS/WALT for bring-up safety.

## UCLASS tunables (stage 2)

- `sched_uclass_wakeup_boost`: gate wakeup granularity boost path.
- `sched_uclass_gran_boost_pct`: percentage reduction of wakeup granularity.
- `sched_uclass_idle_bias`: allow UCLASS idle-first candidate scoring.
- `sched_uclass_prefer_prev_cpu`: keep latency-sensitive tasks on prev CPU
  when it remains a valid candidate.
- `sched_uclass_prev_cpu_energy_margin_pct`: minimum energy saving required
  before migrating away from prev CPU.

## Safety principles for SM8250 / 4.19

- Keep existing task lifecycle/state transitions untouched.
- Keep all changes runtime-tunable via sysctl + sched_features.
- Use stock control-flow paths to avoid boot regressions.
