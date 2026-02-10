# UCLASS scheduler core

UCLASS (Unified Capacity and Latency Aware Scheduling System) is staged as a
layer on top of the existing 4.19 scheduler so devices can adopt it safely.

## Directory layout

- `core.c`: central UCLASS policy helpers that are called from stock CFS paths.
- `uclass.h`: UCLASS API exposed to scheduler internals.
- `Makefile`: build rules for the UCLASS objects.

## Migration strategy toward a standalone scheduler

1. **Stage 1 (current):** helper-based policy hooks only.
2. **Stage 2:** split wakeup, placement, and migration logic into dedicated
   files under `kernel/sched/uclass/` while retaining stock entry points.
3. **Stage 3:** add UCLASS-owned runqueue accounting helpers and tracing.
4. **Stage 4:** optional full scheduler-class style replacement with build-time
   fallback to stock CFS/WALT for bring-up safety.

## Safety principles for SM8250 / 4.19

- Keep existing task lifecycle/state transitions untouched.
- Keep all changes runtime-tunable via sysctl + sched_features.
- Use stock control-flow paths to avoid boot regressions.
