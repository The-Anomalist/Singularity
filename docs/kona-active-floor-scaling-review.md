# Kona ICC active-display floor scaling review (SM8250 Kona and Kona v2)

This note captures a conservative design review for active-display
workload-aware floor scaling in `drivers/interconnect/qcom/kona.c`. The
provider is selected by both `qcom,kona-interconnect` and
`qcom,kona-v2-interconnect`, so its defaults apply consistently to both
silicon variants.

Key recommendations:
- Classify using `req_max = max(req_ab, req_ib)` (pre-floor request), not post-floor values.
- Apply scaling only when display is active and role is non-display.
- Keep scaling order: path floor -> active-display scale -> minimum AB/IB clamps -> existing bias/headroom.
- Start with global active-display thresholds, keep GPU/GMU boost and hysteresis unchanged.
- Introduce per-role profiles only after measurements show role-specific instability.

Suggested conservative defaults for first pass:
- low threshold: 1,200,000 KB/s
- high threshold: 6,000,000 KB/s
- low scale: 70%
- medium scale: 85%
- high scale: 100%

These values preserve responsiveness for common UI/foreground bursts while allowing non-display floor reduction for lighter active workloads.

The shared thermal description also gives GPU and PoP passive trips a 2°C
hysteresis. This prevents repeated throttle/unthrottle transitions near 95°C;
the GPU zone's passive polling interval is 100 ms, which is sufficient for
that hysteresis while avoiding the previous 10 ms wakeup cadence.
