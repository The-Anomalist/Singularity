#!/bin/bash
# SPDX-License-Identifier: GPL-2.0

# Extended MGLRU smoke/regression checks.
# Run as root for write-path coverage.

ksft_skip=4
sysctl_dir=/proc/sys/vm
proc_lru_gen=/proc/lru_gen

require_file()
{
	local path="$1"
	[ -e "$path" ] || {
		echo "[SKIP] missing $path"
		exit $ksft_skip
	}
}

read_int()
{
	cat "$1" 2>/dev/null
}

write_int()
{
	local val="$1"
	local path="$2"
	echo "$val" > "$path"
}

vmstat_read()
{
	local key="$1"
	awk -v key="$key" '$1 == key { print $2; found = 1; exit } END { if (!found) print "" }' /proc/vmstat
}

require_vmstat_key()
{
	local key="$1"
	local val

	val=$(vmstat_read "$key")
	[ -n "$val" ] || {
		echo "[SKIP] missing vmstat key: $key"
		exit $ksft_skip
	}
}

expect_ge()
{
	local lhs="$1"
	local rhs="$2"
	local msg="$3"

	[ "$lhs" -ge "$rhs" ] || {
		echo "[FAIL] $msg (got $lhs expected >= $rhs)"
		exit 1
	}
}

require_file "$sysctl_dir/lru_gen_enabled"
require_file "$sysctl_dir/lru_gen_min_ttl_ms"
require_file "$sysctl_dir/lru_gen_age_period_ms"
require_file "$sysctl_dir/lru_gen_weight_anon_pct"
require_file "$sysctl_dir/lru_gen_dedup_window_ms"
require_file "$sysctl_dir/lru_gen_pressure_normalize"
require_file "$sysctl_dir/lru_gen_ptwalk_pages"
require_file "$proc_lru_gen"
require_vmstat_key mglru_aged
require_vmstat_key mglru_mm_walk_success
require_vmstat_key mglru_mm_walk_fail
require_vmstat_key mglru_mm_walk_fallback

if [ ! -w "$sysctl_dir/lru_gen_enabled" ] || [ ! -w "$proc_lru_gen" ]; then
	echo "[SKIP] test needs root (write access to vm sysctls and /proc/lru_gen)"
	exit $ksft_skip
fi

orig_enabled=$(read_int "$sysctl_dir/lru_gen_enabled")
orig_min_ttl=$(read_int "$sysctl_dir/lru_gen_min_ttl_ms")
orig_age_period=$(read_int "$sysctl_dir/lru_gen_age_period_ms")
orig_weight=$(read_int "$sysctl_dir/lru_gen_weight_anon_pct")
orig_dedup=$(read_int "$sysctl_dir/lru_gen_dedup_window_ms")
orig_norm=$(read_int "$sysctl_dir/lru_gen_pressure_normalize")
orig_ptwalk=$(read_int "$sysctl_dir/lru_gen_ptwalk_pages")

cleanup()
{
	write_int "$orig_enabled" "$sysctl_dir/lru_gen_enabled" || true
	write_int "$orig_min_ttl" "$sysctl_dir/lru_gen_min_ttl_ms" || true
	write_int "$orig_age_period" "$sysctl_dir/lru_gen_age_period_ms" || true
	write_int "$orig_weight" "$sysctl_dir/lru_gen_weight_anon_pct" || true
	write_int "$orig_dedup" "$sysctl_dir/lru_gen_dedup_window_ms" || true
	write_int "$orig_norm" "$sysctl_dir/lru_gen_pressure_normalize" || true
	write_int "$orig_ptwalk" "$sysctl_dir/lru_gen_ptwalk_pages" || true
}
trap cleanup EXIT

# Toggle enabled state.
write_int 1 "$sysctl_dir/lru_gen_enabled"
[ "$(read_int "$sysctl_dir/lru_gen_enabled")" = "1" ] || exit 1
write_int 0 "$sysctl_dir/lru_gen_enabled"
[ "$(read_int "$sysctl_dir/lru_gen_enabled")" = "0" ] || exit 1
write_int 1 "$sysctl_dir/lru_gen_enabled"
[ "$(read_int "$sysctl_dir/lru_gen_enabled")" = "1" ] || exit 1

# Verify range clamping behavior exposed by implementation.
write_int 1 "$sysctl_dir/lru_gen_age_period_ms"
[ "$(read_int "$sysctl_dir/lru_gen_age_period_ms")" = "100" ] || exit 1
write_int 999999 "$sysctl_dir/lru_gen_age_period_ms"
[ "$(read_int "$sysctl_dir/lru_gen_age_period_ms")" = "60000" ] || exit 1

write_int 999 "$sysctl_dir/lru_gen_weight_anon_pct"
[ "$(read_int "$sysctl_dir/lru_gen_weight_anon_pct")" = "100" ] || exit 1

write_int 5000 "$sysctl_dir/lru_gen_dedup_window_ms"
[ "$(read_int "$sysctl_dir/lru_gen_dedup_window_ms")" = "1000" ] || exit 1

write_int 1 "$sysctl_dir/lru_gen_ptwalk_pages"
[ "$(read_int "$sysctl_dir/lru_gen_ptwalk_pages")" = "32" ] || exit 1
write_int 999999 "$sysctl_dir/lru_gen_ptwalk_pages"
[ "$(read_int "$sysctl_dir/lru_gen_ptwalk_pages")" = "16384" ] || exit 1

# Validate /proc/lru_gen controls and output shape.
aged_before=$(vmstat_read mglru_aged)
walk_ok_before=$(vmstat_read mglru_mm_walk_success)
walk_fail_before=$(vmstat_read mglru_mm_walk_fail)
walk_fb_before=$(vmstat_read mglru_mm_walk_fallback)

echo enable > "$proc_lru_gen"
echo age=2 > "$proc_lru_gen"
echo sample_mm > "$proc_lru_gen"
echo min_ttl_ms=1000 > "$proc_lru_gen"
[ "$(read_int "$sysctl_dir/lru_gen_min_ttl_ms")" = "1000" ] || exit 1
echo normalize=0 > "$proc_lru_gen"
[ "$(read_int "$sysctl_dir/lru_gen_pressure_normalize")" = "0" ] || exit 1
echo normalize=1 > "$proc_lru_gen"
[ "$(read_int "$sysctl_dir/lru_gen_pressure_normalize")" = "1" ] || exit 1
echo reset > "$proc_lru_gen"
if echo definitely_invalid_command > "$proc_lru_gen" 2>/dev/null; then
	echo "[FAIL] invalid /proc/lru_gen command unexpectedly succeeded"
	exit 1
fi

grep -q '^enabled=' "$proc_lru_gen" || exit 1
grep -q '^node=' "$proc_lru_gen" || exit 1
grep -q 'mm_walk=' "$proc_lru_gen" || exit 1

aged_after=$(vmstat_read mglru_aged)
walk_ok_after=$(vmstat_read mglru_mm_walk_success)
walk_fail_after=$(vmstat_read mglru_mm_walk_fail)
walk_fb_after=$(vmstat_read mglru_mm_walk_fallback)

expect_ge "$aged_after" "$aged_before" "mglru_aged must be monotonic"
expect_ge "$walk_ok_after" "$walk_ok_before" "mglru_mm_walk_success must be monotonic"
expect_ge "$walk_fail_after" "$walk_fail_before" "mglru_mm_walk_fail must be monotonic"
expect_ge "$walk_fb_after" "$walk_fb_before" "mglru_mm_walk_fallback must be monotonic"

echo "[PASS] MGLRU extended control-plane smoke test"
