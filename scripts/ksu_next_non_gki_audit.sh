#!/usr/bin/env bash
# Read-only KernelSU Next audit for non-GKI Android 4.19 manual-hook kernels.
set -u

root="$(git rev-parse --show-toplevel 2>/dev/null || pwd)"
cd "$root" || exit 2

fail=0
warn=0

note() { printf '%s\n' "$*"; }
pass() { printf 'PASS: %s\n' "$*"; }
missing() { printf 'FAIL: %s\n' "$*"; fail=$((fail + 1)); }
warning() { printf 'WARN: %s\n' "$*"; warn=$((warn + 1)); }

require_file() {
    [ -e "$1" ] && pass "$1 exists" || missing "$1 is missing"
}

require_pattern() {
    local file="$1" pattern="$2" desc="$3"
    if [ ! -e "$file" ]; then
        missing "$desc ($file missing)"
    elif rg -q "$pattern" "$file"; then
        pass "$desc"
    else
        missing "$desc"
    fi
}

note '== KernelSU Next non-GKI manual-hook audit =='

ksu_dir='drivers/kernelsu'
if [ -L "$ksu_dir" ]; then
    target="$(readlink "$ksu_dir")"
    pass "$ksu_dir is a symlink to $target"
elif [ -d "$ksu_dir" ]; then
    pass "$ksu_dir is a directory"
else
    missing "$ksu_dir is missing"
fi

require_file 'arch/arm64/configs/vendor/kona-perf_defconfig'
require_pattern 'arch/arm64/configs/vendor/kona-perf_defconfig' '^CONFIG_KSU=y$' 'CONFIG_KSU is enabled'
require_pattern 'arch/arm64/configs/vendor/kona-perf_defconfig' '^CONFIG_KSU_MANUAL_HOOK=y$' 'manual hooks are enabled'
require_pattern 'arch/arm64/configs/vendor/kona-perf_defconfig' '^# CONFIG_KSU_KPROBES_HOOK is not set$' 'kprobes hook mode is disabled'
require_pattern 'arch/arm64/configs/vendor/kona-perf_defconfig' '^CONFIG_KSU_SUSFS=y$' 'SUSFS is enabled'
require_pattern 'Makefile' '^KSU_GIT_VERSION_VALID \?= 1$' 'KernelSU version metadata fallback is exported'
require_pattern 'Makefile' '^KSU_GIT_TAG \?= v3\.2\.0-legacy$' 'KernelSU version tag fallback is v3.2.0-legacy'
require_pattern 'Makefile' '^KSU_NEXT_MANAGER_HASH \?= 79e590113c4c4c0c222978e413a5faa801666957b1212a328e46c00c69821bf7$' 'KernelSU manager hash fallback matches official manager'

note '== Manual hook markers =='
require_pattern 'kernel/reboot.c' 'ksu_handle_sys_reboot' 'reboot manual hook marker exists'
require_pattern 'fs/exec.c' 'ksu_execveat_hook' 'execveat manual hook gate exists'
require_pattern 'fs/exec.c' 'ksu_handle_execveat_sucompat' 'execveat sucompat manual hook exists'
require_pattern 'fs/exec.c' 'ksu_handle_execveat_ksud' 'execveat ksud manual hook exists'
require_pattern 'fs/open.c' 'ksu_handle_faccessat' 'faccessat manual hook exists'
require_pattern 'fs/stat.c' 'ksu_handle_stat' 'stat manual hook exists'
require_pattern 'fs/read_write.c' 'ksu_handle_sys_read' 'read manual hook exists'
require_pattern 'drivers/input/input.c' 'ksu_handle_input_handle_event' 'input manual hook exists'

note '== KernelSU Next Kbuild/Kconfig expectations =='
if [ -e "$ksu_dir/Kbuild" ]; then
    require_pattern "$ksu_dir/Kbuild" 'KSU_VERSION' 'Kbuild defines/reports KSU_VERSION'
    require_pattern "$ksu_dir/Kbuild" 'KSU_VERSION_TAG' 'Kbuild defines/reports KSU_VERSION_TAG'
    require_pattern "$ksu_dir/Kbuild" 'KSU_NEXT_MANAGER_HASH' 'Kbuild accepts KSU_NEXT_MANAGER_HASH'
    require_pattern "$ksu_dir/Kbuild" 'EXPECTED_MANAGER_HASH' 'Kbuild exports EXPECTED_MANAGER_HASH'
    require_pattern "$ksu_dir/Kbuild" 'CONFIG_KSU_MANUAL_HOOK' 'Kbuild has manual-hook detection'
    require_pattern "$ksu_dir/Kbuild" 'ksu_handle_sys_reboot' 'Kbuild manual-hook marker matches this tree'

    version_define="$(rg -o -- '-DKSU_VERSION=[0-9]+' "$ksu_dir/Kbuild" 2>/dev/null | tail -n1 | sed 's/.*=//' || true)"
    if [ -n "$version_define" ] && [ "$version_define" -lt 33110 ]; then
        warning "static KSU_VERSION $version_define is below v3.2.0 manager minimum 33110"
    fi
else
    warning "$ksu_dir/Kbuild not present; initialize/update KernelSU Next subtree before building"
fi

if [ -e "$ksu_dir/Kconfig" ]; then
    require_pattern "$ksu_dir/Kconfig" 'config KSU_MANUAL_HOOK' 'Kconfig exposes KSU_MANUAL_HOOK'
    require_pattern "$ksu_dir/Kconfig" 'config KSU_KPROBES_HOOK' 'Kconfig exposes KSU_KPROBES_HOOK'
    require_pattern "$ksu_dir/Kconfig" '!KSU_MANUAL_HOOK' 'Kconfig prevents kprobes hook with manual hook'
else
    warning "$ksu_dir/Kconfig not present; initialize/update KernelSU Next subtree before building"
fi

note '== SUSFS headers =='
require_file 'include/linux/susfs.h'
require_file 'include/linux/susfs_def.h'
require_file 'include/linux/sus_su.h'
require_pattern 'include/linux/sus_su.h' 'drivers/kernelsu/core_hook.h|drivers/kernelsu/.*/core_hook.h' 'sus_su header includes KernelSU hook declarations'

note "== Result: $fail failure(s), $warn warning(s) =="
[ "$fail" -eq 0 ] || exit 1
[ "$warn" -eq 0 ] || exit 2
exit 0
