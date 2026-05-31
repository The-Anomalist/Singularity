KernelSU Next non-GKI 4.19 update checklist
============================================

This tree carries KernelSU Next through ``drivers/kernelsu`` with manual
hooks enabled for the non-GKI SM8250 4.19 kernel.  Newer KernelSU Next
managers reject old kernels primarily when the built-in driver reports an old
``KSU_VERSION`` or when the manager APK signature expected by the kernel no
longer matches the manager being used.  Do not switch this tree to GKI mode,
``CONFIG_KSU_KPROBES_HOOK``, or generated kprobes hooks to fix that; update the
KernelSU Next kernel driver/core and keep the existing manual hooks.

Required configuration invariants
---------------------------------

Keep these values in the device defconfig:

* ``CONFIG_KSU=y``
* ``CONFIG_KSU_MANUAL_HOOK=y``
* ``# CONFIG_KSU_KPROBES_HOOK is not set``
* ``CONFIG_KSU_SUSFS=y`` and the existing SUSFS feature options

The current SM8250 defconfig already selects this combination.  In particular,
``CONFIG_KSU_MANUAL_HOOK`` is set and ``CONFIG_KSU_KPROBES_HOOK`` is disabled,
so any KernelSU Next core update must preserve the manual-hook code paths rather
than replacing the hooks.

KernelSU Next driver/core files to compare and update
-----------------------------------------------------

When importing a newer KernelSU Next release, compare the working 3.1.0 legacy
``drivers/kernelsu`` tree against the target release and update these paths as a
set.  Partial updates commonly compile but make the manager show "driver too
old" or "not detected".

Version and manager API reporting
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* top-level ``Makefile``

  * This tree exports ``KSU_GIT_VERSION``, ``KSU_GIT_TAG``,
    ``KSU_GIT_VERSION_VALID``, ``KSU_NEXT_MANAGER_SIZE`` and
    ``KSU_NEXT_MANAGER_HASH`` defaults so a copied/symlinked KernelSU Next
    legacy tree does not fall back to ``KSU_VERSION=1`` when its ``.git``
    metadata is absent.  Build commands can still override these variables.

* ``drivers/kernelsu/Kbuild``

  * Check the ``KSU_VERSION`` calculation and the final
    ``ccflags-y += -DKSU_VERSION=...`` line.  New managers require the kernel
    driver to report at least the manager's minimum supported driver version
    (for v3.2.0 this is 33110).  If the KernelSU Next source is not a real git
    repository at build time, the upstream Kbuild fallback can define
    ``KSU_VERSION=1``; that is the classic cause of "driver too old".
  * Preserve or add the ``KSU_VERSION_TAG`` define so manager diagnostics can
    read the release tag.
  * Keep the manager signature variables and compiler defines in sync:
    ``KSU_NEXT_MANAGER_SIZE``, ``KSU_NEXT_MANAGER_HASH``,
    ``EXPECTED_MANAGER_SIZE`` and ``EXPECTED_MANAGER_HASH``.
  * Keep the manual-hook detection block keyed by
    ``CONFIG_KSU_MANUAL_HOOK`` and do not allow ``CONFIG_KSU_KPROBES_HOOK`` to
    satisfy hook detection in this non-GKI tree.

* ``drivers/kernelsu/include/ksu.h`` or the release-equivalent public header

  * Verify that ``KERNEL_SU_VERSION`` expands to ``KSU_VERSION``.
  * Verify that ``KERNEL_SU_VERSION_TAG`` expands to ``KSU_VERSION_TAG``.

* ``drivers/kernelsu/supercall/*`` and ``drivers/kernelsu/runtime/ksud_integration.*``

  * Compare the manager-facing command structs and get-info response fields.
    KernelSU Next v3.2.0 release notes call out a fix for the wrong
    ``GetInfoCmd`` struct, so do not update only ``Kbuild`` without these
    supercall/runtime files.

Manager signature/hash handling
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``drivers/kernelsu/manager/apk_sign.c``
* ``drivers/kernelsu/manager/throne_tracker.c``
* ``drivers/kernelsu/manager/pkg_observer.c``
* legacy layouts: ``drivers/kernelsu/apk_sign.c`` and related manager files

Update these together with ``Kbuild``.  The Kbuild defaults should match the
manager APK you install:

* official KernelSU Next manager hash:
  ``79e590113c4c4c0c222978e413a5faa801666957b1212a328e46c00c69821bf7``
* official manager signature size default: ``0x3e6``

If you use a spoofed or re-signed manager, pass the matching values at build
time with ``KSU_NEXT_MANAGER_HASH=<sha256>`` and
``KSU_NEXT_MANAGER_SIZE=<size>`` instead of editing the manual hooks.

Kbuild/Kconfig hook detection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

* ``drivers/kernelsu/Kconfig``

  * Keep ``KSU_MANUAL_HOOK`` available when ``KSU=y`` and built-in.
  * Keep ``KSU_KPROBES_HOOK`` dependent on ``!KSU_MANUAL_HOOK``.
  * Do not add a default that turns kprobes on when manual hooks are selected.

* ``drivers/kernelsu/Kbuild``

  * Ensure the manual hook probe checks for a marker that exists in this tree.
    The stable marker here is ``ksu_handle_sys_reboot`` in ``kernel/reboot.c``.
  * For newer KernelSU Next releases that check more than one marker, make sure
    the check accepts all manual hooks listed below before enabling
    ``HAVE_KSU_HOOK := 0``.

Manual hook markers expected in this tree
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

These hooks must remain in the Android 4.19 kernel files.  They are the markers
to compare against newer KernelSU Next manual-hook expectations:

* ``kernel/reboot.c``: ``ksu_handle_sys_reboot``
* ``fs/exec.c``: ``ksu_execveat_hook``, ``ksu_handle_execveat_ksud`` and
  ``ksu_handle_execveat_sucompat`` in ``do_execveat_common()``
* ``fs/open.c``: ``ksu_handle_faccessat``
* ``fs/stat.c``: ``ksu_handle_stat``
* ``fs/read_write.c``: ``ksu_handle_sys_read``
* ``drivers/input/input.c``: ``ksu_handle_input_handle_event``

Do not replace these call sites when updating KernelSU Next.  If a newer
KernelSU Next core renames a handler, add a compatibility wrapper in the
KernelSU source or update only the function declaration/call signature needed by
that release while keeping the call site location intact.

CONFIG_KSU_MANUAL_HOOK compatibility notes
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Newer KernelSU Next legacy code contains syscall-hook revisions used by both
manual and kprobe modes.  On this kernel:

* Keep the manual ``execve`` and ``execveat`` paths centralized through
  ``do_execveat_common()`` after ``getname()`` has produced a
  ``struct filename``.
* Keep ``ksu_execveat_hook`` gating around ``ksu_handle_execveat_ksud`` and
  call ``ksu_handle_execveat_sucompat`` on the same ``struct filename``.
* Keep ``faccessat`` and ``stat`` hooks because newer sucompat paths still rely
  on seeing ``/system/bin/su``/``/system/xbin/su`` style accesses reliably.
* Keep the ``read`` hook for manager/ksud channel compatibility.
* Keep the input hook if the imported release still enables the volume-key or
  safe-mode path.

SUSFS integration compatibility
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

SUSFS touches common KernelSU paths and must be carried forward when refreshing
``drivers/kernelsu``:

* ``include/linux/susfs.h``
* ``include/linux/susfs_def.h``
* ``include/linux/sus_su.h``
* KernelSU core hooks that call SUSFS helpers, especially sucompat, mount
  namespace, allowlist/profile, and any ``sus_su`` integration files in the
  imported release.

When comparing releases, do not drop local ``CONFIG_KSU_SUSFS*`` ifdefs from
KernelSU files.  Re-apply them after taking the newer KernelSU Next core, then
run the audit script below.

Audit command
-------------

Run ``scripts/ksu_next_non_gki_audit.sh`` after refreshing the KernelSU Next
subtree.  It checks the defconfig invariants, manual hook markers, Kbuild/Kconfig
version and manager-signature plumbing, and SUSFS headers.  It is intentionally
read-only and does not modify the manual hooks.
