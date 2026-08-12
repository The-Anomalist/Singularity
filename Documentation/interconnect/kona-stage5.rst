.. SPDX-License-Identifier: GPL-2.0-only

Kona packed BCM Stage 5 ownership audit
=======================================

Stage 4 remains the authoritative Kona ICC writer for the CPU SH4, SH0 and
MC0 BCMs.  Stage 5 reserves a separate ``packed_family_mask`` (GPU bit 0, GMU
bit 1) so later validated clients do not overload the CPU group mask.

GPU and GMU audit result
------------------------

The four Kona logical paths are client-facing compatibility paths.  Their
``GPU_MEM_AB/IB`` and ``GPU_LLCC_AB/IB`` names must not be treated as proof of
independent physical BCM ownership:

* KGSL selects either GMU DCVS, ICC, or its legacy msm_bus client for runtime
  bandwidth.  ICC failure can fall back to msm_bus.
* The GMU driver queries the msm_bus backend for DDR TCS commands and copies
  those commands into the HFI bandwidth table consumed by firmware.
* The command-db verification bridge identifies that DDR table as MC0, SH0
  and ACV.  ACV is solver-owned and is not a normal X/Y bandwidth BCM.
* GMU bootstrap can also use msm_bus and the firmware table is restored as
  part of GMU bring-up.  Consequently an apps-RSC replay from Kona ICC would
  be a duplicate physical owner.

There is no source or DT contract in this tree that transfers ownership of an
independent GPU-only BCM from KGSL/GMU to the Kona virtual provider.  Stage 5
therefore exposes the audited family, aggregate and blocking reason in each
GPU/GMU ``physical`` attribute, but deliberately performs no new packed write.
Arming a family bit records validation intent; it cannot override external
ownership.  The deprecated raw GPU/GMU switches remain off by default and are
not enabled by Stage 5.

Safe enablement requirements
----------------------------

Real packed ownership can be added to the generic family mapping only after
all of the following are available:

#. a named command-db BCM with valid BCM auxiliary metadata (address, unit,
   width and VCD), rather than a logical AB/IB alias;
#. a KGSL/GMU contract disabling both runtime msm_bus fallback and firmware
   HFI/TCS programming/replay for that same physical resource;
#. a documented hand-off order covering probe, GPU power collapse, system
   suspend/resume and driver removal; and
#. hardware validation of zero transitions and failure recovery.

The diagnostic aggregation ignores ``U64_MAX`` cache entries.  This preserves
the invalid/uninitialized sentinel and prevents it from becoming a saturated
physical request in a future transport implementation.
