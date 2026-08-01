Kona RPMh interconnect bring-up
==============================

Boot selection
--------------

``kona.rpmh_model`` is boot-only: 0 selects legacy, 1 discovers command-db
metadata, 2 validates SH4 ownership, 3 validates SH4 and SH0 ownership, and 4
programs SH4/SH0/MC0.  Stages 2 and 3 intentionally retain legacy programming:
the local ``CPU_LLCC_*`` and ``CPU_MEM_*`` aliases do not provide separable
physical ownership.  Programming one packed BCM alongside those aliases would
allow two voters to own the same fabric.  Stage 5 is reserved for subsequently
validated clients.  The deprecated ``kona.rpmh_cpu_model=1`` maps to stage 1.

The local msm_bus implementation defines an eight-byte BCM auxiliary record:
32-bit unit size, 16-bit width, 8-bit clock domain (VCD), and one reserved byte.
Addresses and the actual unit/width/VCD values are firmware command-db data and
must be captured from each target; they are not SM8550 constants.  A packed
command has Y in bits 13:0, X in bits 27:14, valid in bit 29, and commit in bit
30.  A zero X/Y vote has valid clear.  Each VCD ends in one commit command and
is submitted as a separate ACTIVE_ONLY RPMh message, matching msm_bus's batch
counts.  ACV remains solver/firmware-owned.

CPU topology
------------

.. list-table::
   :header-rows: 1

   * - Qnode
     - Bus width
     - Channels
     - BCM
     - Status
   * - APPSS
     - 32
     - 2
     - SH4
     - packed at stage 4
   * - LLCC
     - 16
     - 4
     - SH0
     - packed at stage 4
   * - EBI
     - 4
     - 4
     - MC0
     - packed at stage 4; ACV is not synthesized

CPU ICC values enter the provider in kB/s. Independent per-CPU average votes
are summed and peaks are maximized. Generic and per-CPU views overlap and are
therefore maximized rather than added. CPU-to-memory is an end-to-end request;
APPSS and LLCC see the maximum endpoint demand and EBI sees memory demand.

No GPU/GMU, display/multimedia, storage/I/O, compute/networking, or peripheral
BCM is converted by this repair. Their physical qnodes, QoS ports and BCM
ownership are not proven locally, so they remain on their existing legacy or
firmware path. In particular ACV and disabled raw GPU, GMU, UFS, crypto and NPU
aliases remain untouched.

Recovery evidence
-----------------

All discovery, first-request, command construction, submission and fallback
records use error log level so earlycon, ramoops/pstore, last_kmsg, or recovery
dmesg can retain them. Capture them with::

  adb shell 'cat /sys/fs/pstore/* 2>/dev/null; dmesg' > kona-boot.log
  adb shell 'find /sys/class/kona_icc -name physical -exec sh -c "echo {}; cat {}" \;' > kona-bcm.log
  adb shell 'dmesg | grep -E "kona-rpmh|TCS Busy|RPMH message"' > kona-rpmh.log

The first ``kona-rpmh: first CPU request`` line identifies the logical request.
The following ``cmd[0]`` and ``submit`` lines identify the first physical BCM,
metadata, normalized X/Y, payload, commit/wait flags, array position, and RPMh
return code. On the original implementation, the first update which dirtied
BCMs in different VCDs was incorrectly sent as one ``rpmh_write`` message. The
local voter instead forms one message per VCD. Hardware boot logs are required
to name which of SH4, SH0, or MC0 was first on a particular device.

Validation matrix
-----------------

Repeat the following for stages 0, 1, 2, 3 and 4, changing the boot argument
and saving the three evidence files above after every workload::

  adb reboot bootloader
  fastboot boot Image --cmdline 'kona.rpmh_model=N ignore_loglevel earlycon'
  adb wait-for-device; adb reboot                         # warm reboot
  for i in 1 2 3 4 5; do adb reboot; adb wait-for-device; done
  adb shell 'sleep 60'                                   # CPU idle
  adb shell 'taskset 1 stress-ng --cpu 1 --timeout 60s'  # single core
  adb shell 'stress-ng --cpu 8 --timeout 120s'           # multicore
  adb shell 'stress-ng --vm 4 --vm-bytes 70% --timeout 120s'
  adb shell 'am start -n com.primatelabs.geekbench/.HomeActivity'
  adb shell 'input keyevent 26; sleep 60; input keyevent 26'
  adb shell 'echo mem > /sys/power/state'                # deep suspend/resume
  for i in 1 2 3 4 5; do adb shell input keyevent 26; sleep 5; adb shell input keyevent 26; sleep 5; done
  adb shell 'am start -a android.intent.action.VIEW -d file:///sdcard/test.mp4'
  adb shell 'am start -a android.media.action.IMAGE_CAPTURE'
  adb shell 'dd if=/dev/zero of=/data/local/tmp/io.bin bs=4M count=512 conv=fsync'
  adb shell 'svc usb setFunctions mtp; iperf3 -c SERVER -t 120'
  adb shell 'iperf3 -c SERVER -t 120; svc data disable; svc data enable'
  adb shell 'stress-ng --cpu 8 --timeout 600s'            # thermal throttling
  adb shell dumpsys battery                              # repeat plugged/unplugged

A run passes only when requested and committed generations converge, dirty and
fallback are clear, suspend/resume completes, and no new RPMh error appears.
Compare transaction traffic under an identical workload with::

  adb shell 'dmesg -c >/dev/null; WORKLOAD; dmesg | grep -c "TCS Busy"'
  adb shell 'dmesg | sed -n "s/.*addr=\(0x[0-9a-f]*\).*/\1/p" | sort | uniq -c'
  adb shell 'dmesg | grep -c "kona-rpmh: submit"'

Stage 0 is the legacy baseline. Stage 1 measures discovery overhead without
packed submissions. Stage 4 must use no more than one submission per dirty VCD
and must never issue both a legacy CPU alias and packed CPU BCM vote.
