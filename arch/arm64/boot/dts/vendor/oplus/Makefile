ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY),y)
    dtbo-$(CONFIG_ARCH_KONA) += \
        kona-instantnoodle-overlay.dtbo \
        kona-instantnoodlep-overlay.dtbo \
        kona-kebab-overlay.dtbo \
        kona-lemonades-overlay.dtbo

    # instantnoodle shipped on the original Kona/A650 revision.  Do not pair
    # its overlay with the v2 GPU/interconnect descriptions.
    kona-instantnoodle-overlay.dtbo-base := kona.dtb

    # instantnoodlep follows kebab's Kona revision coverage; panel differences
    # remain confined to each device's display overlay.
    kona-instantnoodlep-overlay.dtbo-base := kona.dtb kona-v2.dtb kona-v2.1.dtb
    kona-kebab-overlay.dtbo-base := kona.dtb kona-v2.dtb kona-v2.1.dtb
    kona-lemonades-overlay.dtbo-base := kona.dtb kona-v2.dtb kona-v2.1.dtb
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files    := *.dtb *.dtbo
