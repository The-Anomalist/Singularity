# SPDX-License-Identifier: GPL-2.0
#
# WireGuard in-kernel build integration

ccflags-y := -D'pr_fmt(fmt)=KBUILD_MODNAME ": " fmt'
ccflags-y += -Wframe-larger-than=2048
ccflags-$(CONFIG_WIREGUARD_DEBUG) += -DDEBUG -g
ccflags-y += -I$(src)
ccflags-y += -I$(src)/crypto/include

wireguard-y := main.o
wireguard-y += noise.o
wireguard-y += device.o
wireguard-y += peer.o
wireguard-y += timers.o
wireguard-y += queueing.o
wireguard-y += send.o
wireguard-y += receive.o
wireguard-y += socket.o
wireguard-y += peerlookup.o
wireguard-y += allowedips.o
wireguard-y += ratelimiter.o
wireguard-y += cookie.o
wireguard-y += netlink.o

include $(src)/crypto/Kbuild.include
include $(src)/compat/Kbuild.include

obj-$(CONFIG_WIREGUARD) := wireguard.o
