# SPDX-License-Identifier: GPL-2.0-or-later
#
# Makefile - AMD SFH HID drivers
# Copyright (c) 2019-2020, Advanced Micro Devices, Inc.
#
#

ccflags-m := -Werror
   obj-$(CONFIG_AMD_SFH_HID) +=amd-sfhtp-hid.o
   amd-sfhtp-hid-objs := amdsfh_hid.o
   amd-sfhtp-hid-objs+= amdsfh_hid_client.o
   amd-sfhtp-hid-objs+= amd_mp2_pcie.o
   amd-sfhtp-hid-objs+= hid_descriptor/amd_sfh_hid_descriptor.o

ccflags-y += -I$(srctree)/$(src)/
