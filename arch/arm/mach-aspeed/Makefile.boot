# SPDX-License-Identifier: GPL-2.0

ifeq ($(CONFIG_ARCH_AST2605),y)
	zreladdr-y	+= 0x82008000
	params_phys-y	:= 0x82000100
endif
