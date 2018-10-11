#
# Makefile for the Linux "Tulip" family network device drivers.
#
EXTRA_CFLAGS += -Dfinal_version

obj-$(CONFIG_PILOT4_NET_TULIP)		+= tulip.o

# Declare multi-part drivers.

tulip-objs			:= eeprom.o interrupt.o media.o \
				   timer.o tulip_core.o		\
				   21142.o pnic.o pnic2.o pilot_phy.o

