#
# Makefile for the linux kernel.
#

obj-y					:= core.o clock.o
ifneq ($(CONFIG_VERSATILE_PARROT6), y)
obj-$(CONFIG_ARCH_VERSATILE_PB)		+= versatile_pb.o
else
obj-$(CONFIG_ARCH_VERSATILE_PB)		+= versatile_p6.o
obj-$(CONFIG_ARCH_VERSATILE_PB)		+= devs.o
endif
obj-$(CONFIG_MACH_VERSATILE_AB)		+= versatile_ab.o
obj-$(CONFIG_PCI)			+= pci.o
