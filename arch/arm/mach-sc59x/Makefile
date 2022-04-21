#
# Makefile for the linux kernel.
#

obj-y					:= core.o dma.o clock.o sec.o
obj-$(CONFIG_ARCH_SC59X)		+= sc59x-ezkit.o
obj-$(CONFIG_ICC:m=y) += icc.o
