#
# Makefile for the linux kernel.
#

obj-y					:= core.o gptimers.o dma.o clock.o sec.o
obj-$(CONFIG_ARCH_SC57X)		+= sc57x-ezkit.o core_control.o
obj-$(CONFIG_ICC:m=y) += icc.o
