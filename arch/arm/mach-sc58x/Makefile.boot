ifeq ($(CONFIG_MACH_SC589_EZKIT),y)
zreladdr-y	+= 0xC2008000
params_phys-y	:= 0xC2000100
else
zreladdr-y	+= 0x80008000
params_phys-y	:= 0x80000100
endif
