ifeq ($(CONFIG_MACH_SC594_SOM_EZKIT),y)
zreladdr-y      += 0x82008000
params_phys-y   := 0x82000100
else
zreladdr-y	+= 0x80008000
params_phys-y	:= 0x80000100
endif
