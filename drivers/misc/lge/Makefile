
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
MTK_CUSTOM_PATH  := $(srctree)/drivers/misc/mediatek

ccflags-y += -I$(MTK_CUSTOM_PATH)/auxadc/$(MTK_PLATFORM)
ccflags-y += -I$(MTK_CUSTOM_PATH)/include

obj-$(CONFIG_LGE_LUT_KCAL) +=  lge_lcd_kcal.o lge_mtk_lcdc_luts.o

obj-$(CONFIG_MACH_LGE) += devices_lge.o

obj-$(CONFIG_MACH_LGE) += lge_blocking_monitor.o

obj-$(CONFIG_LGE_BOOTLOADER_LOG) += lge_bootloader_log.o

obj-$(CONFIG_LGE_HANDLE_PANIC) += lge_handle_panic.o
obj-$(CONFIG_LGE_HIDDEN_RESET) += lge_hidden_reset.o

obj-$(CONFIG_PRE_SELF_DIAGNOSIS) += lge_pre_selfd.o

obj-$(CONFIG_MACH_LGE) += lge_power_sysfs.o

obj-$(CONFIG_LGE_QFPROM_INTERFACE) += lge_efuse_access.o
