
include $(srctree)/drivers/misc/mediatek/cam_cal/src/Makefile.custom

define FILTER_PROJECT_DRV

$(info $(srctree)/drivers/misc/mediatek/cam_cal/src/$(subst ",,$(CONFIG_MTK_PLATFORM))/$(1)_eeprom)
ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cam_cal/src/$(subst ",,$(CONFIG_MTK_PLATFORM))/$(1)_eeprom),)
platform_cam_cal_drv += $(subst ",,$(CONFIG_MTK_PLATFORM))/$(1)_eeprom/
obj-y += $(subst ",,$(CONFIG_MTK_PLATFORM))/$(1)_eeprom/
else ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cam_cal/src/common/$(1)_eeprom),)
common_cam_cal_drv += common/$(1)_eeprom/
obj-y += common/$(1)_eeprom/
else ifneq ($(wildcard $(srctree)/drivers/misc/mediatek/cam_cal/src/common/$(1)),)
common_cam_cal_drv += common/$(1)/
obj-y += common/$(1)/
else
without_cam_cal_drv += $(1)_eeprom
endif
endef

cam_cal_drv = $(subst ",,$(CONFIG_CUSTOM_KERNEL_CAM_CAL_DRV))

$(foreach drv,$(cam_cal_drv),$(eval $(call FILTER_PROJECT_DRV,$(drv))))

