subdir-ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_model_config_i2c.o
obj-y	+=  lgtp_model_config_misc.o
obj-y	+=  lgtp_platform_api_i2c.o
obj-y	+=  lgtp_platform_api_misc.o
obj-y	+=  lgtp_platform_api_power.o

ifeq ($(CONFIG_JDI_INCELL), y)
obj-y 	+= S3320/
endif

ifeq ($(CONFIG_MTK_TD4191),y)
obj-y	+=  td4191/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LU201X), y)
obj-y 	+= LU202X/
endif


ifeq ($(CONFIG_TOUCHSCREEN_LGE_MELFAS), y)
obj-y 	+= MIT200/
endif

ifeq ($(CONFIG_TOUCHSCREEN_LGE_MIT300), y)
obj-y   += MIT300/
endif
