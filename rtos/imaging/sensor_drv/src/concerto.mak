ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72 R5F))

include $(PRELUDE)
TARGET      := ti_imaging_sensordrv
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
CSOURCES    += imx390/iss_sensor_imx390.c
CSOURCES    += ar0233/iss_sensor_ar0233.c
CSOURCES    += ar0820/iss_sensor_ar0820.c
CSOURCES    += ub9xx_raw_test_pattern/iss_sensor_raw_testpat.c
CSOURCES    += ub9xx_yuv_test_pattern/iss_sensor_testpat.c
CSOURCES    += gw_ar0233_yuv/iss_sensor_gw_ar0233.c

IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(VISION_APPS_PATH)/utils/remote_service/include
IDIRS       += $(VISION_APPS_PATH)/utils/ipc/include
IDIRS       += $(PDK_PATH)/packages


include $(FINALE)

endif
