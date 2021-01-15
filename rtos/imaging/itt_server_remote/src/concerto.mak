ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), A72 R5F))

include $(PRELUDE)
TARGET      := ti_imaging_ittsrvr
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(VISION_APPS_PATH)/utils/remote_service/include
IDIRS       += $(VISION_APPS_PATH)/utils/ipc/include
IDIRS       += $(PDK_PATH)/packages
IDIRS       += $(HOST_ROOT)/itt_server_remote/include


include $(FINALE)

endif
