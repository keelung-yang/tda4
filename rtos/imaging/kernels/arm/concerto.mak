ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 A72 R5F))

include $(PRELUDE)
TARGET      := vx_target_kernels_imaging_aewb
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)
IDIRS       += $(HOST_ROOT)/kernels/include
IDIRS       += $(HOST_ROOT)/kernels/host
IDIRS       += $(VXLIB_PATH)/packages
IDIRS       += $(HOST_ROOT)/algos/awb/include
IDIRS       += $(HOST_ROOT)/algos/ae/include
IDIRS       += $(HOST_ROOT)/algos/dcc/include
IDIRS       += $(HOST_ROOT)/sensor_drv/include
IDIRS       += $(HOST_ROOT)/itt_server_remote/include
IDIRS       += $(PDK_PATH)/packages
IDIRS       += $(VISION_APPS_PATH)/utils/remote_service/include
IDIRS       += $(VISION_APPS_PATH)/utils/ipc/include

ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), x86_64))
IDIRS       += $(J7_C_MODELS_PATH)/include
endif
include $(FINALE)

endif
