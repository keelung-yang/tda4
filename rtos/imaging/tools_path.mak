
PSDK_PATH = $(abspath ..)
$(info PSDK_PATH=$(PSDK_PATH))

TIOVX_PATH ?= $(PSDK_PATH)/tiovx

# paths for components shared between tiovx and imaging are specified in below
# file in tiovx, ex, bios, tidl, pdk, cgtools, ...
include $(TIOVX_PATH)/psdkra_tools_path.mak

TIOVX_CUSTOM_KERNEL_PATH ?= $(TIOVX_PATH)/kernels_j7
