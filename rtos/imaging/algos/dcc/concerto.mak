ifeq ($(TARGET_CPU), $(filter $(TARGET_CPU), X86 x86_64 A72 R5F))

include $(PRELUDE)
TARGET      := vx_target_kernels_imaging_iss_dcc
TARGETTYPE  := library
CSOURCES    := $(call all-c-files)

include $(FINALE)

endif
