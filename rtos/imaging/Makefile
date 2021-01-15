
include tools_path.mak

# Edit below file in tiovx/ to change additional build options
include $(TIOVX_PATH)/build_flags.mak

DIRECTORIES :=
DIRECTORIES += kernels/arm
DIRECTORIES += algos/awb/src
DIRECTORIES += algos/ae/src
DIRECTORIES += algos/dcc/src
DIRECTORIES += kernels/include
DIRECTORIES += kernels/host
DIRECTORIES += sensor_drv/src
DIRECTORIES += itt_server_remote/src

TARGET_COMBOS :=

ifeq ($(BUILD_TARGET_MODE),yes)
  ifeq ($(PROFILE), $(filter $(PROFILE), debug all))
	TARGET_COMBOS += J7:SYSBIOS:R5F:1:debug:TIARMCGT
	TARGET_COMBOS += J7:SYSBIOS:A72:1:debug:GCC_SYSBIOS_ARM
	ifeq ($(BUILD_LINUX_A72),yes)
	  TARGET_COMBOS += J7:LINUX:A72:1:debug:GCC_LINUX_ARM
	endif
    ifeq ($(BUILD_QNX_A72),yes)
      TARGET_COMBOS += J7:QNX:A72:1:debug:GCC_QNX_ARM
    endif
  endif

  ifeq ($(PROFILE), $(filter $(PROFILE), release all))
	TARGET_COMBOS += J7:SYSBIOS:R5F:1:release:TIARMCGT
	TARGET_COMBOS += J7:SYSBIOS:A72:1:release:GCC_SYSBIOS_ARM
	ifeq ($(BUILD_LINUX_A72),yes)
	  TARGET_COMBOS += J7:LINUX:A72:1:release:GCC_LINUX_ARM
	endif
    ifeq ($(BUILD_QNX_A72),yes)
      TARGET_COMBOS += J7:QNX:A72:1:release:GCC_QNX_ARM
    endif
  endif

endif

ifeq ($(BUILD_EMULATION_MODE),yes)
  ifeq ($(OS),Windows_NT)
    ifeq ($(PROFILE), $(filter $(PROFILE), debug all))
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), x86_64 all))
            TARGET_COMBOS += PC:WINDOWS:x86_64:1:debug:GCC_WINDOWS
        endif
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), X86 all))
            TARGET_COMBOS += PC:WINDOWS:X86:1:debug:GCC_WINDOWS
        endif
    endif

    ifeq ($(PROFILE), $(filter $(PROFILE), release all))
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), x86_64 all))
            TARGET_COMBOS += PC:WINDOWS:x86_64:1:release:GCC_WINDOWS
        endif
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), X86 all))
            TARGET_COMBOS += PC:WINDOWS:X86:1:release:GCC_WINDOWS
        endif
    endif
  else
    ifeq ($(PROFILE), $(filter $(PROFILE), debug all))
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), x86_64 all))
            TARGET_COMBOS += PC:LINUX:x86_64:1:debug:GCC_LINUX
        endif
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), X86 all))
            TARGET_COMBOS += PC:LINUX:X86:1:debug:GCC_LINUX
        endif
    endif

    ifeq ($(PROFILE), $(filter $(PROFILE), release all))
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), x86_64 all))
            TARGET_COMBOS += PC:LINUX:x86_64:1:release:GCC_LINUX
        endif
        ifeq ($(BUILD_EMULATION_ARCH), $(filter $(BUILD_EMULATION_ARCH), X86 all))
            TARGET_COMBOS += PC:LINUX:X86:1:release:GCC_LINUX
        endif
    endif
  endif
endif

CONCERTO_ROOT ?= concerto
BUILD_MULTI_PROJECT := 1
BUILD_TARGET := target.mak
BUILD_PLATFORM :=

include $(CONCERTO_ROOT)/rules.mak
