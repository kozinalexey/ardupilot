# Standard things
sp              := $(sp).x
dirstack_$(sp)  := $(d)
d               := $(dir)
BUILDDIRS       += $(BUILD_PATH)/$(d)
BUILDDIRS       += $(BUILD_PATH)/$(d)/comm
BUILDDIRS       += $(BUILD_PATH)/$(d)/boards/$(BOARD)

LIBRARY_INCLUDES += -I$(d)/comm -I$(d)/boards/$(BOARD)
WIR    := AP_HAL_REVOMINI/wirish

# Local flags
# always include board #defines
CFLAGS_$(d) := -Wall -Werror -include $(WIR)/boards/$(BOARD)/$(BOARD).h


# Local rules and targets
cSRCS_$(d)   :=  
cSRCS_$(d)   += $(WIR)/boards/$(BOARD)/system_stm32f4xx.c # C startup code
#cSRCS_$(d)   += $(WIR)/boards/$(BOARD)/$(BOARD).c

cppSRCS_$(d) := 
cppSRCS_$(d) += $(WIR)/boards/$(BOARD)/$(BOARD).cpp
cppSRCS_$(d) += $(WIR)/boards.cpp

sSRCS_$(d)   := 
sSRCS_$(d)   += $(WIR)/boards/$(BOARD)/$(BOARD)_startup.S


cFILES_$(d)   := $(cSRCS_$(d):%=$(d)/%)
cppFILES_$(d) := $(cppSRCS_$(d):%=$(d)/%)
sFILES_$(d)   := $(sSRCS_$(d):%=$(d)/%)

OBJS_$(d)     := $(cFILES_$(d):%.c=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)     += $(cppFILES_$(d):%.cpp=$(LIBRARIES_PATH)/%.o)
OBJS_$(d)     += $(sFILES_$(d):%.S=$(LIBRARIES_PATH)/%.o)

DEPS_$(d)     := $(OBJS_$(d):%.o=%.d)

$(OBJS_$(d)): TGT_CFLAGS := $(CFLAGS_$(d))

TGT_BIN += $(OBJS_$(d))

# Standard things
-include        $(DEPS_$(d))
d               := $(dirstack_$(sp))
sp              := $(basename $(sp))
