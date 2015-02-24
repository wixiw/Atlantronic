# @file Makefile
# @author Jean-Baptiste Trédez
# @author Willy Lambert
#
# fichier de configuration : .cfg
# les fichiers objets *.o sont construits à partir des fichiers *.c ou *.S
#
# fonctionnement :
#   - on attend un fichier build.mk à côté de ce fichier qui contient la liste des *.o à compiler.
#   - ce fichier indique les programmes à compiler et la liste des fichiers à inclure :
#		obj-stm32-${prog} : liste des obbjets
#	    bin-stm32 += stm32_ard : compile le programme ard (en prenant obj-stm32-stm32_ard comme liste de fichiers)


# dossiers
src := .
obj := ../obj
bin := ../bin

INCLUDES:=-I.

MAKEFLAGS += -rR --no-print-directory

DEBUG ?= 0
VERBOSE ?= 0
SKIP_SIZE_HEADER ?= "+1"

ifeq ($(VERBOSE),0)
MAKEFLAGS += --quiet
endif

ARCH=stm32
CROSSCOMPILE:=arm-none-eabi-
MCPU:=cortex-m4

LDSCRIPT:=../scripts/ld/stm32f407.ld

CFLAGS:=-mcpu=$(MCPU)
CFLAGS+=-mfloat-abi=hard
CFLAGS+=-mfpu=fpv4-sp-d16
CFLAGS+=-mthumb
CFLAGS+=-mno-thumb-interwork
CFLAGS+=-fomit-frame-pointer
CFLAGS+=-fno-exceptions
CFLAGS+=-fno-non-call-exceptions
CFLAGS+=-fsingle-precision-constant
CFLAGS+=-ffast-math
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-Wswitch-default
CFLAGS+=-Wswitch-enum
CFLAGS+=-D__stm32__
CFLAGS+=-DSTM32F4XX
CFLAGS+=-DSTM32F407xx
CFLAGS+=-g3
ifneq ($(DEBUG),1)
CFLAGS+=-O2
endif

CXXFLAGS:=$(CFLAGS)
CXXFLAGS+=-fno-rtti
CXXFLAGS+=-ftemplate-depth-0
CXXFLAGS+=-Woverloaded-virtual
CXXFLAGS+=-Wfatal-errors
CXXFLAGS+=-ftemplate-depth=17 

#cflag qu'on ne met pas pour le c++
CFLAGS+=-Wunsuffixed-float-constants

LDFLAGS:=-mcpu=$(MCPU)
LDFLAGS+=-nostartfiles
LDFLAGS+=-mfloat-abi=hard
LDFLAGS+=-mfpu=fpv4-sp-d16
LDFLAGS+=-mthumb
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-lsupc++
LDFLAGS+=-lgcc
LDFLAGS+=-lc
LDFLAGS+=-lm
LDFLAGS+=-lg
LDFLAGS+=-g3


AS:=$(CROSSCOMPILE)as
CC:=$(CROSSCOMPILE)gcc
CXX:=$(CROSSCOMPILE)g++
OBJCOPY:=$(CROSSCOMPILE)objcopy
STRIP:=$(CROSSCOMPILE)strip
SIZE:=$(CROSSCOMPILE)size
DOT:=dot
VERSION=$(shell git rev-parse HEAD)

ifneq ($(MAKECMDGOALS),clean)
-include build.mk
endif

# règles
$(obj)/$(ARCH)/%.d: $(obj)/$(ARCH)/%.o

$(obj)/$(ARCH)/%.o: $(src)/%.c
	@echo "    CC    " $@
	mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -DVERSION=\"$(VERSION)\" $($(patsubst $(obj)/$(ARCH)/%,cflags-$(ARCH)-%, $@)) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.cxx
	@echo "   CPP    " $@
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -DVERSION=\"$(VERSION)\" $($(patsubst $(obj)/$(ARCH)/%,cxxflags-$(ARCH)-%, $@)) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )

$(obj)/$(ARCH)/%.o: $(src)/%.cpp
	@echo "   CPP    " $@
	mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -DVERSION=\"$(VERSION)\" $($(patsubst $(obj)/$(ARCH)/%,cxxflags-$(ARCH)-%, $@)) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES) || ( rm -vfr $@ $(@:.o=.d) ; exit 1 )


$(obj)/$(ARCH)/%.o: $(src)/%.S
	@echo [AS] $@
	$(AS) $(AFLAGS) -c $< -o $@ -MMD -MF$(@:.o=.d) $(INCLUDES)


.PHONY: all
all: $(addprefix $(bin)/$(ARCH)/,$(bin-$(ARCH)))

.PHONY: stat
stat: $(addprefix $(bin)/$(ARCH)/,$(bin-$(ARCH)))
	$(SIZE) -B $^ | tail -n $(SKIP_SIZE_HEADER)


$(foreach var,$(bin-$(ARCH)),$(eval $(bin)/$(ARCH)/$(var):$(addprefix $(obj)/$(ARCH)/,$(obj-$(ARCH)-$(var)) )))
$(foreach var,$(bin-$(ARCH)),$(eval DEP += $(addprefix $(obj)/$(ARCH)/,$(obj-$(ARCH)-$(var):.o=.d))))

ifneq ($(MAKECMDGOALS),clean)
-include $(DEP)
endif

$(bin)/$(ARCH)/%:
	@echo "    LD    " $@
	mkdir -p $(dir $@)
	$(CXX) $^ -o $@ $($(patsubst $(bin)/$(ARCH)/%,lib-$(ARCH)-%, $@)) -Wl,-Map="$@.map" $(LDFLAGS)
	$(OBJCOPY) --only-keep-debug $@ $@.debug
	$(OBJCOPY) --add-gnu-debuglink $@.debug $@
	$(STRIP) $@


.PHONY: clean
clean:
	rm -frv $(obj)/stm32
	rm -frv $(bin)/stm32

