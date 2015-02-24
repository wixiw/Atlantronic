# @file Makefile
# @author Jean-Baptiste Trédez
#
# les fichiers objets *.o sont construits à partir des fichiers *.c ou *.S
#
# fonctionnement :
#   - recherche tout les fichiers build.mk dans les sous dossiers
#   - ces fichiers indiquent les programmes à compiler et comment les compiler :
#       - on suppose qu'on veut compiler le programme homologation de la carte foo
#       - bin-foo += homologation
#       - obj-foo-homologation += x.o y.o z.o
#       - lib-foo-homologation += -lncurses

# pour le module noyau
export INSTALL_MOD_PATH:=$(DESTDIR)

# dossiers
# dossiers
src := .
obj := ../obj
bin := ../bin

INCLUDES:=-I. -I ../src_stm32


MAKEFLAGS += -rR --no-print-directory

DEBUG ?= 0
VERBOSE ?= 0
SKIP_SIZE_HEADER ?= "+1"

ifeq ($(VERBOSE),0)
MAKEFLAGS += --quiet
endif

ARCH=x86

MARCH ?= atom

# on tente de détecter la configuration native
BIT ?= $(shell getconf LONG_BIT)

DEF+=LINUX
LDSCRIPT:=../scripts/ld/elf_linux_$(BIT).ld

CFLAGS:=-march=$(MARCH)
CFLAGS+=-m$(BIT)
CFLAGS+=-Wall
CFLAGS+=-Wextra
CFLAGS+=-g

LDFLAGS:=-march=$(MARCH)
LDFLAGS+=-m$(BIT)
LDFLAGS+=-T $(LDSCRIPT)
LDFLAGS+=-pthread
LDFLAGS+=-lrt

ifneq ($(DEBUG),1)
DEF+= NDEBUG
CFLAGS+=-O2
CFLAGS+=-fomit-frame-pointer
else
CFLAGS+=-O0
endif

CFLAGS+=$(addprefix -D,$(DEF))
CXXFLAGS:=$(CFLAGS)


AS:=$(CROSSCOMPILE)as
CC:=$(CROSSCOMPILE)gcc
CXX:=$(CROSSCOMPILE)g++
OBJCOPY:=$(CROSSCOMPILE)objcopy
STRIP:=$(CROSSCOMPILE)strip
SIZE:=$(CROSSCOMPILE)size
VERSION=$(shell git rev-parse HEAD)

ifneq ($(MAKECMDGOALS),clean)
MK:=$(shell find . -name 'build.mk')
-include $(MK)
endif

SRC_DOC=$(shell find . -name '*.dot')
BIN_DOC=$(SRC_DOC:.dot=.png)

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

.PHONY: modules
modules:
	+make MAKEFLAGS=--no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/modules	

.PHONY: modules_install
modules_install: modules	
	+make MAKEFLAGS=--no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/modules modules_install
	

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
	rm -frv $(obj)/$(ARCH)
	rm -frv $(bin)/$(ARCH)
	+make MAKEFLAGS=--no-print-directory -C /lib/modules/`uname -r`/build M=`pwd`/modules clean

