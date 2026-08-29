################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
# Deliberately empty. These used to carry -I$(INCDIR), for headers that included
# the external libraries with angle brackets -- common.mk only passes the
# include dir as -iquote, which serves #include "..." and nothing else.
#
# That built fine and broke every editor: the compile_commands.json PROS
# generates for clangd does not carry EXTRA_CXXFLAGS, so main.cpp and
# subsystems.cpp showed unresolved includes while make was perfectly happy.
# Every header now uses quoted includes, so stock PROS flags suffice. Keep it
# that way -- if this needs filling in again, the editor will silently disagree
# with the build.
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
#
# OFF deliberately. Hot/cold splits the program into two files on the brain: a
# 2.3MB cold library everything links against, and a small hot program. It makes
# repeat uploads faster, but the hot program is USELESS without its cold half --
# and if the cold upload does not complete, the brain is left holding a program
# it cannot run, shows it under a generic name, and drops it after one attempt.
# That is exactly what was happening. A monolith is one self-contained file with
# nothing to go missing. Uploads are somewhat larger; they also succeed.
USE_PACKAGE:=0

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

################################################################################
# liblvgl has to be whole-archived, or the brain screen stays black.
#
# LVGL is brought up by display_initialize(), which lives in ONE archive member,
# liblvgl.a(display.c.o), and is called by the PROS startup code. Nothing in the
# project references it directly -- and libpros.a(startup.c.o) carries a WEAK
# no-op display_initialize() of its own as a fallback for projects built without
# liblvgl.
#
# So the linker never has an undefined symbol to resolve: the weak stub already
# satisfies the call, display.c.o is never pulled out of the archive, and the
# strong definition that would have overridden it never gets a chance. LVGL is
# then never initialised and its timers are never pumped. lv_init and
# lv_timer_handler are literally absent from the binary -- check with
#   arm-none-eabi-nm bin/monolith.elf | grep -w lv_init
# and the screen shows nothing at all, forever.
#
# This is why it broke when hot/cold linking was turned off above. The cold
# image is linked with --whole-archive (common.mk, the $(COLD_ELF) rule), which
# pulled display.c.o in as a side effect. The monolith rule has no such flag, so
# switching to USE_PACKAGE:=0 silently took the display with it.
#
# Overriding LNK_FLAGS after the include, because common.mk defines it with a
# plain = and would otherwise win. Only liblvgl is whole-archived; --gc-sections
# still drops what the UI does not use, so the cost is about 185 KB of text.
LNK_FLAGS=--gc-sections --start-group \
	$(filter-out $(FWDIR)/liblvgl.a,$(strip $(LIBRARIES))) \
	--whole-archive $(FWDIR)/liblvgl.a --no-whole-archive \
	-lgcc -lstdc++ --end-group \
	-T$(FWDIR)/v5-common.ld --no-warn-rwx-segments \
	--sort-section=alignment --sort-common
