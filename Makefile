# PROS library configuration. Requires a separately installed PROS kernel project.
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR:=$(ROOT)/bin
SRCDIR:=$(ROOT)/src
INCDIR:=$(ROOT)/include
# Use the installed kernel's language standard (C++17 or newer is required).
USE_PACKAGE:=1
# Kernel rules use -iquote; public upstream headers use angle includes.
EXTRA_CXXFLAGS += -I$(INCDIR)

IS_LIBRARY:=1
LIBNAME:=autotune
VERSION:=0.1.0
TEMPLATE_FILES=$(wildcard $(INCDIR)/autotune/*.hpp) README.md LICENSE CHANGELOG.md CONTRIBUTING.md $(wildcard docs/*.md)

.DEFAULT_GOAL:=library

ifeq ($(wildcard common.mk),)
$(error Missing PROS dependencies. Run python3 scripts/prepare_build.py /tmp/guantune-build first)
endif
include common.mk
