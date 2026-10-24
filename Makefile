# PROS library configuration. Requires a separately installed PROS kernel project.
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR:=$(ROOT)/bin
SRCDIR:=$(ROOT)/src
INCDIR:=$(ROOT)/include
CXX_STANDARD:=gnu++20
USE_PACKAGE:=1

IS_LIBRARY:=1
LIBNAME:=autotune
VERSION:=0.1.0
TEMPLATE_FILES=$(wildcard $(INCDIR)/autotune/*.hpp) README.md LICENSE CHANGELOG.md

.DEFAULT_GOAL:=library

ifeq ($(wildcard common.mk),)
$(error Missing PROS kernel common.mk. Follow the README build-project setup)
endif
include common.mk
