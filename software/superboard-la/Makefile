# sb_la — Windows only (WinUSB). Build with mingw-w64.
#
#   make
#   make clean

ifneq ($(OS),Windows_NT)
  UNAME := $(shell uname -s)
  ifeq (,$(findstring MINGW,$(UNAME))$(findstring MSYS,$(UNAME))$(findstring CYGWIN,$(UNAME)))
    $(error sb_la uses WinUSB and builds on Windows only)
  endif
endif

CC      ?= gcc
CFLAGS  ?= -O2 -std=gnu11 -Wall -Wextra -Iinclude
LDLIBS  := -lwinusb -lsetupapi -lcfgmgr32

BUILD   := build
TARGET  := $(BUILD)/sb_la.exe

.PHONY: all clean
all: $(TARGET)

$(TARGET): src/sb_la.c include/la_protocol.h | $(BUILD)
	$(CC) $(CFLAGS) -o $@ src/sb_la.c $(LDLIBS)

$(BUILD):
	@mkdir -p $(BUILD)

clean:
	@rm -rf $(BUILD)
