# VESC CAN SDK Makefile
#
# Copyright (c) 2025 waas AG (waas.rent)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Compiler settings
CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -fPIC
DEBUG_CFLAGS = -Wall -Wextra -std=c99 -g -O0 -fPIC -DDEBUG
AR = ar
ARFLAGS = rcs

# ============================================================================
# Version and Git Information
# ============================================================================

# Get Git information
GIT_HASH := $(shell git rev-parse --short HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH := $(shell git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")

# Version definitions
VERSION_MAJOR = 1
VERSION_MINOR = 0
VERSION_PATCH = 0

# Add version and Git information to compiler flags
VERSION_CFLAGS = -DVESC_SDK_GIT_HASH=\"$(GIT_HASH)\" -DVESC_SDK_GIT_BRANCH=\"$(GIT_BRANCH)\"
CFLAGS += $(VERSION_CFLAGS)
DEBUG_CFLAGS += $(VERSION_CFLAGS)

# Directories
SRCDIR = .
OBJDIR = obj
LIBDIR = lib
INCDIR = include

# Source files
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Library name
LIBRARY = libvesc_can_sdk.a
SHARED_LIBRARY = libvesc_can_sdk.so

# Default target
all: directories $(LIBDIR)/$(LIBRARY) $(LIBDIR)/$(SHARED_LIBRARY)

# Create directories
directories:
	@mkdir -p $(OBJDIR)
	@mkdir -p $(LIBDIR)

# Static library
$(LIBDIR)/$(LIBRARY): $(OBJECTS)
	@echo "Creating static library $@"
	@$(AR) $(ARFLAGS) $@ $^

# Shared library
$(LIBDIR)/$(SHARED_LIBRARY): $(OBJECTS)
	@echo "Creating shared library $@"
	@$(CC) -shared -o $@ $^

# Compile source files
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	@echo "Compiling $<"
	@$(CC) $(CFLAGS) -I$(INCDIR) -c $< -o $@

# Clean build artifacts
clean:
	@echo "Cleaning build artifacts"
	@rm -rf $(OBJDIR)
	@rm -rf $(LIBDIR)

# Install library and headers
install: all
	@echo "Installing VESC CAN SDK"
	@mkdir -p /usr/local/lib
	@mkdir -p /usr/local/include
	@cp $(LIBDIR)/$(LIBRARY) /usr/local/lib/
	@cp $(LIBDIR)/$(SHARED_LIBRARY) /usr/local/lib/
	@cp $(INCDIR)/*.h /usr/local/include/
	@ldconfig

# Uninstall
uninstall:
	@echo "Uninstalling VESC CAN SDK"
	@rm -f /usr/local/lib/$(LIBRARY)
	@rm -f /usr/local/lib/$(SHARED_LIBRARY)
	@rm -f /usr/local/include/vesc_can_sdk.h
	@rm -f /usr/local/include/vesc_buffer.h
	@rm -f /usr/local/include/vesc_crc.h
	@rm -f /usr/local/include/vesc_version.h
	@ldconfig

# Build version example
version_example: all
	@echo "Building version example"
	@$(CC) $(CFLAGS) -I$(INCDIR) -L$(LIBDIR) -lvesc_can_sdk -lm -o version_example examples/version_example.c

# Show version information
version:
	@echo "VESC CAN SDK Version Information"
	@echo "================================="
	@echo "Version: $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)"
	@echo "Git Hash: $(GIT_HASH)"
	@echo "Git Branch: $(GIT_BRANCH)"
	@echo "Build Date: $(shell date '+%b %d %Y')"
	@echo "Build Time: $(shell date '+%H:%M:%S')"
	@echo ""

# Debug build
debug: clean
	@echo "Building debug version"
	@mkdir -p $(OBJDIR)
	@mkdir -p $(LIBDIR)
	@$(CC) $(DEBUG_CFLAGS) -I$(INCDIR) -c vesc_can_sdk.c -o $(OBJDIR)/vesc_can_sdk.o
	@$(CC) $(DEBUG_CFLAGS) -I$(INCDIR) -c vesc_buffer.c -o $(OBJDIR)/vesc_buffer.o
	@$(CC) $(DEBUG_CFLAGS) -I$(INCDIR) -c vesc_crc.c -o $(OBJDIR)/vesc_crc.o
	@$(AR) $(ARFLAGS) $(LIBDIR)/$(LIBRARY) $(OBJDIR)/*.o
	@$(CC) -shared -o $(LIBDIR)/$(SHARED_LIBRARY) $(OBJDIR)/*.o

# Show help
help:
	@echo "VESC CAN SDK Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all            - Build static and shared libraries (default)"
	@echo "  clean          - Remove build artifacts"
	@echo "  install        - Install library and headers to system"
	@echo "  uninstall      - Remove library and headers from system"
	@echo "  debug          - Build debug version of library"
	@echo "  version        - Show version information"
	@echo "  version_example - Build version example"
	@echo "  help           - Show this help message"
	@echo ""
	@echo "Build products:"
	@echo "  $(LIBDIR)/$(LIBRARY)      - Static library"
	@echo "  $(LIBDIR)/$(SHARED_LIBRARY) - Shared library"
	@echo ""
	@echo "Version information:"
	@echo "  Version: $(VERSION_MAJOR).$(VERSION_MINOR).$(VERSION_PATCH)"
	@echo "  Git Hash: $(GIT_HASH)"
	@echo "  Git Branch: $(GIT_BRANCH)"

.PHONY: all clean install uninstall help directories debug version version_example