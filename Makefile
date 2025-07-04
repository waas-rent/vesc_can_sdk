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
AR = ar
ARFLAGS = rcs

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
	@ldconfig

# Test compilation
test: all
	@echo "Building test example"
	@$(CC) $(CFLAGS) -I$(INCDIR) -L$(LIBDIR) -lvesc_can_sdk -lm -o test_example examples/basic_control.c

# Show help
help:
	@echo "VESC CAN SDK Makefile"
	@echo ""
	@echo "Available targets:"
	@echo "  all        - Build static and shared libraries (default)"
	@echo "  clean      - Remove build artifacts"
	@echo "  install    - Install library and headers to system"
	@echo "  uninstall  - Remove library and headers from system"
	@echo "  test       - Build test example"
	@echo "  help       - Show this help message"
	@echo ""
	@echo "Build products:"
	@echo "  $(LIBDIR)/$(LIBRARY)      - Static library"
	@echo "  $(LIBDIR)/$(SHARED_LIBRARY) - Shared library"

.PHONY: all clean install uninstall test help directories 