# VESC CAN SDK for Zephyr RTOS
#
# This CMakeLists.txt file allows the VESC CAN SDK to be built as a static library
# for use with Zephyr RTOS applications.
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

# Set minimum CMake version
cmake_minimum_required(VERSION 3.20.0)

# Find Zephyr package
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

# Set C standard for Zephyr compatibility
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Define the library name
set(LIBRARY_NAME vesc_can_sdk)

# Source files for the VESC CAN SDK
set(SOURCES
    vesc_buffer.c
    vesc_crc.c
    vesc_can_sdk.c
)

# Header files
set(HEADERS
    vesc_buffer.h
    vesc_crc.h
    vesc_can_sdk.h
)

# Create static library
add_library(${LIBRARY_NAME} STATIC ${SOURCES})

# Set library properties
set_target_properties(${LIBRARY_NAME} PROPERTIES
    OUTPUT_NAME ${LIBRARY_NAME}
    POSITION_INDEPENDENT_CODE OFF  # Not needed for embedded systems
)

# Set include directories
target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link with Zephyr libraries
target_link_libraries(${LIBRARY_NAME} zephyr)

# Set compiler flags for embedded systems
target_compile_options(${LIBRARY_NAME} PRIVATE
    -Wall
    -Wextra
    -Wno-unused-parameter
    -Wno-unused-variable
    -Wno-unused-function
)

# Define preprocessor macros for Zephyr compatibility
target_compile_definitions(${LIBRARY_NAME} PRIVATE
    VESC_CAN_SDK_ZEPHYR=1
)

# Install headers to Zephyr include directory
install(FILES ${HEADERS} DESTINATION include/${LIBRARY_NAME})

# Create a config header for Zephyr-specific configurations
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/vesc_can_sdk_zephyr_config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/vesc_can_sdk_zephyr_config.h
    @ONLY
)

# Install the config header
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/vesc_can_sdk_zephyr_config.h
    DESTINATION include/${LIBRARY_NAME}
)

# Print configuration summary
message(STATUS "VESC CAN SDK for Zephyr Configuration:")
message(STATUS "  Library name: ${LIBRARY_NAME}")
message(STATUS "  Source files: ${SOURCES}")
message(STATUS "  Header files: ${HEADERS}")
message(STATUS "  C standard: ${CMAKE_C_STANDARD}")
message(STATUS "  Zephyr base: $ENV{ZEPHYR_BASE}") 