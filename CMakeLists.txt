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

zephyr_sources_ifdef(CONFIG_VESC_CAN_SDK
    vesc_can_sdk.c
    vesc_buffer.c
    vesc_crc.c
)

zephyr_include_directories_ifdef(CONFIG_VESC_CAN_SDK
    include
)