# VESC CAN SDK for Zephyr RTOS

This document describes how to build and use the VESC CAN SDK as a static library in Zephyr RTOS projects.

**Copyright (c) 2025 waas AG (waas.rent)**  
**License: MIT**

## Overview

The VESC CAN SDK is a pure C library for communicating with VESC motor controllers over CAN bus. This Zephyr integration allows you to use the SDK in your Zephyr-based embedded applications.

**Compatibility:** This SDK is compatible with VESC firmware version 6.06.

## Directory Structure

- `vesc_can_sdk.c`, `vesc_can_sdk.h` — Main SDK source and header
- `vesc_buffer.c`, `vesc_buffer.h` — Buffer utilities
- `vesc_crc.c`, `vesc_crc.h` — CRC utilities
- `CMakeLists.zephyr.txt` — CMake file for Zephyr integration
- `vesc_can_sdk_zephyr_config.h.in` — Zephyr-specific configuration header template

## Building the SDK as a Zephyr Library

1. **Add the SDK to your Zephyr project**
   - Copy the `vesc_can_sdk` directory into your Zephyr application's source tree.

2. **Include the SDK in your application's CMakeLists.txt**
   - In your application's `CMakeLists.txt`, add:
     ```cmake
     add_subdirectory(vesc_can_sdk)
     ```
   - This will build the SDK as a static library named `vesc_can_sdk`.

3. **Link the library to your application**
   - In your application's CMake target, link against the SDK:
     ```cmake
     target_link_libraries(your_app PRIVATE vesc_can_sdk)
     ```

4. **Include the SDK headers in your code**
   - In your source files, include the main SDK header:
     ```c
     #include <vesc_can_sdk.h>
     ```

## Zephyr-Specific Configuration

- The file `vesc_can_sdk_zephyr_config.h` is generated automatically during the build and provides Zephyr-specific macros and type definitions.
- The SDK uses Zephyr's CAN, memory, threading, and logging APIs for integration.

## Example Usage

```c
#include <vesc_can_sdk.h>

void main(void) {
    // Initialize CAN and VESC SDK as needed
    // Use SDK functions to communicate with VESC devices
}
```

## Notes

- Ensure your Zephyr project is configured with CAN support enabled.
- The SDK expects Zephyr's CAN API to be available (`<zephyr/canbus/can.h>`).
- You may need to adapt the CAN send/receive functions to your board/application.

## License

This SDK is licensed under the MIT License. See `LICENSE` for details. 