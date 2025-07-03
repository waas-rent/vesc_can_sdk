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
   - Add this module to your west.yml manifest file. 
     ````
     manifest:
      self:
         path: myproject

      remotes:
         - name: zephyrproject-upstream
            url-base: https://github.com/zephyrproject-rtos
         - name: vesc_can_sdk
            url-base: https://github.com/waas-rent

      projects:
         - name: zephyr
            remote: zephyrproject-upstream
            revision: v4.0.0
            import: false
            path: zephyr
            west-commands: scripts/west-commands.yml
            import:
            name-allowlist:
               - ...
               - vesc_can_sdk
         
         - name: vesc_can_sdk
            remote: vesc_can_sdk
            revision: main
            path: modules/lib/vesc_can_sdk
     ```

3. **Run `west update` to retrieve the module**

4. **Include the SDK headers in your code**
   - In your source files, include the main SDK header:
     ```c
     #include <vesc_can_sdk.h>
     ```

## Example Usage

### Basic Initialization and Version Request

Here's a complete example showing how to initialize the VESC CAN SDK and request the firmware version from a VESC device:

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <vesc_can_sdk.h>
#include <stdio.h>

// CAN device
static const struct device *can_dev;

// VESC controller ID (typically 0 for single VESC, or 1-255 for multiple VESCs)
#define VESC_CONTROLLER_ID 0

// Response callback function
static void vesc_response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("Received response from VESC %d, command: %d, length: %d\n", 
           controller_id, command, len);
    
    // Handle firmware version response
    if (command == COMM_FW_VERSION) {
        vesc_fw_version_t version;
        if (vesc_parse_fw_version(data, len, &version)) {
            printf("VESC Firmware Version: %d.%d\n", version.major, version.minor);
            printf("Hardware: %s\n", version.hw_name);
            printf("Hardware Type: %d\n", version.hw_type);
            printf("Configuration: %d\n", version.cfg_num);
            printf("UUID: ");
            for (int i = 0; i < 12; i++) {
                printf("%02X", version.uuid[i]);
            }
            printf("\n");
        } else {
            printf("Failed to parse firmware version response\n");
        }
    }
}

// CAN send function for VESC SDK
static bool vesc_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    struct can_frame frame;
    
    frame.id = id;
    frame.dlc = len;
    frame.flags = CAN_FRAME_ID_EXT; // VESC uses extended CAN IDs
    
    memcpy(frame.data, data, len);
    
    int ret = can_send(can_dev, &frame, K_MSEC(100), NULL, NULL);
    if (ret != 0) {
        printf("Failed to send CAN frame: %d\n", ret);
        return false;
    }
    
    return true;
}

// CAN receive thread
static void can_receive_thread(void) {
    struct can_frame frame;
    
    while (1) {
        int ret = can_read(can_dev, &frame, K_FOREVER, NULL);
        if (ret == 0) {
            // Process received CAN frame with VESC SDK
            vesc_process_can_frame(frame.id, frame.data, frame.dlc);
        }
    }
}

void main(void) {
    printf("VESC CAN SDK Example\n");
    
    // Get CAN device
    can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
    if (!device_is_ready(can_dev)) {
        printf("CAN device not ready\n");
        return;
    }
    
    // Initialize VESC CAN SDK with controller ID
    if (!vesc_can_init(vesc_can_send, VESC_CONTROLLER_ID)) {
        printf("Failed to initialize VESC CAN SDK\n");
        return;
    }
    
    // Set response callback
    vesc_set_response_callback(vesc_response_callback);
    
    printf("VESC CAN SDK initialized successfully\n");
    
    // Start CAN receive thread
    k_thread_create(&can_thread, can_thread_stack,
                   K_THREAD_STACK_SIZEOF(can_thread_stack),
                   (k_thread_entry_t)can_receive_thread,
                   NULL, NULL, NULL,
                   K_PRIO_COOP(7), 0, K_NO_WAIT);
    k_thread_name_set(&can_thread, "can_receive");
    
    // Wait a moment for system to stabilize
    k_sleep(K_MSEC(1000));
    
    // Request VESC firmware version
    printf("Requesting VESC firmware version...\n");
    vesc_get_fw_version(VESC_CONTROLLER_ID);
    
    // Keep the main thread alive
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

// Thread definitions
K_THREAD_DEFINE(can_thread, 1024, can_receive_thread, NULL, NULL, NULL, 7, 0, 0);
```

### Key Components Explained

1. **CAN Send Function**: The `vesc_can_send` function is passed to the SDK during initialization. It handles the actual transmission of CAN frames using Zephyr's CAN driver.

2. **Response Callback**: The `vesc_response_callback` function is called by the SDK when responses are received from VESC devices. It handles parsing and processing of different response types.

3. **CAN Receive Thread**: A dedicated thread continuously reads CAN frames and passes them to the SDK for processing.

4. **Version Request**: The `vesc_get_fw_version()` function sends a firmware version request to the specified VESC controller.

### Additional Commands

You can also send other commands to VESC devices:

```c
// Get motor values (current, voltage, RPM, etc.)
vesc_get_values(VESC_CONTROLLER_ID);

// Set motor duty cycle (0.0 to 1.0)
vesc_set_duty(VESC_CONTROLLER_ID, 0.5f);

// Set motor current (in Amperes)
vesc_set_current(VESC_CONTROLLER_ID, 10.0f);

// Set motor RPM
vesc_set_rpm(VESC_CONTROLLER_ID, 1000.0f);
```

### Controller ID Filtering

The SDK automatically filters incoming CAN frames to only process those from the configured controller ID. This prevents interference from other VESC controllers on the same CAN bus.

**Initialization with Controller ID:**
```c
// Initialize SDK to listen for controller ID 1
if (!vesc_can_init(vesc_can_send, 1)) {
    printf("Failed to initialize VESC CAN SDK\n");
    return;
}
```

**Runtime Controller ID Changes:**
You can change the controller ID at runtime if needed:
```c
// Change to listen for controller ID 2
vesc_set_controller_id(2);
```

**Benefits:**
- Only processes CAN frames from the intended VESC controller
- Prevents interference from other controllers on the same CAN bus
- Improves communication reliability in multi-controller setups
- Allows dynamic switching between different VESC controllers

### Multi-Controller Setups

When working with multiple VESC controllers on the same CAN bus, the controller ID filtering ensures reliable communication:

```c
// Initialize SDK for controller ID 1
if (!vesc_can_init(vesc_can_send, 1)) {
    printf("Failed to initialize VESC CAN SDK\n");
    return;
}

// Communicate with controller 1
vesc_get_values(1);
vesc_set_duty(1, 0.5f);

// Switch to controller 2
vesc_set_controller_id(2);
vesc_get_values(2);
vesc_set_duty(2, 0.3f);

// Switch back to controller 1
vesc_set_controller_id(1);
vesc_get_values(1);
```

This approach allows you to communicate with multiple VESC controllers sequentially while ensuring that only responses from the intended controller are processed.

### Status Message Parsing Example

VESC controllers automatically send status messages when status reporting is enabled. Here's how to parse Status Message 1 (CAN_PACKET_STATUS) which contains RPM, current, and duty cycle information:

```c
// Response callback function with status message parsing
static void vesc_response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("Received response from VESC %d, command: %d, length: %d\n", 
           controller_id, command, len);
    
    // Handle firmware version response
    if (command == COMM_FW_VERSION) {
        vesc_fw_version_t version;
        if (vesc_parse_fw_version(data, len, &version)) {
            printf("VESC Firmware Version: %d.%d\n", version.major, version.minor);
            printf("Hardware: %s\n", version.hw_name);
        }
    }
    
    // Handle Status Message 1 (RPM, Current, Duty Cycle)
    else if (command == CAN_PACKET_STATUS) {
        vesc_status_msg_1_t status;
        if (vesc_parse_status_msg_1(data, len, &status)) {
            printf("VESC %d Status 1:\n", controller_id);
            printf("  RPM: %.0f\n", status.rpm);
            printf("  Current: %.2f A\n", status.current);
            printf("  Duty Cycle: %.1f%%\n", status.duty * 100.0f);
            
            // Example: Check if motor is running
            if (status.rpm > 0) {
                printf("  Motor Status: Running\n");
            } else {
                printf("  Motor Status: Stopped\n");
            }
            
            // Example: Check current limits
            if (status.current > 20.0f) {
                printf("  WARNING: High current detected!\n");
            }
        } else {
            printf("Failed to parse Status Message 1\n");
        }
    }
    
    // Handle other status messages
    else if (command == CAN_PACKET_STATUS_4) {
        vesc_status_msg_4_t status;
        if (vesc_parse_status_msg_4(data, len, &status)) {
            printf("VESC %d Status 4:\n", controller_id);
            printf("  FET Temperature: %.1f°C\n", status.temp_fet);
            printf("  Motor Temperature: %.1f°C\n", status.temp_motor);
            printf("  Input Current: %.2f A\n", status.current_in);
            printf("  PID Position: %.2f\n", status.pid_pos_now);
            
            // Example: Temperature monitoring
            if (status.temp_fet > 80.0f) {
                printf("  WARNING: High FET temperature!\n");
            }
            if (status.temp_motor > 100.0f) {
                printf("  WARNING: High motor temperature!\n");
            }
        }
    }
}
```

### Status Message Types

The VESC SDK supports parsing all 6 status message types:

- **Status Message 1** (`CAN_PACKET_STATUS`): RPM, current, duty cycle
- **Status Message 2** (`CAN_PACKET_STATUS_2`): Amp hours consumed and charged
- **Status Message 3** (`CAN_PACKET_STATUS_3`): Watt hours consumed and charged
- **Status Message 4** (`CAN_PACKET_STATUS_4`): Temperatures, input current, PID position
- **Status Message 5** (`CAN_PACKET_STATUS_5`): Tachometer value, input voltage
- **Status Message 6** (`CAN_PACKET_STATUS_6`): ADC values, PPM value

### Enabling Status Messages

To receive status messages from VESC controllers, you need to enable status reporting in the VESC configuration. This is typically done through the VESC Tool or by sending configuration commands.

**Note:** Status messages are automatically sent by VESC controllers when enabled - you don't need to request them explicitly like other commands.

## Notes

- You may need to adapt the CAN send/receive functions to your board/application.
- The VESC controller ID must be specified during SDK initialization and should match the ID configured in your VESC device.
- The SDK automatically filters incoming CAN frames to only process those from the configured controller ID.
- Extended CAN IDs are used by VESC (29-bit IDs).
- The SDK handles packet fragmentation automatically for long commands.
- You can change the controller ID at runtime using `vesc_set_controller_id()` if needed.

## License

This SDK is licensed under the MIT License. See `LICENSE` for details. 