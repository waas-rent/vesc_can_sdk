/*
 * VESC CAN SDK - Controller ID Filtering Example
 * 
 * This example demonstrates how to use the controller ID filtering
 * functionality to only process CAN frames from a specific VESC controller.
 * 
 * Copyright (c) 2025 waas AG (waas.rent)
 */

#include "../include/vesc_can_sdk.h"
#include <stdio.h>
#include <stdint.h>

// Example CAN send function (user must implement this)
static bool example_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    printf("Sending CAN frame: ID=0x%03X, Len=%d\n", id, len);
    // User should implement actual CAN transmission here
    return true;
}

// Example response callback function
static void example_response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("Received response from controller %d, command %d, length %d\n", 
           controller_id, command, len);
    
    // Parse specific responses
    switch (command) {
        case COMM_GET_VALUES: {
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("Motor RPM: %.1f, Current: %.2fA, Voltage: %.1fV\n",
                       values.rpm, values.current_motor, values.v_in);
            }
        } break;
        
        case COMM_FW_VERSION: {
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                printf("Firmware: %d.%d, Hardware: %s\n",
                       version.major, version.minor, version.hw_name);
            }
        } break;
    }
}

int main() {
    printf("VESC CAN SDK - Controller ID Filtering Example\n");
    printf("==============================================\n\n");
    
    // Initialize the SDK with controller ID 1
    if (!vesc_can_init(example_can_send, 1)) {
        printf("Failed to initialize VESC CAN SDK\n");
        return -1;
    }
    
    // Set the response callback
    vesc_set_response_callback(example_response_callback);
    
    printf("Configured to listen for controller ID: 1\n\n");
    
    // Send some commands to controller ID 1
    printf("Sending commands to controller ID 1:\n");
    vesc_get_values(1);
    vesc_get_fw_version(1);
    
    // Simulate receiving CAN frames
    printf("\nSimulating CAN frame reception:\n");
    
    // This frame should be processed (controller ID 1)
    uint8_t test_data[] = {0x1B, 0x00, 0x00, 0x00, 0x00}; // Example GET_VALUES response
    uint32_t can_id_1 = 1 | (COMM_GET_VALUES << 8);
    printf("Processing CAN frame from controller 1...\n");
    vesc_process_can_frame(can_id_1, test_data, sizeof(test_data));
    
    // This frame should be ignored (controller ID 2)
    uint32_t can_id_2 = 2 | (COMM_GET_VALUES << 8);
    printf("Processing CAN frame from controller 2...\n");
    vesc_process_can_frame(can_id_2, test_data, sizeof(test_data));
    
    // Change controller ID to listen for controller 2
    printf("\nChanging to listen for controller ID: 2\n");
    vesc_set_controller_id(2);
    
    // Now this frame should be processed (controller ID 2)
    printf("Processing CAN frame from controller 2...\n");
    vesc_process_can_frame(can_id_2, test_data, sizeof(test_data));
    
    // This frame should be ignored (controller ID 1)
    printf("Processing CAN frame from controller 1...\n");
    vesc_process_can_frame(can_id_1, test_data, sizeof(test_data));
    
    printf("\nNote: You can change the controller ID at runtime using vesc_set_controller_id()\n");
    printf("This allows you to communicate with different VESC controllers on the same CAN bus.\n");
    
    printf("\nExample completed successfully!\n");
    return 0;
} 