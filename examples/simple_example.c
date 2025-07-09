/*
 * VESC CAN SDK - Simple Example
 * 
 * This example demonstrates the basic usage of the VESC CAN SDK including:
 * - SDK initialization
 * - Motor control commands (duty, current, RPM)
 * - Status queries (get values, ping)
 * - Response handling with callbacks
 * - Debug output configuration
 * 
 * Copyright (c) 2025 waas AG (waas.rent)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include "vesc_can_sdk.h"
#include "vesc_buffer.h"

// ============================================================================
// Global Variables
// ============================================================================

static bool response_received = false;
static uint8_t last_controller_id = 0;
static uint8_t last_command = 0;
static uint8_t last_data[256];
static uint8_t last_data_len = 0;

// ============================================================================
// CAN Send Function (Mock Implementation)
// ============================================================================

/**
 * Mock CAN send function - in a real application, this would send actual CAN frames
 */
static bool mock_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    (void)data; // Suppress unused parameter warning
    printf("  [MOCK] CAN TX: ID=0x%03X, Len=%d\n", id, len);
    
    // Simulate some responses for demonstration
    uint8_t controller_id = id & 0xFF;
    uint8_t packet_type = (id >> 8) & 0xFF;
    
    // Simulate PONG response
    if (packet_type == CAN_PACKET_PING) {
        printf("  [MOCK] Simulating PONG response from VESC#%d\n", controller_id);
        uint8_t pong_data[1] = {controller_id};
        vesc_process_can_frame(controller_id | ((uint32_t)CAN_PACKET_PONG << 8), pong_data, 1);
        return true;
    }
    
    // Simulate STATUS response for motor control commands
    if (packet_type == CAN_PACKET_SET_DUTY || 
        packet_type == CAN_PACKET_SET_CURRENT || 
        packet_type == CAN_PACKET_SET_RPM) {
        printf("  [MOCK] Simulating STATUS response from VESC#%d\n", controller_id);
        
        // Create a mock status response
        uint8_t status_data[8];
        int32_t index = 0;
        vesc_buffer_append_int32(status_data, 1000, &index);  // RPM
        vesc_buffer_append_int16(status_data, 500, &index);   // Current (0.1A)
        vesc_buffer_append_int16(status_data, 5000, &index);  // Duty (0.001)
        
        vesc_process_can_frame(controller_id | ((uint32_t)CAN_PACKET_STATUS << 8), status_data, 8);
        return true;
    }
    
    return true;
}

// ============================================================================
// Response Callback Function
// ============================================================================

/**
 * Response callback function - called when responses are received from VESC controllers
 */
static void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("  [CALLBACK] VESC#%d response: command=0x%02X, len=%d\n", 
           controller_id, command, len);
    
    // Store response data for processing
    last_controller_id = controller_id;
    last_command = command;
    last_data_len = len > 255 ? 255 : len;
    memcpy(last_data, data, last_data_len);
    response_received = true;
    
    // Parse specific responses
    switch (command) {
        case COMM_FW_VERSION: {
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                printf("    Firmware: %d.%d, Hardware: %s\n", 
                       version.major, version.minor, version.hw_name);
            }
        } break;
        
        case COMM_GET_VALUES: {
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("    RPM: %.0f, Current: %.2fA, Duty: %.1f%%, Temp: %.1f°C\n",
                       values.rpm, values.current_motor, values.duty_cycle * 100.0f, values.temp_fet);
            }
        } break;
        
        case CAN_PACKET_PONG: {
            vesc_pong_response_t pong;
            if (vesc_parse_pong_response(data, len, &pong)) {
                printf("    PONG from VESC#%d\n", pong.controller_id);
            }
        } break;
        
        case CAN_PACKET_STATUS: {
            vesc_status_msg_1_t status;
            if (vesc_parse_status_msg_1(data, len, &status)) {
                printf("    Status: RPM=%.0f, Current=%.2fA, Duty=%.1f%%\n",
                       status.rpm, status.current, status.duty * 100.0f);
            }
        } break;
        
        default:
            printf("    Unhandled command: 0x%02X\n", command);
            break;
    }
}

// ============================================================================
// Helper Functions
// ============================================================================

/**
 * Wait for response with timeout
 */
static bool wait_for_response(int timeout_ms) {
    response_received = false;
    
    // Simple polling loop - in a real application, this would be event-driven
    for (int i = 0; i < timeout_ms / 10; i++) {
        if (response_received) {
            return true;
        }
        usleep(10000); // 10ms
    }
    
    return false;
}

/**
 * Print section header
 */
static void print_section(const char *title) {
    printf("\n%s\n", title);
    printf("%.*s\n", (int)strlen(title), "==========================================");
}

// ============================================================================
// Main Function
// ============================================================================

int main(void) {
    printf("VESC CAN SDK - Simple Example\n");
    printf("=============================\n");
    
    // ========================================================================
    // 1. SDK Initialization
    // ========================================================================
    print_section("1. SDK Initialization");
    
    // Initialize the SDK
    uint8_t receiver_id = 0;  // We'll receive from any VESC
    uint8_t sender_id = 1;    // Our sender ID
    if (!vesc_can_init(mock_can_send, receiver_id, sender_id)) {
        fprintf(stderr, "Error: Failed to initialize VESC CAN SDK\n");
        return -1;
    }
    printf("✓ SDK initialized successfully\n");
    printf("  Receiver ID: %d\n", receiver_id);
    printf("  Sender ID: %d\n", sender_id);
    
    // Set response callback
    vesc_set_response_callback(response_callback);
    printf("✓ Response callback set\n");
    
    // ========================================================================
    // 2. Debug Configuration
    // ============================================================================
    print_section("2. Debug Configuration");
    
    // Enable debug output
    vesc_debug_config_t debug_config = {
        .level = VESC_DEBUG_BASIC,
        .categories = VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES | VESC_DEBUG_CAN,
        .output_func = NULL,  // Use printf
        .enable_timestamps = true,
        .enable_statistics = true
    };
    
    if (vesc_debug_configure(&debug_config)) {
        printf("✓ Debug output enabled\n");
        printf("  Level: BASIC\n");
        printf("  Categories: COMMANDS, RESPONSES, CAN\n");
    }
    
    // ========================================================================
    // 3. Basic Commands
    // ============================================================================
    print_section("3. Basic Commands");
    
    uint8_t test_controller = 1;
    
    // Ping command
    printf("Sending PING to VESC#%d...\n", test_controller);
    vesc_ping(test_controller);
    if (wait_for_response(1000)) {
        printf("✓ PING response received\n");
    } else {
        printf("✗ PING timeout\n");
    }
    
    // Get firmware version
    printf("Getting firmware version from VESC#%d...\n", test_controller);
    vesc_get_fw_version(test_controller);
    if (wait_for_response(1000)) {
        printf("✓ Firmware version received\n");
    } else {
        printf("✗ Firmware version timeout\n");
    }
    
    // Get values
    printf("Getting values from VESC#%d...\n", test_controller);
    vesc_get_values(test_controller);
    if (wait_for_response(1000)) {
        printf("✓ Values received\n");
    } else {
        printf("✗ Values timeout\n");
    }
    
    // ========================================================================
    // 4. Motor Control Commands
    // ============================================================================
    print_section("4. Motor Control Commands");
    
    // Set duty cycle (0.5 = 50%)
    printf("Setting duty cycle to 50%% on VESC#%d...\n", test_controller);
    vesc_set_duty(test_controller, 0.5f);
    if (wait_for_response(1000)) {
        printf("✓ Duty cycle command acknowledged\n");
    } else {
        printf("✗ Duty cycle command timeout\n");
    }
    
    // Set current (5.0A)
    printf("Setting current to 5.0A on VESC#%d...\n", test_controller);
    vesc_set_current(test_controller, 5.0f);
    if (wait_for_response(1000)) {
        printf("✓ Current command acknowledged\n");
    } else {
        printf("✗ Current command timeout\n");
    }
    
    // Set RPM (1000 RPM)
    printf("Setting RPM to 1000 on VESC#%d...\n", test_controller);
    vesc_set_rpm(test_controller, 1000.0f);
    if (wait_for_response(1000)) {
        printf("✓ RPM command acknowledged\n");
    } else {
        printf("✗ RPM command timeout\n");
    }
    
    // Stop motor (set duty to 0)
    printf("Stopping motor on VESC#%d...\n", test_controller);
    vesc_set_duty(test_controller, 0.0f);
    if (wait_for_response(1000)) {
        printf("✓ Stop command acknowledged\n");
    } else {
        printf("✗ Stop command timeout\n");
    }
    
    // ========================================================================
    // 5. Multiple Controllers
    // ============================================================================
    print_section("5. Multiple Controllers");
    
    // Send commands to multiple controllers
    for (uint8_t i = 1; i <= 3; i++) {
        printf("Pinging VESC#%d...\n", i);
        vesc_ping(i);
        if (wait_for_response(500)) {
            printf("✓ VESC#%d responded\n", i);
        } else {
            printf("✗ VESC#%d timeout\n", i);
        }
    }
    
    // ========================================================================
    // 6. Statistics and Debug Information
    // ============================================================================
    print_section("6. Statistics and Debug Information");
    
    // Print debug statistics
    vesc_debug_print_stats();
    
    // Print buffer state
    vesc_debug_print_buffer_state();
    
    // ========================================================================
    // 7. Version Information
    // ============================================================================
    print_section("7. Version Information");
    
    // Print SDK version
    vesc_sdk_print_version();
    
    // Check version compatibility
    if (vesc_sdk_version_at_least(1, 0, 0)) {
        printf("✓ SDK version is compatible (1.0.0+)\n");
    } else {
        printf("✗ SDK version too old (requires 1.0.0+)\n");
    }
    
    // ========================================================================
    // 8. Cleanup
    // ============================================================================
    print_section("8. Cleanup");
    
    // Disable debug output
    vesc_debug_disable();
    printf("✓ Debug output disabled\n");
    
    printf("\nExample completed successfully!\n");
    printf("This example demonstrated:\n");
    printf("- SDK initialization and configuration\n");
    printf("- Basic commands (PING, GET_FW_VERSION, GET_VALUES)\n");
    printf("- Motor control commands (SET_DUTY, SET_CURRENT, SET_RPM)\n");
    printf("- Response handling with callbacks\n");
    printf("- Debug output and statistics\n");
    printf("- Multiple controller communication\n");
    
    return 0;
} 