/*
 * VESC CAN SDK - Debug Example
 * 
 * This example demonstrates how to use the debugging functionality
 * of the VESC CAN SDK to monitor communication and troubleshoot issues.
 * 
 * Copyright (c) 2025 waas AG (waas.rent)
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include "../include/vesc_can_sdk.h"

// Global variables for cleanup
static bool running = true;
static uint8_t vesc_id = 1; // VESC controller ID

// Signal handler for graceful shutdown
void signal_handler(int sig) {
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = false;
}

// Example CAN send function (replace with your actual CAN implementation)
bool example_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    // In a real implementation, you would send this over your CAN interface
    // For example, with SocketCAN:
    // struct can_frame frame;
    // frame.can_id = id | CAN_EFF_FLAG; // Extended frame
    // frame.can_dlc = len;
    // memcpy(frame.data, data, len);
    // return write(can_socket, &frame, sizeof(frame)) == sizeof(frame);
    
    // For this example, we'll just simulate successful transmission
    return true;
}

// Response callback function
void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("VESC %d Response: Command=0x%02X, Len=%d\n", controller_id, command, len);
    
    // Parse specific responses
    switch (command) {
        case COMM_GET_VALUES: {
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("  Motor Values: RPM=%.0f, Current=%.2fA, Duty=%.1f%%, Temp=%.1f°C\n", 
                       values.rpm, values.current_motor, values.duty_cycle * 100.0f, values.temp_fet);
            }
        } break;
        
        case COMM_DETECT_MOTOR_R_L: {
            vesc_motor_rl_response_t response;
            if (vesc_parse_motor_rl_response(data, len, &response)) {
                printf("  Motor R/L: R=%.6fΩ, L=%.8fH\n", response.resistance, response.inductance);
            }
        } break;
        
        case COMM_FW_VERSION: {
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                printf("  Firmware: %d.%d, Hardware: %s\n", version.major, version.minor, version.hw_name);
            }
        } break;
    }
}

// Custom debug output function example
void custom_debug_output(const char *message) {
    // You can redirect debug output to a file, syslog, or custom logging system
    FILE *debug_file = fopen("vesc_debug.log", "a");
    if (debug_file) {
        fprintf(debug_file, "%s", message);
        fclose(debug_file);
    }
    // Also print to console
    printf("%s", message);
}

int main(int argc, char *argv[]) {
    printf("VESC CAN SDK - Debug Example\n");
    printf("============================\n\n");
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize VESC CAN SDK
    if (!vesc_can_init(example_can_send, vesc_id)) {
        printf("Failed to initialize VESC CAN SDK\n");
        return 1;
    }
    
    // Set response callback
    vesc_set_response_callback(response_callback);
    
    printf("VESC CAN SDK initialized successfully\n");
    printf("Using VESC controller ID: %d\n\n", vesc_id);
    
    // Example 1: Basic debugging (commands and responses only)
    printf("=== Example 1: Basic Debugging ===\n");
    vesc_debug_enable(VESC_DEBUG_BASIC, VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES);
    
    printf("Getting firmware version...\n");
    vesc_get_fw_version(vesc_id);
    sleep(1);
    
    printf("Getting motor values...\n");
    vesc_get_values(vesc_id);
    sleep(1);
    
    // Disable debugging
    vesc_debug_disable();
    printf("\n");
    
    // Example 2: Detailed debugging with CAN communication
    printf("=== Example 2: Detailed Debugging ===\n");
    vesc_debug_enable(VESC_DEBUG_DETAILED, VESC_DEBUG_CAN | VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES);
    
    printf("Setting motor current to 5.0A...\n");
    vesc_set_current(vesc_id, 5.0f);
    sleep(1);
    
    printf("Setting motor duty to 25%%...\n");
    vesc_set_duty(vesc_id, 0.25f);
    sleep(1);
    
    // Disable debugging
    vesc_debug_disable();
    printf("\n");
    
    // Example 3: Custom debug output function
    printf("=== Example 3: Custom Debug Output ===\n");
    vesc_debug_enable(VESC_DEBUG_BASIC, VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES);
    vesc_debug_set_output_func(custom_debug_output);
    
    printf("Getting ADC values (debug output to file)...\n");
    vesc_get_decoded_adc(vesc_id);
    sleep(1);
    
    // Reset to default output
    vesc_debug_set_output_func(NULL);
    vesc_debug_disable();
    printf("\n");
    
    // Example 4: Full debugging with statistics
    printf("=== Example 4: Full Debugging with Statistics ===\n");
    vesc_debug_config_t config = {
        .level = VESC_DEBUG_VERBOSE,
        .categories = VESC_DEBUG_ALL,
        .output_func = NULL,
        .enable_timestamps = true,
        .enable_statistics = true
    };
    vesc_debug_configure(&config);
    
    printf("Running motor control demonstration with full debugging...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    float duty = 0.0f;
    float duty_step = 0.1f;
    bool increasing = true;
    int cycle_count = 0;
    
    while (running && cycle_count < 10) {
        // Set motor duty cycle
        vesc_set_duty(vesc_id, duty);
        
        // Get motor values every second
        vesc_get_values(vesc_id);
        
        printf("Cycle %d: Duty cycle: %.1f%%\n", cycle_count + 1, duty * 100.0f);
        
        // Update duty cycle (ramp up and down)
        if (increasing) {
            duty += duty_step;
            if (duty >= 0.3f) {
                duty = 0.3f;
                increasing = false;
            }
        } else {
            duty -= duty_step;
            if (duty <= -0.3f) {
                duty = -0.3f;
                increasing = true;
            }
        }
        
        cycle_count++;
        sleep(1);
    }
    
    // Print debug statistics
    vesc_debug_print_stats();
    
    // Disable debugging
    vesc_debug_disable();
    
    printf("Debug example completed successfully!\n");
    return 0;
} 