/*
 * VESC CAN SDK - Basic Control Example
 * 
 * This example demonstrates how to use the VESC CAN SDK to control a motor.
 * 
 * Copyright (c) 2024 VESC CAN SDK Contributors
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
    printf("CAN TX: ID=0x%03X, Len=%d, Data=", id, len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // In a real implementation, you would send this over your CAN interface
    // For example, with SocketCAN:
    // struct can_frame frame;
    // frame.can_id = id | CAN_EFF_FLAG; // Extended frame
    // frame.can_dlc = len;
    // memcpy(frame.data, data, len);
    // return write(can_socket, &frame, sizeof(frame)) == sizeof(frame);
    
    return true;
}

// Response callback function
void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    printf("VESC %d Response: Command=0x%02X, Len=%d, Data=", controller_id, command, len);
    for (int i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    
    // Parse specific responses
    switch (command) {
        case COMM_GET_VALUES: {
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("  Motor Values:\n");
                printf("    Temperature FET: %.1f°C\n", values.temp_fet);
                printf("    Temperature Motor: %.1f°C\n", values.temp_motor);
                printf("    Current Motor: %.2fA\n", values.current_motor);
                printf("    Current Input: %.2fA\n", values.current_in);
                printf("    Duty Cycle: %.1f%%\n", values.duty_cycle * 100.0f);
                printf("    RPM: %.0f\n", values.rpm);
                printf("    Input Voltage: %.1fV\n", values.v_in);
                printf("    Consumed Ah: %.2fAh\n", values.amp_hours);
                printf("    Consumed Wh: %.2fWh\n", values.watt_hours);
            }
        } break;
        
        case COMM_DETECT_MOTOR_R_L: {
            vesc_motor_rl_response_t response;
            if (vesc_parse_motor_rl_response(data, len, &response)) {
                printf("  Motor R/L Detection:\n");
                printf("    Resistance: %.6f Ω\n", response.resistance);
                printf("    Inductance: %.8f H\n", response.inductance);
                printf("    Ld-Lq Difference: %.8f H\n", response.ld_lq_diff);
            }
        } break;
        
        case COMM_FW_VERSION: {
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                printf("  Firmware Version:\n");
                printf("    Version: %d.%d\n", version.major, version.minor);
                printf("    Hardware: %s\n", version.hw_name);
                printf("    Hardware Type: %d\n", version.hw_type);
                printf("    Config Number: %d\n", version.cfg_num);
            }
        } break;
        
        case COMM_GET_DECODED_ADC: {
            vesc_adc_values_t adc;
            if (vesc_parse_adc_values(data, len, &adc)) {
                printf("  ADC Values:\n");
                printf("    ADC1: %.3f\n", adc.adc1);
                printf("    ADC2: %.3f\n", adc.adc2);
                printf("    ADC3: %.3f\n", adc.adc3);
                printf("    Input Voltage: %.1fV\n", adc.v_in);
            }
        } break;
        
        case COMM_GET_DECODED_PPM: {
            vesc_ppm_values_t ppm;
            if (vesc_parse_ppm_values(data, len, &ppm)) {
                printf("  PPM Values:\n");
                printf("    PPM: %.3f\n", ppm.ppm);
                printf("    Pulse Length: %.1f μs\n", ppm.pulse_len);
            }
        } break;
    }
}

int main(int argc, char *argv[]) {
    printf("VESC CAN SDK - Basic Control Example\n");
    printf("====================================\n\n");
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize VESC CAN SDK
    if (!vesc_can_init(example_can_send)) {
        printf("Failed to initialize VESC CAN SDK\n");
        return 1;
    }
    
    // Set response callback
    vesc_set_response_callback(response_callback);
    
    printf("VESC CAN SDK initialized successfully\n");
    printf("Using VESC controller ID: %d\n\n", vesc_id);
    
    // Get firmware version
    printf("Getting firmware version...\n");
    vesc_get_fw_version(vesc_id);
    sleep(1);
    
    // Get current motor values
    printf("Getting motor values...\n");
    vesc_get_values(vesc_id);
    sleep(1);
    
    // Get ADC values
    printf("Getting ADC values...\n");
    vesc_get_decoded_adc(vesc_id);
    sleep(1);
    
    // Get PPM values
    printf("Getting PPM values...\n");
    vesc_get_decoded_ppm(vesc_id);
    sleep(1);
    
    // Motor control demonstration
    printf("\nStarting motor control demonstration...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    float duty = 0.0f;
    float duty_step = 0.1f;
    bool increasing = true;
    
    while (running) {
        // Set motor duty cycle
        vesc_set_duty(vesc_id, duty);
        
        // Get motor values every second
        vesc_get_values(vesc_id);
        
        printf("Duty cycle: %.1f%%\n", duty * 100.0f);
        
        // Update duty cycle (ramp up and down)
        if (increasing) {
            duty += duty_step;
            if (duty >= 0.5f) {
                duty = 0.5f;
                increasing = false;
            }
        } else {
            duty -= duty_step;
            if (duty <= -0.5f) {
                duty = -0.5f;
                increasing = true;
            }
        }
        
        sleep(1);
    }
    
    // Stop motor
    printf("\nStopping motor...\n");
    vesc_set_duty(vesc_id, 0.0f);
    vesc_set_current_brake(vesc_id, 0.0f);
    
    printf("Example completed\n");
    return 0;
} 