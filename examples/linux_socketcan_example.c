/*
 * VESC CAN SDK - Linux SocketCAN Example
 * 
 * This example demonstrates how to use the VESC CAN SDK with Linux SocketCAN.
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
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <pthread.h>
#include "../include/vesc_can_sdk.h"

// Global variables
static bool running = true;
static int can_socket = -1;
static uint8_t vesc_id = 1;
static pthread_t receive_thread;

// Signal handler for graceful shutdown
void signal_handler(int sig) {
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = false;
}

// CAN send function for SocketCAN
bool socketcan_send(uint32_t id, uint8_t *data, uint8_t len) {
    if (can_socket < 0) {
        printf("CAN socket not initialized\n");
        return false;
    }
    
    struct can_frame frame;
    frame.can_id = id | CAN_EFF_FLAG; // Extended frame
    frame.can_dlc = len;
    memcpy(frame.data, data, len);
    
    ssize_t bytes_sent = write(can_socket, &frame, sizeof(frame));
    if (bytes_sent != sizeof(frame)) {
        printf("Failed to send CAN frame: %s\n", strerror(errno));
        return false;
    }
    
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

// CAN receive thread function
void *can_receive_thread(void *arg) {
    struct can_frame frame;
    
    while (running) {
        ssize_t bytes_read = read(can_socket, &frame, sizeof(frame));
        if (bytes_read == sizeof(frame)) {
            // Process received CAN frame
            vesc_process_can_frame(frame.can_id, frame.data, frame.can_dlc);
        } else if (bytes_read < 0 && errno != EAGAIN) {
            printf("Error reading CAN frame: %s\n", strerror(errno));
            break;
        }
    }
    
    return NULL;
}

// Initialize SocketCAN
bool init_socketcan(const char *interface) {
    // Create socket
    can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket < 0) {
        printf("Failed to create CAN socket: %s\n", strerror(errno));
        return false;
    }
    
    // Get interface index
    struct ifreq ifr;
    strcpy(ifr.ifr_name, interface);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        printf("Failed to get interface index for %s: %s\n", interface, strerror(errno));
        close(can_socket);
        return false;
    }
    
    // Bind socket to interface
    struct sockaddr_can addr;
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        printf("Failed to bind CAN socket: %s\n", strerror(errno));
        close(can_socket);
        return false;
    }
    
    // Set non-blocking mode
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);
    
    printf("SocketCAN initialized on interface %s\n", interface);
    return true;
}

int main(int argc, char *argv[]) {
    printf("VESC CAN SDK - Linux SocketCAN Example\n");
    printf("======================================\n\n");
    
    // Check command line arguments
    if (argc < 2) {
        printf("Usage: %s <can_interface> [vesc_id]\n", argv[0]);
        printf("Example: %s can0 1\n", argv[0]);
        return 1;
    }
    
    const char *interface = argv[1];
    if (argc >= 3) {
        vesc_id = atoi(argv[2]);
    }
    
    // Set up signal handler for graceful shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize SocketCAN
    if (!init_socketcan(interface)) {
        printf("Failed to initialize SocketCAN\n");
        return 1;
    }
    
    // Initialize VESC CAN SDK
    if (!vesc_can_init(socketcan_send)) {
        printf("Failed to initialize VESC CAN SDK\n");
        close(can_socket);
        return 1;
    }
    
    // Set response callback
    vesc_set_response_callback(response_callback);
    
    printf("VESC CAN SDK initialized successfully\n");
    printf("Using VESC controller ID: %d\n\n", vesc_id);
    
    // Start CAN receive thread
    if (pthread_create(&receive_thread, NULL, can_receive_thread, NULL) != 0) {
        printf("Failed to create receive thread\n");
        close(can_socket);
        return 1;
    }
    
    // Get firmware version
    printf("Getting firmware version...\n");
    vesc_get_fw_version(vesc_id);
    sleep(2);
    
    // Get current motor values
    printf("Getting motor values...\n");
    vesc_get_values(vesc_id);
    sleep(2);
    
    // Get ADC values
    printf("Getting ADC values...\n");
    vesc_get_decoded_adc(vesc_id);
    sleep(2);
    
    // Get PPM values
    printf("Getting PPM values...\n");
    vesc_get_decoded_ppm(vesc_id);
    sleep(2);
    
    // Motor control demonstration
    printf("\nStarting motor control demonstration...\n");
    printf("Press Ctrl+C to stop\n\n");
    
    float duty = 0.0f;
    float duty_step = 0.05f;
    bool increasing = true;
    
    while (running) {
        // Set motor duty cycle
        vesc_set_duty(vesc_id, duty);
        
        // Get motor values every 2 seconds
        vesc_get_values(vesc_id);
        
        printf("Duty cycle: %.1f%%\n", duty * 100.0f);
        
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
        
        sleep(2);
    }
    
    // Stop motor
    printf("\nStopping motor...\n");
    vesc_set_duty(vesc_id, 0.0f);
    vesc_set_current_brake(vesc_id, 0.0f);
    
    // Wait for receive thread to finish
    pthread_join(receive_thread, NULL);
    
    // Cleanup
    close(can_socket);
    
    printf("Example completed\n");
    return 0;
} 