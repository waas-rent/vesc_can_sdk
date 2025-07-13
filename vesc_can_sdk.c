/*
 * VESC CAN SDK - Main Implementation
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

#include "vesc_can_sdk.h"
#include "vesc_buffer.h"
#include "vesc_crc.h"
#include "vesc_version.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <stdarg.h>

#ifdef _WIN32
#include <windows.h>
#elif __linux__
#include <pthread.h>
#endif

// ============================================================================
// Internal Constants and Types
// ============================================================================

#define VESC_RX_BUFFER_SIZE 256
#define VESC_RX_BUFFER_NUM 4

typedef struct {
    uint8_t buffer[VESC_RX_BUFFER_SIZE];
    int32_t offset;
    bool active;
    uint8_t controller_id;
} vesc_rx_buffer_t;

typedef struct {
    vesc_can_send_func_t can_send_func;
    vesc_response_callback_t response_callback;
    vesc_rx_buffer_t rx_buffers[VESC_RX_BUFFER_NUM];
    uint8_t receiver_controller_id;  // Receiver controller ID (used in CAN ID field)
    uint8_t sender_id;               // Sender controller ID (used in buffer protocol first byte)
    vesc_motor_rl_response_t motor_rl_response; // Internal storage for motor R/L detection response
    vesc_flux_linkage_response_t flux_linkage_response; // Internal storage for flux linkage detection response
    bool initialized;
#ifdef _WIN32
    CRITICAL_SECTION mutex;
#elif __linux__
    pthread_mutex_t mutex;
#endif
} vesc_sdk_state_t;

typedef struct {
    vesc_debug_config_t config;
    vesc_debug_stats_t stats;
    bool enabled;
    time_t start_time;
} vesc_debug_state_t;

// ============================================================================
// Global State
// ============================================================================

static vesc_sdk_state_t sdk_state = {0};
static vesc_debug_state_t debug_state = {0};

// ============================================================================
// Debug Helper Functions
// ============================================================================

/**
 * Get current timestamp string
 */
static const char* vesc_debug_get_timestamp(void) {
    static char timestamp[32];
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    strftime(timestamp, sizeof(timestamp), "%H:%M:%S", tm_info);
    return timestamp;
}

/**
 * Debug output function
 */
static void vesc_debug_output(const char *format, ...) {
    if (!debug_state.enabled) {
        return;
    }
    
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (debug_state.config.output_func) {
        debug_state.config.output_func(buffer);
    } else {
        printf("%s", buffer);
    }
}

/**
 * Check if debug category is enabled
 */
static bool vesc_debug_category_enabled(uint16_t category) {
    return debug_state.enabled && 
           (debug_state.config.categories & category) != 0;
}

/**
 * Print hex dump of data
 */
static void vesc_debug_hex_dump(const char *prefix, uint8_t *data, uint8_t len) {
    if (!debug_state.enabled || debug_state.config.level < VESC_DEBUG_DETAILED) {
        return;
    }
    
    // Handle NULL data case (e.g., for PING packets)
    if (!data) {
        vesc_debug_output("%s(no data)\n", prefix);
        return;
    }
    
    char hex_line[128];
    char ascii_line[64];
    int hex_pos = 0;
    int ascii_pos = 0;
    
    vesc_debug_output("%s", prefix);
    
    for (int i = 0; i < len; i++) {
        // Add hex byte
        hex_pos += snprintf(hex_line + hex_pos, sizeof(hex_line) - hex_pos, 
                           "%02X ", data[i]);
        
        // Add ASCII character
        ascii_pos += snprintf(ascii_line + ascii_pos, sizeof(ascii_line) - ascii_pos,
                             "%c", (data[i] >= 32 && data[i] <= 126) ? data[i] : '.');
        
        // Print line every 16 bytes or at end
        if ((i + 1) % 16 == 0 || i == len - 1) {
            // Pad hex line to 48 characters
            while (hex_pos < 48) {
                hex_pos += snprintf(hex_line + hex_pos, sizeof(hex_line) - hex_pos, " ");
            }
            vesc_debug_output("  %s  |%s|\n", hex_line, ascii_line);
            hex_pos = 0;
            ascii_pos = 0;
        }
    }
}

/**
 * Get command name string
 */
static const char* vesc_debug_get_command_name(uint8_t command) {
    switch (command) {
        case COMM_FW_VERSION: return "FW_VERSION";
        case COMM_GET_VALUES: return "GET_VALUES";
        case COMM_GET_MCC_CONFIG: return "GET_MCC_CONFIG";
        case COMM_REBOOT: return "REBOOT";
        case COMM_DETECT_MOTOR_R_L: return "DETECT_MOTOR_R_L";
        case COMM_DETECT_MOTOR_PARAM: return "DETECT_MOTOR_PARAM";
        case COMM_DETECT_MOTOR_FLUX_LINKAGE: return "DETECT_MOTOR_FLUX_LINKAGE";
        case COMM_GET_DECODED_ADC: return "GET_DECODED_ADC";
        case COMM_GET_DECODED_PPM: return "GET_DECODED_PPM";
        case COMM_GET_DECODED_CHUK: return "GET_DECODED_CHUK";
        case COMM_FORWARD_CAN: return "FORWARD_CAN";
        case COMM_SET_CHUCK_DATA: return "SET_CHUCK_DATA";
        case COMM_CAN_UPDATE_BAUD_ALL: return "CAN_UPDATE_BAUD_ALL";
        default: return "UNKNOWN";
    }
}

/**
 * Get CAN packet type name string
 */
static const char* vesc_debug_get_packet_name(uint8_t packet_type) {
    switch (packet_type) {
        case CAN_PACKET_SET_DUTY: return "SET_DUTY";
        case CAN_PACKET_SET_CURRENT: return "SET_CURRENT";
        case CAN_PACKET_SET_CURRENT_BRAKE: return "SET_CURRENT_BRAKE";
        case CAN_PACKET_SET_RPM: return "SET_RPM";
        case CAN_PACKET_SET_POS: return "SET_POS";
        case CAN_PACKET_FILL_RX_BUFFER: return "FILL_RX_BUFFER";
        case CAN_PACKET_FILL_RX_BUFFER_LONG: return "FILL_RX_BUFFER_LONG";
        case CAN_PACKET_PROCESS_RX_BUFFER: return "PROCESS_RX_BUFFER";
        case CAN_PACKET_PROCESS_SHORT_BUFFER: return "PROCESS_SHORT_BUFFER";
        case CAN_PACKET_STATUS: return "STATUS";
        case CAN_PACKET_STATUS_2: return "STATUS_2";
        case CAN_PACKET_STATUS_3: return "STATUS_3";
        case CAN_PACKET_STATUS_4: return "STATUS_4";
        case CAN_PACKET_STATUS_5: return "STATUS_5";
        case CAN_PACKET_STATUS_6: return "STATUS_6";
        case CAN_PACKET_PING: return "PING";
        case CAN_PACKET_PONG: return "PONG";
        case CAN_PACKET_UPDATE_BAUD: return "UPDATE_BAUD";
        default: return "UNKNOWN";
    }
}

// ============================================================================
// Internal Functions
// ============================================================================

/**
 * Send CAN packet with extended ID
 */
static bool vesc_can_send_packet(uint32_t id, uint8_t *data, uint8_t len) {
    if (!sdk_state.initialized || !sdk_state.can_send_func) {
        return false;
    }
    
    // Debug output for CAN transmission
    if (vesc_debug_category_enabled(VESC_DEBUG_CAN)) {
        uint8_t controller_id = id & 0xFF;
        uint8_t packet_type = (id >> 8) & 0xFF;
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        const char *packet_name = vesc_debug_get_packet_name(packet_type);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_output("[%s] CAN TX: ID=0x%03X (VESC#%d, %s), Len=%d\n", 
                             timestamp, id, controller_id, packet_name, len);
            vesc_debug_hex_dump("  Data: ", data, len);
        } else {
            vesc_debug_output("[%s] CAN TX: VESC#%d %s (%d bytes)\n", 
                             timestamp, controller_id, packet_name, len);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.can_tx_count++;
            debug_state.stats.total_tx_bytes += len;
        }
    }
    
    return sdk_state.can_send_func(id, data, len);
}

/**
 * Send buffer using VESC packet fragmentation protocol
 */
static void vesc_send_buffer(uint8_t controller_id, uint8_t *data, uint32_t len) {
    if (len <= 6) {
        // Calculate CRC for the payload
        uint8_t index = len;
        uint16_t crc = vesc_crc16(data, len);
        data[index++] = (uint8_t)(crc >> 8);
        data[index++] = (uint8_t)(crc & 0xFF);

        // Append stop byte (char 3)
        data[index++] = 3;

        // Buffer command structure is as follows for short buffer:
        // 0: identifier of sending controller
        // 1: flag to indicate if a result should be sent (possible values: 0, 1, 2, default is 0)
        // 2: command such as COMM_GET_VALUES or COMM_FW_VERSION
        uint8_t short_buffer[8];
        short_buffer[0] = sdk_state.sender_id;  // Use sender ID for first byte
        short_buffer[1] = 0;  // Flag to indicate if a result should be sent (possible values: 0, 1, 2, default is 0)
        memcpy(short_buffer + 2, data, index);
        
        uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8);
        vesc_can_send_packet(can_id, short_buffer, index + 1);
    } else {
        // Long packet - fragment
        uint8_t send_buffer[8];
        uint32_t end_a = 0;

        uint8_t command = data[0];
        
        // Send first 255 bytes in 7-byte chunks
        for (unsigned int i = 0; i < len; i += 7) {
            if (i > 255) {
                break;
            }
            
            end_a = i + 7;
            uint8_t send_len = 7;
            send_buffer[0] = i;
            
            if ((i + 7) <= len) {
                memcpy(send_buffer + 1, data + i, send_len);
            } else {
                send_len = len - i;
                memcpy(send_buffer + 1, data + i, send_len);
            }
            
            uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8);
            vesc_can_send_packet(can_id, send_buffer, send_len + 1);
        }
        
        // Send remaining bytes in 6-byte chunks
        for (uint32_t i = end_a; i < len; i += 6) {
            uint8_t send_len = 6;
            send_buffer[0] = i >> 8;
            send_buffer[1] = i & 0xFF;
            
            if ((i + 6) <= len) {
                memcpy(send_buffer + 2, data + i, send_len);
            } else {
                send_len = len - i;
                memcpy(send_buffer + 2, data + i, send_len);
            }
            
            uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8);
            vesc_can_send_packet(can_id, send_buffer, send_len + 2);
        }
        
        // Send process command
        int32_t index = 0;
        memset(send_buffer, 0, sizeof(send_buffer));
        send_buffer[index++] = sdk_state.sender_id;  // Use sender ID for first byte
        send_buffer[index++] = 0;  // Command should be processed by the receiver
        send_buffer[index++] = (uint8_t)(len >> 8);
        send_buffer[index++] = (uint8_t)(len & 0xFF);

        uint16_t crc = vesc_crc16(data, len);
        send_buffer[index++] = (uint8_t)(crc >> 8);
        send_buffer[index++] = (uint8_t)(crc & 0xFF);

        if (vesc_debug_category_enabled(VESC_DEBUG_CAN)) {
            uint16_t length = (send_buffer[2] << 8) | send_buffer[3];
            uint16_t crc_rx = (send_buffer[4] << 8) | send_buffer[5];
            const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
            vesc_debug_output("[%s] Finalizing packet: VESC#%d %s \n(payload=%d bytes, total_for_this_packet=%d bytes, \n\tsend_buffer_byte_1=0x%02X, \n\tsend_buffer_byte_2=0x%02X, \n\tsend_buffer_byte_3=0x%02X, \n\tsend_buffer_byte_4=0x%02X, \n\tlength_calculated=%d, \n\tcrc_calculated=0x%04X, \n\tcrc_rx=0x%04X)\n", 
                             timestamp, controller_id, vesc_debug_get_command_name(command), (int)len, (int)len, send_buffer[1], send_buffer[2], send_buffer[3], send_buffer[4], length, crc, crc_rx);
        }
        
        uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8);
        vesc_can_send_packet(can_id, send_buffer, index++);
    }
}

static void vesc_process_can_frame_and_store_information(uint32_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    (void)controller_id;
    // Results for motor_r_l and flux_linkage detection should be stored internally
    if (command == COMM_DETECT_MOTOR_R_L) {
        printf("Processing motor R/L response for controller %d", controller_id);
        vesc_parse_motor_rl_response(data, len, &sdk_state.motor_rl_response);
    } else if (command == COMM_DETECT_MOTOR_FLUX_LINKAGE) {
        printf("Processing flux linkage response for controller %d", controller_id);
        vesc_parse_flux_linkage_response(data, len, &sdk_state.flux_linkage_response);
    } else {
        LOG_WARNING("Received unexpected command %d for controller %d", command, controller_id);
    } 
}

/**
 * Process received CAN frame
 */
static void vesc_process_can_frame_internal(uint32_t id, uint8_t *data, uint8_t len) {
    uint8_t controller_id = id & 0xFF;
    uint8_t packet_type = (id >> 8) & 0xFF;
    
    // Debug output for CAN reception
    if (vesc_debug_category_enabled(VESC_DEBUG_CAN)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        const char *packet_name = vesc_debug_get_packet_name(packet_type);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_output("[%s] CAN RX: ID=0x%03X (VESC#%d, %s), Len=%d\n", 
                             timestamp, id, controller_id, packet_name, len);
            vesc_debug_hex_dump("  Data: ", data, len);
        } else {
            vesc_debug_output("[%s] CAN RX: VESC#%d %s (%d bytes)\n", 
                             timestamp, controller_id, packet_name, len);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.can_rx_count++;
            debug_state.stats.total_rx_bytes += len;
        }
    }

    switch (packet_type) {
        case CAN_PACKET_FILL_RX_BUFFER: {
            if (len < 1) {
                printf("Buffer underflow: len=%d\n", len);
                return;
            }
            uint8_t offset = data[0];
            if (offset < 255) {
                // Find buffer for this controller
                int buf_idx = -1;
                for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                    if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                        vesc_debug_output("Buffer[%d]: active=%s, controller_id=%d, offset=%d, controller_id_to_use=%d\n", 
                               i, 
                               sdk_state.rx_buffers[i].active ? "true" : "false",
                               sdk_state.rx_buffers[i].controller_id,
                               sdk_state.rx_buffers[i].offset,
                               controller_id);
                    }

                    if (sdk_state.rx_buffers[i].active == true && 
                        sdk_state.rx_buffers[i].controller_id == controller_id) {
                        buf_idx = i;
                        break;
                    }
                }
                
                // If no active buffer found for this controller, allocate one
                if (buf_idx < 0) {
                    for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                        if (!sdk_state.rx_buffers[i].active) {
                            buf_idx = i;
                            sdk_state.rx_buffers[i].active = true;
                            sdk_state.rx_buffers[i].controller_id = controller_id;
                            sdk_state.rx_buffers[i].offset = 0;
                            
                            // Debug output for buffer allocation
                            if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                                vesc_debug_output("[%s] Buffer: VESC#%d allocated buffer[%d]\n", 
                                                 timestamp, controller_id, i);

                            }
                            break;
                        }
                    }
                }
                
                if (buf_idx >= 0 && offset + len - 1 < VESC_RX_BUFFER_SIZE) {
                    memcpy(sdk_state.rx_buffers[buf_idx].buffer + offset, data + 1, len - 1);
                    
                    // Debug output for buffer operation
                    if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                        vesc_debug_output("[%s] Buffer: VESC#%d FILL_RX_BUFFER offset=%d, len=%d\n", 
                                         timestamp, controller_id, offset, len - 1);
                    }
                } else {
                    if (vesc_debug_category_enabled(VESC_DEBUG_ERRORS)) {
                        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                        if (buf_idx < 0) {
                            vesc_debug_output("[%s] Buffer Error: VESC#%d FILL_RX_BUFFER no free buffer available\n", 
                                             timestamp, controller_id);
                        } else {
                            vesc_debug_output("[%s] Buffer Error: VESC#%d FILL_RX_BUFFER invalid offset=%d, buf_idx=%d, len=%d, rx_buffer_size=%d, rx_buffer_num=%d, buf_is_active=%d, buf_controller_id=%d\n", 
                                             timestamp, controller_id, offset, buf_idx, len, VESC_RX_BUFFER_SIZE, VESC_RX_BUFFER_NUM, sdk_state.rx_buffers[buf_idx].active, sdk_state.rx_buffers[buf_idx].controller_id);
                        }
                        
                        if (debug_state.config.enable_statistics) {
                            debug_state.stats.error_count++;
                        }
                    } else {
                        printf("Buffer overflow: offset=%d, len=%d\n", offset, len);
                    }
                }
            } else {
                printf("Buffer overflow: offset=%d, len=%d\n", offset, len);
            }
        } break;
        
        case CAN_PACKET_FILL_RX_BUFFER_LONG: {
            if (len < 2) {
                printf("Buffer underflow: len=%d\n", len);
                return;
            }
            uint16_t offset = (data[0] << 8) | data[1];
            if (offset >= 255 && offset + len - 2 < VESC_RX_BUFFER_SIZE) {
                // Find buffer for this controller
                int buf_idx = -1;
                for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                    if (sdk_state.rx_buffers[i].active && 
                        sdk_state.rx_buffers[i].controller_id == controller_id) {
                        buf_idx = i;
                        break;
                    }
                }
                
                // If no active buffer found for this controller, allocate one
                if (buf_idx < 0) {
                    for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                        if (!sdk_state.rx_buffers[i].active) {
                            buf_idx = i;
                            sdk_state.rx_buffers[i].active = true;
                            sdk_state.rx_buffers[i].controller_id = controller_id;
                            sdk_state.rx_buffers[i].offset = 0;
                            
                            // Debug output for buffer allocation
                            if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                                vesc_debug_output("[%s] Buffer: VESC#%d allocated buffer[%d] for FILL_RX_BUFFER_LONG\n", 
                                                 timestamp, controller_id, i);
                            }
                            break;
                        }
                    }
                }
                
                if (buf_idx >= 0) {
                    memcpy(sdk_state.rx_buffers[buf_idx].buffer + offset, data + 2, len - 2);
                    
                    // Debug output for buffer operation
                    if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                        vesc_debug_output("[%s] Buffer: VESC#%d FILL_RX_BUFFER_LONG offset=%d, len=%d\n", 
                                         timestamp, controller_id, offset, len - 2);
                    }
                } else if (vesc_debug_category_enabled(VESC_DEBUG_ERRORS)) {
                    const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                    if (buf_idx < 0) {
                        vesc_debug_output("[%s] Buffer Error: VESC#%d FILL_RX_BUFFER_LONG no free buffer available\n", 
                                         timestamp, controller_id);
                    } else {
                        vesc_debug_output("[%s] Buffer Error: VESC#%d FILL_RX_BUFFER_LONG buffer overflow (offset=%d, data_len=%d, max_offset=%d, buffer_size=%d)\n", 
                                         timestamp, controller_id, offset, len - 2, offset + len - 2, VESC_RX_BUFFER_SIZE);
                    }
                    
                    if (debug_state.config.enable_statistics) {
                        debug_state.stats.error_count++;
                    }
                }
            } else {
                printf("Buffer overflow: offset=%d, len=%d\n", offset, len);
            }
        } break;
        
        case CAN_PACKET_PROCESS_RX_BUFFER: {
            if (len < 6) {
                printf("Buffer underflow: len=%d\n", len);
                return;
            }
            
            // Add defensive checks for data array access
            if (!data) {
                printf("PROCESS_RX_BUFFER: data is NULL\n");
                return;
            }
            
            //uint8_t last_identifier_of_controller = data[0];
            uint8_t send_flag = data[1];
            uint16_t length = (data[2] << 8) | data[3];
            uint16_t crc_rx = (data[4] << 8) | data[5];
            
            // Add debug output for incoming packet
            if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                vesc_debug_output("[%s] PROCESS_RX_BUFFER: VESC#%d, send_flag=0x%02X, length=%d, crc_rx=0x%04X\n", 
                                 timestamp, controller_id, send_flag, length, crc_rx);
            }
            
            // Find buffer for this controller
            int buf_idx = -1;
            for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                if (sdk_state.rx_buffers[i].active && 
                    sdk_state.rx_buffers[i].controller_id == controller_id) {
                    buf_idx = i;
                    break;
                }
            }
            
            // Add debug output for buffer search
            if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                vesc_debug_output("[%s] PROCESS_RX_BUFFER: VESC#%d buffer search result: buf_idx=%d\n", 
                                 timestamp, controller_id, buf_idx);
            }
            
            if (buf_idx >= 0 && length <= VESC_RX_BUFFER_SIZE) {
                // Add defensive check for buffer access
                if (buf_idx >= VESC_RX_BUFFER_NUM) {
                    printf("PROCESS_RX_BUFFER: Invalid buf_idx=%d, max=%d\n", buf_idx, VESC_RX_BUFFER_NUM);
                    return;
                }

                if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                    const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                    vesc_debug_output("[%s] PROCESS_RX_BUFFER: VESC#%d calculating CRC for length=%d\n", 
                                     timestamp, controller_id, length);
                    
                    if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
                        vesc_debug_hex_dump("  Buffer data for CRC: ", sdk_state.rx_buffers[buf_idx].buffer, length);
                    }
                }
                
                uint16_t crc_calc = vesc_crc16(sdk_state.rx_buffers[buf_idx].buffer, length);
                
                // Add debug output for CRC result
                if (vesc_debug_category_enabled(VESC_DEBUG_BUFFERS)) {
                    const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                    vesc_debug_output("[%s] PROCESS_RX_BUFFER: VESC#%d CRC calc=0x%04X, rx=0x%04X, match=%s\n", 
                                     timestamp, controller_id, crc_calc, crc_rx, (crc_calc == crc_rx) ? "true" : "false");
                }
                
                if (crc_calc == crc_rx) {
                    uint8_t command = sdk_state.rx_buffers[buf_idx].buffer[0];
                
                    // Debug output for response
                    if (vesc_debug_category_enabled(VESC_DEBUG_RESPONSES)) {
                        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                        const char *command_name = vesc_debug_get_command_name(command);
                        vesc_debug_output("[%s] Response: VESC#%d %s (%d bytes)\n", 
                                         timestamp, controller_id, command_name, length);
                        
                        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
                            vesc_debug_hex_dump("  Response Data: ", sdk_state.rx_buffers[buf_idx].buffer, length);
                        }
                        
                        // Update statistics
                        if (debug_state.config.enable_statistics) {
                            debug_state.stats.response_count++;
                        }
                    }

                    // Valid packet - call callback
                    if (buf_idx >= 0 && buf_idx < VESC_RX_BUFFER_NUM && length <= VESC_RX_BUFFER_SIZE) {
                        vesc_process_can_frame_and_store_information(controller_id, command, sdk_state.rx_buffers[buf_idx].buffer, length);
                        if (sdk_state.response_callback) {
                            // Add safety check for callback parameters
                                sdk_state.response_callback(controller_id, command, 
                                                        sdk_state.rx_buffers[buf_idx].buffer, length);
                        }
                    } else {
                        printf("ERROR: Invalid buffer access in callback (buf_idx=%d, length=%d)\n", 
                                buf_idx, length);
                    }
                    
                } else {
                    // CRC error
                    if (vesc_debug_category_enabled(VESC_DEBUG_ERRORS)) {
                        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                        vesc_debug_output("[%s] CRC Error: VESC#%d (calc=0x%04X, rx=0x%04X)\n", 
                                         timestamp, controller_id, crc_calc, crc_rx);
                        
                        if (debug_state.config.enable_statistics) {
                            debug_state.stats.crc_error_count++;
                            debug_state.stats.error_count++;
                        }
                    }
                }
                sdk_state.rx_buffers[buf_idx].active = false;
            } else if (vesc_debug_category_enabled(VESC_DEBUG_ERRORS)) {
                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                if (buf_idx < 0) {
                    vesc_debug_output("[%s] Buffer Error: VESC#%d PROCESS_RX_BUFFER no active buffer found\n", 
                                     timestamp, controller_id);
                } else {
                    vesc_debug_output("[%s] Buffer Error: VESC#%d PROCESS_RX_BUFFER invalid length=%d, buffer_size=%d\n", 
                                     timestamp, controller_id, length, VESC_RX_BUFFER_SIZE);
                }
                
                if (debug_state.config.enable_statistics) {
                    debug_state.stats.error_count++;
                }
            }
        } break;
        
        case CAN_PACKET_PROCESS_SHORT_BUFFER: {
            if (len < 1) {
                printf("Buffer underflow: len=%d\n", len);
                return;
            }

            int index = 0;
            index++; // ignore controller_id
            index++; // ignore send_command_flag
            uint8_t command = data[index++];

            // Debug output for short buffer response
            if (vesc_debug_category_enabled(VESC_DEBUG_RESPONSES)) {
                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                const char *command_name = vesc_debug_get_command_name(command);
                vesc_debug_output("[%s] Short Response: VESC#%d %s (%d bytes)\n", 
                                 timestamp, controller_id, command_name, len - 1);
                
                if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
                    vesc_debug_hex_dump("  Response Data: ", data + 1, len - 1);
                }
                
                // Update statistics
                if (debug_state.config.enable_statistics) {
                    debug_state.stats.response_count++;
                }
            }

            vesc_process_can_frame_and_store_information(controller_id, packet_type, data, len);
            if (sdk_state.response_callback) {
                sdk_state.response_callback(controller_id, command, data + index, len - 3);
            } else {
                printf("No response callback: len=%d\n", len);
            }
            
        } break;
        
        default:
            // Debug output for direct packet
            if (vesc_debug_category_enabled(VESC_DEBUG_RESPONSES)) {
                const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
                const char *packet_name = vesc_debug_get_packet_name(packet_type);
                vesc_debug_output("[%s] Direct Packet: VESC#%d %s (%d bytes)\n", 
                                 timestamp, controller_id, packet_name, len);
                
                if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
                    vesc_debug_hex_dump("  Packet Data: ", data, len);
                }
                
                // Update statistics
                if (debug_state.config.enable_statistics) {
                    debug_state.stats.response_count++;
                }
            }

            // Direct packet - call callback
            vesc_process_can_frame_and_store_information(controller_id, packet_type, data, len);
            if (sdk_state.response_callback) {
                sdk_state.response_callback(controller_id, packet_type, data, len);
            }
            
            break;
    }
}

/**
 * Send command with CRC and stop byte
 */
static void vesc_send_command(uint8_t controller_id, uint8_t *data, uint32_t len) {
    uint8_t buffer[256]; // Adjust size as needed
    int32_t index = 0;

    // Copy data into buffer
    memcpy(buffer, data, len);
    index += len;

    // Debug output for command sending
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS) && len > 0) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        const char *command_name = vesc_debug_get_command_name(data[0]);
        vesc_debug_output("[%s] Sending Command: VESC#%d %s (payload=%d bytes, total=%d bytes)\n", 
                         timestamp, controller_id, command_name, (int)len, index);
        
        if (debug_state.config.level >= VESC_DEBUG_VERBOSE) {
            vesc_debug_hex_dump("  Full Packet: ", buffer, index);
        }
    }

    // Use vesc_send_buffer for actual sending
    vesc_send_buffer(controller_id, buffer, index);
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t receiver_controller_id, uint8_t sender_id) {
    if (!can_send_func) {
        return false;
    }
    
    memset(&sdk_state, 0, sizeof(sdk_state));
    sdk_state.can_send_func = can_send_func;
    sdk_state.receiver_controller_id = receiver_controller_id;  // Receiver controller ID
    sdk_state.sender_id = sender_id;                            // Sender controller ID
    sdk_state.initialized = true;
    
#ifdef _WIN32
    InitializeCriticalSection(&sdk_state.mutex);
#elif __linux__
    pthread_mutex_init(&sdk_state.mutex, NULL);
#endif
    
    return true;
}

void vesc_set_response_callback(vesc_response_callback_t callback) {
    sdk_state.response_callback = callback;
}

void vesc_set_controller_id(uint8_t receiver_controller_id) {
    sdk_state.receiver_controller_id = receiver_controller_id;
}

void vesc_set_sender_controller_id(uint8_t sender_controller_id) {
    // This sets the controller ID that will be used as the sender ID
    // in buffer protocol commands (first byte of process command)
    sdk_state.sender_id = sender_controller_id;
}

uint8_t vesc_get_sender_controller_id(void) {
    return sdk_state.sender_id;
}

void vesc_process_can_frame(uint32_t id, uint8_t *data, uint8_t len) {
    // Add defensive checks for the main entry point
    if (!sdk_state.initialized) {
        printf("ERROR: VESC SDK not initialized\n");
        return;
    }
    
    if (!data) {
        printf("ERROR: vesc_process_can_frame called with NULL data\n");
        return;
    }
    
    if (len > 8) {
        printf("ERROR: vesc_process_can_frame called with invalid len=%d (max=8)\n", len);
        return;
    }
    
#ifdef _WIN32
    EnterCriticalSection(&sdk_state.mutex);
#elif __linux__
    pthread_mutex_lock(&sdk_state.mutex);
#endif
    
    // Process all CAN frames - don't filter by controller ID
    // The response callback will handle the appropriate responses
    vesc_process_can_frame_internal(id, data, len);
    
#ifdef _WIN32
    LeaveCriticalSection(&sdk_state.mutex);
#elif __linux__
    pthread_mutex_unlock(&sdk_state.mutex);
#endif
}

/** Get the latest motor R/L response 
 * This function returns the most recent motor resistance and inductance response.
 * If the response is not valid, it returns an empty response structure.
 * The response is valid if the motor R/L detection has been performed.
 */
vesc_motor_rl_response_t vesc_get_latest_motor_rl_response() {
    if (!sdk_state.motor_rl_response.valid) {
        return (vesc_motor_rl_response_t){0};
    }
    return sdk_state.motor_rl_response;
}

/**
 * Get the latest flux linkage response
 * This function returns the most recent flux linkage response.
 * If the response is not valid, it returns an empty response structure.
 * The response is valid if the flux linkage detection has been performed.
 */
vesc_flux_linkage_response_t vesc_get_latest_flux_linkage_response() {
    if (!sdk_state.flux_linkage_response.valid) {
        return (vesc_flux_linkage_response_t){0};
    }
    return sdk_state.flux_linkage_response;
}

// ============================================================================
// Motor Control Functions (using VESC packet protocol)
// ============================================================================

void vesc_set_duty(uint8_t controller_id, float duty) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &index);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_DUTY %.3f (%.1f%%)\n", 
                         timestamp, controller_id, (double)duty, (double)(duty * 100.0f));
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_current(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_CURRENT %.2fA\n", 
                         timestamp, controller_id, (double)current);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_current_brake(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_CURRENT_BRAKE %.2fA\n", 
                         timestamp, controller_id, (double)current);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_rpm(uint8_t controller_id, float rpm) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)rpm, &index);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_RPM %.0f RPM\n", 
                         timestamp, controller_id, (double)rpm);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_handbrake(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_HANDBRAKE %.2fA\n", 
                         timestamp, controller_id, (double)current);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

// ============================================================================
// Get Firmware Version (TESTED)
// ============================================================================

void vesc_get_fw_version(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_FW_VERSION;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_FW_VERSION\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

// ============================================================================
// Ping Function
// ============================================================================

void vesc_ping(uint8_t controller_id) {
    // PING packet has no data, just send empty packet
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_PING << 8);
    
    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d PING\n", 
                         timestamp, controller_id);
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }
    
    // Send empty PING packet
    vesc_can_send_packet(can_id, NULL, 0);
}


// ============================================================================
// Motor Detection Functions
// ============================================================================

void vesc_detect_motor_r_l(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_DETECT_MOTOR_R_L;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d DETECT_MOTOR_R_L\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    vesc_send_command(controller_id, buffer, 1);
}

void vesc_detect_motor_param(uint8_t controller_id, float current, float min_rpm, float low_duty) {
    uint8_t buffer[13];
    int32_t index = 0;
    buffer[index++] = COMM_DETECT_MOTOR_PARAM;
    vesc_buffer_append_float32(buffer, current, 1e3f, &index);
    vesc_buffer_append_float32(buffer, min_rpm, 1e3f, &index);
    vesc_buffer_append_float32(buffer, low_duty, 1e3f, &index);

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d DETECT_MOTOR_PARAM (current=%.2fA, min_rpm=%.0f, low_duty=%.3f)\n", 
                         timestamp, controller_id, (double)current, (double)min_rpm, (double)low_duty);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    vesc_send_command(controller_id, buffer, index);
}

void vesc_detect_motor_flux_linkage(uint8_t controller_id, float current, float min_rpm, float duty, float resistance) {
    uint8_t buffer[17];
    int32_t index = 0;
    buffer[index++] = COMM_DETECT_MOTOR_FLUX_LINKAGE;
    vesc_buffer_append_float32(buffer, current, 1e3f, &index);
    vesc_buffer_append_float32(buffer, min_rpm, 1e3f, &index);
    vesc_buffer_append_float32(buffer, duty, 1e3f, &index);
    vesc_buffer_append_float32(buffer, resistance, 1e6f, &index);

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d DETECT_MOTOR_FLUX_LINKAGE (current=%.2fA, min_rpm=%.0f, duty=%.3f, resistance=%.6fΩ)\n", 
                         timestamp, controller_id, (double)current, (double)min_rpm, (double)duty, (double)resistance);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    vesc_send_command(controller_id, buffer, index);
}

// ============================================================================
// Configuration Functions
// ============================================================================

void vesc_can_update_baud_all(uint16_t kbits, uint16_t delay_msec) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int16(buffer, kbits, &index);
    vesc_buffer_append_int16(buffer, delay_msec, &index);
    
    uint32_t can_id = 255 | ((uint32_t)CAN_PACKET_UPDATE_BAUD << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

// ============================================================================
// Status Functions
// ============================================================================

void vesc_get_values(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_VALUES;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_VALUES\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_get_mcc_config(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_MCC_CONFIG;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_MCC_CONFIG\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_reboot(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_REBOOT;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d REBOOT\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_get_decoded_adc(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_DECODED_ADC;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_DECODED_ADC\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_get_decoded_ppm(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_DECODED_PPM;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_DECODED_PPM\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_get_decoded_chuck(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_DECODED_CHUK;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d GET_DECODED_CHUK\n", 
                         timestamp, controller_id);
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, 1);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, 1);
}

void vesc_set_chuck_data(uint8_t controller_id, const vesc_chuck_data_t *chuck_data) {
    if (!chuck_data) {
        printf("vesc_set_chuck_data: chuck_data is NULL\n");
        return;
    }
    
    uint8_t buffer[13];
    int32_t index = 0;
    buffer[index++] = COMM_SET_CHUCK_DATA;
    buffer[index++] = chuck_data->js_x;
    buffer[index++] = chuck_data->js_y;
    buffer[index++] = chuck_data->bt_c;
    buffer[index++] = chuck_data->bt_z;
    vesc_buffer_append_int16(buffer, chuck_data->acc_x, &index);
    vesc_buffer_append_int16(buffer, chuck_data->acc_y, &index);
    vesc_buffer_append_int16(buffer, chuck_data->acc_z, &index);
    buffer[index++] = chuck_data->rev_has_state ? 1 : 0;
    buffer[index++] = chuck_data->is_rev ? 1 : 0;

    // Debug output for command
    if (vesc_debug_category_enabled(VESC_DEBUG_COMMANDS)) {
        const char *timestamp = debug_state.config.enable_timestamps ? vesc_debug_get_timestamp() : "";
        vesc_debug_output("[%s] Command: VESC#%d SET_CHUCK_DATA (js_x=%d, js_y=%d, bt_c=%d, bt_z=%d, acc_x=%d, acc_y=%d, acc_z=%d, rev_has_state=%s, is_rev=%s)\n", 
                         timestamp, controller_id, 
                         chuck_data->js_x, chuck_data->js_y, 
                         chuck_data->bt_c, chuck_data->bt_z,
                         chuck_data->acc_x, chuck_data->acc_y, chuck_data->acc_z,
                         chuck_data->rev_has_state ? "true" : "false",
                         chuck_data->is_rev ? "true" : "false");
        
        if (debug_state.config.level >= VESC_DEBUG_DETAILED) {
            vesc_debug_hex_dump("  Command Data: ", buffer, index);
        }
        
        // Update statistics
        if (debug_state.config.enable_statistics) {
            debug_state.stats.command_count++;
        }
    }

    // Use vesc_send_command to handle CRC and stop byte
    vesc_send_command(controller_id, buffer, index);
}

// ============================================================================
// Response Parsing Functions
// ============================================================================

bool vesc_parse_get_values(uint8_t *data, uint8_t len, vesc_values_t *values) {
    if (!data || !values || len < 32) {
        printf("vesc_parse_get_values: data is NULL or values is NULL or len is less than 32\n");
        return false;
    }
    
    int32_t index = 1;
    values->temp_fet = vesc_buffer_get_float16(data, 1e1f, &index);
    values->temp_motor = vesc_buffer_get_float16(data, 1e1f, &index);
    values->current_motor = vesc_buffer_get_float32(data, 1e2f, &index);
    values->current_in = vesc_buffer_get_float32(data, 1e2f, &index);
    values->current_id = vesc_buffer_get_float32(data, 1e2f, &index);
    values->current_iq = vesc_buffer_get_float32(data, 1e2f, &index);
    values->duty_cycle = vesc_buffer_get_float16(data, 1e3f, &index);
    values->rpm = vesc_buffer_get_float32(data, 1e0f, &index);
    values->v_in = vesc_buffer_get_float16(data, 1e1f, &index);
    values->amp_hours = vesc_buffer_get_float32(data, 1e4f, &index);
    values->amp_hours_charged = vesc_buffer_get_float32(data, 1e4f, &index);
    values->watt_hours = vesc_buffer_get_float32(data, 1e4f, &index);
    values->watt_hours_charged = vesc_buffer_get_float32(data, 1e4f, &index);
    values->tachometer = vesc_buffer_get_int32(data, &index);
    values->tachometer_abs = vesc_buffer_get_int32(data, &index);
    values->fault_code = data[index++];
    values->pid_pos = vesc_buffer_get_float32(data, 1e6f, &index);
    values->controller_id = data[index++];
    values->temp_mos1 = vesc_buffer_get_float16(data, 1e1f, &index);
    values->temp_mos2 = vesc_buffer_get_float16(data, 1e1f, &index);
    values->temp_mos3 = vesc_buffer_get_float16(data, 1e1f, &index);
    values->vd = vesc_buffer_get_float32(data, 1e3f, &index);
    values->vq = vesc_buffer_get_float32(data, 1e3f, &index);
    values->status = data[index++];
    
    return true;
}

bool vesc_parse_motor_rl_response(uint8_t *data, uint8_t len, vesc_motor_rl_response_t *response) {
    if (!data || !response || len < 12) {
        return false;
    }
    
    int32_t index = 1;
    response->resistance = vesc_buffer_get_float32(data, 1e6f, &index);
    response->inductance = vesc_buffer_get_float32(data, 1e3f, &index);
    response->ld_lq_diff = vesc_buffer_get_float32(data, 1e3f, &index);
    response->valid = true;
    
    return true;
}

bool vesc_parse_motor_param_response(uint8_t *data, uint8_t len, vesc_motor_param_response_t *response) {
    if (!data || !response || len < 20) {
        return false;
    }
    
    int32_t index = 1;
    response->cycle_int_limit = vesc_buffer_get_float32(data, 1e3f, &index);
    response->coupling_k = vesc_buffer_get_float32(data, 1e3f, &index);
    
    for (int i = 0; i < 8; i++) {
        response->hall_table[i] = (int8_t)data[index++];
    }
    
    response->hall_res = vesc_buffer_get_int32(data, &index);
    response->valid = true;
    
    return true;
}

bool vesc_parse_flux_linkage_response(uint8_t *data, uint8_t len, vesc_flux_linkage_response_t *response) {
    if (!data || !response || len < 4) {
        return false;
    }
    
    int32_t index = 1;
    response->flux_linkage = vesc_buffer_get_float32(data, 1e7f, &index);
    response->valid = true;
    
    return true;
}

bool vesc_parse_adc_values(uint8_t *data, uint8_t len, vesc_adc_values_t *values) {
    if (!data || !values || len < 16) {
        printf("vesc_parse_adc_values: data is NULL or values is NULL or len is less than 16\n");
        return false;
    }
    
    int32_t index = 1;
    // Parse int32 values with 1e6 scaling to match the filling code
    values->adc1 = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->adc2 = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->adc3 = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->v_in = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->valid = true;
    
    return true;
}

bool vesc_parse_ppm_values(uint8_t *data, uint8_t len, vesc_ppm_values_t *values) {
    if (!data || !values || len < 8) {
        printf("vesc_parse_ppm_values: data is NULL or values is NULL or len is less than 8\n");
        return false;
    }
    
    int32_t index = 1;
    // Parse int32 values with 1e6 scaling to match the filling code
    values->ppm = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->pulse_len = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->valid = true;
    
    return true;
}

bool vesc_parse_chuck_values(uint8_t *data, uint8_t len, vesc_chuck_values_t *values) {
    if (!data || !values || len < 5) {
        printf("vesc_parse_chuck_values: data is NULL or values is NULL or len is less than 5\n");
        return false;
    }
    
    int32_t index = 1;
    // Parse int32 value with 1e6 scaling to match the filling code
    values->js_y = (float)vesc_buffer_get_int32(data, &index) / 1000000.0f;
    values->valid = true;
    
    return true;
}

bool vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version) {
    if (!data || !version || len < 30) {
        printf("vesc_parse_fw_version: data is NULL or version is NULL or len is less than 30\n");
        return false;
    }
    
    int32_t index = 1;
    version->major = data[index++];
    version->minor = data[index++];
    
    // Copy hardware name (null-terminated)
    int name_len = 0;
    while (index < len && name_len < 31 && data[index] != 0) {
        version->hw_name[name_len++] = data[index++];
    }
    version->hw_name[name_len] = '\0';
    index++; // Skip null terminator
    
    // Copy UUID
    for (int i = 0; i < 12 && index < len; i++) {
        version->uuid[i] = data[index++];
    }
    
    version->pairing_done = data[index++];
    version->test_version = data[index++];
    version->hw_type = data[index++];
    version->cfg_num = data[index++];
    version->valid = true;
    
    return true;
}

// ============================================================================
// Status Message Parsing Functions
// ============================================================================

bool vesc_parse_status_msg_1(uint8_t *data, uint8_t len, vesc_status_msg_1_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->rpm = (float)vesc_buffer_get_int32(data, &index);
    status->current = (float)vesc_buffer_get_int16(data, &index) / 10.0f;
    status->duty = (float)vesc_buffer_get_int16(data, &index) / 1000.0f;
    status->valid = true;
    
    return true;
}

bool vesc_parse_status_msg_2(uint8_t *data, uint8_t len, vesc_status_msg_2_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->amp_hours = (float)vesc_buffer_get_int32(data, &index) / 1e4f;
    status->amp_hours_charged = (float)vesc_buffer_get_int32(data, &index) / 1e4f;
    status->valid = true;
    
    return true;
}

bool vesc_parse_status_msg_3(uint8_t *data, uint8_t len, vesc_status_msg_3_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->watt_hours = (float)vesc_buffer_get_int32(data, &index) / 1e4f;
    status->watt_hours_charged = (float)vesc_buffer_get_int32(data, &index) / 1e4f;
    status->valid = true;
    
    return true;
}

bool vesc_parse_status_msg_4(uint8_t *data, uint8_t len, vesc_status_msg_4_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->temp_fet = (float)vesc_buffer_get_int16(data, &index) / 10.0f;
    status->temp_motor = (float)vesc_buffer_get_int16(data, &index) / 10.0f;
    status->current_in = (float)vesc_buffer_get_int16(data, &index) / 10.0f;
    status->pid_pos_now = (float)vesc_buffer_get_int16(data, &index) / 50.0f;
    status->valid = true;
    
    return true;
}

bool vesc_parse_status_msg_5(uint8_t *data, uint8_t len, vesc_status_msg_5_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->tacho_value = vesc_buffer_get_int32(data, &index);
    status->v_in = (float)vesc_buffer_get_int16(data, &index) / 10.0f;
    status->valid = true;
    
    return true;
}

bool vesc_parse_status_msg_6(uint8_t *data, uint8_t len, vesc_status_msg_6_t *status) {
    if (!data || !status || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    status->adc_1 = vesc_buffer_get_float16(data, 1e3f, &index);
    status->adc_2 = vesc_buffer_get_float16(data, 1e3f, &index);
    status->adc_3 = vesc_buffer_get_float16(data, 1e3f, &index);
    status->ppm = vesc_buffer_get_float16(data, 1e3f, &index);
    status->valid = true;
    
    return true;
}

bool vesc_parse_pong_response(uint8_t *data, uint8_t len, vesc_pong_response_t *pong) {
    if (!data || !pong || len < 1) {
        return false;
    }
    
    pong->controller_id = data[0];
    pong->valid = true;
    
    return true;
}

// ============================================================================
// Debug Functions
// ============================================================================

bool vesc_debug_configure(vesc_debug_config_t *config) {
    if (!config) {
        printf("vesc_debug_configure: config is NULL\n");
        return false;
    }
    
    // Validate debug level
    if (config->level > VESC_DEBUG_VERBOSE) {
        printf("vesc_debug_configure: level is greater than VESC_DEBUG_VERBOSE, it is %d\n", config->level);
        return false;
    }
    
    // Copy configuration
    debug_state.config = *config;
    debug_state.enabled = (config->level > VESC_DEBUG_NONE);
    debug_state.start_time = time(NULL);
    
    // Initialize statistics if enabled
    if (config->enable_statistics) {
        memset(&debug_state.stats, 0, sizeof(debug_state.stats));
    }
    
    // Debug output for configuration
    if (debug_state.enabled) {
        vesc_debug_output("VESC Debug: Enabled (level=%d, categories=0x%04X)\n", 
                         config->level, config->categories);
    }
    
    return true;
}

bool vesc_debug_enable(uint8_t level, uint16_t categories) {
    vesc_debug_config_t config = {
        .level = level,
        .categories = categories,
        .output_func = NULL,
        .enable_timestamps = true,
        .enable_statistics = true
    };
    
    return vesc_debug_configure(&config);
}

void vesc_debug_disable(void) {
    debug_state.enabled = false;
    vesc_debug_output("VESC Debug: Disabled\n");
}

void vesc_debug_set_output_func(vesc_debug_output_func_t output_func) {
    debug_state.config.output_func = output_func;
}

bool vesc_debug_get_stats(vesc_debug_stats_t *stats) {
    if (!stats) {
        return false;
    }
    
    *stats = debug_state.stats;
    return true;
}

bool vesc_debug_get_config(vesc_debug_config_t *config) {
    if (!config) {
        return false;
    }
    
    *config = debug_state.config;
    return true;
}

void vesc_debug_reset_stats(void) {
    memset(&debug_state.stats, 0, sizeof(debug_state.stats));
    debug_state.start_time = time(NULL);
    vesc_debug_output("VESC Debug: Statistics reset\n");
}

void vesc_debug_print_stats(void) {
    if (!debug_state.enabled) {
        printf("VESC Debug: Not enabled\n");
        return;
    }
    
    time_t now = time(NULL);
    time_t uptime = now - debug_state.start_time;
    
    printf("\n=== VESC Debug Statistics ===\n");
#ifdef __ZEPHYR__
    printf("Uptime: %lld seconds\n", uptime);
#else
    printf("Uptime: %ld seconds\n", uptime);
#endif
    printf("CAN Transmissions: %u\n", debug_state.stats.can_tx_count);
    printf("CAN Receptions: %u\n", debug_state.stats.can_rx_count);
    printf("Commands Sent: %u\n", debug_state.stats.command_count);
    printf("Responses Received: %u\n", debug_state.stats.response_count);
    printf("Errors: %u\n", debug_state.stats.error_count);
    printf("CRC Errors: %u\n", debug_state.stats.crc_error_count);
    printf("Buffer Overflows: %u\n", debug_state.stats.buffer_overflow_count);
    printf("Total TX Bytes: %lu\n", (unsigned long)debug_state.stats.total_tx_bytes);
    printf("Total RX Bytes: %lu\n", (unsigned long)debug_state.stats.total_rx_bytes);
    
    if (uptime > 0) {
        printf("TX Rate: %.1f bytes/sec\n", (double)debug_state.stats.total_tx_bytes / uptime);
        printf("RX Rate: %.1f bytes/sec\n", (double)debug_state.stats.total_rx_bytes / uptime);
        printf("CAN TX Rate: %.1f msgs/sec\n", (double)debug_state.stats.can_tx_count / uptime);
        printf("CAN RX Rate: %.1f msgs/sec\n", (double)debug_state.stats.can_rx_count / uptime);
    }
    printf("=============================\n\n");
}

void vesc_debug_print_buffer_state(void) {
    printf("\n=== VESC RX Buffer State ===\n");
    for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
        printf("Buffer[%d]: active=%s, controller_id=%d, offset=%d\n", 
               i, 
               sdk_state.rx_buffers[i].active ? "true" : "false",
               sdk_state.rx_buffers[i].controller_id,
               sdk_state.rx_buffers[i].offset);
        
        if (sdk_state.rx_buffers[i].active && debug_state.enabled && debug_state.config.level >= VESC_DEBUG_DETAILED) {
            printf("  Buffer data (first 16 bytes): ");
            for (int j = 0; j < 16 && j < VESC_RX_BUFFER_SIZE; j++) {
                printf("%02X ", sdk_state.rx_buffers[i].buffer[j]);
            }
            printf("\n");
        }
    }
    printf("=============================\n\n");
}

// ============================================================================
// Version Functions Implementation
// ============================================================================

// Git information - these will be set by the build system if available
#ifndef VESC_SDK_GIT_HASH
#define VESC_SDK_GIT_HASH "unknown"
#endif

#ifndef VESC_SDK_GIT_BRANCH
#define VESC_SDK_GIT_BRANCH "unknown"
#endif

// Static version information structure
static const vesc_sdk_version_t sdk_version = {
    .major = VESC_SDK_VERSION_MAJOR,
    .minor = VESC_SDK_VERSION_MINOR,
    .patch = VESC_SDK_VERSION_PATCH,
    .version_number = VESC_SDK_VERSION_NUMBER,
    .version_string = VESC_SDK_VERSION_STRING,
    .build_date = VESC_SDK_BUILD_DATE,
    .build_time = VESC_SDK_BUILD_TIME,
    .git_hash = VESC_SDK_GIT_HASH,
    .git_branch = VESC_SDK_GIT_BRANCH
};

const vesc_sdk_version_t* vesc_sdk_get_version(void) {
    return &sdk_version;
}

const char* vesc_sdk_get_version_string(void) {
    return sdk_version.version_string;
}

uint32_t vesc_sdk_get_version_number(void) {
    return sdk_version.version_number;
}

bool vesc_sdk_version_at_least(uint8_t major, uint8_t minor, uint8_t patch) {
    uint32_t required_version = (major * 10000) + (minor * 100) + patch;
    return sdk_version.version_number >= required_version;
}

void vesc_sdk_print_version(void) {
    printf("\n=== VESC CAN SDK Version Information ===\n");
    printf("Version: %s\n", sdk_version.version_string);
    printf("Build Date: %s\n", sdk_version.build_date);
    printf("Build Time: %s\n", sdk_version.build_time);
    printf("Git Hash: %s\n", sdk_version.git_hash);
    printf("Git Branch: %s\n", sdk_version.git_branch);
    printf("========================================\n\n");
}