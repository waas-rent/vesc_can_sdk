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
#include <string.h>

// ============================================================================
// Internal Constants and Types
// ============================================================================

#define VESC_RX_BUFFER_SIZE 256
#define VESC_RX_BUFFER_NUM 4

typedef struct {
    uint8_t buffer[VESC_RX_BUFFER_SIZE];
    int32_t offset;
    bool active;
} vesc_rx_buffer_t;

typedef struct {
    vesc_can_send_func_t can_send_func;
    vesc_response_callback_t response_callback;
    vesc_rx_buffer_t rx_buffers[VESC_RX_BUFFER_NUM];
    uint8_t controller_id;
    bool initialized;
} vesc_sdk_state_t;

// ============================================================================
// Global State
// ============================================================================

static vesc_sdk_state_t sdk_state = {0};

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
    return sdk_state.can_send_func(id, data, len);
}

/**
 * Send buffer using VESC packet fragmentation protocol
 */
static void vesc_send_buffer(uint8_t controller_id, uint8_t *data, uint32_t len, uint8_t send) {
    if (len <= 6) {
        // Short packet - send directly
        uint8_t buffer[8];
        int32_t index = 0;
        buffer[index++] = send;
        memcpy(buffer + index, data, len);
        index += len;
        
        uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8);
        vesc_can_send_packet(can_id, buffer, index);
    } else {
        // Long packet - fragment
        uint8_t send_buffer[8];
        uint32_t end_a = 0;
        
        // Send first 255 bytes in 7-byte chunks
        for (uint32_t i = 0; i < len; i += 7) {
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
        send_buffer[index++] = sdk_state.controller_id;
        send_buffer[index++] = send;
        send_buffer[index++] = len >> 8;
        send_buffer[index++] = len & 0xFF;
        uint16_t crc = vesc_crc16(data, len);
        send_buffer[index++] = (uint8_t)(crc >> 8);
        send_buffer[index++] = (uint8_t)(crc & 0xFF);
        
        uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8);
        vesc_can_send_packet(can_id, send_buffer, index);
    }
}



/**
 * Process received CAN frame
 */
static void vesc_process_can_frame_internal(uint32_t id, uint8_t *data, uint8_t len) {
    uint8_t controller_id = id & 0xFF;
    uint8_t packet_type = (id >> 8) & 0xFF;
    
    switch (packet_type) {
        case CAN_PACKET_FILL_RX_BUFFER: {
            if (len < 1) return;
            uint8_t offset = data[0];
            if (offset < 255) {
                // Find buffer for this controller
                int buf_idx = -1;
                for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                    if (sdk_state.rx_buffers[i].active && 
                        sdk_state.rx_buffers[i].buffer[0] == controller_id) {
                        buf_idx = i;
                        break;
                    }
                }
                
                if (buf_idx >= 0 && offset + len - 1 < VESC_RX_BUFFER_SIZE) {
                    memcpy(sdk_state.rx_buffers[buf_idx].buffer + offset, data + 1, len - 1);
                }
            }
        } break;
        
        case CAN_PACKET_FILL_RX_BUFFER_LONG: {
            if (len < 2) return;
            uint16_t offset = (data[0] << 8) | data[1];
            if (offset >= 255 && offset + len - 2 < VESC_RX_BUFFER_SIZE) {
                // Find buffer for this controller
                int buf_idx = -1;
                for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                    if (sdk_state.rx_buffers[i].active && 
                        sdk_state.rx_buffers[i].buffer[0] == controller_id) {
                        buf_idx = i;
                        break;
                    }
                }
                
                if (buf_idx >= 0) {
                    memcpy(sdk_state.rx_buffers[buf_idx].buffer + offset, data + 2, len - 2);
                }
            }
        } break;
        
        case CAN_PACKET_PROCESS_RX_BUFFER: {
            if (len < 6) return;
            uint8_t send = data[1];
            uint16_t length = (data[2] << 8) | data[3];
            uint16_t crc_rx = (data[4] << 8) | data[5];
            
            // Find buffer for this controller
            int buf_idx = -1;
            for (int i = 0; i < VESC_RX_BUFFER_NUM; i++) {
                if (sdk_state.rx_buffers[i].active && 
                    sdk_state.rx_buffers[i].buffer[0] == controller_id) {
                    buf_idx = i;
                    break;
                }
            }
            
            if (buf_idx >= 0 && length <= VESC_RX_BUFFER_SIZE) {
                uint16_t crc_calc = vesc_crc16(sdk_state.rx_buffers[buf_idx].buffer, length);
                if (crc_calc == crc_rx) {
                    // Valid packet - call callback
                    if (sdk_state.response_callback) {
                        sdk_state.response_callback(controller_id, send, 
                                                   sdk_state.rx_buffers[buf_idx].buffer, length);
                    }
                }
                sdk_state.rx_buffers[buf_idx].active = false;
            }
        } break;
        
        case CAN_PACKET_PROCESS_SHORT_BUFFER: {
            if (len < 1) return;
            uint8_t send = data[0];
            if (sdk_state.response_callback) {
                sdk_state.response_callback(controller_id, send, data + 1, len - 1);
            }
        } break;
        
        default:
            // Direct packet - call callback
            if (sdk_state.response_callback) {
                sdk_state.response_callback(controller_id, packet_type, data, len);
            }
            break;
    }
}

// ============================================================================
// Public API Implementation
// ============================================================================

bool vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t controller_id) {
    if (!can_send_func) {
        return false;
    }
    
    memset(&sdk_state, 0, sizeof(sdk_state));
    sdk_state.can_send_func = can_send_func;
    sdk_state.controller_id = controller_id;
    sdk_state.initialized = true;
    
    return true;
}

void vesc_set_response_callback(vesc_response_callback_t callback) {
    sdk_state.response_callback = callback;
}

void vesc_set_controller_id(uint8_t controller_id) {
    sdk_state.controller_id = controller_id;
}

void vesc_process_can_frame(uint32_t id, uint8_t *data, uint8_t len) {
    // Extract controller ID from CAN frame
    uint8_t frame_controller_id = id & 0xFF;
    
    // Only process frames from the configured controller ID
    if (frame_controller_id != sdk_state.controller_id) {
        return;
    }
    
    vesc_process_can_frame_internal(id, data, len);
}

// ============================================================================
// Motor Control Functions
// ============================================================================

void vesc_set_duty(uint8_t controller_id, float duty) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &index);
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_current(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_current_brake(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_rpm(uint8_t controller_id, float rpm) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)rpm, &index);
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

void vesc_set_handbrake(uint8_t controller_id, float current) {
    uint8_t buffer[4];
    int32_t index = 0;
    vesc_buffer_append_int32(buffer, (int32_t)(current * 1000.0f), &index);
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_HANDBRAKE << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

// ============================================================================
// Motor Detection Functions
// ============================================================================

void vesc_detect_motor_r_l(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_DETECT_MOTOR_R_L;
    vesc_send_buffer(controller_id, buffer, 1, 2);
}

void vesc_detect_motor_param(uint8_t controller_id, float current, float min_rpm, float low_duty) {
    uint8_t buffer[13];
    int32_t index = 0;
    buffer[index++] = COMM_DETECT_MOTOR_PARAM;
    vesc_buffer_append_float32(buffer, current, 1e3f, &index);
    vesc_buffer_append_float32(buffer, min_rpm, 1e3f, &index);
    vesc_buffer_append_float32(buffer, low_duty, 1e3f, &index);
    vesc_send_buffer(controller_id, buffer, index, 2);
}

void vesc_detect_motor_flux_linkage(uint8_t controller_id, float current, float min_rpm, float duty, float resistance) {
    uint8_t buffer[17];
    int32_t index = 0;
    buffer[index++] = COMM_DETECT_MOTOR_FLUX_LINKAGE;
    vesc_buffer_append_float32(buffer, current, 1e3f, &index);
    vesc_buffer_append_float32(buffer, min_rpm, 1e3f, &index);
    vesc_buffer_append_float32(buffer, duty, 1e3f, &index);
    vesc_buffer_append_float32(buffer, resistance, 1e6f, &index);
    vesc_send_buffer(controller_id, buffer, index, 2);
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

void vesc_set_chuck_data(uint8_t controller_id, vesc_chuck_data_t *data) {
    uint8_t buffer[12];
    int32_t index = 0;
    buffer[index++] = data->js_x;
    buffer[index++] = data->js_y;
    buffer[index++] = data->bt_c;
    buffer[index++] = data->bt_z;
    vesc_buffer_append_int16(buffer, data->acc_x, &index);
    vesc_buffer_append_int16(buffer, data->acc_y, &index);
    vesc_buffer_append_int16(buffer, data->acc_z, &index);
    buffer[index++] = data->rev_has_state ? 1 : 0;
    buffer[index++] = data->is_rev ? 1 : 0;
    
    uint32_t can_id = controller_id | ((uint32_t)CAN_PACKET_SET_CHUCK_DATA << 8);
    vesc_can_send_packet(can_id, buffer, index);
}

// ============================================================================
// Status Functions
// ============================================================================

void vesc_get_values(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_VALUES;
    vesc_send_buffer(controller_id, buffer, 1, 2);
}

void vesc_get_decoded_adc(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_DECODED_ADC;
    vesc_send_buffer(controller_id, buffer, 1, 2);
}

void vesc_get_decoded_ppm(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_GET_DECODED_PPM;
    vesc_send_buffer(controller_id, buffer, 1, 2);
}

void vesc_get_fw_version(uint8_t controller_id) {
    uint8_t buffer[1];
    buffer[0] = COMM_FW_VERSION;
    vesc_send_buffer(controller_id, buffer, 1, 2);
}

// ============================================================================
// Response Parsing Functions
// ============================================================================

bool vesc_parse_get_values(uint8_t *data, uint8_t len, vesc_values_t *values) {
    if (!data || !values || len < 50) {
        return false;
    }
    
    int32_t index = 0;
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
    
    int32_t index = 0;
    response->resistance = vesc_buffer_get_float32(data, 1e6f, &index);
    response->inductance = vesc_buffer_get_float32(data, 1e8f, &index);
    response->ld_lq_diff = vesc_buffer_get_float32(data, 1e8f, &index);
    response->valid = true;
    
    return true;
}

bool vesc_parse_motor_param_response(uint8_t *data, uint8_t len, vesc_motor_param_response_t *response) {
    if (!data || !response || len < 20) {
        return false;
    }
    
    int32_t index = 0;
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
    
    int32_t index = 0;
    response->flux_linkage = vesc_buffer_get_float32(data, 1e7f, &index);
    response->valid = true;
    
    return true;
}

bool vesc_parse_adc_values(uint8_t *data, uint8_t len, vesc_adc_values_t *values) {
    if (!data || !values || len < 8) {
        return false;
    }
    
    int32_t index = 0;
    values->adc1 = vesc_buffer_get_float16(data, 1e3f, &index);
    values->adc2 = vesc_buffer_get_float16(data, 1e3f, &index);
    values->adc3 = vesc_buffer_get_float16(data, 1e3f, &index);
    values->v_in = vesc_buffer_get_float16(data, 1e1f, &index);
    values->valid = true;
    
    return true;
}

bool vesc_parse_ppm_values(uint8_t *data, uint8_t len, vesc_ppm_values_t *values) {
    if (!data || !values || len < 4) {
        return false;
    }
    
    int32_t index = 0;
    values->ppm = vesc_buffer_get_float16(data, 1e3f, &index);
    values->pulse_len = vesc_buffer_get_float16(data, 1e1f, &index);
    values->valid = true;
    
    return true;
}

bool vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version) {
    if (!data || !version || len < 50) {
        return false;
    }
    
    int32_t index = 0;
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