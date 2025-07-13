/*
 * VESC CAN SDK
 * 
 * A pure C SDK for communicating with VESC motor controllers over CAN bus.
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

#ifndef VESC_CAN_SDK_H
#define VESC_CAN_SDK_H

#include <stdint.h>
#include <stdbool.h>
#include "vesc_version.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Debug Configuration
// ============================================================================

// Debug levels
#define VESC_DEBUG_NONE        0
#define VESC_DEBUG_BASIC       1
#define VESC_DEBUG_DETAILED    2
#define VESC_DEBUG_VERBOSE     3

// Debug categories (bit flags)
#define VESC_DEBUG_CAN         0x0001  // CAN communication
#define VESC_DEBUG_COMMANDS    0x0002  // Command sending
#define VESC_DEBUG_RESPONSES   0x0004  // Response parsing
#define VESC_DEBUG_BUFFERS     0x0008  // Buffer management
#define VESC_DEBUG_ERRORS      0x0010  // Error conditions
#define VESC_DEBUG_PERFORMANCE 0x0020  // Performance metrics
#define VESC_DEBUG_ALL         0x003F  // All categories

// Debug output function type
typedef void (*vesc_debug_output_func_t)(const char *message);

// Debug configuration structure
typedef struct {
    uint8_t level;                    // Debug level (0-3)
    uint16_t categories;              // Enabled categories (bit flags)
    vesc_debug_output_func_t output_func; // Custom output function (NULL = printf)
    bool enable_timestamps;           // Include timestamps in output
    bool enable_statistics;           // Collect debug statistics
} vesc_debug_config_t;

// Debug statistics structure
typedef struct {
    uint32_t can_tx_count;           // CAN transmit count
    uint32_t can_rx_count;           // CAN receive count
    uint32_t command_count;          // Commands sent
    uint32_t response_count;         // Responses received
    uint32_t error_count;            // Error count
    uint32_t crc_error_count;        // CRC error count
    uint32_t buffer_overflow_count;  // Buffer overflow count
    uint64_t total_tx_bytes;         // Total bytes transmitted
    uint64_t total_rx_bytes;         // Total bytes received
} vesc_debug_stats_t;

// ============================================================================
// Command Definitions
// ============================================================================

#define COMM_FW_VERSION            0
#define COMM_GET_VALUES            4
#define COMM_GET_MCC_CONFIG        14
#define COMM_REBOOT                29
#define COMM_GET_VALUES_SETUP      47
#define COMM_DETECT_MOTOR_R_L      25
#define COMM_DETECT_MOTOR_PARAM    24
#define COMM_DETECT_MOTOR_FLUX_LINKAGE 26
#define COMM_GET_DECODED_ADC       32
#define COMM_GET_DECODED_PPM       31
#define COMM_GET_DECODED_CHUK      33
#define COMM_FORWARD_CAN           34
#define COMM_SET_CHUCK_DATA        35
#define COMM_CAN_UPDATE_BAUD_ALL   158

// ============================================================================
// Packet Definitions
// ============================================================================

// Packets
#define CAN_PACKET_SET_DUTY                        0
#define CAN_PACKET_SET_CURRENT                     1
#define CAN_PACKET_SET_CURRENT_BRAKE               2
#define CAN_PACKET_SET_RPM                         3
#define CAN_PACKET_SET_POS                         4
#define CAN_PACKET_FILL_RX_BUFFER                  5
#define CAN_PACKET_FILL_RX_BUFFER_LONG             6
#define CAN_PACKET_PROCESS_RX_BUFFER               7
#define CAN_PACKET_PROCESS_SHORT_BUFFER            8
#define CAN_PACKET_STATUS                          9
#define CAN_PACKET_SET_CURRENT_REL                 10
#define CAN_PACKET_SET_CURRENT_BRAKE_REL           11
#define CAN_PACKET_SET_CURRENT_HANDBRAKE           12
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REL       13
#define CAN_PACKET_STATUS_2                        14
#define CAN_PACKET_STATUS_3                        15
#define CAN_PACKET_STATUS_4                        16
#define CAN_PACKET_PING                            17
#define CAN_PACKET_PONG                            18
#define CAN_PACKET_DETECT_APPLY_ALL_FOC            19
#define CAN_PACKET_DETECT_APPLY_ALL_FOC_RES        20
#define CAN_PACKET_CONF_CURRENT_LIMITS             21
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS       22
#define CAN_PACKET_CONF_CURRENT_LIMITS_IN          23
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN    24
#define CAN_PACKET_CONF_FOC_ERPMS                  25
#define CAN_PACKET_CONF_STORE_FOC_ERPMS            26
#define CAN_PACKET_STATUS_5                        27
#define CAN_PACKET_POLL_TS5700N8501_STATUS         28
#define CAN_PACKET_CONF_BATTERY_CUT                29
#define CAN_PACKET_CONF_STORE_BATTERY_CUT          30
#define CAN_PACKET_SHUTDOWN                        31
#define CAN_PACKET_IO_BOARD_ADC_1_TO_4             32
#define CAN_PACKET_IO_BOARD_ADC_5_TO_8             33
#define CAN_PACKET_IO_BOARD_ADC_9_TO_12            34
#define CAN_PACKET_IO_BOARD_DIGITAL_IN             35
#define CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL     36
#define CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM         37
#define CAN_PACKET_BMS_V_TOT                       38
#define CAN_PACKET_BMS_I                           39
#define CAN_PACKET_BMS_AH_WH                       40
#define CAN_PACKET_BMS_V_CELL                      41
#define CAN_PACKET_BMS_BAL                         42
#define CAN_PACKET_BMS_TEMPS                       43
#define CAN_PACKET_BMS_HUM                         44
#define CAN_PACKET_BMS_SOC_SOH_TEMP_STAT           45
#define CAN_PACKET_PSW_STAT                        46
#define CAN_PACKET_PSW_SWITCH                      47
#define CAN_PACKET_BMS_HW_DATA_1                   48
#define CAN_PACKET_BMS_HW_DATA_2                   49
#define CAN_PACKET_BMS_HW_DATA_3                   50
#define CAN_PACKET_BMS_HW_DATA_4                   51
#define CAN_PACKET_BMS_HW_DATA_5                   52
#define CAN_PACKET_BMS_AH_WH_CHG_TOTAL             53
#define CAN_PACKET_BMS_AH_WH_DIS_TOTAL             54
#define CAN_PACKET_UPDATE_PID_POS_OFFSET           55
#define CAN_PACKET_POLL_ROTOR_POS                  56
#define CAN_PACKET_NOTIFY_BOOT                     57
#define CAN_PACKET_STATUS_6                        58
#define CAN_PACKET_GNSS_TIME                       59
#define CAN_PACKET_GNSS_LAT                        60
#define CAN_PACKET_GNSS_LON                        61
#define CAN_PACKET_GNSS_ALT_SPEED_HDOP             62
#define CAN_PACKET_UPDATE_BAUD                     63
#define CAN_PACKET_MAKE_ENUM_32_BITS               0xFFFFFFFF,
// ============================================================================
// Response Structures
// ============================================================================

typedef struct {
    float temp_fet;           // FET temperature (°C)
    float temp_motor;         // Motor temperature (°C)
    float current_motor;      // Motor current (A)
    float current_in;         // Input current (A)
    float current_id;         // D-axis current (A)
    float current_iq;         // Q-axis current (A)
    float duty_cycle;         // Duty cycle (0.0-1.0)
    float rpm;                // Motor RPM
    float v_in;               // Input voltage (V)
    float amp_hours;          // Consumed amp hours (Ah)
    float amp_hours_charged;  // Charged amp hours (Ah)
    float watt_hours;         // Consumed watt hours (Wh)
    float watt_hours_charged; // Charged watt hours (Wh)
    int32_t tachometer;       // Tachometer value
    int32_t tachometer_abs;   // Absolute tachometer value
    uint8_t fault_code;       // Fault code
    float pid_pos;            // PID position
    uint8_t controller_id;    // Controller ID
    float temp_mos1;          // MOSFET 1 temperature (°C)
    float temp_mos2;          // MOSFET 2 temperature (°C)
    float temp_mos3;          // MOSFET 3 temperature (°C)
    float vd;                 // D-axis voltage (V)
    float vq;                 // Q-axis voltage (V)
    uint8_t status;           // Status flags
} vesc_values_t;

typedef struct {
    float resistance;         // Motor resistance (Ω)
    float inductance;         // Motor inductance (microhenryH)
    float ld_lq_diff;         // Ld-Lq difference (microhenryH)
    bool valid;               // Response validity
} vesc_motor_rl_response_t;

typedef struct {
    float cycle_int_limit;    // Cycle integrator limit
    float coupling_k;         // BEMF coupling constant
    int8_t hall_table[8];     // Hall sensor table
    int hall_res;             // Hall sensor result
    bool valid;               // Response validity
} vesc_motor_param_response_t;

typedef struct {
    float flux_linkage;       // Flux linkage (Wb)
    bool valid;               // Response validity
} vesc_flux_linkage_response_t;

typedef struct {
    float adc1;               // ADC1 value (0.0-1.0)
    float adc2;               // ADC2 value (0.0-1.0)
    float adc3;               // ADC3 value (0.0-1.0)
    float v_in;               // Input voltage (V)
    bool valid;               // Response validity
} vesc_adc_values_t;

typedef struct {
    float ppm;                // PPM value (0.0-1.0)
    float pulse_len;          // Pulse length (μs)
    bool valid;               // Response validity
} vesc_ppm_values_t;

typedef struct {
    float js_y;               // Joystick Y value (0.0-1.0)
    bool valid;               // Response validity
} vesc_chuck_values_t;

typedef struct {
    uint8_t major;            // Major version
    uint8_t minor;            // Minor version
    char hw_name[32];         // Hardware name
    uint8_t uuid[12];         // Device UUID
    bool pairing_done;        // Pairing status
    uint8_t test_version;     // Test version
    uint8_t hw_type;          // Hardware type
    uint8_t cfg_num;          // Configuration number
    bool valid;               // Response validity
} vesc_fw_version_t;

typedef struct {
    uint8_t js_x;             // Joystick X (0-255)
    uint8_t js_y;             // Joystick Y (0-255)
    uint8_t bt_c;             // Button C (0/1)
    uint8_t bt_z;             // Button Z (0/1)
    int16_t acc_x;            // Accelerometer X
    int16_t acc_y;            // Accelerometer Y
    int16_t acc_z;            // Accelerometer Z
    bool rev_has_state;       // Reverse has state
    bool is_rev;              // Is reverse
} vesc_chuck_data_t;

// ============================================================================
// Status Message Response Structures
// ============================================================================

typedef struct {
    uint8_t controller_id;    // Controller ID
    float rpm;                // Motor RPM
    float current;            // Motor current (A)
    float duty;               // Duty cycle (0.0-1.0)
    bool valid;               // Response validity
} vesc_status_msg_1_t;

typedef struct {
    uint8_t controller_id;    // Controller ID
    float amp_hours;          // Consumed amp hours (Ah)
    float amp_hours_charged;  // Charged amp hours (Ah)
    bool valid;               // Response validity
} vesc_status_msg_2_t;

typedef struct {
    uint8_t controller_id;    // Controller ID
    float watt_hours;         // Consumed watt hours (Wh)
    float watt_hours_charged; // Charged watt hours (Wh)
    bool valid;               // Response validity
} vesc_status_msg_3_t;

typedef struct {
    uint8_t controller_id;    // Controller ID
    float temp_fet;           // FET temperature (°C)
    float temp_motor;         // Motor temperature (°C)
    float current_in;         // Input current (A)
    float pid_pos_now;        // Current PID position
    bool valid;               // Response validity
} vesc_status_msg_4_t;

typedef struct {
    uint8_t controller_id;    // Controller ID
    int32_t tacho_value;      // Tachometer value
    float v_in;               // Input voltage (V)
    bool valid;               // Response validity
} vesc_status_msg_5_t;

typedef struct {
    uint8_t controller_id;    // Controller ID
    float adc_1;              // ADC1 value (0.0-1.0)
    float adc_2;              // ADC2 value (0.0-1.0)
    float adc_3;              // ADC3 value (0.0-1.0)
    float ppm;                // PPM value (0.0-1.0)
    bool valid;               // Response validity
} vesc_status_msg_6_t;

typedef struct {
    uint8_t controller_id;    // Controller ID from PONG response
    bool valid;               // Response validity
} vesc_pong_response_t;

// ============================================================================
// Function Types
// ============================================================================

// CAN send function type - user must implement this
typedef bool (*vesc_can_send_func_t)(uint32_t id, uint8_t *data, uint8_t len);

// Response callback function type
typedef void (*vesc_response_callback_t)(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len);

// ============================================================================
// Core SDK Functions
// ============================================================================

/**
 * Initialize the VESC CAN SDK
 * 
 * @param can_send_func Pointer to user's CAN send function
 * @param receiver_controller_id Receiver controller ID (0-255) to listen for
 * @return true on success, false on failure
 */
bool vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t receiver_controller_id, uint8_t sender_id);

/**
 * Set the response callback function
 * 
 * @param callback Pointer to callback function
 */
void vesc_set_response_callback(vesc_response_callback_t callback);

/**
 * Set the receiver controller ID to filter incoming CAN frames
 * 
 * @param receiver_controller_id Receiver controller ID (0-255) to listen for
 */
void vesc_set_controller_id(uint8_t receiver_controller_id);

/**
 * Set the sender's controller ID for buffer protocol commands
 * 
 * This function sets the controller ID that will be used as the sender ID
 * in buffer protocol commands (first byte of process command).
 * 
 * @param sender_controller_id The controller ID to use as sender ID (typically 42)
 */
void vesc_set_sender_controller_id(uint8_t sender_controller_id);

/**
 * Get the current sender's controller ID
 * 
 * @return The current sender controller ID
 */
uint8_t vesc_get_sender_controller_id(void);

/**
 * Process received CAN frame
 * 
 * @param id CAN ID
 * @param data CAN data
 * @param len Data length
 */
void vesc_process_can_frame(uint32_t id, uint8_t *data, uint8_t len);

/**
 * This function returns the most recent motor R/L response.
 * If the response is not valid, it returns an empty response structure.
 * The response is valid if the motor R/L detection has been performed.
 */
vesc_motor_rl_response_t vesc_get_latest_motor_rl_response();

/**
 * This function returns the most recent motor parameter response.
 * If the response is not valid, it returns an empty response structure.
 * The response is valid if the motor parameter detection has been performed.
 */
vesc_flux_linkage_response_t vesc_get_latest_flux_linkage_response();


// ============================================================================
// Motor Control Functions
// ============================================================================

/**
 * Set motor duty cycle
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param duty Duty cycle (-1.0 to 1.0)
 */
void vesc_set_duty(uint8_t controller_id, float duty);

/**
 * Set motor current
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param current Current in Amperes
 */
void vesc_set_current(uint8_t controller_id, float current);

/**
 * Set braking current
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param current Braking current in Amperes
 */
void vesc_set_current_brake(uint8_t controller_id, float current);

/**
 * Set motor RPM
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param rpm Target RPM
 */
void vesc_set_rpm(uint8_t controller_id, float rpm);

/**
 * Set handbrake current
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param current Handbrake current in Amperes
 */
void vesc_set_handbrake(uint8_t controller_id, float current);

// ============================================================================
// Motor Detection Functions
// ============================================================================

/**
 * Detect motor resistance and inductance. This will make the motor make weird noises but these are expected.
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_detect_motor_r_l(uint8_t controller_id);

/**
 * Detect motor parameters for BLDC motors
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param current Detection current in Amperes
 * @param min_rpm Minimum RPM for detection
 * @param low_duty Low duty cycle for detection
 */
void vesc_detect_motor_param(uint8_t controller_id, float current, float min_rpm, float low_duty);

/**
 * Detect motor flux linkage. ATTENTION: This command will move the motor!
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param current Detection current in Amperes
 * @param min_rpm Minimum RPM for detection
 * @param duty Duty cycle for detection
 * @param resistance Motor resistance in Ohms
 */
void vesc_detect_motor_flux_linkage(uint8_t controller_id, float current, float min_rpm, float duty, float resistance);

// ============================================================================
// Configuration Functions
// ============================================================================

/**
 * Update CAN baud rate on all devices
 * 
 * @param kbits Baud rate in kbits/s (125, 250, 500, 1000)
 * @param delay_msec Delay before applying new baud rate
 */
void vesc_can_update_baud_all(uint16_t kbits, uint16_t delay_msec);

// ============================================================================
// Status Functions
// ============================================================================

/**
 * Get motor status values
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_values(uint8_t controller_id);

/**
 * Get MCC config
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_mcc_config(uint8_t controller_id);

/**
 * Reboot VESC controller
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_reboot(uint8_t controller_id);

/**
 * Get decoded ADC values
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_decoded_adc(uint8_t controller_id);

/**
 * Get decoded PPM values
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_decoded_ppm(uint8_t controller_id);

/**
 * Get decoded chuck data
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_decoded_chuck(uint8_t controller_id);

/**
 * Set chuck data
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param chuck_data Pointer to chuck data structure
 */
void vesc_set_chuck_data(uint8_t controller_id, const vesc_chuck_data_t *chuck_data);

/**
 * Get firmware version
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_fw_version(uint8_t controller_id);

/**
 * Send ping to VESC controller
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_ping(uint8_t controller_id);

// ============================================================================
// Response Parsing Functions
// ============================================================================

/**
 * Parse GET_VALUES response
 * 
 * @param data Response data
 * @param len Data length
 * @param values Pointer to values structure
 * @return true on success, false on failure
 */
bool vesc_parse_get_values(uint8_t *data, uint8_t len, vesc_values_t *values);

/**
 * Parse motor R/L detection response
 * 
 * @param data Response data
 * @param len Data length
 * @param response Pointer to response structure
 * @return true on success, false on failure
 */
bool vesc_parse_motor_rl_response(uint8_t *data, uint8_t len, vesc_motor_rl_response_t *response);

/**
 * Parse motor parameter detection response
 * 
 * @param data Response data
 * @param len Data length
 * @param response Pointer to response structure
 * @return true on success, false on failure
 */
bool vesc_parse_motor_param_response(uint8_t *data, uint8_t len, vesc_motor_param_response_t *response);

/**
 * Parse flux linkage detection response
 * 
 * @param data Response data
 * @param len Data length
 * @param response Pointer to response structure
 * @return true on success, false on failure
 */
bool vesc_parse_flux_linkage_response(uint8_t *data, uint8_t len, vesc_flux_linkage_response_t *response);

/**
 * Parse ADC values response
 * 
 * @param data Response data
 * @param len Data length
 * @param values Pointer to ADC values structure
 * @return true on success, false on failure
 */
bool vesc_parse_adc_values(uint8_t *data, uint8_t len, vesc_adc_values_t *values);

/**
 * Parse PPM values response
 * 
 * @param data Response data
 * @param len Data length
 * @param values Pointer to PPM values structure
 * @return true on success, false on failure
 */
bool vesc_parse_ppm_values(uint8_t *data, uint8_t len, vesc_ppm_values_t *values);

/**
 * Parse chuck values response
 * 
 * @param data Response data
 * @param len Data length
 * @param values Pointer to chuck values structure
 * @return true on success, false on failure
 */
bool vesc_parse_chuck_values(uint8_t *data, uint8_t len, vesc_chuck_values_t *values);

/**
 * Parse firmware version response
 * 
 * @param data Response data
 * @param len Data length
 * @param version Pointer to firmware version structure
 * @return true on success, false on failure
 */
bool vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version);

// ============================================================================
// Status Message Parsing Functions
// ============================================================================

/**
 * Parse CAN_PACKET_STATUS (Status Message 1) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 1 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_1(uint8_t *data, uint8_t len, vesc_status_msg_1_t *status);

/**
 * Parse CAN_PACKET_STATUS_2 (Status Message 2) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 2 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_2(uint8_t *data, uint8_t len, vesc_status_msg_2_t *status);

/**
 * Parse CAN_PACKET_STATUS_3 (Status Message 3) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 3 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_3(uint8_t *data, uint8_t len, vesc_status_msg_3_t *status);

/**
 * Parse CAN_PACKET_STATUS_4 (Status Message 4) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 4 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_4(uint8_t *data, uint8_t len, vesc_status_msg_4_t *status);

/**
 * Parse CAN_PACKET_STATUS_5 (Status Message 5) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 5 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_5(uint8_t *data, uint8_t len, vesc_status_msg_5_t *status);

/**
 * Parse CAN_PACKET_STATUS_6 (Status Message 6) response
 * 
 * @param data Response data
 * @param len Data length
 * @param status Pointer to status message 6 structure
 * @return true on success, false on failure
 */
bool vesc_parse_status_msg_6(uint8_t *data, uint8_t len, vesc_status_msg_6_t *status);

/**
 * Parse CAN_PACKET_PONG response
 * 
 * @param data Response data
 * @param len Data length
 * @param pong Pointer to pong response structure
 * @return true on success, false on failure
 */
bool vesc_parse_pong_response(uint8_t *data, uint8_t len, vesc_pong_response_t *pong);

// ============================================================================
// Debug Functions
// ============================================================================

/**
 * Configure debugging
 * 
 * @param config Pointer to debug configuration structure
 * @return true on success, false on failure
 */
bool vesc_debug_configure(vesc_debug_config_t *config);

/**
 * Enable debugging with basic configuration
 * 
 * @param level Debug level (0-3)
 * @param categories Debug categories (bit flags)
 * @return true on success, false on failure
 */
bool vesc_debug_enable(uint8_t level, uint16_t categories);

/**
 * Disable debugging
 */
void vesc_debug_disable(void);

/**
 * Set custom debug output function
 * 
 * @param output_func Custom output function (NULL = use printf)
 */
void vesc_debug_set_output_func(vesc_debug_output_func_t output_func);

/**
 * Get debug statistics
 * 
 * @param stats Pointer to statistics structure
 * @return true on success, false on failure
 */
bool vesc_debug_get_stats(vesc_debug_stats_t *stats);

/**
 * Get current debug configuration
 * 
 * @param config Pointer to debug configuration structure
 * @return true on success, false on failure
 */
bool vesc_debug_get_config(vesc_debug_config_t *config);

/**
 * Reset debug statistics
 */
void vesc_debug_reset_stats(void);

/**
 * Print debug statistics
 */
void vesc_debug_print_stats(void);
void vesc_debug_print_buffer_state(void);

#ifdef __cplusplus
}
#endif

#endif // VESC_CAN_SDK_H