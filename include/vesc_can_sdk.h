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

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Command Definitions
// ============================================================================

// Motor Control Commands
#define COMM_SET_DUTY              0
#define COMM_SET_CURRENT           1
#define COMM_SET_CURRENT_BRAKE     2
#define COMM_SET_RPM               3
#define COMM_SET_POS               4
#define COMM_SET_HANDBRAKE         5

// Motor Detection Commands
#define COMM_DETECT_MOTOR_R_L      25
#define COMM_DETECT_MOTOR_PARAM    26
#define COMM_DETECT_MOTOR_FLUX_LINKAGE 27

// Configuration Commands
#define COMM_CAN_UPDATE_BAUD_ALL   158
#define COMM_SET_CHUCK_DATA        45

// Status Commands
#define COMM_GET_VALUES            27
#define COMM_GET_DECODED_ADC       30
#define COMM_GET_DECODED_PPM       31
#define COMM_FW_VERSION            0

// CAN Packet Types
#define CAN_PACKET_SET_DUTY        0
#define CAN_PACKET_SET_CURRENT     1
#define CAN_PACKET_SET_CURRENT_BRAKE 2
#define CAN_PACKET_SET_RPM         3
#define CAN_PACKET_SET_POS         4
#define CAN_PACKET_SET_HANDBRAKE   5
#define CAN_PACKET_SET_SERVO_POS   5
#define CAN_PACKET_SET_CHUCK_DATA  6
#define CAN_PACKET_SET_CURRENT_NOREV 7
#define CAN_PACKET_SET_CURRENT_NOREV_BRAKE 8
#define CAN_PACKET_SET_CURRENT_HANDBRAKE 9
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_NOREV 10
#define CAN_PACKET_SET_CURRENT_FWD 11
#define CAN_PACKET_SET_CURRENT_REV 12
#define CAN_PACKET_SET_CURRENT_BRAKE_FWD 13
#define CAN_PACKET_SET_CURRENT_BRAKE_REV 14
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_FWD 15
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REV 16
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_FWD_NOREV 17
#define CAN_PACKET_SET_CURRENT_HANDBRAKE_REV_NOREV 18
#define CAN_PACKET_FILL_RX_BUFFER  5
#define CAN_PACKET_FILL_RX_BUFFER_LONG 6
#define CAN_PACKET_PROCESS_RX_BUFFER 7
#define CAN_PACKET_PROCESS_SHORT_BUFFER 8
#define CAN_PACKET_PING            28
#define CAN_PACKET_PONG            29
#define CAN_PACKET_DETECT_APPLY_ALL_FOC 30
#define CAN_PACKET_CONF_CURRENT_LIMITS 31
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS 32
#define CAN_PACKET_CONF_CURRENT_LIMITS_IN 33
#define CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN 34
#define CAN_PACKET_CONF_FOC_ERPMS 35
#define CAN_PACKET_CONF_STORE_FOC_ERPMS 36
#define CAN_PACKET_CONF_BATTERY_CUT 37
#define CAN_PACKET_CONF_STORE_BATTERY_CUT 38
#define CAN_PACKET_SHUTDOWN        39
#define CAN_PACKET_IO_BOARD_ADC_1_TO_4 40
#define CAN_PACKET_IO_BOARD_ADC_5_TO_8 41
#define CAN_PACKET_IO_BOARD_ADC_9_TO_12 42
#define CAN_PACKET_IO_BOARD_DIGITAL_IN 43
#define CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL 44
#define CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM 45
#define CAN_PACKET_IO_BOARD_SET_SERVO 46
#define CAN_PACKET_BMS_GET_VALUES  47
#define CAN_PACKET_BMS_SET_CHARGE_ALLOWED 48
#define CAN_PACKET_BMS_SET_BALANCE_OVERRIDE 49
#define CAN_PACKET_BMS_RESET_COUNTERS 50
#define CAN_PACKET_BMS_FORCE_BALANCE 51
#define CAN_PACKET_PSW_GET_STATUS  52
#define CAN_PACKET_PSW_SWITCH      53
#define CAN_PACKET_UPDATE_PID_POS_OFFSET 54
#define CAN_PACKET_POLL_ROTOR_POS  55
#define CAN_PACKET_BMS_FWD_CAN_RX  56
#define CAN_PACKET_BMS_HW_DATA     57
#define CAN_PACKET_GNSS_LAT        58
#define CAN_PACKET_GNSS_LON        59
#define CAN_PACKET_GNSS_ALT_SPEED_HDOP 60
#define CAN_PACKET_UPDATE_BAUD     61

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
    float inductance;         // Motor inductance (H)
    float ld_lq_diff;         // Ld-Lq difference (H)
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
 * @param controller_id VESC controller ID (0-255) to listen for
 * @return true on success, false on failure
 */
bool vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t controller_id);

/**
 * Set the response callback function
 * 
 * @param callback Pointer to callback function
 */
void vesc_set_response_callback(vesc_response_callback_t callback);

/**
 * Set the controller ID to filter incoming CAN frames
 * 
 * @param controller_id VESC controller ID (0-255) to listen for
 */
void vesc_set_controller_id(uint8_t controller_id);

/**
 * Process received CAN frame
 * 
 * @param id CAN ID
 * @param data CAN data
 * @param len Data length
 */
void vesc_process_can_frame(uint32_t id, uint8_t *data, uint8_t len);

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

/**
 * Set nunchuk/joystick data
 * 
 * @param controller_id VESC controller ID (0-255)
 * @param data Pointer to chuck data structure
 */
void vesc_set_chuck_data(uint8_t controller_id, vesc_chuck_data_t *data);

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
 * Get firmware version
 * 
 * @param controller_id VESC controller ID (0-255)
 */
void vesc_get_fw_version(uint8_t controller_id);

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
 * Parse firmware version response
 * 
 * @param data Response data
 * @param len Data length
 * @param version Pointer to firmware version structure
 * @return true on success, false on failure
 */
bool vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version);

#ifdef __cplusplus
}
#endif

#endif // VESC_CAN_SDK_H 