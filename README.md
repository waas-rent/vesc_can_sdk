# VESC CAN SDK

**Compatible with VESC firmware version 6.06**

A pure C SDK for communicating with VESC motor controllers over CAN bus. This SDK provides a simple interface to send commands and receive responses from VESC devices without requiring any special libraries beyond a user-provided CAN send function.

## Features

- **Pure C implementation** - No external dependencies
- **Simple interface** - Just provide a CAN send function
- **Comprehensive command support** - All major VESC commands implemented
- **Response parsing** - Automatic parsing of VESC responses
- **Python integration** - Python wrapper and monitoring tools included

## Supported Commands

### Motor Control Commands
- `COMM_SET_DUTY` - Set motor duty cycle (-1.0 to 1.0)
- `COMM_SET_CURRENT` - Set motor current in Amperes
- `COMM_SET_CURRENT_BRAKE` - Set braking current in Amperes
- `COMM_SET_RPM` - Set motor RPM
- `COMM_SET_HANDBRAKE` - Set handbrake current

### Motor Detection Commands
- `COMM_DETECT_MOTOR_R_L` - Detect motor resistance and inductance
- `COMM_DETECT_MOTOR_PARAM` - Detect motor parameters (BLDC)
- `COMM_DETECT_MOTOR_FLUX_LINKAGE` - Detect motor flux linkage

### Configuration Commands
- `COMM_CAN_UPDATE_BAUD_ALL` - Update CAN baud rate on all devices
- `COMM_SET_CHUCK_DATA` - Set nunchuk/joystick data

### Status Commands
- `COMM_GET_VALUES` - Get motor status values
- `COMM_GET_DECODED_ADC` - Get decoded ADC values
- `COMM_GET_DECODED_PPM` - Get decoded PPM values
- `COMM_FW_VERSION` - Get firmware version

## Quick Start

### 1. Include the SDK

```c
#include "vesc_can_sdk.h"
```

### 2. Implement CAN Send Function

```c
bool my_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    // Your CAN send implementation here
    // Return true on success, false on failure
    return true;
}
```

### 3. Initialize the SDK

```c
vesc_can_init(my_can_send);
```

### 4. Send Commands

```c
// Set motor to 50% duty cycle
vesc_set_duty(1, 0.5f);

// Get motor status
vesc_get_values(1);
```

### 5. Handle Responses

```c
void handle_vesc_response(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    switch (command) {
        case COMM_GET_VALUES:
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("RPM: %.1f, Current: %.2fA, Voltage: %.1fV\n", 
                       values.rpm, values.current_motor, values.v_in);
            }
            break;
    }
}

// Register the callback
vesc_set_response_callback(handle_vesc_response);
```

## Command Reference

### Motor Control

#### `vesc_set_duty(uint8_t controller_id, float duty)`
Set motor duty cycle.
- `controller_id`: VESC controller ID (0-255)
- `duty`: Duty cycle (-1.0 to 1.0)

#### `vesc_set_current(uint8_t controller_id, float current)`
Set motor current.
- `controller_id`: VESC controller ID (0-255)
- `current`: Current in Amperes

#### `vesc_set_current_brake(uint8_t controller_id, float current)`
Set braking current.
- `controller_id`: VESC controller ID (0-255)
- `current`: Braking current in Amperes

#### `vesc_set_rpm(uint8_t controller_id, float rpm)`
Set motor RPM.
- `controller_id`: VESC controller ID (0-255)
- `rpm`: Target RPM

#### `vesc_set_handbrake(uint8_t controller_id, float current)`
Set handbrake current.
- `controller_id`: VESC controller ID (0-255)
- `current`: Handbrake current in Amperes

### Motor Detection

#### `vesc_detect_motor_r_l(uint8_t controller_id)`
Detect motor resistance and inductance.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_motor_rl_response_t` structure

#### `vesc_detect_motor_param(uint8_t controller_id, float current, float min_rpm, float low_duty)`
Detect motor parameters for BLDC motors.
- `controller_id`: VESC controller ID (0-255)
- `current`: Detection current in Amperes
- `min_rpm`: Minimum RPM for detection
- `low_duty`: Low duty cycle for detection
- **Response**: `vesc_motor_param_response_t` structure

#### `vesc_detect_motor_flux_linkage(uint8_t controller_id, float current, float min_rpm, float duty, float resistance)`
Detect motor flux linkage.
- `controller_id`: VESC controller ID (0-255)
- `current`: Detection current in Amperes
- `min_rpm`: Minimum RPM for detection
- `duty`: Duty cycle for detection
- `resistance`: Motor resistance in Ohms
- **Response**: `vesc_flux_linkage_response_t` structure

### Configuration

#### `vesc_can_update_baud_all(uint16_t kbits, uint16_t delay_msec)`
Update CAN baud rate on all devices.
- `kbits`: Baud rate in kbits/s (125, 250, 500, 1000)
- `delay_msec`: Delay before applying new baud rate

#### `vesc_set_chuck_data(uint8_t controller_id, vesc_chuck_data_t *data)`
Set nunchuk/joystick data.
- `controller_id`: VESC controller ID (0-255)
- `data`: Pointer to chuck data structure

### Status Commands

#### `vesc_get_values(uint8_t controller_id)`
Get motor status values.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_values_t` structure

#### `vesc_get_decoded_adc(uint8_t controller_id)`
Get decoded ADC values.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_adc_values_t` structure

#### `vesc_get_decoded_ppm(uint8_t controller_id)`
Get decoded PPM values.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_ppm_values_t` structure

#### `vesc_get_fw_version(uint8_t controller_id)`
Get firmware version.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_fw_version_t` structure

## Response Structures

### `vesc_values_t`
```c
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
```

### `vesc_motor_rl_response_t`
```c
typedef struct {
    float resistance;         // Motor resistance (Ω)
    float inductance;         // Motor inductance (H)
    float ld_lq_diff;         // Ld-Lq difference (H)
    bool valid;               // Response validity
} vesc_motor_rl_response_t;
```

### `vesc_motor_param_response_t`
```c
typedef struct {
    float cycle_int_limit;    // Cycle integrator limit
    float coupling_k;         // BEMF coupling constant
    int8_t hall_table[8];     // Hall sensor table
    int hall_res;             // Hall sensor result
    bool valid;               // Response validity
} vesc_motor_param_response_t;
```

### `vesc_flux_linkage_response_t`
```c
typedef struct {
    float flux_linkage;       // Flux linkage (Wb)
    bool valid;               // Response validity
} vesc_flux_linkage_response_t;
```

### `vesc_adc_values_t`
```c
typedef struct {
    float adc1;               // ADC1 value (0.0-1.0)
    float adc2;               // ADC2 value (0.0-1.0)
    float adc3;               // ADC3 value (0.0-1.0)
    float v_in;               // Input voltage (V)
    bool valid;               // Response validity
} vesc_adc_values_t;
```

### `vesc_ppm_values_t`
```c
typedef struct {
    float ppm;                // PPM value (0.0-1.0)
    float pulse_len;          // Pulse length (μs)
    bool valid;               // Response validity
} vesc_ppm_values_t;
```

### `vesc_fw_version_t`
```c
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
```

### `vesc_chuck_data_t`
```c
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
```

## Status Message Structures

The SDK provides structures and parsing functions for VESC CAN status messages. These messages are automatically sent by VESC controllers when status reporting is enabled.

### `vesc_status_msg_1_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    float rpm;                // Motor RPM
    float current;            // Motor current (A)
    float duty;               // Duty cycle (0.0-1.0)
    bool valid;               // Response validity
} vesc_status_msg_1_t;
```

### `vesc_status_msg_2_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    float amp_hours;          // Consumed amp hours (Ah)
    float amp_hours_charged;  // Charged amp hours (Ah)
    bool valid;               // Response validity
} vesc_status_msg_2_t;
```

### `vesc_status_msg_3_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    float watt_hours;         // Consumed watt hours (Wh)
    float watt_hours_charged; // Charged watt hours (Wh)
    bool valid;               // Response validity
} vesc_status_msg_3_t;
```

### `vesc_status_msg_4_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    float temp_fet;           // FET temperature (°C)
    float temp_motor;         // Motor temperature (°C)
    float current_in;         // Input current (A)
    float pid_pos_now;        // Current PID position
    bool valid;               // Response validity
} vesc_status_msg_4_t;
```

### `vesc_status_msg_5_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    int32_t tacho_value;      // Tachometer value
    float v_in;               // Input voltage (V)
    bool valid;               // Response validity
} vesc_status_msg_5_t;
```

### `vesc_status_msg_6_t`
```c
typedef struct {
    uint8_t controller_id;    // Controller ID
    float adc_1;              // ADC1 value (0.0-1.0)
    float adc_2;              // ADC2 value (0.0-1.0)
    float adc_3;              // ADC3 value (0.0-1.0)
    float ppm;                // PPM value (0.0-1.0)
    bool valid;               // Response validity
} vesc_status_msg_6_t;
```

## Python Integration

The SDK includes Python bindings and monitoring tools:

```python
import vesc_can_sdk

# Initialize SDK
vesc = vesc_can_sdk.VESCCAN()

# Set motor duty cycle
vesc.set_duty(1, 0.5)

# Get motor values
values = vesc.get_values(1)
print(f"RPM: {values.rpm}, Current: {values.current_motor}A")
```

### Python Monitoring Script

Use the included `monitor_vesc.py` script to monitor VESC status:

```bash
python monitor_vesc.py --can-interface can0 --controller-id 1
```

## Building

### C SDK
```bash
cd vesc_can_sdk
make
```

### Python Bindings
```bash
cd vesc_can_sdk/python
python setup.py build
python setup.py install
```

## Examples

See the `examples/` directory for complete working examples:

- `basic_control.c` - Basic motor control
- `motor_detection.c` - Motor parameter detection
- `status_monitoring.c` - Status monitoring
- `python_example.py` - Python usage example

## Usage Examples

### Basic Motor Control
```c
#include "vesc_can_sdk.h"

// Initialize the SDK
vesc_can_init(my_can_send_function, 1);

// Set response callback
vesc_set_response_callback(my_response_callback);

// Control motor
vesc_set_duty(1, 0.5f);      // 50% duty cycle
vesc_set_current(1, 10.0f);  // 10A current
vesc_set_rpm(1, 1000.0f);    // 1000 RPM
```

### Status Message Handling
```c
void response_callback(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    switch (command) {
        case CAN_PACKET_STATUS: {
            vesc_status_msg_1_t status;
            if (vesc_parse_status_msg_1(data, len, &status)) {
                printf("VESC %d: RPM=%.0f, Current=%.2fA, Duty=%.1f%%\n",
                       status.controller_id, status.rpm, status.current, status.duty * 100.0f);
            }
        } break;
        
        case CAN_PACKET_STATUS_4: {
            vesc_status_msg_4_t status;
            if (vesc_parse_status_msg_4(data, len, &status)) {
                printf("VESC %d: FET Temp=%.1f°C, Motor Temp=%.1f°C, Input Current=%.2fA\n",
                       status.controller_id, status.temp_fet, status.temp_motor, status.current_in);
            }
        } break;
    }
}
```

## License

This SDK is released under the MIT License. See LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Internal Knowledge

### Packets vs Commands

In the context of VESC communication, there is a distinction between sending packets and commands. Packets are the raw data structures transmitted over the CAN bus, while commands are higher-level abstractions that represent specific actions or queries. This SDK supports both packets and commands seamlessly, abstracting away the differences so users can focus on their application logic without worrying about the underlying details.

## Status Message Parsing Functions

These functions parse VESC CAN status messages that are automatically sent by VESC controllers.

#### `vesc_parse_status_msg_1(uint8_t *data, uint8_t len, vesc_status_msg_1_t *status)`
Parse CAN_PACKET_STATUS (Status Message 1) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 1 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: RPM, current, duty cycle

#### `vesc_parse_status_msg_2(uint8_t *data, uint8_t len, vesc_status_msg_2_t *status)`
Parse CAN_PACKET_STATUS_2 (Status Message 2) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 2 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: Amp hours consumed and charged

#### `vesc_parse_status_msg_3(uint8_t *data, uint8_t len, vesc_status_msg_3_t *status)`
Parse CAN_PACKET_STATUS_3 (Status Message 3) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 3 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: Watt hours consumed and charged

#### `vesc_parse_status_msg_4(uint8_t *data, uint8_t len, vesc_status_msg_4_t *status)`
Parse CAN_PACKET_STATUS_4 (Status Message 4) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 4 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: Temperatures, input current, PID position

#### `vesc_parse_status_msg_5(uint8_t *data, uint8_t len, vesc_status_msg_5_t *status)`
Parse CAN_PACKET_STATUS_5 (Status Message 5) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 5 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: Tachometer value, input voltage

#### `vesc_parse_status_msg_6(uint8_t *data, uint8_t len, vesc_status_msg_6_t *status)`
Parse CAN_PACKET_STATUS_6 (Status Message 6) response.
- `data`: Response data
- `len`: Data length
- `status`: Pointer to status message 6 structure
- **Returns**: `true` on success, `false` on failure
- **Data**: ADC values, PPM value

#### `vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version)`
Parse firmware version response.
- `data`: Response data
- `len`: Data length
- `version`: Pointer to firmware version structure
- **Returns**: `true` on success, `false` on failure