# VESC CAN SDK

A pure C SDK for communicating with VESC controllers over CAN bus. This SDK provides a simple interface to send commands and receive responses from VESC devices without requiring any special libraries beyond a user-provided CAN send function. It also features a Python based shell which uses the SDK to provide interactive access to a VESC controller.

Use cases for this SDK include but are not limited to

- Custom Firmware that wants to control a VESC controller such as VESC EDU, VESC Express, Maxim or Flipsky
- Arduino integration, see README.arduino.md
- Accessing VESC information when not being able to use VESC tool (see Python shell)
- General integration into other software

VESC is a registered trademark of Benjamin Vedder. Read the [trademark policies](https://vesc-project.com/trademark_policies) for more information.

## Compatibility

Compatible with VESC firmware version 6.06 (see https://github.com/vedderb/bldc). It might be compatible with any 6.x but this has not been tested.

## Liability

Using this SDK may result in hardware damage or unsafe operation if used improperly. The authors are not responsible for any damage or injury resulting from use of this SDK. Use at your own risk. The commands that you can send using this SDK can cause harm if you do not know what you are doing.

## Features

- **Pure C implementation** - No external dependencies
- **Simple interface** - Just provide a CAN send function
- **Comprehensive command support** - Major VESC commands and packets implemented
- **Response parsing** - Support for parsing VESC responses such as STATUS_1..6 or other values
- **Shell** - Provides a shell that can be used to interact with a controller
- **Python integration** - Works well with Python 3.x, see python/ directory
- **Zephyr 3.x and 4.x support** - Works seamlessly as a Zephyr module, see README.zephyr.md 

## Supported Commands

### Motor Control Commands
- `vesc_set_duty()` - Set motor duty cycle (-1.0 to 1.0)
- `vesc_set_current()` - Set motor current in Amperes
- `vesc_set_current_brake()` - Set braking current in Amperes
- `vesc_set_rpm()` - Set motor RPM
- `vesc_set_handbrake()` - Set handbrake current

### Motor Detection Commands
- `vesc_detect_motor_r_l()` - Detect motor resistance and inductance. This requires a connected motor. It will produce weird sounds that are ok.
- `vesc_detect_motor_param()` - Detect motor parameters (BLDC). This requires a connected motor. ATTENTION: This will turn the motor. Make sure that it is free from any obstacles.
- `vesc_detect_motor_flux_linkage()` - Detect motor flux linkage. This requires a connected motor.

### Configuration Commands
- `vesc_can_update_baud_all()` - Update CAN baud rate on all VESC devices. This operation might break your setup.

### Status Commands
- `vesc_get_values()` - Get motor status values
- `vesc_get_decoded_adc()` - Get decoded ADC values
- `vesc_get_decoded_ppm()` - Get decoded PPM values
- `vesc_get_decoded_chuck()` - Get decoded chuck (nunchuk) data
- `vesc_get_fw_version()` - Get firmware version
- `vesc_get_mcc_config()` - Get MCC configuration

### Chuck (Nunchuk) Commands
- `vesc_set_chuck_data()` - Set chuck (nunchuk) data

### Status Message Parsing
The SDK automatically handles VESC CAN status messages and provides parsing functions for:
- `vesc_parse_status_msg_1()` - Parse status message 1 (RPM, current, duty)
- `vesc_parse_status_msg_2()` - Parse status message 2 (amp hours)
- `vesc_parse_status_msg_3()` - Parse status message 3 (watt hours)
- `vesc_parse_status_msg_4()` - Parse status message 4 (temperatures, input current)
- `vesc_parse_status_msg_5()` - Parse status message 5 (tachometer, voltage)
- `vesc_parse_status_msg_6()` - Parse status message 6 (ADC values, PPM)

### Debug and Development
- `vesc_debug_configure()` - Configure debug output
- `vesc_debug_enable()` - Enable debug output
- `vesc_debug_disable()` - Disable debug output
- `vesc_debug_set_output_func()` - Set custom debug output function
- `vesc_debug_get_stats()` - Get debug statistics
- `vesc_debug_reset_stats()` - Reset debug statistics
- `vesc_debug_print_stats()` - Print debug statistics
- `vesc_debug_print_buffer_state()` - Print buffer state

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
vesc_can_init(my_can_send, 1, 42);  // Initialize with receiver controller ID 1 and sender ID 42
```

### 4. Set Sender Controller ID (for Buffer Protocol)

```c
// Set the sender's controller ID to 42 for buffer protocol commands
// This ensures proper identification in multi-controller setups
vesc_set_sender_controller_id(42);
```

### 5. Send Commands

```c
// Set motor to 50% duty cycle
vesc_set_duty(1, 0.5f);

// Get motor status
vesc_get_values(1);
```

### 6. Handle Responses

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

### Use Python Interactive Shell

Use the included `vesc_shell.py` script for interactive communication with VESC controllers:

```bash
python vesc_shell.py can0 --id 118 --bustype canalystii --baudrate 1000000
```

The shell provides an interactive command-line interface for:
- Getting motor values (temperature, current, RPM, voltage, etc.)
- Reading ADC and PPM values
- Getting firmware version information
- Detecting motor parameters
- Sending raw CAN messages
- Listening for VESC status messages
- Logging CAN traffic to files

For more information about available commands, run `help` within the shell.

#### Shell Usage Example

Here's a typical session using the VESC shell using Canalyst II CAN logger as input:

```bash
# Start the shell with a VESC on CAN interface can0, controller ID 1
$ python vesc_shell.py can0 --id 118 --bustype canalystii --baudrate 1000000

VESC CAN Shell. Type help or ? to list commands.
vesc> help

Available commands:
  values          - Get motor values (temperature, current, RPM, etc.)
  adc             - Get ADC values
  ppm             - Get PPM values
  chuck           - Get decoded chuck (nunchuk) data
  setchuck        - Set chuck (nunchuk) data
  motor_rl        - Detect motor resistance and inductance
  fw_version      - Get firmware version
  reboot          - Reboot the VESC controller
  raw_can         - Send raw CAN message
  listen          - Listen for VESC status messages (1-6, with optional filtering)
  verbose         - Toggle verbose logging of raw CAN messages
  logging         - Toggle CAN message logging to file
  quit/exit       - Exit the shell

vesc> fw_version
Getting firmware version...
VESC 6.06 VESC_6.6 with UUID: 1234567890abcdef123456

vesc> values
Getting motor values...

Motor Status:
  Temperature FET: 45.2°C
  Temperature Motor: 38.1°C
  Current Motor: 2.34A
  Current Input: 2.45A
  Duty Cycle: 15.2%
  RPM: 1250
  Input Voltage: 24.1V
  Consumed Ah: 1.23Ah
  Consumed Wh: 29.6Wh
  Fault Code: 0

vesc> adc
Getting ADC values...

ADC Values:
  ADC1: 0.523
  ADC2: 0.187
  ADC3: 0.000
  Input Voltage: 24.1V

vesc> listen 1 4
Listening for commands: 9(STATUS_1), 16(STATUS_4)... Press Ctrl+C to stop
[Status1] Controller: 1, RPM: 1250, Current: 2.34A, Duty: 15.2%
[Status4] Controller: 1, Temp_FET: 45.2°C, Temp_Motor: 38.1°C, Current_In: 2.45A, PID_Pos: 0.00
^C
Stopped listening for status messages

vesc> verbose
Verbose logging enabled

vesc> logging
CAN logging enabled: vesc_can_log_20250115_143022.csv

vesc> quit
Exiting...
```

The shell supports various CAN interfaces and configurations:

```bash
# Use different CAN interface
python vesc_shell.py can1 --id 2

# Use different bus type (PCAN)
python vesc_shell.py PCAN_USBBUS1 --bustype pcan --id 1

# Use CANalyst-II device
python vesc_shell.py 0 --bustype canalystii --id 1

# Custom baudrate
python vesc_shell.py can0 --id 1 --baudrate 250000

# Custom sender ID
python vesc_shell.py can0 --id 1 --sender-id 100
```

## Command Reference

### Initialization and Configuration

#### `vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t receiver_controller_id, uint8_t sender_id)`
Initialize the VESC CAN SDK.
- `can_send_func`: Function pointer to CAN send implementation
- `receiver_controller_id`: Receiver controller ID for receiving frames (can be changed later)
- `sender_id`: Sender controller ID for buffer protocol commands
- **Returns**: `true` on success, `false` on failure

#### `vesc_set_controller_id(uint8_t receiver_controller_id)`
Set the receiver controller ID for receiving frames.
- `receiver_controller_id`: Receiver controller ID to listen for

#### `vesc_set_sender_controller_id(uint8_t sender_controller_id)`
Set the sender's controller ID for buffer protocol commands.
- `sender_controller_id`: Controller ID to use as sender ID (typically 42)
- **Important**: This ensures that when using buffer protocol commands (like `vesc_get_values()`, `vesc_get_fw_version()`, etc.), the VESC will respond with the correct sender ID in the response.
- **Note**: This is separate from the receiver controller ID. The receiver ID is used in CAN ID fields, while the sender ID is used in the first byte of buffer protocol commands.

#### `vesc_get_sender_controller_id(void)`
Get the current sender's controller ID.
- **Returns**: The current sender controller ID

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

#### `vesc_get_decoded_chuck(uint8_t controller_id)`
Get decoded chuck (nunchuk) data.
- `controller_id`: VESC controller ID (0-255)
- **Response**: `vesc_chuck_values_t` structure

#### `vesc_set_chuck_data(uint8_t controller_id, const vesc_chuck_data_t *chuck_data)`
Set chuck (nunchuk) data.
- `controller_id`: VESC controller ID (0-255)
- `chuck_data`: Pointer to chuck data structure

#### `vesc_get_mcc_config(uint8_t controller_id)`
Get MCC configuration.
- `controller_id`: VESC controller ID (0-255)

### SDK Initialization and Control

#### `vesc_can_init(vesc_can_send_func_t can_send_func, uint8_t controller_id)`
Initialize the VESC CAN SDK.
- `can_send_func`: User-provided CAN send function
- `controller_id`: Default controller ID
- **Returns**: `true` on success, `false` on failure

#### `vesc_set_response_callback(vesc_response_callback_t callback)`
Set the response callback function.
- `callback`: Function to handle VESC responses

#### `vesc_set_controller_id(uint8_t controller_id)`
Set the default controller ID.
- `controller_id`: New default controller ID

#### `vesc_process_can_frame(uint32_t id, uint8_t *data, uint8_t len)`
Process incoming CAN frame.
- `id`: CAN frame ID
- `data`: CAN frame data
- `len`: Data length

### Debug Functions

#### `vesc_debug_configure(vesc_debug_config_t *config)`
Configure debug output.
- `config`: Debug configuration structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_debug_enable(uint8_t level, uint16_t categories)`
Enable debug output.
- `level`: Debug level (0-3)
- `categories`: Debug categories (bit flags)

#### `vesc_debug_disable(void)`
Disable debug output.

#### `vesc_debug_get_stats(vesc_debug_stats_t *stats)`
Get debug statistics.
- `stats`: Pointer to statistics structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_debug_reset_stats(void)`
Reset debug statistics.

#### `vesc_debug_print_stats(void)`
Print debug statistics to console.

#### `vesc_debug_set_output_func(vesc_debug_output_func_t output_func)`
Set custom debug output function.
- `output_func`: Custom output function (NULL = printf)

#### `vesc_debug_print_buffer_state(void)`
Print buffer state to console.

## Debug Configuration

### Debug Levels
- `VESC_DEBUG_NONE` (0) - No debug output
- `VESC_DEBUG_BASIC` (1) - Basic debug information
- `VESC_DEBUG_DETAILED` (2) - Detailed debug information
- `VESC_DEBUG_VERBOSE` (3) - Verbose debug information

### Debug Categories (bit flags)
- `VESC_DEBUG_CAN` (0x0001) - CAN communication
- `VESC_DEBUG_COMMANDS` (0x0002) - Command sending
- `VESC_DEBUG_RESPONSES` (0x0004) - Response parsing
- `VESC_DEBUG_BUFFERS` (0x0008) - Buffer management
- `VESC_DEBUG_ERRORS` (0x0010) - Error conditions
- `VESC_DEBUG_PERFORMANCE` (0x0020) - Performance metrics
- `VESC_DEBUG_ALL` (0x003F) - All categories

### `vesc_debug_config_t`
```c
typedef struct {
    uint8_t level;                    // Debug level (0-3)
    uint16_t categories;              // Enabled categories (bit flags)
    vesc_debug_output_func_t output_func; // Custom output function (NULL = printf)
    bool enable_timestamps;           // Include timestamps in output
    bool enable_statistics;           // Collect debug statistics
} vesc_debug_config_t;
```

### `vesc_debug_stats_t`
```c
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
```

## Command and Packet Definitions

### Commands
- `COMM_FW_VERSION` (0) - Get firmware version
- `COMM_GET_VALUES` (4) - Get motor values
- `COMM_GET_MCC_CONFIG` (14) - Get MCC configuration
- `COMM_DETECT_MOTOR_R_L` (25) - Detect motor R/L
- `COMM_DETECT_MOTOR_PARAM` (26) - Detect motor parameters
- `COMM_DETECT_MOTOR_FLUX_LINKAGE` (27) - Detect motor flux linkage
- `COMM_REBOOT` (29) - Reboot VESC
- `COMM_GET_DECODED_PPM` (31) - Get decoded PPM
- `COMM_GET_DECODED_ADC` (32) - Get decoded ADC
- `COMM_GET_DECODED_CHUK` (33) - Get decoded chuck
- `COMM_FORWARD_CAN` (34) - Forward CAN message
- `COMM_SET_CHUCK_DATA` (35) - Set chuck data
- `COMM_CAN_UPDATE_BAUD_ALL` (158) - Update CAN baud rate

### Packets
- `CAN_PACKET_SET_DUTY` (0) - Set duty cycle
- `CAN_PACKET_SET_CURRENT` (1) - Set current
- `CAN_PACKET_SET_CURRENT_BRAKE` (2) - Set brake current
- `CAN_PACKET_SET_RPM` (3) - Set RPM
- `CAN_PACKET_STATUS` (9) - Status message 1
- `CAN_PACKET_STATUS_2` (14) - Status message 2
- `CAN_PACKET_STATUS_3` (15) - Status message 3
- `CAN_PACKET_STATUS_4` (16) - Status message 4
- `CAN_PACKET_STATUS_5` (27) - Status message 5
- `CAN_PACKET_STATUS_6` (58) - Status message 6

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

### `vesc_chuck_values_t`
```c
typedef struct {
    float js_y;               // Joystick Y value (0.0-1.0)
    bool valid;               // Response validity
} vesc_chuck_values_t;
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

## Building

### C SDK
```bash
cd vesc_can_sdk
make
```

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

## Response Parsing Functions

These functions parse VESC responses and status messages.

#### `vesc_parse_get_values(uint8_t *data, uint8_t len, vesc_values_t *values)`
Parse COMM_GET_VALUES response.
- `data`: Response data
- `len`: Data length
- `values`: Pointer to values structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_motor_rl_response(uint8_t *data, uint8_t len, vesc_motor_rl_response_t *response)`
Parse motor resistance/inductance detection response.
- `data`: Response data
- `len`: Data length
- `response`: Pointer to motor RL response structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_motor_param_response(uint8_t *data, uint8_t len, vesc_motor_param_response_t *response)`
Parse motor parameter detection response.
- `data`: Response data
- `len`: Data length
- `response`: Pointer to motor parameter response structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_flux_linkage_response(uint8_t *data, uint8_t len, vesc_flux_linkage_response_t *response)`
Parse motor flux linkage detection response.
- `data`: Response data
- `len`: Data length
- `response`: Pointer to flux linkage response structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_adc_values(uint8_t *data, uint8_t len, vesc_adc_values_t *values)`
Parse COMM_GET_DECODED_ADC response.
- `data`: Response data
- `len`: Data length
- `values`: Pointer to ADC values structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_ppm_values(uint8_t *data, uint8_t len, vesc_ppm_values_t *values)`
Parse COMM_GET_DECODED_PPM response.
- `data`: Response data
- `len`: Data length
- `values`: Pointer to PPM values structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_fw_version(uint8_t *data, uint8_t len, vesc_fw_version_t *version)`
Parse COMM_FW_VERSION response.
- `data`: Response data
- `len`: Data length
- `version`: Pointer to firmware version structure
- **Returns**: `true` on success, `false` on failure

#### `vesc_parse_chuck_values(uint8_t *data, uint8_t len, vesc_chuck_values_t *values)`
Parse COMM_GET_DECODED_CHUK response.
- `data`: Response data
- `len`: Data length
- `values`: Pointer to chuck values structure
- **Returns**: `true` on success, `false` on failure

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