# VESC CAN Protocol Documentation

## Introduction

This document provides a comprehensive overview of the CAN (Controller Area Network) communication protocol used by VESC (Vedder Electronic Speed Controller) devices. The VESC CAN protocol enables real-time communication between VESC controllers and other devices on a CAN bus, supporting both direct motor control and complex configuration operations.

### What is VESC?

VESC is an open-source electronic speed controller designed for electric vehicles, robotics, and other motor control applications. It supports various motor types including BLDC, PMSM, and DC motors, and can be configured for different applications through its flexible firmware.

### What is CAN?

CAN (Controller Area Network) is a robust vehicle bus standard designed to allow microcontrollers and devices to communicate with each other within a vehicle without a host computer. CAN is designed to handle high-speed data transmission while being resistant to electrical interference and providing error detection capabilities.

### Two Types of Communication

The VESC CAN protocol supports two distinct types of communication:

1. **CAN Packets**: Direct, low-level messages for real-time motor control and status broadcasting. These are simple, fixed-size messages (up to 8 bytes) that provide immediate control and monitoring capabilities.

2. **Commands**: High-level protocol messages for complex operations, configuration management, and diagnostic functions. These are variable-length messages (up to 256 bytes) that handle setup, configuration, and advanced operations.

Both communication types can be transmitted over the same CAN bus, allowing for a flexible and powerful control system that combines real-time performance with comprehensive management capabilities.

### Document Scope

This document covers:
- **CAN Packets**: Low-level communication messages for real-time motor control
- **Commands**: High-level protocol messages for configuration and complex operations
- **Protocol Structure**: Detailed breakdown of packet formats and data structures
- **Command Fragmentation**: How large commands are split across multiple CAN frames
- **Practical Examples**: Real-world usage scenarios with hex data examples
- **Byte-level Details**: Complete byte-by-byte breakdown of all packet types

### Target Audience

This documentation is intended for:
- Developers integrating VESC controllers into their projects
- Engineers working with CAN bus communication
- Anyone needing to understand VESC's communication protocol
- Users developing custom firmware or applications for VESC devices

---

## CAN Packet Structure

### CAN Frame Format

**Standard CAN Frame:**
```
[Extended ID: 32 bits] [Data Length: 4 bits] [Data: 0-8 bytes]
```

**CAN ID Structure:**
```
Extended ID: [Packet Type: 24 bits] [Controller ID: 8 bits]
             │   Upper 24 bits     │  Lower 8 bits   │
```

**Example CAN ID Calculation:**
```
Controller ID: 1
Packet Type: CAN_PACKET_SET_DUTY (0x00)
Extended ID: (0x00 << 8) | 0x01 = 0x00000001
```

### CAN Packet Types

The VESC CAN protocol defines 64 different packet types (0-63) for various operations:

#### Motor Control Packets
- `CAN_PACKET_SET_DUTY` (0) - Set motor duty cycle
- `CAN_PACKET_SET_CURRENT` (1) - Set motor current
- `CAN_PACKET_SET_CURRENT_BRAKE` (2) - Set brake current
- `CAN_PACKET_SET_RPM` (3) - Set motor RPM
- `CAN_PACKET_SET_POS` (4) - Set motor position
- `CAN_PACKET_SET_CURRENT_REL` (10) - Set relative current
- `CAN_PACKET_SET_CURRENT_BRAKE_REL` (11) - Set relative brake current
- `CAN_PACKET_SET_CURRENT_HANDBRAKE` (12) - Set handbrake current
- `CAN_PACKET_SET_CURRENT_HANDBRAKE_REL` (13) - Set relative handbrake current

#### Status Packets
- `CAN_PACKET_STATUS` (9) - Status message 1 (RPM, current, duty)
- `CAN_PACKET_STATUS_2` (14) - Status message 2 (Amp hours)
- `CAN_PACKET_STATUS_3` (15) - Status message 3 (Watt hours)
- `CAN_PACKET_STATUS_4` (16) - Status message 4 (Temperatures, input current, PID)
- `CAN_PACKET_STATUS_5` (27) - Status message 5 (Tachometer, input voltage)
- `CAN_PACKET_STATUS_6` (58) - Status message 6 (ADC values, PPM)

#### Buffer Management Packets
- `CAN_PACKET_FILL_RX_BUFFER` (5) - Fill receive buffer (7-byte chunks)
- `CAN_PACKET_FILL_RX_BUFFER_LONG` (6) - Fill receive buffer (6-byte chunks)
- `CAN_PACKET_PROCESS_RX_BUFFER` (7) - Process receive buffer
- `CAN_PACKET_PROCESS_SHORT_BUFFER` (8) - Process short buffer (≤6 bytes)

#### System Packets
- `CAN_PACKET_PING` (17) - Ping request
- `CAN_PACKET_PONG` (18) - Ping response
- `CAN_PACKET_UPDATE_BAUD` (63) - Update CAN baud rate

#### Configuration Packets
- `CAN_PACKET_CONF_CURRENT_LIMITS` (21) - Configure current limits
- `CAN_PACKET_CONF_STORE_CURRENT_LIMITS` (22) - Store current limits
- `CAN_PACKET_CONF_CURRENT_LIMITS_IN` (23) - Configure input current limits
- `CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN` (24) - Store input current limits
- `CAN_PACKET_CONF_FOC_ERPMS` (25) - Configure FOC ERPMs
- `CAN_PACKET_CONF_STORE_FOC_ERPMS` (26) - Store FOC ERPMs
- `CAN_PACKET_CONF_BATTERY_CUT` (29) - Configure battery cut
- `CAN_PACKET_CONF_STORE_BATTERY_CUT` (30) - Store battery cut

#### BMS Packets
- `CAN_PACKET_BMS_V_TOT` (38) - BMS total voltage
- `CAN_PACKET_BMS_I` (39) - BMS current
- `CAN_PACKET_BMS_AH_WH` (40) - BMS amp hours and watt hours
- `CAN_PACKET_BMS_V_CELL` (41) - BMS cell voltages
- `CAN_PACKET_BMS_BAL` (42) - BMS balancing
- `CAN_PACKET_BMS_TEMPS` (43) - BMS temperatures
- `CAN_PACKET_BMS_HUM` (44) - BMS humidity
- `CAN_PACKET_BMS_SOC_SOH_TEMP_STAT` (45) - BMS SOC, SOH, temperature status

#### IO Board Packets
- `CAN_PACKET_IO_BOARD_ADC_1_TO_4` (32) - IO board ADC 1-4
- `CAN_PACKET_IO_BOARD_ADC_5_TO_8` (33) - IO board ADC 5-8
- `CAN_PACKET_IO_BOARD_ADC_9_TO_12` (34) - IO board ADC 9-12
- `CAN_PACKET_IO_BOARD_DIGITAL_IN` (35) - IO board digital inputs
- `CAN_PACKET_IO_BOARD_SET_OUTPUT_DIGITAL` (36) - Set IO board digital outputs
- `CAN_PACKET_IO_BOARD_SET_OUTPUT_PWM` (37) - Set IO board PWM outputs

#### GNSS Packets
- `CAN_PACKET_GNSS_TIME` (59) - GNSS time
- `CAN_PACKET_GNSS_LAT` (60) - GNSS latitude
- `CAN_PACKET_GNSS_LON` (61) - GNSS longitude
- `CAN_PACKET_GNSS_ALT_SPEED_HDOP` (62) - GNSS altitude, speed, HDOP

---

## Detailed CAN Packet Examples

### Motor Control Packets

#### CAN_PACKET_SET_DUTY (0)
**Purpose:** Set motor duty cycle (0.0 to 1.0)

**CAN ID:** `(0x00 << 8) | controller_id`

**Data Format:**
```
[0-3]: Duty cycle as int32 (duty * 100000)
```

**Example: Set 50% duty cycle on controller 1**
```
CAN ID: 0x00000001
Data: [0x88 0x13 0x00 0x00]  // 50000 = 0.5 * 100000
      │Duty cycle (little endian)│
```

**Byte Breakdown:**
- Bytes 0-3: `0x88 0x13 0x00 0x00` = 50000 (0.5 * 100000)
- Little endian: 0x00001388 = 50000

#### CAN_PACKET_SET_CURRENT (1)
**Purpose:** Set motor current in amperes

**CAN ID:** `(0x01 << 8) | controller_id`

**Data Format:**
```
[0-3]: Current as int32 (current * 1000)
```

**Example: Set 10A current on controller 2**
```
CAN ID: 0x00000002
Data: [0x10 0x27 0x00 0x00]  // 10000 = 10.0 * 1000
      │Current (little endian)│
```

**Byte Breakdown:**
- Bytes 0-3: `0x10 0x27 0x00 0x00` = 10000 (10.0 * 1000)
- Little endian: 0x00002710 = 10000

#### CAN_PACKET_SET_RPM (3)
**Purpose:** Set motor RPM

**CAN ID:** `(0x03 << 8) | controller_id`

**Data Format:**
```
[0-3]: RPM as int32
```

**Example: Set 5000 RPM on controller 1**
```
CAN ID: 0x00000001
Data: [0x88 0x13 0x00 0x00]  // 5000 RPM
      │RPM (little endian)   │
```

**Byte Breakdown:**
- Bytes 0-3: `0x88 0x13 0x00 0x00` = 5000
- Little endian: 0x00001388 = 5000

### Status Packets

#### CAN_PACKET_STATUS (9)
**Purpose:** Broadcast motor status (RPM, current, duty)

**CAN ID:** `(0x09 << 8) | controller_id`

**Data Format:**
```
[0-3]: RPM as int32
[4-5]: Current as int16 (current * 10)
[6-7]: Duty as int16 (duty * 1000)
```

**Example: Status from controller 1**
```
CAN ID: 0x00000001
Data: [0x88 0x13 0x00 0x00 0x64 0x00 0xE8 0x03]
      │RPM │    │Current│    │Duty │    │
      │5000│    │10.0A  │    │10%  │    │
```

**Byte Breakdown:**
- Bytes 0-3: `0x88 0x13 0x00 0x00` = 5000 RPM
- Bytes 4-5: `0x64 0x00` = 100 (10.0A * 10)
- Bytes 6-7: `0xE8 0x03` = 1000 (0.1 * 1000)

#### CAN_PACKET_STATUS_2 (14)
**Purpose:** Broadcast amp hours consumed and charged

**CAN ID:** `(0x0E << 8) | controller_id`

**Data Format:**
```
[0-3]: Amp hours consumed as int32 (Ah * 10000)
[4-7]: Amp hours charged as int32 (Ah * 10000)
```

**Example: Status 2 from controller 1**
```
CAN ID: 0x00000001
Data: [0x10 0x27 0x00 0x00 0x20 0x4E 0x00 0x00]
      │Ah consumed│    │Ah charged│    │
      │10.0 Ah    │    │20.0 Ah   │    │
```

**Byte Breakdown:**
- Bytes 0-3: `0x10 0x27 0x00 0x00` = 100000 (10.0 * 10000)
- Bytes 4-7: `0x20 0x4E 0x00 0x00` = 200000 (20.0 * 10000)

#### CAN_PACKET_STATUS_3 (15)
**Purpose:** Broadcast watt hours consumed and charged

**CAN ID:** `(0x0F << 8) | controller_id`

**Data Format:**
```
[0-3]: Watt hours consumed as int32 (Wh * 10000)
[4-7]: Watt hours charged as int32 (Wh * 10000)
```

**Example: Status 3 from controller 1**
```
CAN ID: 0x00000001
Data: [0x40 0x9C 0x00 0x00 0x80 0x38 0x01 0x00]
      │Wh consumed│    │Wh charged│    │
      │40.0 Wh    │    │80.0 Wh   │    │
```

**Byte Breakdown:**
- Bytes 0-3: `0x40 0x9C 0x00 0x00` = 400000 (40.0 * 10000)
- Bytes 4-7: `0x80 0x38 0x01 0x00` = 800000 (80.0 * 10000)

#### CAN_PACKET_STATUS_4 (16)
**Purpose:** Broadcast temperatures, input current, and PID position

**CAN ID:** `(0x10 << 8) | controller_id`

**Data Format:**
```
[0-1]: FET temperature as int16 (temp * 10)
[2-3]: Motor temperature as int16 (temp * 10)
[4-5]: Input current as int16 (current * 10)
[6-7]: PID position as int16 (position * 50)
```

**Example: Status 4 from controller 1**
```
CAN ID: 0x00000001
Data: [0x64 0x00 0x32 0x00 0x96 0x00 0xE8 0x03]
      │FET │Motor│Input│PID │
      │10°C│5°C  │15A  │20  │
```

**Byte Breakdown:**
- Bytes 0-1: `0x64 0x00` = 100 (10.0°C * 10)
- Bytes 2-3: `0x32 0x00` = 50 (5.0°C * 10)
- Bytes 4-5: `0x96 0x00` = 150 (15.0A * 10)
- Bytes 6-7: `0xE8 0x03` = 1000 (20.0 * 50)

#### CAN_PACKET_STATUS_5 (27)
**Purpose:** Broadcast tachometer value and input voltage

**CAN ID:** `(0x1B << 8) | controller_id`

**Data Format:**
```
[0-3]: Tachometer value as int32
[4-5]: Input voltage as int16 (voltage * 10)
```

**Example: Status 5 from controller 1**
```
CAN ID: 0x00000001
Data: [0x88 0x13 0x00 0x00 0xE8 0x03]
      │Tachometer│Input V│
      │5000      │50.0V  │
```

**Byte Breakdown:**
- Bytes 0-3: `0x88 0x13 0x00 0x00` = 5000 (tachometer)
- Bytes 4-5: `0xE8 0x03` = 1000 (50.0V * 10)

#### CAN_PACKET_STATUS_6 (58)
**Purpose:** Broadcast ADC values and PPM

**CAN ID:** `(0x3A << 8) | controller_id`

**Data Format:**
```
[0-1]: ADC1 as int16 (value * 1000)
[2-3]: ADC2 as int16 (value * 1000)
[4-5]: ADC3 as int16 (value * 1000)
[6-7]: PPM as int16 (value * 1000)
```

**Example: Status 6 from controller 1**
```
CAN ID: 0x00000001
Data: [0x64 0x00 0xC8 0x00 0x2C 0x01 0x90 0x01]
      │ADC1│ADC2│ADC3│PPM │
      │0.1 │0.2 │0.3 │0.4 │
```

**Byte Breakdown:**
- Bytes 0-1: `0x64 0x00` = 100 (0.1 * 1000)
- Bytes 2-3: `0xC8 0x00` = 200 (0.2 * 1000)
- Bytes 4-5: `0x2C 0x01` = 300 (0.3 * 1000)
- Bytes 6-7: `0x90 0x01` = 400 (0.4 * 1000)

### System Packets

#### CAN_PACKET_PING (17)
**Purpose:** Check if a VESC is present on the bus

**CAN ID:** `(0x11 << 8) | controller_id`

**Data Format:**
```
[0-7]: Empty (no data)
```

**Example: Ping controller 1**
```
CAN ID: 0x00000001
Data: []  // No data
```

#### CAN_PACKET_PONG (18)
**Purpose:** Response to ping

**CAN ID:** `(0x12 << 8) | controller_id`

**Data Format:**
```
[0]: Controller ID
```

**Example: Pong from controller 1**
```
CAN ID: 0x00000001
Data: [0x01]  // Controller ID
```

---

## Command Structure

Commands are high-level protocol messages that can be much longer than CAN packets (up to 256 bytes). They use a more complex structure with CRC validation.

### Command Packet Format

**Command Structure:**
```
[Command ID: 8 bits] [Payload Length: 16 bits] [Payload Data: variable] [CRC: 16 bits]
```

**Example: COMM_GET_VALUES Command**
```
Command ID: 0x04        (COMM_GET_VALUES)
Payload Length: 0x0001  (1 byte)
Payload Data: [0x04]    (Command ID repeated)
CRC: 0xABCD
```

**Complete Command:**
```
0x04 00 01 04 AB CD
│ID │Length│Data│CRC│
│8b │ 16b  │8b  │16b│
```

### Command Definitions

#### COMM_FW_VERSION (0)
**Purpose:** Get firmware version information

**Payload:** Empty

**Example:**
```
Command: 0x00 00 00
Response: [Major][Minor][HW Name][UUID][Pairing][Test][HW Type][Config]
```

#### COMM_GET_VALUES (4)
**Purpose:** Get comprehensive motor values

**Payload:** Empty

**Example:**
```
Command: 0x04 00 00
Response: [Temp FET][Temp Motor][Current Motor][Current In][Current ID][Current IQ][Duty][RPM][V In][Ah][Ah Chg][Wh][Wh Chg][Tacho][Tacho Abs][Fault][PID Pos][ID][Temp MOS1][Temp MOS2][Temp MOS3][Vd][Vq][Status]
```

#### COMM_GET_MCC_CONFIG (14)
**Purpose:** Get motor configuration

**Payload:** Empty

#### COMM_DETECT_MOTOR_R_L (25)
**Purpose:** Detect motor resistance and inductance

**Payload:** Empty

#### COMM_DETECT_MOTOR_PARAM (26)
**Purpose:** Detect motor parameters

**Payload:**
```
[0]: Command ID (0x1A)
[1-4]: Current as float32 (current * 1000)
[5-8]: Min RPM as float32 (rpm * 1000)
[9-12]: Low duty as float32 (duty * 1000)
```

**Example: Detect motor parameters**
```
Command: 0x1A 00 0C 1A 88 13 00 00 10 27 00 00 64 00 00 00
        │ID │Length│ID │Current│Min RPM│Low Duty│
        │26 │12    │26 │10A    │5000   │0.1     │
```

#### COMM_DETECT_MOTOR_FLUX_LINKAGE (27)
**Purpose:** Detect motor flux linkage

**Payload:**
```
[0]: Command ID (0x1B)
[1-4]: Current as float32 (current * 1000)
[5-8]: Min RPM as float32 (rpm * 1000)
[9-12]: Duty as float32 (duty * 1000)
[13-16]: Resistance as float32 (resistance * 1000)
```

#### COMM_GET_DECODED_ADC (32)
**Purpose:** Get decoded ADC values

**Payload:** Empty

#### COMM_GET_DECODED_PPM (31)
**Purpose:** Get decoded PPM values

**Payload:** Empty

#### COMM_GET_DECODED_CHUK (33)
**Purpose:** Get decoded chuck values

**Payload:** Empty

#### COMM_SET_CHUCK_DATA (35)
**Purpose:** Set chuck data

**Payload:**
```
[0]: Command ID (0x23)
[1]: Joystick X (0-255)
[2]: Joystick Y (0-255)
[3]: Button C (0/1)
[4]: Button Z (0/1)
[5-6]: Accelerometer X (int16)
[7-8]: Accelerometer Y (int16)
[9-10]: Accelerometer Z (int16)
[11]: Reverse has state (0/1)
[12]: Is reverse (0/1)
```

#### COMM_REBOOT (29)
**Purpose:** Reboot VESC

**Payload:** Empty

#### COMM_CAN_UPDATE_BAUD_ALL (158)
**Purpose:** Update CAN baud rate on all devices

**Payload:**
```
[0]: Command ID (0x9E)
[1-2]: Baud rate in kbits (uint16)
[3-4]: Delay in milliseconds (uint16)
```

---

## Command Fragmentation Process

When a command is too large to fit in a single CAN frame (8 bytes), it gets split into multiple CAN packets using a buffer system.

### Buffer System Overview

The VESC buffer system uses three packet types:
1. `CAN_PACKET_FILL_RX_BUFFER` - Fill buffer with 7-byte chunks
2. `CAN_PACKET_FILL_RX_BUFFER_LONG` - Fill buffer with 6-byte chunks
3. `CAN_PACKET_PROCESS_RX_BUFFER` - Process the complete buffer

### Short Buffer (≤6 bytes)

For commands 6 bytes or less, use `CAN_PACKET_PROCESS_SHORT_BUFFER`:

**Packet Structure:**
```
[0]: Sender controller ID
[1]: Send flag (0=process, 1=send, 2=no reply, 3=local)
[2-7]: Command data (1-6 bytes)
```

**Example: Send COMM_FW_VERSION (1 byte)**
```
CAN ID: 0x00000001  (Controller 1, PROCESS_SHORT_BUFFER)
Data: [0x02][0x00][0x00]
      │Sender│Flag│Command│
      │ID=2  │0   │FW_VER │
```

### Long Buffer (>6 bytes)

For commands larger than 6 bytes, use the fragmentation system:

#### Step 1: Fill Buffer (First 255 bytes)

**CAN_PACKET_FILL_RX_BUFFER Structure:**
```
[0]: Buffer offset (0-254)
[1-7]: Command data (7 bytes)
```

**Example: Fill buffer with first 7 bytes**
```
CAN ID: 0x00000001  (Controller 1, FILL_RX_BUFFER)
Data: [0x00][0x1A][0x88][0x13][0x00][0x00][0x10][0x27]
      │Offset│Command data (7 bytes)        │
      │0     │DETECT_MOTOR_PARAM + params  │
```

#### Step 2: Fill Buffer Long (Remaining bytes)

**CAN_PACKET_FILL_RX_BUFFER_LONG Structure:**
```
[0-1]: Buffer offset (uint16, 255+)
[2-7]: Command data (6 bytes)
```

**Example: Fill buffer with bytes 255-260**
```
CAN ID: 0x00000001  (Controller 1, FILL_RX_BUFFER_LONG)
Data: [0x00][0xFF][0x64][0x00][0x00][0x00][0x00][0x00]
      │Offset│    │Command data (6 bytes)    │
      │255   │    │Remaining parameters      │
```

#### Step 3: Process Buffer

**CAN_PACKET_PROCESS_RX_BUFFER Structure:**
```
[0]: Sender controller ID
[1]: Send flag (0=process, 1=send, 2=no reply, 3=local)
[2-3]: Command length (uint16)
[4-5]: CRC (uint16)
```

**Example: Process 13-byte command**
```
CAN ID: 0x00000001  (Controller 1, PROCESS_RX_BUFFER)
Data: [0x02][0x00][0x00][0x0D][0xAB][0xCD]
      │Sender│Flag│Length│    │CRC │
      │ID=2  │0   │13    │    │0xABCD│
```

### Complete Example: Send COMM_DETECT_MOTOR_PARAM

**Command to send:**
```
[0x1A][0x88][0x13][0x00][0x00][0x10][0x27][0x00][0x00][0x64][0x00][0x00][0x00]
│ID  │Current│    │Min RPM│    │Low Duty│
│26  │10A    │    │5000   │    │0.1     │
```

**Step 1: First 7 bytes**
```
CAN Frame 1:
ID: 0x00000001  (Controller 1, FILL_RX_BUFFER)
Data: [0x00][0x1A][0x88][0x13][0x00][0x00][0x10][0x27]
      │Offset│Command data (7 bytes)        │
      │0     │DETECT_MOTOR_PARAM + params  │
```

**Step 2: Remaining 6 bytes**
```
CAN Frame 2:
ID: 0x00000001  (Controller 1, FILL_RX_BUFFER_LONG)
Data: [0x00][0x07][0x00][0x00][0x64][0x00][0x00][0x00]
      │Offset│    │Remaining data (6 bytes) │
      │7     │    │Low duty parameter       │
```

**Step 3: Process command**
```
CAN Frame 3:
ID: 0x00000001  (Controller 1, PROCESS_RX_BUFFER)
Data: [0x02][0x00][0x00][0x0D][0xAB][0xCD]
      │Sender│Flag│Length│    │CRC │
      │ID=2  │0   │13    │    │0xABCD│
```

---

## Data Encoding

### Integer Encoding

All integers are encoded in **little-endian** format:

**16-bit integer examples:**
```
Value: 1000
Bytes: [0xE8 0x03]  // 0x03E8 = 1000

Value: -500
Bytes: [0x0C 0xFE]  // 0xFE0C = -500 (two's complement)
```

**32-bit integer examples:**
```
Value: 50000
Bytes: [0x88 0x13 0x00 0x00]  // 0x00001388 = 50000

Value: -10000
Bytes: [0xF0 0xD8 0xFF 0xFF]  // 0xFFFFD8F0 = -10000 (two's complement)
```

### Float Encoding

Floats are encoded as scaled integers:

**Float16 (16-bit scaled float):**
```
Value: 10.5A
Scale: 10
Encoded: 105 (10.5 * 10)
Bytes: [0x69 0x00]  // 0x0069 = 105
```

**Float32 (32-bit scaled float):**
```
Value: 10.5A
Scale: 1000
Encoded: 10500 (10.5 * 1000)
Bytes: [0x94 0x29 0x00 0x00]  // 0x00002994 = 10500
```

### Common Scaling Factors

| Data Type | Scale Factor | Example |
|-----------|--------------|---------|
| Current (A) | 10 | 10.5A → 105 |
| Current (A) | 1000 | 10.5A → 10500 |
| Voltage (V) | 10 | 50.0V → 500 |
| Temperature (°C) | 10 | 25.5°C → 255 |
| Duty Cycle | 100000 | 0.5 → 50000 |
| RPM | 1 | 5000 → 5000 |
| Amp Hours | 10000 | 10.5Ah → 105000 |
| Watt Hours | 10000 | 40.2Wh → 402000 |
| ADC/PPM | 1000 | 0.5 → 500 |
| PID Position | 50 | 20.0 → 1000 |

---

## Response Parsing

### Status Message Parsing

The SDK provides parsing functions for all status messages:

#### vesc_parse_status_msg_1()
**Parses:** CAN_PACKET_STATUS
**Data:** RPM, current, duty
**Scaling:** Current/10, Duty/1000

#### vesc_parse_status_msg_2()
**Parses:** CAN_PACKET_STATUS_2
**Data:** Amp hours consumed, charged
**Scaling:** Ah/10000

#### vesc_parse_status_msg_3()
**Parses:** CAN_PACKET_STATUS_3
**Data:** Watt hours consumed, charged
**Scaling:** Wh/10000

#### vesc_parse_status_msg_4()
**Parses:** CAN_PACKET_STATUS_4
**Data:** Temperatures, input current, PID position
**Scaling:** Temp/10, Current/10, Position/50

#### vesc_parse_status_msg_5()
**Parses:** CAN_PACKET_STATUS_5
**Data:** Tachometer, input voltage
**Scaling:** Voltage/10

#### vesc_parse_status_msg_6()
**Parses:** CAN_PACKET_STATUS_6
**Data:** ADC1, ADC2, ADC3, PPM
**Scaling:** Value/1000

### Command Response Parsing

#### vesc_parse_get_values()
**Parses:** COMM_GET_VALUES response
**Data:** Complete motor status (temperatures, currents, voltages, etc.)

#### vesc_parse_fw_version()
**Parses:** COMM_FW_VERSION response
**Data:** Firmware version, hardware name, UUID, etc.

#### vesc_parse_motor_rl_response()
**Parses:** COMM_DETECT_MOTOR_R_L response
**Data:** Motor resistance and inductance

#### vesc_parse_motor_param_response()
**Parses:** COMM_DETECT_MOTOR_PARAM response
**Data:** Motor parameters (cycle integrator limit, coupling constant, hall table)

#### vesc_parse_flux_linkage_response()
**Parses:** COMM_DETECT_MOTOR_FLUX_LINKAGE response
**Data:** Motor flux linkage

---

## Practical Examples

### Example 1: Basic Motor Control

**Set motor to 50% duty cycle:**
```
vesc_set_duty(1, 0.5f);
```

**CAN Frame:**
```
ID: 0x00000001
Data: [0x88 0x13 0x00 0x00]  // 50000 = 0.5 * 100000
```

**Set motor to 10A current:**
```
vesc_set_current(1, 10.0f);
```

**CAN Frame:**
```
ID: 0x00000001
Data: [0x10 0x27 0x00 0x00]  // 10000 = 10.0 * 1000
```

### Example 2: Status Monitoring

**Receive status message:**
```
CAN ID: 0x00000001  (Controller 1, STATUS)
Data: [0x88 0x13 0x00 0x00 0x64 0x00 0xE8 0x03]
```

**Parse with SDK:**
```c
vesc_status_msg_1_t status;
if (vesc_parse_status_msg_1(data, 8, &status)) {
    printf("RPM: %.0f, Current: %.1fA, Duty: %.1f%%\n", 
           status.rpm, status.current, status.duty * 100.0f);
}
```

**Output:**
```
RPM: 5000, Current: 10.0A, Duty: 10.0%
```

### Example 3: Command with Fragmentation

**Send motor detection command:**
```c
vesc_detect_motor_param(1, 10.0f, 5000.0f, 0.1f);
```

**Generated CAN frames:**
```
Frame 1: FILL_RX_BUFFER
ID: 0x00000001
Data: [0x00][0x1A][0x88][0x13][0x00][0x00][0x10][0x27]

Frame 2: FILL_RX_BUFFER_LONG  
ID: 0x00000001
Data: [0x00][0x07][0x00][0x00][0x64][0x00][0x00][0x00]

Frame 3: PROCESS_RX_BUFFER
ID: 0x00000001
Data: [0x02][0x00][0x00][0x0D][0xAB][0xCD]
```

### Example 4: Complete Communication Flow

**1. Initialize SDK:**
```c
vesc_can_init(can_send_func, 2, 1);  // Receiver ID=2, Sender ID=1
```

**2. Set response callback:**
```c
vesc_set_response_callback(response_handler);
```

**3. Send commands:**
```c
vesc_get_fw_version(1);      // Get firmware version
vesc_set_duty(1, 0.3f);      // Set 30% duty
vesc_get_values(1);          // Get motor values
```

**4. Process responses:**
```c
void response_handler(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    switch (command) {
        case COMM_FW_VERSION:
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                printf("VESC %d: FW %d.%d, HW: %s\n", 
                       controller_id, version.major, version.minor, version.hw_name);
            }
            break;
            
        case COMM_GET_VALUES:
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                printf("VESC %d: RPM=%.0f, Current=%.2fA, Temp=%.1f°C\n",
                       controller_id, values.rpm, values.current_motor, values.temp_fet);
            }
            break;
    }
}
```

---

## Error Handling

### Common Error Conditions

1. **CRC Errors:** Invalid CRC in command responses
2. **Buffer Overflow:** Command too large for receive buffer
3. **Invalid Data:** Malformed packet data
4. **Timeout:** No response received within expected time
5. **Controller Not Found:** PING/PONG failure

### Debug Support

The SDK includes comprehensive debug support:

```c
// Enable debug output
vesc_debug_enable(VESC_DEBUG_DETAILED, VESC_DEBUG_CAN | VESC_DEBUG_COMMANDS);

// Set custom output function
vesc_debug_set_output_func(my_debug_output);

// Get debug statistics
vesc_debug_stats_t stats;
vesc_debug_get_stats(&stats);
printf("CAN TX: %u, RX: %u, Errors: %u\n", 
       stats.can_tx_count, stats.can_rx_count, stats.error_count);
```

---

## Best Practices

### 1. Controller ID Management
- Use unique controller IDs for each VESC
- Avoid ID conflicts on the same CAN bus
- Consider using sequential IDs (1, 2, 3, ...)

### 2. Error Handling
- Always check return values from parsing functions
- Implement timeout mechanisms for commands
- Handle CRC errors gracefully

### 3. Performance Optimization
- Use direct CAN packets for real-time control
- Use commands for configuration and diagnostics
- Minimize command fragmentation when possible

### 4. Debug and Monitoring
- Enable debug output during development
- Monitor CAN bus statistics
- Log important events and errors

### 5. Safety Considerations
- Implement emergency stop functionality
- Validate all input parameters
- Use appropriate current and voltage limits
- Monitor temperature and fault conditions

---

This documentation provides a complete reference for implementing VESC CAN communication. The byte-level details and examples should enable developers to integrate VESC controllers into their projects effectively.
