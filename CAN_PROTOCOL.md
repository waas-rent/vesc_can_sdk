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

### Target Audience

This documentation is intended for:
- Developers integrating VESC controllers into their projects
- Engineers working with CAN bus communication
- Anyone needing to understand VESC's communication protocol
- Users developing custom firmware or applications for VESC devices

---

## CAN Packets

CAN packets are **low-level communication messages** that are transmitted directly over the CAN bus. They are defined in `comm_can.c` in the VESC firmware and have these characteristics:

1. **Direct hardware communication**: CAN packets are sent directly to the CAN hardware interface
2. **Simple structure**: Each packet has a specific ID and up to 8 bytes of data
3. **Real-time control**: Used for immediate motor control (duty, current, RPM, position)
4. **Status broadcasting**: Used to broadcast status information to other devices on the CAN bus

Examples of CAN packets:
- `CAN_PACKET_SET_DUTY` - Set motor duty cycle
- `CAN_PACKET_SET_CURRENT` - Set motor current
- `CAN_PACKET_STATUS` - Broadcast motor status (RPM, current, duty)
- `CAN_PACKET_PING` - Check if a VESC is present on the bus

## Commands

Commands are **high-level protocol messages** that are processed by the VESC command system. They are defined in `commands.c` and have these characteristics:

1. **Complex operations**: Commands can perform complex operations that may involve multiple steps
2. **Configuration management**: Used for reading/writing motor and application configurations
3. **File operations**: Handle firmware updates, code storage, and data transfer
4. **Diagnostic functions**: Motor detection, parameter measurement, debugging
5. **Variable length**: Commands can be much longer than CAN packets (up to 256 bytes)

Examples of commands:
- `COMM_GET_VALUES` - Get comprehensive motor values
- `COMM_SET_MCCONF` - Set motor configuration
- `COMM_DETECT_MOTOR_PARAM` - Detect motor parameters
- `COMM_JUMP_TO_BOOTLOADER` - Enter bootloader mode
- `COMM_TERMINAL_CMD` - Execute terminal commands

## Key Differences

| Aspect | CAN Packets | Commands |
|--------|-------------|----------|
| **Purpose** | Real-time control & status | Configuration & complex operations |
| **Data size** | Max 8 bytes | Up to 256 bytes |
| **Processing** | Direct hardware access | Processed through command system |
| **Use case** | Motor control, status broadcast | Setup, configuration, diagnostics |
| **Protocol** | Simple ID + data | Complex packet structure with CRC |

## Relationship

CAN packets and commands can work together:
- Commands can be sent over CAN using the buffer system (`CAN_PACKET_FILL_RX_BUFFER`, `CAN_PACKET_PROCESS_RX_BUFFER`)
- This allows complex command operations to be performed over the CAN bus
- The VESC can forward commands to other VESCs on the CAN bus

In essence, CAN packets are the "fast lane" for real-time control, while commands are the "management interface" for configuration and complex operations.

Here are the structures for both CAN packets and commands with hex examples:

## CAN Packet Structure

**CAN Frame Format:**
```
[Extended ID: 32 bits] [Data Length: 4 bits] [Data: 0-8 bytes]
```

**Example: STATUS_6 Packet**
```
Extended ID: 0x00000006  (CAN_PACKET_STATUS_6 << 8)
Data Length: 0x08        (8 bytes)
Data: [0x12 0x34 0x56 0x78 0x9A 0xBC 0xDE 0xF0]
      │ADC1│ADC2│ADC3│PPM │
      │2 bytes each      │
```

**CAN Packet ID Breakdown:**
```
0x00000006 = [00 00 00 06]
             │Packet Type│Controller ID│
             │  24 bits  │   8 bits    │
```

## Command Structure

**Command Packet Format:**
```
[Command ID: 8 bits] [Payload Length: 16 bits] [Payload Data: variable] [CRC: 16 bits]
```

**Example: COMM_GET_VALUES Command**
```
Command ID: 0x27        (COMM_GET_VALUES)
Payload Length: 0x0032  (50 bytes)
Payload Data: [0x12 0x34 0x56 0x78 0x9A 0xBC ... 0xEF]
CRC: 0xABCD
```

**Command Payload Breakdown:**
```
0x27 00 32 [12 34 56 78 9A BC ... EF] AB CD
│ID │Length│     Variable Data      │CRC│
│8b │ 16b  │     0-256 bytes        │16b│
```

## Comparison Table

| Field | CAN Packet | Command |
|-------|------------|---------|
| **Header** | 32-bit Extended ID | 8-bit Command ID |
| **Length** | 4-bit DLC (0-8) | 16-bit Length (0-256) |
| **Data** | Fixed 8 bytes max | Variable 0-256 bytes |
| **Validation** | CAN hardware CRC | Software CRC-16 |
| **Addressing** | ID embedded in Extended ID | Separate addressing |

## Real Examples

**CAN Packet - Set Current:**
```
ID: 0x00000001  (Controller 1, SET_CURRENT)
Data: [0xE8 0x03 0x00 0x00]  (1000 mA = 0x3E8)
```

**Command - Get Values:**
```
ID: 0x27
Length: 0x0040  (64 bytes)
Data: [0x12 0x34 0x56 0x78 ... 0xEF]  (64 bytes of motor data)
CRC: 0xABCD
```

**CAN Packet - Status Broadcast:**
```
ID: 0x00000002  (Controller 2, STATUS)
Data: [0x88 0x13 0x00 0x00 0x64 0x00 0x00 0x00]
      │RPM │    │Current│    │Duty │    │
      │5000│    │100 mA │    │10%  │    │
```

**Command - Set Configuration:**
```
ID: 0x32
Length: 0x0080  (128 bytes)
Data: [0x01 0x02 0x03 ... 0x80]  (128 bytes of config data)
CRC: 0x1234
```

The key difference is that CAN packets are **compact and fixed-size** for real-time efficiency, while commands are **flexible and variable-size** for complex operations.

## Command Fragmentation Process

When a command is too large to fit in a single CAN frame (8 bytes), it gets split into multiple CAN packets using a buffer system.

## CAN Buffer System Structure

**Step 1: Fill Buffer Packets**
```
CAN_PACKET_FILL_RX_BUFFER:
ID: [Controller_ID | 0x000000]  (24-bit packet type)
Data: [Offset: 8 bits] [Data: 7 bytes]
      │0x00│[Command data...]│
      │    │                 │
      │    └─ 7 bytes of command data
      └─ Buffer offset (0, 7, 14, 21...)
```

**Step 2: Fill Buffer Long Packets** (for larger offsets)
```
CAN_PACKET_FILL_RX_BUFFER_LONG:
ID: [Controller_ID | 0x000000]  (24-bit packet type)
Data: [Offset_high: 8 bits] [Offset_low: 8 bits] [Data: 6 bytes]
      │0x00│0xFF│[Command data...]│
      │    │    │                 │
      │    │    └─ 6 bytes of command data
      │    │    └─ Offset low byte (0xFF)
      └─ Offset high byte (0x00)
```

**Step 3: Process Buffer Packet**
```
CAN_PACKET_PROCESS_RX_BUFFER:
ID: [Controller_ID | 0x000000]  (24-bit packet type)
Data: [Sender_ID: 8 bits] [Send_type: 8 bits] [Length_high: 8 bits] [Length_low: 8 bits] [CRC_high: 8 bits] [CRC_low: 8 bits]
      │0x01│0x00│0x00│0x40│0xAB│0xCD│
      │    │    │    │    │    │    │
      │    │    │    │    │    └─ CRC low byte
      │    │    │    │    └─ CRC high byte
      │    │    │    └─ Length low byte (64 bytes)
      │    │    └─ Length high byte
      │    └─ Send type (0=process, 1=send, 2=no reply, 3=local)
      └─ Sender controller ID
```

## Example: Sending a 50-byte Command

**Command to send:**
```
[0x27][0x00][0x32][...50 bytes of data...][0xAB][0xCD]
│ID  │Length│     Payload Data            │CRC │
```

**Step 1: First 7 bytes**
```
CAN Frame 1:
ID: 0x00000001  (Controller 1, FILL_RX_BUFFER)
Data: [0x00][0x27][0x00][0x32][...4 more bytes...]
      │Offset│Command data...
```

**Step 2: Next 7 bytes**
```
CAN Frame 2:
ID: 0x00000001  (Controller 1, FILL_RX_BUFFER)
Data: [0x07][...7 bytes of data...]
      │Offset│
```

**Step 3: Continue with 6-byte packets**
```
CAN Frame 3:
ID: 0x00000001  (Controller 1, FILL_RX_BUFFER_LONG)
Data: [0x00][0x0E][...6 bytes of data...]
      │Offset│
```

**Step 4: Final processing**
```
CAN Frame N:
ID: 0x00000001  (Controller 1, PROCESS_RX_BUFFER)
Data: [0x01][0x00][0x00][0x32][0xAB][0xCD]
      │Sender│Type│Length│    │CRC │
```

## Buffer Management

**Short Commands (≤6 bytes):**
```
CAN_PACKET_PROCESS_SHORT_BUFFER:
ID: [Controller_ID | 0x000000]
Data: [Sender_ID: 8 bits] [Send_type: 8 bits] [Command_data: 1-6 bytes]
      │0x01│0x00│[Short command...]│
```

## Key Points

1. **Two-phase system**: Fill buffer → Process buffer
2. **Offset tracking**: Each packet includes its position in the command
3. **CRC validation**: Ensures data integrity across all fragments
4. **Send types**: Control how the command is processed (local vs forwarded)
5. **Automatic switching**: Uses short buffer for ≤6 bytes, long buffer for larger data
