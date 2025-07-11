# VESC CAN SDK - Arduino Integration Guide

This guide explains how to integrate the VESC CAN SDK with Arduino for motor control applications. The VESC CAN SDK provides a pure C implementation that works seamlessly with Arduino's C/C++ environment.

## Table of Contents

1. [Overview](#overview)
2. [Hardware Requirements](#hardware-requirements)
3. [Installation](#installation)
4. [Basic Setup](#basic-setup)
5. [Motor Control Examples](#motor-control-examples)
6. [Advanced Features](#advanced-features)
7. [Best Practices](#best-practices)
8. [Troubleshooting](#troubleshooting)
9. [Complete Examples](#complete-examples)

## Overview

The VESC CAN SDK enables Arduino to communicate with VESC motor controllers over CAN bus. It provides:

- **Pure C implementation** - No external dependencies
- **Function pointer interface** - Flexible CAN communication
- **Comprehensive motor control** - Duty, current, RPM, position control
- **Real-time monitoring** - Temperature, voltage, current, RPM
- **Motor parameter detection** - Automatic motor characterization
- **Debug capabilities** - Built-in debugging and statistics

## Hardware Requirements

### Arduino Boards with Built-in CAN
- **Arduino Giga R1** - Native CAN controller
- **Arduino Due** - Native CAN controller
- **Arduino Zero** - With CAN add-on board

### CAN Interface Options
- **MCP2515 CAN Module** - SPI-based, compatible with most Arduino boards
- **SN65HVD230 CAN Transceiver** - For custom CAN circuits
- **USB-CAN Adapters** - For development and testing

### VESC Controllers
- VESC 6.0 and later
- VESC EDU
- VESC Express
- Compatible with VESC firmware 6.06+

## Installation

### 1. Download the SDK
```bash
git clone https://github.com/waas/vesc_can_sdk.git
```

### 2. Copy SDK Files to Arduino Project
Copy the following files to your Arduino project directory:
```
your_arduino_project/
├── vesc_can_sdk.c
├── vesc_buffer.c
├── vesc_crc.c
├── include/
│   ├── vesc_can_sdk.h
│   ├── vesc_buffer.h
│   ├── vesc_crc.h
│   └── vesc_version.h
└── your_arduino_sketch.ino
```

### 3. Install Required Libraries
Install the appropriate CAN library for your hardware:

**For Arduino Giga R1:**
```cpp
// Built-in CAN library
#include <CAN.h>
```

**For MCP2515:**
```cpp
// Install "CAN_BUS_Shield" library
#include <CAN_BUS_Shield.h>
```

**For ACAN2515:**
```cpp
// Install "ACAN2515" library
#include <ACAN2515.h>
```

## Basic Setup

### 1. Basic Arduino Sketch Structure

```cpp
#include "vesc_can_sdk.h"

// CAN send function for VESC SDK
bool arduino_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    CAN.beginPacket(id);
    CAN.write(data, len);
    return CAN.endPacket();
}

// VESC response callback
void vesc_response_handler(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    switch(command) {
        case COMM_GET_VALUES: {
            vesc_values_t values;
            if (vesc_parse_get_values(data, len, &values)) {
                Serial.print("RPM: "); Serial.println(values.rpm);
                Serial.print("Current: "); Serial.println(values.current_motor);
                Serial.print("Voltage: "); Serial.println(values.v_in);
                Serial.print("Temp: "); Serial.println(values.temp_fet);
            }
        } break;
        
        case COMM_FW_VERSION: {
            vesc_fw_version_t version;
            if (vesc_parse_fw_version(data, len, &version)) {
                Serial.print("VESC Firmware: "); 
                Serial.print(version.major); Serial.print("."); Serial.println(version.minor);
            }
        } break;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("VESC Arduino Integration");
    
    // Initialize CAN
    if (!CAN.begin(500E3)) { // 500 kbps
        Serial.println("Failed to start CAN");
        while (1);
    }
    
    // Initialize VESC SDK
    if (!vesc_can_init(arduino_can_send, 1, 42)) {
        Serial.println("Failed to initialize VESC SDK");
        while (1);
    }
    
    // Set response callback
    vesc_set_response_callback(vesc_response_handler);
    
    Serial.println("VESC SDK initialized successfully");
}

void loop() {
    // Process incoming CAN frames
    if (CAN.parsePacket()) {
        uint32_t id = CAN.packetId();
        uint8_t data[8];
        uint8_t len = CAN.read(data, 8);
        vesc_process_can_frame(id, data, len);
    }
    
    // Send commands periodically
    static unsigned long last_command = 0;
    if (millis() - last_command > 1000) {
        vesc_get_values(1); // Get VESC values every second
        last_command = millis();
    }
}
```

### 2. MCP2515 CAN Module Setup

```cpp
#include <CAN_BUS_Shield.h>
#include "vesc_can_sdk.h"

CAN can(10); // CS pin 10

bool arduino_can_send(uint32_t id, uint8_t *data, uint8_t len) {
    unsigned char len_byte = len;
    unsigned char buf[8];
    memcpy(buf, data, len);
    
    if (can.sendMsgBuf(id, 0, len_byte, buf) == CAN_OK) {
        return true;
    }
    return false;
}

void setup() {
    Serial.begin(115200);
    
    // Initialize MCP2515
    while (can.begin(CAN_500KBPS) != CAN_OK) {
        Serial.println("CAN init failed");
        delay(100);
    }
    
    // Initialize VESC SDK
    vesc_can_init(arduino_can_send, 1, 42);
    vesc_set_response_callback(vesc_response_handler);
}

void loop() {
    // Process CAN messages
    unsigned char len = 0;
    unsigned char buf[8];
    unsigned long canId;
    
    if (can.readMsgBuf(&canId, &len, buf) == CAN_OK) {
        vesc_process_can_frame(canId, buf, len);
    }
}
```

## Motor Control Examples

### 1. Basic Motor Control

```cpp
void basicMotorControl() {
    // Set motor to 50% duty cycle
    vesc_set_duty(1, 0.5f);
    delay(2000);
    
    // Set motor current to 10A
    vesc_set_current(1, 10.0f);
    delay(2000);
    
    // Set motor RPM to 2000
    vesc_set_rpm(1, 2000.0f);
    delay(2000);
    
    // Stop motor
    vesc_set_duty(1, 0.0f);
}
```

### 2. Throttle Control

```cpp
class ThrottleController {
private:
    int throttle_pin;
    float last_throttle;
    
public:
    ThrottleController(int pin) : throttle_pin(pin), last_throttle(0.0f) {
        pinMode(pin, INPUT);
    }
    
    void update() {
        int raw_value = analogRead(throttle_pin);
        float throttle = map(raw_value, 0, 1023, 0.0f, 1.0f);
        
        // Apply deadzone
        if (throttle < 0.05f) throttle = 0.0f;
        
        // Smooth throttle changes
        float smooth_throttle = throttle * 0.8f + last_throttle * 0.2f;
        
        // Set motor duty
        vesc_set_duty(1, smooth_throttle);
        
        last_throttle = smooth_throttle;
    }
    
    void emergencyStop() {
        vesc_set_current_brake(1, 50.0f);
    }
};

ThrottleController throttle(A0);

void loop() {
    throttle.update();
    
    // Emergency stop button
    if (digitalRead(2) == LOW) {
        throttle.emergencyStop();
    }
}
```

### 3. PID Position Control

```cpp
class PositionController {
private:
    float target_position;
    float current_position;
    float kp, ki, kd;
    float integral;
    float last_error;
    
public:
    PositionController(float p, float i, float d) 
        : kp(p), ki(i), kd(d), integral(0), last_error(0) {}
    
    void setTarget(float position) {
        target_position = position;
    }
    
    void update() {
        float error = target_position - current_position;
        integral += error;
        float derivative = error - last_error;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        // Limit output
        if (output > 50.0f) output = 50.0f;
        if (output < -50.0f) output = -50.0f;
        
        vesc_set_current(1, output);
        
        last_error = error;
    }
    
    void updatePosition(float position) {
        current_position = position;
    }
};

PositionController position_controller(2.0f, 0.1f, 0.05f);
```

## Advanced Features

### 1. Motor Parameter Detection

```cpp
void detectMotorParameters() {
    Serial.println("Starting motor detection...");
    
    // Detect motor resistance and inductance
    vesc_detect_motor_r_l(1);
    delay(5000); // Wait for detection to complete
    
    // Detect motor parameters (BLDC)
    vesc_detect_motor_param(1, 5.0f, 1000.0f, 0.1f);
    delay(10000); // Wait for detection to complete
    
    // Detect flux linkage
    vesc_detect_motor_flux_linkage(1, 5.0f, 1000.0f, 0.1f, 0.1f);
    delay(5000);
    
    Serial.println("Motor detection complete");
}

void vesc_response_handler(uint8_t controller_id, uint8_t command, uint8_t *data, uint8_t len) {
    switch(command) {
        case COMM_DETECT_MOTOR_R_L: {
            vesc_motor_rl_response_t response;
            if (vesc_parse_motor_rl_response(data, len, &response)) {
                Serial.print("Resistance: "); Serial.print(response.resistance); Serial.println(" Ω");
                Serial.print("Inductance: "); Serial.print(response.inductance); Serial.println(" μH");
            }
        } break;
        
        case COMM_DETECT_MOTOR_PARAM: {
            vesc_motor_param_response_t response;
            if (vesc_parse_motor_param_response(data, len, &response)) {
                Serial.print("Cycle Int Limit: "); Serial.println(response.cycle_int_limit);
                Serial.print("Coupling K: "); Serial.println(response.coupling_k);
            }
        } break;
    }
}
```

### 2. Temperature Monitoring and Protection

```cpp
class TemperatureProtection {
private:
    float max_temp_fet;
    float max_temp_motor;
    bool protection_active;
    
public:
    TemperatureProtection(float max_fet, float max_motor) 
        : max_temp_fet(max_fet), max_temp_motor(max_motor), protection_active(false) {}
    
    void checkTemperatures(vesc_values_t &values) {
        if (values.temp_fet > max_temp_fet || values.temp_motor > max_temp_motor) {
            if (!protection_active) {
                Serial.println("Temperature protection activated!");
                vesc_set_duty(1, 0.0f); // Stop motor
                protection_active = true;
            }
        } else if (protection_active && 
                   values.temp_fet < max_temp_fet - 10.0f && 
                   values.temp_motor < max_temp_motor - 10.0f) {
            Serial.println("Temperature protection deactivated");
            protection_active = false;
        }
    }
    
    bool isProtectionActive() {
        return protection_active;
    }
};

TemperatureProtection temp_protection(80.0f, 100.0f);
```

### 3. Battery Monitoring

```cpp
class BatteryMonitor {
private:
    float min_voltage;
    float max_voltage;
    float voltage_warning;
    
public:
    BatteryMonitor(float min_v, float max_v, float warning_v) 
        : min_voltage(min_v), max_voltage(max_v), voltage_warning(warning_v) {}
    
    void checkVoltage(vesc_values_t &values) {
        if (values.v_in < min_voltage) {
            Serial.println("CRITICAL: Low battery voltage!");
            vesc_set_current(1, 0.0f); // Stop motor
        } else if (values.v_in < voltage_warning) {
            Serial.print("WARNING: Low battery voltage: "); 
            Serial.println(values.v_in);
        }
        
        // Calculate battery percentage
        float battery_percent = (values.v_in - min_voltage) / (max_voltage - min_voltage) * 100.0f;
        Serial.print("Battery: "); Serial.print(battery_percent); Serial.println("%");
    }
};

BatteryMonitor battery_monitor(20.0f, 50.0f, 25.0f);
```

### 4. Debug and Statistics

```cpp
void setupDebug() {
    // Configure debug output
    vesc_debug_config_t debug_config = {
        .level = VESC_DEBUG_BASIC,
        .categories = VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES | VESC_DEBUG_ERRORS,
        .output_func = NULL, // Use Serial
        .enable_timestamps = true,
        .enable_statistics = true
    };
    
    vesc_debug_configure(&debug_config);
}

void printStatistics() {
    vesc_debug_stats_t stats;
    if (vesc_debug_get_stats(&stats)) {
        Serial.println("=== VESC Statistics ===");
        Serial.print("CAN TX: "); Serial.println(stats.can_tx_count);
        Serial.print("CAN RX: "); Serial.println(stats.can_rx_count);
        Serial.print("Commands: "); Serial.println(stats.command_count);
        Serial.print("Responses: "); Serial.println(stats.response_count);
        Serial.print("Errors: "); Serial.println(stats.error_count);
        Serial.print("CRC Errors: "); Serial.println(stats.crc_error_count);
        Serial.println("=====================");
    }
}
```

## Best Practices

### 1. Use Interrupts for CAN Reception

```cpp
void canISR() {
    // Process CAN frames in ISR
    if (CAN.parsePacket()) {
        uint32_t id = CAN.packetId();
        uint8_t data[8];
        uint8_t len = CAN.read(data, 8);
        vesc_process_can_frame(id, data, len);
    }
}

void setup() {
    // Attach CAN interrupt
    CAN.onReceive(canISR);
}
```

### 2. Implement Communication Timeout

```cpp
class CommunicationMonitor {
private:
    unsigned long last_response;
    unsigned long timeout_ms;
    
public:
    CommunicationMonitor(unsigned long timeout) 
        : last_response(0), timeout_ms(timeout) {}
    
    void update() {
        last_response = millis();
    }
    
    bool isConnected() {
        return (millis() - last_response) < timeout_ms;
    }
    
    void checkConnection() {
        if (!isConnected()) {
            Serial.println("VESC communication lost!");
            // Reinitialize or take safety action
            vesc_can_init(arduino_can_send, 1, 42);
        }
    }
};

CommunicationMonitor comm_monitor(5000); // 5 second timeout
```

### 3. Non-blocking Operations

```cpp
class NonBlockingController {
private:
    unsigned long last_command;
    unsigned long command_interval;
    uint8_t current_command;
    
public:
    NonBlockingController(unsigned long interval) 
        : last_command(0), command_interval(interval), current_command(0) {}
    
    void update() {
        if (millis() - last_command >= command_interval) {
            sendNextCommand();
            last_command = millis();
        }
    }
    
    void sendNextCommand() {
        switch(current_command) {
            case 0:
                vesc_get_values(1);
                break;
            case 1:
                vesc_get_decoded_adc(1);
                break;
            case 2:
                vesc_ping(1);
                break;
        }
        current_command = (current_command + 1) % 3;
    }
};
```

### 4. Safety Features

```cpp
class SafetyController {
private:
    bool emergency_stop_active;
    float max_current;
    float max_rpm;
    
public:
    SafetyController(float max_curr, float max_speed) 
        : emergency_stop_active(false), max_current(max_curr), max_rpm(max_speed) {}
    
    void checkSafety(vesc_values_t &values) {
        // Emergency stop button
        if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
            emergencyStop();
        }
        
        // Overcurrent protection
        if (values.current_motor > max_current) {
            Serial.println("Overcurrent detected!");
            vesc_set_current(1, 0.0f);
        }
        
        // Overspeed protection
        if (values.rpm > max_rpm) {
            Serial.println("Overspeed detected!");
            vesc_set_rpm(1, max_rpm);
        }
    }
    
    void emergencyStop() {
        if (!emergency_stop_active) {
            Serial.println("EMERGENCY STOP ACTIVATED!");
            vesc_set_current_brake(1, 50.0f);
            emergency_stop_active = true;
        }
    }
    
    void resetEmergencyStop() {
        emergency_stop_active = false;
    }
};
```

## Troubleshooting

### Common Issues

1. **No VESC Response**
   - Check CAN wiring and termination
   - Verify VESC controller ID
   - Check CAN baud rate (500 kbps recommended)
   - Ensure VESC is powered and connected

2. **Communication Errors**
   - Check for CAN bus conflicts
   - Verify CAN frame format (extended IDs)
   - Monitor debug output for error messages

3. **Motor Not Responding**
   - Check motor connections
   - Verify VESC configuration
   - Check for fault codes in VESC values

4. **High CPU Usage**
   - Use interrupts for CAN reception
   - Implement non-blocking operations
   - Reduce command frequency

### Debug Commands

```cpp
void debugVESC() {
    // Enable detailed debug output
    vesc_debug_enable(VESC_DEBUG_DETAILED, VESC_DEBUG_ALL);
    
    // Print buffer state
    vesc_debug_print_buffer_state();
    
    // Print statistics
    vesc_debug_print_stats();
    
    // Test communication
    vesc_ping(1);
    delay(100);
    vesc_get_fw_version(1);
    delay(100);
    vesc_get_values(1);
}
```

## Complete Examples

### 1. Electric Vehicle Controller

```cpp
#include "vesc_can_sdk.h"

class EVController {
private:
    int throttle_pin;
    int brake_pin;
    int emergency_pin;
    float max_duty;
    float max_current;
    
public:
    EVController(int throttle, int brake, int emergency) 
        : throttle_pin(throttle), brake_pin(brake), emergency_pin(emergency),
          max_duty(0.8f), max_current(30.0f) {
        pinMode(throttle_pin, INPUT);
        pinMode(brake_pin, INPUT);
        pinMode(emergency_pin, INPUT_PULLUP);
    }
    
    void update() {
        // Emergency stop
        if (digitalRead(emergency_pin) == LOW) {
            emergencyStop();
            return;
        }
        
        // Read inputs
        int throttle_raw = analogRead(throttle_pin);
        int brake_raw = analogRead(brake_pin);
        
        float throttle = map(throttle_raw, 0, 1023, 0.0f, max_duty);
        float brake = map(brake_raw, 0, 1023, 0.0f, max_current);
        
        // Apply controls
        if (brake > 0.1f) {
            vesc_set_current_brake(1, brake);
        } else {
            vesc_set_duty(1, throttle);
        }
    }
    
    void emergencyStop() {
        vesc_set_current_brake(1, 50.0f);
    }
};

EVController ev_controller(A0, A1, 2);

void setup() {
    // Initialize CAN and VESC SDK
    CAN.begin(500E3);
    vesc_can_init(arduino_can_send, 1, 42);
    vesc_set_response_callback(vesc_response_handler);
}

void loop() {
    // Process CAN messages
    if (CAN.parsePacket()) {
        uint32_t id = CAN.packetId();
        uint8_t data[8];
        uint8_t len = CAN.read(data, 8);
        vesc_process_can_frame(id, data, len);
    }
    
    // Update EV controller
    ev_controller.update();
}
```

### 2. Robotic Arm Controller

```cpp
class RoboticArm {
private:
    float target_position;
    float current_position;
    float kp, ki, kd;
    float integral;
    float last_error;
    
public:
    RoboticArm(float p, float i, float d) 
        : kp(p), ki(i), kd(d), integral(0), last_error(0) {}
    
    void setTarget(float position) {
        target_position = position;
    }
    
    void update() {
        float error = target_position - current_position;
        integral += error;
        float derivative = error - last_error;
        
        float output = kp * error + ki * integral + kd * derivative;
        
        // Limit output
        if (output > 20.0f) output = 20.0f;
        if (output < -20.0f) output = -20.0f;
        
        vesc_set_current(1, output);
        
        last_error = error;
    }
    
    void updatePosition(float position) {
        current_position = position;
    }
    
    void home() {
        setTarget(0.0f);
    }
};

RoboticArm arm(2.0f, 0.1f, 0.05f);

void loop() {
    // Process CAN messages
    if (CAN.parsePacket()) {
        uint32_t id = CAN.packetId();
        uint8_t data[8];
        uint8_t len = CAN.read(data, 8);
        vesc_process_can_frame(id, data, len);
    }
    
    // Update arm controller
    arm.update();
    
    // Example: Move to different positions
    static unsigned long last_move = 0;
    if (millis() - last_move > 5000) {
        static int position_index = 0;
        float positions[] = {0.0f, 90.0f, 180.0f, 270.0f};
        arm.setTarget(positions[position_index]);
        position_index = (position_index + 1) % 4;
        last_move = millis();
    }
}
```

## License

This Arduino integration guide is part of the VESC CAN SDK project and is released under the MIT License. See the main LICENSE file for details.

## Contributing

Contributions to improve Arduino integration are welcome! Please feel free to submit pull requests or open issues for bugs and feature requests.

## Support

For support and questions about Arduino integration:
- Check the troubleshooting section above
- Review the debug output and statistics
- Consult the main VESC CAN SDK documentation
- Open an issue on the project repository 