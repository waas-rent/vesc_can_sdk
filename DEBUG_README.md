# VESC CAN SDK Debugging Guide

This document describes the comprehensive debugging functionality added to the VESC CAN SDK to help developers monitor communication, troubleshoot issues, and understand the VESC protocol.

## Overview

The debugging system provides detailed, human-readable information about the communication flow between your application and VESC motor controllers. It can be enabled/disabled at runtime and provides multiple levels of detail to suit different debugging needs.

## Features

### Debug Levels
- **VESC_DEBUG_NONE (0)** - No debugging output
- **VESC_DEBUG_BASIC (1)** - Essential information (commands, responses)
- **VESC_DEBUG_DETAILED (2)** - Detailed packet information with hex dumps
- **VESC_DEBUG_VERBOSE (3)** - Complete packet dumps and timing information

### Debug Categories
- **VESC_DEBUG_CAN** - Raw CAN frame transmission/reception
- **VESC_DEBUG_COMMANDS** - Motor control and configuration commands
- **VESC_DEBUG_RESPONSES** - Response parsing and validation
- **VESC_DEBUG_BUFFERS** - RX buffer management for fragmented packets
- **VESC_DEBUG_ERRORS** - Error conditions and validation failures
- **VESC_DEBUG_PERFORMANCE** - Timing and statistics

### Statistics Collection
- CAN transmission/reception counts
- Command and response counts
- Error counts (CRC errors, buffer overflows)
- Byte transfer statistics
- Performance metrics (bytes/sec, messages/sec)

## Quick Start

### Basic Usage

```c
#include "vesc_can_sdk.h"

// Enable basic debugging for commands and responses
vesc_debug_enable(VESC_DEBUG_BASIC, VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES);

// Your VESC operations here...
vesc_set_current(1, 5.0f);
vesc_get_values(1);

// Disable debugging when done
vesc_debug_disable();
```

### Advanced Configuration

```c
// Configure detailed debugging with custom output
vesc_debug_config_t config = {
    .level = VESC_DEBUG_DETAILED,
    .categories = VESC_DEBUG_CAN | VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES,
    .output_func = my_custom_output_function,
    .enable_timestamps = true,
    .enable_statistics = true
};
vesc_debug_configure(&config);
```

## API Reference

### Core Functions

#### `vesc_debug_enable(level, categories)`
Enable debugging with basic configuration.

**Parameters:**
- `level` - Debug level (0-3)
- `categories` - Bit flags for enabled categories

**Returns:** `true` on success, `false` on failure

#### `vesc_debug_configure(config)`
Configure debugging with full options.

**Parameters:**
- `config` - Pointer to debug configuration structure

**Returns:** `true` on success, `false` on failure

#### `vesc_debug_disable()`
Disable debugging.

#### `vesc_debug_set_output_func(output_func)`
Set custom debug output function.

**Parameters:**
- `output_func` - Custom output function (NULL = use printf)

#### `vesc_debug_get_stats(stats)`
Get current debug statistics.

**Parameters:**
- `stats` - Pointer to statistics structure

**Returns:** `true` on success, `false` on failure

#### `vesc_debug_reset_stats()`
Reset debug statistics to zero.

#### `vesc_debug_print_stats()`
Print formatted debug statistics to console.

### Configuration Structure

```c
typedef struct {
    uint8_t level;                    // Debug level (0-3)
    uint16_t categories;              // Enabled categories (bit flags)
    vesc_debug_output_func_t output_func; // Custom output function
    bool enable_timestamps;           // Include timestamps
    bool enable_statistics;           // Collect statistics
} vesc_debug_config_t;
```

### Statistics Structure

```c
typedef struct {
    uint32_t can_tx_count;           // CAN transmit count
    uint32_t can_rx_count;           // CAN receive count
    uint32_t command_count;          // Commands sent
    uint32_t response_count;         // Responses received
    uint32_t error_count;            // Total error count
    uint32_t crc_error_count;        // CRC error count
    uint32_t buffer_overflow_count;  // Buffer overflow count
    uint64_t total_tx_bytes;         // Total bytes transmitted
    uint64_t total_rx_bytes;         // Total bytes received
} vesc_debug_stats_t;
```

## Debug Output Examples

### Basic Command Output
```
[14:30:15] Command: VESC#1 SET_CURRENT 5.00A
[14:30:15] Sending Command: VESC#1 SET_CURRENT (payload=4 bytes, total=7 bytes)
[14:30:15] CAN TX: VESC#1 SET_CURRENT (4 bytes)
```

### Detailed CAN Output
```
[14:30:15] CAN TX: ID=0x001 (VESC#1, SET_CURRENT), Len=4
  Data: 88 13 00 00
[14:30:15] CAN RX: ID=0x001 (VESC#1, STATUS), Len=8
  Data: 00 00 00 00 00 00 00 00
```

### Response Output
```
[14:30:15] Response: VESC#1 GET_VALUES (50 bytes)
  Response Data: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
                00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
                00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
                00 00
```

### Error Output
```
[14:30:15] CRC Error: VESC#1 (calc=0x1234, rx=0x5678)
[14:30:15] Buffer Error: VESC#1 FILL_RX_BUFFER invalid offset=300 or buffer overflow
```

### Statistics Output
```
=== VESC Debug Statistics ===
Uptime: 120 seconds
CAN Transmissions: 45
CAN Receptions: 42
Commands Sent: 15
Responses Received: 12
Errors: 3
CRC Errors: 2
Buffer Overflows: 1
Total TX Bytes: 180
Total RX Bytes: 504
TX Rate: 1.5 bytes/sec
RX Rate: 4.2 bytes/sec
CAN TX Rate: 0.4 msgs/sec
CAN RX Rate: 0.4 msgs/sec
=============================
```

## Use Cases

### 1. Protocol Learning
Enable detailed debugging to understand the VESC protocol:
```c
vesc_debug_enable(VESC_DEBUG_DETAILED, VESC_DEBUG_CAN | VESC_DEBUG_COMMANDS);
```

### 2. Troubleshooting Communication Issues
Enable error debugging to identify problems:
```c
vesc_debug_enable(VESC_DEBUG_BASIC, VESC_DEBUG_ERRORS | VESC_DEBUG_CAN);
```

### 3. Performance Monitoring
Enable statistics to monitor communication performance:
```c
vesc_debug_config_t config = {
    .level = VESC_DEBUG_BASIC,
    .categories = VESC_DEBUG_PERFORMANCE,
    .enable_statistics = true
};
vesc_debug_configure(&config);
```

### 4. Custom Logging
Use custom output function to integrate with existing logging systems:
```c
void my_logger(const char *message) {
    syslog(LOG_DEBUG, "VESC: %s", message);
}

vesc_debug_set_output_func(my_logger);
```

### 5. File Logging
Log debug output to a file:
```c
void file_logger(const char *message) {
    FILE *f = fopen("vesc_debug.log", "a");
    if (f) {
        fprintf(f, "%s", message);
        fclose(f);
    }
}

vesc_debug_set_output_func(file_logger);
```

## Performance Impact

- **Disabled**: Zero performance impact
- **Basic Level**: Minimal impact (< 1% CPU overhead)
- **Detailed Level**: Low impact (1-5% CPU overhead)
- **Verbose Level**: Moderate impact (5-15% CPU overhead)

The actual impact depends on the communication frequency and enabled categories.

## Best Practices

1. **Enable only needed categories** - Don't enable all categories unless necessary
2. **Use appropriate debug levels** - Use BASIC for production debugging, DETAILED for development
3. **Monitor statistics** - Use statistics to identify performance bottlenecks
4. **Custom output functions** - Use custom output for integration with existing systems
5. **Disable when not needed** - Always disable debugging in production unless actively troubleshooting

## Example Programs

See `examples/debug_example.c` for a complete demonstration of all debugging features.

## Troubleshooting

### Common Issues

1. **No debug output**: Ensure debugging is enabled and appropriate categories are selected
2. **Performance issues**: Reduce debug level or disable unnecessary categories
3. **Memory issues**: Debug output uses stack buffers; ensure sufficient stack space
4. **File logging errors**: Check file permissions and disk space

### Debug Categories Reference

| Category | Description | Use Case |
|----------|-------------|----------|
| VESC_DEBUG_CAN | Raw CAN frames | Protocol analysis, hardware debugging |
| VESC_DEBUG_COMMANDS | Command sending | Application logic verification |
| VESC_DEBUG_RESPONSES | Response parsing | Data validation, protocol verification |
| VESC_DEBUG_BUFFERS | Buffer management | Fragmented packet debugging |
| VESC_DEBUG_ERRORS | Error conditions | Problem identification |
| VESC_DEBUG_PERFORMANCE | Statistics | Performance monitoring |

## Integration Notes

The debugging system is designed to be non-intrusive and can be easily integrated into existing applications. All debug functions are optional and can be conditionally compiled out if needed.

For embedded systems with limited resources, consider:
- Using only BASIC debug level
- Enabling only essential categories
- Implementing custom output functions that minimize overhead
- Disabling timestamps and statistics if not needed 