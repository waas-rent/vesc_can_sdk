# VESC CAN SDK Examples

This directory contains example programs demonstrating how to use the VESC CAN SDK.

## Examples

### 1. `version_example.c`
A simple example that demonstrates the version system functionality:
- Version checking and compatibility
- Accessing version information programmatically
- Conditional compilation based on version
- Logging version information

### 2. `simple_example.c`
A comprehensive example that demonstrates the core SDK functionality:
- SDK initialization and configuration
- Basic commands (PING, GET_FW_VERSION, GET_VALUES)
- Motor control commands (SET_DUTY, SET_CURRENT, SET_RPM)
- Response handling with callbacks
- Debug output and statistics
- Multiple controller communication

## Building the Examples

### Prerequisites
- GCC compiler
- Make utility
- VESC CAN SDK source files (in parent directory)

### Compilation
```bash
# Build all examples
make

# Build specific example
make version_example
make simple_example

# Clean build files
make clean
```

### Running the Examples
```bash
# Run version example
make run-version
# or
./version_example

# Run simple example
make run-simple
# or
./simple_example
```

## Example Output

### Version Example
```
VESC CAN SDK Version Example
============================

1. Checking minimum required version:
   ✓ SDK version is compatible

2. SDK Version Information:
=== VESC CAN SDK Version Information ===
Version: 1.0.0
Build Date: 2025-01-27
Build Time: 12:34:56
Git Hash: abc123def456
Git Branch: main
========================================

3. Programmatic version access:
   Version: 1.0.0
   Build: 2025-01-27 at 12:34:56
   Git: abc123def456 on main
```

### Simple Example
```
VESC CAN SDK - Simple Example
=============================

1. SDK Initialization
==========================================
✓ SDK initialized successfully
  Receiver ID: 0
  Sender ID: 1
✓ Response callback set

2. Debug Configuration
==========================================
✓ Debug output enabled
  Level: BASIC
  Categories: COMMANDS, RESPONSES, CAN

3. Basic Commands
==========================================
Sending PING to VESC#1...
  [MOCK] CAN TX: ID=0x001, Len=0
  [MOCK] Simulating PONG response from VESC#1
  [CALLBACK] VESC#1 response: command=0x01, len=1
    PONG from VESC#1
✓ PING response received
```

## Key Features Demonstrated

### SDK Initialization
```c
// Initialize the SDK with CAN send function and controller IDs
vesc_can_init(mock_can_send, receiver_id, sender_id);

// Set response callback
vesc_set_response_callback(response_callback);
```

### Debug Configuration
```c
vesc_debug_config_t debug_config = {
    .level = VESC_DEBUG_BASIC,
    .categories = VESC_DEBUG_COMMANDS | VESC_DEBUG_RESPONSES | VESC_DEBUG_CAN,
    .enable_timestamps = true,
    .enable_statistics = true
};
vesc_debug_configure(&debug_config);
```

### Motor Control Commands
```c
// Set duty cycle (0.0 to 1.0)
vesc_set_duty(controller_id, 0.5f);

// Set current in amperes
vesc_set_current(controller_id, 5.0f);

// Set RPM
vesc_set_rpm(controller_id, 1000.0f);
```

### Response Handling
```c
static void response_callback(uint8_t controller_id, uint8_t command, 
                            uint8_t *data, uint8_t len) {
    switch (command) {
        case COMM_FW_VERSION:
            // Parse firmware version response
            break;
        case COMM_GET_VALUES:
            // Parse values response
            break;
        // ... handle other commands
    }
}
```

## Notes

- The examples use a mock CAN send function for demonstration purposes
- In a real application, you would replace `mock_can_send` with your actual CAN interface
- The examples include simulated responses to demonstrate the callback system
- Debug output can be configured to show different levels of detail
- All examples are designed to be educational and demonstrate best practices

## Troubleshooting

### Compilation Errors
- Ensure all SDK source files are present in the parent directory
- Check that the include path is correct (`-I../include`)
- Verify that all required header files are available

### Runtime Issues
- The examples use mock functions, so they should run without actual hardware
- If you see timeout messages, this is expected behavior with the mock implementation
- Debug output can be disabled by setting the debug level to `VESC_DEBUG_NONE`

## Next Steps

After running these examples, you can:
1. Integrate the SDK into your own application
2. Replace the mock CAN function with your actual CAN interface
3. Implement your own response handling logic
4. Configure debug output for your specific needs
5. Explore additional SDK features like motor detection and configuration 