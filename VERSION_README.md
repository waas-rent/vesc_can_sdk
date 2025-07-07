# VESC CAN SDK Version System

This document describes the version system implemented in the VESC CAN SDK.

## Overview

The VESC CAN SDK includes a comprehensive version system that provides:

- **Compile-time version information** via macros
- **Runtime version information** via functions
- **Automatic Git integration** for build tracking
- **Version comparison utilities** for compatibility checking

## Version Information

### Version Format

The SDK uses semantic versioning (SemVer) with the format: `MAJOR.MINOR.PATCH`

- **MAJOR**: Incompatible API changes
- **MINOR**: Backward-compatible new functionality
- **PATCH**: Backward-compatible bug fixes

### Current Version

The current version is defined in `include/vesc_version.h`:

```c
#define VESC_SDK_VERSION_MAJOR    1
#define VESC_SDK_VERSION_MINOR    0
#define VESC_SDK_VERSION_PATCH    0
```

## Usage

### Compile-time Access

You can access version information at compile time using macros:

```c
#include "vesc_version.h"

// Version numbers
int major = VESC_SDK_VERSION_MAJOR;    // 1
int minor = VESC_SDK_VERSION_MINOR;    // 0
int patch = VESC_SDK_VERSION_PATCH;    // 0

// Version string
const char* version = VESC_SDK_VERSION_STRING;  // "1.0.0"

// Version number for comparison
uint32_t version_num = VESC_SDK_VERSION_NUMBER; // 10000

// Build information
const char* build_date = VESC_SDK_BUILD_DATE;   // "Jan 15 2025"
const char* build_time = VESC_SDK_BUILD_TIME;   // "14:30:25"
```

### Runtime Access

You can access version information at runtime using functions:

```c
#include "vesc_can_sdk.h"

// Get complete version information
const vesc_sdk_version_t* version = vesc_sdk_get_version();
printf("Version: %s\n", version->version_string);
printf("Build Date: %s\n", version->build_date);
printf("Git Hash: %s\n", version->git_hash);

// Get version string
const char* version_str = vesc_sdk_get_version_string();

// Get version number for comparison
uint32_t version_num = vesc_sdk_get_version_number();

// Check if version is at least a certain version
if (vesc_sdk_version_at_least(1, 0, 0)) {
    printf("SDK version is at least 1.0.0\n");
}

// Print complete version information
vesc_sdk_print_version();
```

### Version Structure

The `vesc_sdk_version_t` structure contains:

```c
typedef struct {
    uint8_t major;                    // Major version number
    uint8_t minor;                    // Minor version number
    uint8_t patch;                    // Patch version number
    uint32_t version_number;          // Combined version number (e.g., 1.0.0 = 10000)
    const char *version_string;       // Version string (e.g., "1.0.0")
    const char *build_date;           // Build date string
    const char *build_time;           // Build time string
    const char *git_hash;             // Git commit hash
    const char *git_branch;           // Git branch name
} vesc_sdk_version_t;
```

## Build System Integration

### CMake Integration

The CMake build system automatically:

1. **Extracts Git information** during configuration
2. **Defines compile-time macros** for version information
3. **Embeds version data** into the compiled library

The main `CMakeLists.txt` includes:

```cmake
# Get Git information
get_git_info()

# Add compile definitions
add_compile_definitions(
    VESC_SDK_GIT_HASH="${GIT_HASH}"
    VESC_SDK_GIT_BRANCH="${GIT_BRANCH}"
)
```

### Git Information

If the project is built from a Git repository, the build system will automatically extract:

- **Git commit hash** (short form)
- **Git branch name**

If Git information is not available, these will be set to "unknown".

## Version Comparison

### Version Number Format

Version numbers are encoded as: `(MAJOR * 10000) + (MINOR * 100) + PATCH`

Examples:
- Version 1.0.0 = 10000
- Version 1.2.3 = 10203
- Version 2.1.0 = 20100

### Comparison Functions

```c
// Check if SDK version is at least the specified version
bool vesc_sdk_version_at_least(uint8_t major, uint8_t minor, uint8_t patch);

// Examples
if (vesc_sdk_version_at_least(1, 0, 0)) {
    // SDK version is 1.0.0 or higher
}

if (vesc_sdk_version_at_least(1, 2, 0)) {
    // SDK version is 1.2.0 or higher
}
```

## Testing

### Test Program

A test program is provided to demonstrate the version functionality:

```bash
# Build the test program
mkdir build && cd build
cmake ../test_version
make

# Run the test
./bin/vesc_version_test
```

The test program will output:

```
VESC CAN SDK Version Test Program
==================================

=== VESC CAN SDK Version Information ===
Version: 1.0.0
Build Date: Jan 15 2025
Build Time: 14:30:25
Git Hash: a1b2c3d
Git Branch: main
========================================

Programmatic access:
  Major: 1
  Minor: 0
  Patch: 0
  Version Number: 10000
  Version String: 1.0.0
  Build Date: Jan 15 2025
  Build Time: 14:30:25
  Git Hash: a1b2c3d
  Git Branch: main

Version comparison tests:
  Current version: 1.0.0
  Version number: 10000
  Is at least 0.9.0: true
  Is at least 1.0.0: true
  Is at least 1.1.0: false
  Is at least 2.0.0: false

Macro access:
  VESC_SDK_VERSION_MAJOR: 1
  VESC_SDK_VERSION_MINOR: 0
  VESC_SDK_VERSION_PATCH: 0
  VESC_SDK_VERSION_STRING: 1.0.0
  VESC_SDK_VERSION_NUMBER: 10000
  VESC_SDK_BUILD_DATE: Jan 15 2025
  VESC_SDK_BUILD_TIME: 14:30:25

Version test completed successfully!
```

## Updating Versions

### For New Releases

1. **Update version numbers** in `include/vesc_version.h`:
   ```c
   #define VESC_SDK_VERSION_MAJOR    1
   #define VESC_SDK_VERSION_MINOR    1  // Increment for new features
   #define VESC_SDK_VERSION_PATCH    0
   ```

2. **Commit the changes** to Git:
   ```bash
   git add include/vesc_version.h
   git commit -m "Bump version to 1.1.0"
   git tag v1.1.0
   ```

3. **Build the library** - version information will be automatically embedded

### Version Guidelines

- **MAJOR**: Increment for breaking API changes
- **MINOR**: Increment for new features (backward compatible)
- **PATCH**: Increment for bug fixes (backward compatible)

## Integration with Applications

### Checking SDK Version

Applications can check the SDK version at runtime:

```c
#include "vesc_can_sdk.h"

// Check minimum required version
if (!vesc_sdk_version_at_least(1, 0, 0)) {
    fprintf(stderr, "Error: VESC CAN SDK version 1.0.0 or higher required\n");
    return -1;
}

// Print version information for debugging
vesc_sdk_print_version();
```

### Conditional Compilation

Applications can use version macros for conditional compilation:

```c
#include "vesc_version.h"

#if VESC_SDK_VERSION_NUMBER >= 10100
    // Use features available in version 1.1.0+
    vesc_new_feature();
#else
    // Fallback for older versions
    vesc_legacy_feature();
#endif
```

## Benefits

The version system provides several benefits:

1. **Runtime version checking** for compatibility
2. **Build tracking** with Git information
3. **Debugging support** with detailed version information
4. **Conditional compilation** based on SDK version
5. **Automatic version embedding** during build
6. **Standardized version format** following SemVer

This ensures that applications can reliably check SDK compatibility and that developers have access to detailed version information for debugging and support purposes. 