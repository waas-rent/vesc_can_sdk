/*
 * VESC CAN SDK - Version Example
 * 
 * This example demonstrates how to use the version system in an application.
 * 
 * Copyright (c) 2025 waas AG (waas.rent)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "vesc_can_sdk.h"

int main(void) {
    printf("VESC CAN SDK Version Example\n");
    printf("============================\n\n");
    
    // Example 1: Check minimum required version
    printf("1. Checking minimum required version:\n");
    if (!vesc_sdk_version_at_least(1, 0, 0)) {
        fprintf(stderr, "Error: VESC CAN SDK version 1.0.0 or higher required\n");
        return -1;
    }
    printf("   ✓ SDK version is compatible\n\n");
    
    // Example 2: Print version information for debugging
    printf("2. SDK Version Information:\n");
    vesc_sdk_print_version();
    
    // Example 3: Access version information programmatically
    printf("3. Programmatic version access:\n");
    const vesc_sdk_version_t *version = vesc_sdk_get_version();
    printf("   Version: %s\n", version->version_string);
    printf("   Build: %s at %s\n", version->build_date, version->build_time);
    printf("   Git: %s on %s\n", version->git_hash, version->git_branch);
    printf("\n");
    
    // Example 4: Conditional compilation example
    printf("4. Conditional compilation example:\n");
    
    // This would be done at compile time in a real application
    printf("   Compile-time version check:\n");
    printf("   VESC_SDK_VERSION_MAJOR: %d\n", VESC_SDK_VERSION_MAJOR);
    printf("   VESC_SDK_VERSION_MINOR: %d\n", VESC_SDK_VERSION_MINOR);
    printf("   VESC_SDK_VERSION_PATCH: %d\n", VESC_SDK_VERSION_PATCH);
    printf("   VESC_SDK_VERSION_NUMBER: %u\n", VESC_SDK_VERSION_NUMBER);
    printf("\n");
    
    // Example 5: Version comparison for feature detection
    printf("5. Feature detection example:\n");
    if (vesc_sdk_version_at_least(1, 1, 0)) {
        printf("   ✓ Advanced features available (requires 1.1.0+)\n");
        // vesc_advanced_feature();
    } else {
        printf("   ⚠ Advanced features not available (requires 1.1.0+)\n");
        printf("   Using basic features only\n");
        // vesc_basic_feature();
    }
    
    if (vesc_sdk_version_at_least(2, 0, 0)) {
        printf("   ✓ Experimental features available (requires 2.0.0+)\n");
        // vesc_experimental_feature();
    } else {
        printf("   ⚠ Experimental features not available (requires 2.0.0+)\n");
    }
    printf("\n");
    
    // Example 6: Logging version information
    printf("6. Logging example:\n");
    printf("   Application started with VESC CAN SDK %s\n", vesc_sdk_get_version_string());
    printf("   Build: %s %s\n", version->build_date, version->build_time);
    printf("   Git: %s (%s)\n", version->git_hash, version->git_branch);
    printf("\n");
    
    printf("Version example completed successfully!\n");
    return 0;
} 