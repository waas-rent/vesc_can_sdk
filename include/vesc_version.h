/*
 * VESC CAN SDK - Version Information
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

#ifndef VESC_VERSION_H
#define VESC_VERSION_H

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Version Definitions
// ============================================================================

// SDK Version - Update these when releasing new versions
#define VESC_SDK_VERSION_MAJOR    1
#define VESC_SDK_VERSION_MINOR    0
#define VESC_SDK_VERSION_PATCH    0

// Build information
#define VESC_SDK_BUILD_DATE       __DATE__
#define VESC_SDK_BUILD_TIME       __TIME__

// Version string macros
#define VESC_SDK_VERSION_STRINGIFY(x) #x
#define VESC_SDK_VERSION_TOSTRING(x) VESC_SDK_VERSION_STRINGIFY(x)

#define VESC_SDK_VERSION_STRING \
    VESC_SDK_VERSION_TOSTRING(VESC_SDK_VERSION_MAJOR) "." \
    VESC_SDK_VERSION_TOSTRING(VESC_SDK_VERSION_MINOR) "." \
    VESC_SDK_VERSION_TOSTRING(VESC_SDK_VERSION_PATCH)

// Version number for comparison (e.g., 1.0.0 = 10000)
#define VESC_SDK_VERSION_NUMBER \
    ((VESC_SDK_VERSION_MAJOR * 10000) + (VESC_SDK_VERSION_MINOR * 100) + VESC_SDK_VERSION_PATCH)

// ============================================================================
// Version Information Structure
// ============================================================================

typedef struct {
    uint8_t major;                    // Major version number
    uint8_t minor;                    // Minor version number
    uint8_t patch;                    // Patch version number
    uint32_t version_number;          // Combined version number for comparison
    const char *version_string;       // Version string (e.g., "1.0.0")
    const char *build_date;           // Build date string
    const char *build_time;           // Build time string
    const char *git_hash;             // Git commit hash (if available)
    const char *git_branch;           // Git branch name (if available)
} vesc_sdk_version_t;

// ============================================================================
// Version Functions
// ============================================================================

/**
 * Get the SDK version information
 * 
 * @return Pointer to version information structure
 */
const vesc_sdk_version_t* vesc_sdk_get_version(void);

/**
 * Get the SDK version string
 * 
 * @return Version string (e.g., "1.0.0")
 */
const char* vesc_sdk_get_version_string(void);

/**
 * Get the SDK version number for comparison
 * 
 * @return Version number (e.g., 1.0.0 = 10000)
 */
uint32_t vesc_sdk_get_version_number(void);

/**
 * Check if the SDK version is at least the specified version
 * 
 * @param major Major version number
 * @param minor Minor version number
 * @param patch Patch version number
 * @return true if SDK version is >= specified version, false otherwise
 */
bool vesc_sdk_version_at_least(uint8_t major, uint8_t minor, uint8_t patch);

/**
 * Print SDK version information to stdout
 */
void vesc_sdk_print_version(void);

#ifdef __cplusplus
}
#endif

#endif // VESC_VERSION_H 