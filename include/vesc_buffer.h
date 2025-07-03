/*
 * VESC CAN SDK - Buffer Utilities
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

#ifndef VESC_BUFFER_H
#define VESC_BUFFER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Buffer Append Functions (Little Endian)
// ============================================================================

/**
 * Append 16-bit integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index);

/**
 * Append 16-bit unsigned integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index);

/**
 * Append 32-bit integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index);

/**
 * Append 32-bit unsigned integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index);

/**
 * Append 64-bit integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index);

/**
 * Append 64-bit unsigned integer to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index);

/**
 * Append 16-bit float to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index);

/**
 * Append 32-bit float to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index);

/**
 * Append 64-bit double to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_double64(uint8_t *buffer, double number, double scale, int32_t *index);

/**
 * Append auto-scaled 32-bit float to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index);

/**
 * Append auto-scaled 64-bit double to buffer
 * 
 * @param buffer Target buffer
 * @param number Number to append
 * @param index Current buffer index (will be incremented)
 */
void vesc_buffer_append_float64_auto(uint8_t *buffer, double number, int32_t *index);

// ============================================================================
// Buffer Get Functions (Little Endian)
// ============================================================================

/**
 * Get 16-bit integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 16-bit integer
 */
int16_t vesc_buffer_get_int16(const uint8_t *buffer, int32_t *index);

/**
 * Get 16-bit unsigned integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 16-bit unsigned integer
 */
uint16_t vesc_buffer_get_uint16(const uint8_t *buffer, int32_t *index);

/**
 * Get 32-bit integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 32-bit integer
 */
int32_t vesc_buffer_get_int32(const uint8_t *buffer, int32_t *index);

/**
 * Get 32-bit unsigned integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 32-bit unsigned integer
 */
uint32_t vesc_buffer_get_uint32(const uint8_t *buffer, int32_t *index);

/**
 * Get 64-bit integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 64-bit integer
 */
int64_t vesc_buffer_get_int64(const uint8_t *buffer, int32_t *index);

/**
 * Get 64-bit unsigned integer from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return 64-bit unsigned integer
 */
uint64_t vesc_buffer_get_uint64(const uint8_t *buffer, int32_t *index);

/**
 * Get 16-bit float from buffer
 * 
 * @param buffer Source buffer
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 * @return Float value
 */
float vesc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);

/**
 * Get 32-bit float from buffer
 * 
 * @param buffer Source buffer
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 * @return Float value
 */
float vesc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);

/**
 * Get 64-bit double from buffer
 * 
 * @param buffer Source buffer
 * @param scale Scale factor
 * @param index Current buffer index (will be incremented)
 * @return Double value
 */
double vesc_buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index);

/**
 * Get auto-scaled 32-bit float from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return Float value
 */
float vesc_buffer_get_float32_auto(const uint8_t *buffer, int32_t *index);

/**
 * Get auto-scaled 64-bit double from buffer
 * 
 * @param buffer Source buffer
 * @param index Current buffer index (will be incremented)
 * @return Double value
 */
double vesc_buffer_get_float64_auto(const uint8_t *buffer, int32_t *index);

#ifdef __cplusplus
}
#endif

#endif // VESC_BUFFER_H 