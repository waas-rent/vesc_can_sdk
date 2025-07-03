/*
 * VESC CAN SDK - Buffer Utilities Implementation
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

#include "vesc_buffer.h"
#include <stdbool.h>
#include <math.h>
#include <string.h>

// ============================================================================
// Buffer Append Functions (Little Endian)
// ============================================================================

void vesc_buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 56);
    buffer[(*index)++] = (uint8_t)(number >> 48);
    buffer[(*index)++] = (uint8_t)(number >> 40);
    buffer[(*index)++] = (uint8_t)(number >> 32);
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 56);
    buffer[(*index)++] = (uint8_t)(number >> 48);
    buffer[(*index)++] = (uint8_t)(number >> 40);
    buffer[(*index)++] = (uint8_t)(number >> 32);
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

void vesc_buffer_append_float16(uint8_t *buffer, float number, float scale, int32_t *index) {
    vesc_buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void vesc_buffer_append_float32(uint8_t *buffer, float number, float scale, int32_t *index) {
    vesc_buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

void vesc_buffer_append_double64(uint8_t *buffer, double number, double scale, int32_t *index) {
    vesc_buffer_append_int64(buffer, (int64_t)(number * scale), index);
}

/*
 * Auto-scaled float32 implementation based on VESC firmware
 * See: http://stackoverflow.com/questions/40416682/portable-way-to-serialize-float-as-32-bit-integer
 */
void vesc_buffer_append_float32_auto(uint8_t *buffer, float number, int32_t *index) {
    // Set subnormal numbers to 0 as they are not handled properly
    if (fabsf(number) < 1.5e-38f) {
        number = 0.0f;
    }

    int e = 0;
    float sig = frexpf(number, &e);
    float sig_abs = fabsf(sig);
    uint32_t sig_i = 0;

    if (sig_abs >= 0.5f) {
        sig_i = (uint32_t)((sig_abs - 0.5f) * 2.0f * 8388608.0f);
        e += 126;
    }

    uint32_t res = ((e & 0xFF) << 23) | (sig_i & 0x7FFFFF);
    if (sig < 0) {
        res |= 1U << 31;
    }

    vesc_buffer_append_uint32(buffer, res, index);
}

void vesc_buffer_append_float64_auto(uint8_t *buffer, double number, int32_t *index) {
    float n = (float)number;
    float err = (float)(number - (double)n);
    vesc_buffer_append_float32_auto(buffer, n, index);
    vesc_buffer_append_float32_auto(buffer, err, index);
}

// ============================================================================
// Buffer Get Functions (Little Endian)
// ============================================================================

int16_t vesc_buffer_get_int16(const uint8_t *buffer, int32_t *index) {
    int16_t res = ((uint16_t)buffer[*index]) << 8 |
                  ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

uint16_t vesc_buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
    uint16_t res = ((uint16_t)buffer[*index]) << 8 |
                   ((uint16_t)buffer[*index + 1]);
    *index += 2;
    return res;
}

int32_t vesc_buffer_get_int32(const uint8_t *buffer, int32_t *index) {
    int32_t res = ((uint32_t)buffer[*index]) << 24 |
                  ((uint32_t)buffer[*index + 1]) << 16 |
                  ((uint32_t)buffer[*index + 2]) << 8 |
                  ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

uint32_t vesc_buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
    uint32_t res = ((uint32_t)buffer[*index]) << 24 |
                   ((uint32_t)buffer[*index + 1]) << 16 |
                   ((uint32_t)buffer[*index + 2]) << 8 |
                   ((uint32_t)buffer[*index + 3]);
    *index += 4;
    return res;
}

int64_t vesc_buffer_get_int64(const uint8_t *buffer, int32_t *index) {
    int64_t res = ((uint64_t)buffer[*index]) << 56 |
                  ((uint64_t)buffer[*index + 1]) << 48 |
                  ((uint64_t)buffer[*index + 2]) << 40 |
                  ((uint64_t)buffer[*index + 3]) << 32 |
                  ((uint64_t)buffer[*index + 4]) << 24 |
                  ((uint64_t)buffer[*index + 5]) << 16 |
                  ((uint64_t)buffer[*index + 6]) << 8 |
                  ((uint64_t)buffer[*index + 7]);
    *index += 8;
    return res;
}

uint64_t vesc_buffer_get_uint64(const uint8_t *buffer, int32_t *index) {
    uint64_t res = ((uint64_t)buffer[*index]) << 56 |
                   ((uint64_t)buffer[*index + 1]) << 48 |
                   ((uint64_t)buffer[*index + 2]) << 40 |
                   ((uint64_t)buffer[*index + 3]) << 32 |
                   ((uint64_t)buffer[*index + 4]) << 24 |
                   ((uint64_t)buffer[*index + 5]) << 16 |
                   ((uint64_t)buffer[*index + 6]) << 8 |
                   ((uint64_t)buffer[*index + 7]);
    *index += 8;
    return res;
}

float vesc_buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)vesc_buffer_get_int16(buffer, index) / scale;
}

float vesc_buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)vesc_buffer_get_int32(buffer, index) / scale;
}

double vesc_buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index) {
    return (double)vesc_buffer_get_int64(buffer, index) / scale;
}

float vesc_buffer_get_float32_auto(const uint8_t *buffer, int32_t *index) {
    uint32_t res = vesc_buffer_get_uint32(buffer, index);

    int e = (res >> 23) & 0xFF;
    uint32_t sig_i = res & 0x7FFFFF;
    bool neg = res & (1U << 31);

    float sig = 0.0f;
    if (e != 0 || sig_i != 0) {
        sig = (float)sig_i / (8388608.0f * 2.0f) + 0.5f;
        e -= 126;
    }

    if (neg) {
        sig = -sig;
    }

    return ldexpf(sig, e);
}

double vesc_buffer_get_float64_auto(const uint8_t *buffer, int32_t *index) {
    double n = vesc_buffer_get_float32_auto(buffer, index);
    double err = vesc_buffer_get_float32_auto(buffer, index);
    return n + err;
} 