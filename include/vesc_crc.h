/*
 * VESC CAN SDK - CRC16 Implementation
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

#ifndef VESC_CRC_H
#define VESC_CRC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Calculate CRC16 for data buffer
 * 
 * @param buf Data buffer
 * @param len Buffer length
 * @return CRC16 value
 */
uint16_t vesc_crc16(const uint8_t *buf, uint32_t len);

/**
 * Calculate CRC16 with initial value
 * 
 * @param buf Data buffer
 * @param len Buffer length
 * @param cksum Initial CRC value
 * @return CRC16 value
 */
uint16_t vesc_crc16_with_init(const uint8_t *buf, uint32_t len, uint16_t cksum);

#ifdef __cplusplus
}
#endif

#endif // VESC_CRC_H 