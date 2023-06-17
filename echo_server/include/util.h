/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define UART_REG(x) ((volatile uint32_t *)(UART_BASE + (x)))

// IMX8MM-evk
// #define UART_BASE 0x5000000 //0x30890000 in hardware on imx8mm. 
// #define STAT 0x98
// #define TRANSMIT 0x40
// #define STAT_TDRE (1 << 14)

// TX2
#define UART_BASE 0x5000000 //0x03100000 in hardware on tx2. 
#define STAT 0x14
#define TRANSMIT 0x00
#define STAT_TDRE (1 << 5)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#ifdef __GNUC__
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x)   (!!(x))
#define unlikely(x) (!!(x))
#endif

static void
putC(uint8_t ch)
{
    while (!(*UART_REG(STAT) & STAT_TDRE)) { }
    *UART_REG(TRANSMIT) = ch;
}

static void
print(const char *s)
{
    while (*s) {
        putC(*s);
        s++;
    }
}

static char
hexchar(unsigned int v)
{
    return v < 10 ? '0' + v : ('a' - 10) + v;
}

static void
puthex64(uint64_t val)
{
    char buffer[16 + 3];
    buffer[0] = '0';
    buffer[1] = 'x';
    buffer[16 + 3 - 1] = 0;
    for (unsigned i = 16 + 1; i > 1; i--) {
        buffer[i] = hexchar(val & 0xf);
        val >>= 4;
    }
    print(buffer);
}

static int wait_for_bit_le32(volatile void *addr, uint32_t mask, bool desired_state, uint32_t timeout, bool clearbit)
{
    uint32_t val;

    // not sure what to do with clearbit
    (void) clearbit; 

    while (timeout--) {
        val = *((uint32_t *)addr);  // Read the 32-bit value from the address
        if ((val & mask) == desired_state)  // Check if the desired bit is set
            return 0;
    }

    return 1;
}