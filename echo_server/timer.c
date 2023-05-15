/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* 
 * This is a basic single client timer driver intended purely for 
 * use by the lwIP stack for TCP. 
 */

#include "timer.h"
#include "echo.h"

// base register for timer
uintptr_t gpt_regs;

#define BIT(n) (1ul<<(n))

#define TMR0_OFFSET         0x10000
#define TMR1_OFFSET         0x20000
#define TMR2_OFFSET         0x30000
#define TMR3_OFFSET         0x40000
#define TMR4_OFFSET         0x50000
#define TMR5_OFFSET         0x60000
#define TMR6_OFFSET         0x70000
#define TMR7_OFFSET         0x80000
#define TMR8_OFFSET         0x90000
#define TMR9_OFFSET         0xA0000
#define TMRUS_OFFSET        0x8
#define TMR_SHARED_OFFSET   0

#define TMRUS_USEC_CFG_DEFAULT   11


struct tmr_shared_map {
    uint32_t TKETSC0; // Value of local TSC counter TSC[31:0], synchronized across SOC
    uint32_t TKETSC1; // Value of local TSC counter {8’h0, TSC[55:32]}, synchronized across SOC
    uint32_t TKEUSEC; // This is the same as the tmrus_map.cntr_1us below
    uint32_t TKEOSC;  // Value of local OSC counter, not synchronized across SOC
    uint32_t TKECR;   // Control register
    uint32_t pad[(0x100 / 4) - 5]; // Have to pad to 0x100
    uint32_t TKEIE[10]; // Routing of shared interrupt {i}, a bit mask indicating which
    // of the internal interrupts is propagated to external interrupt {i},
    uint32_t pad1[(0x100 / 4) - 10]; // Have to pad to 0x200
    uint32_t TKEIV;   // Which shared interrupts are currently asserted
    uint32_t TKEIR;   // Which internal interrupts are currently asserted, before applying the
    // TKEIE masks
} PACKED;
_Static_assert(sizeof(struct tmr_shared_map) == 0x208, "struct tmr_shared_map has incorrect layout");

/* A free-running read-only counter changes once very microsecond.
   On TX2 this is just a register in the shared Timer region. */
struct tmrus_map {
    uint32_t cntr_1us; /* offset 0 */
};

// static uint32_t overflow_count = 0;

struct tmr_map {
    uint32_t cr;    /* config register */
    uint32_t sr;    /* status register */
    uint32_t cssr;    /* clock source selection register */
};

static struct tmr_map *gpt;

#define LWIP_TICK_MS 10
#define NS_IN_MS 1000000ULL
#define NS_IN_US 1000ULL

/* 0-28 bits are for value, n + 1 trigger mode */
#define PVT_VAL_MASK        0x1fffffff
#define PVT_VAL_BITS        29

int timers_initialised = 0;

static uint64_t get_ticks(void) {
    /* FIXME: If an overflow interrupt happens in the middle here we are in trouble */
    // uint64_t overflow = overflow_count;
    // uint32_t sr = gpt->sr;
    // print("SR reg addr = ");
    // puthex64(&gpt->sr);
    // print("\n");

    // print("sr = ");
    // puthex64(sr);
    // print("\n");

    struct tmrus_map *tmrus_map = (void *)(gpt_regs + TMRUS_OFFSET);

    // print("cntr_1us = ");
    uint64_t num = tmrus_map->cntr_1us * NS_IN_US;
    // puthex64(num);
    // print("\n");

    // TODO address overflow since usec counter is only 32 bits
    return (uint64_t) tmrus_map->cntr_1us;

    // struct tmr_shared_map *tmr_shared_map = (volatile void *)( gpt_regs + TMR_SHARED_OFFSET);
    // uint32_t lo, hi, ss;

    // hi = tmr_shared_map->TKETSC1;

	// /*
	//  * The 56-bit value of the TSC is spread across two registers that are
	//  * not synchronized. In order to read them atomically, ensure that the
	//  * high 24 bits match before and after reading the low 32 bits.
	//  */
	// do {
	// 	/* snapshot the high 24 bits */
	// 	ss = hi;

    //     lo = tmr_shared_map->TKETSC0;
    //     hi = tmr_shared_map->TKETSC1;
	// } while (hi != ss);

    // uint64_t tsc_count = (uint64_t) hi << 32 | lo;

    // print("tsc = ");
    // puthex64(tsc_count);
    // print("\n");

    // return sr & PVT_VAL_MASK;

    // uint32_t cnt = gpt[CNT];
    // uint32_t sr2 = gpt->sr;
    // if ((sr2 & (1 << 5)) && (!(sr1 & (1 << 5)))) {
    //     /* rolled-over during - 64-bit time must be the overflow */
    //     cnt = gpt[CNT];
    //     overflow++;
    // }
    // return (overflow << 32) | cnt;
}

u32_t sys_now(void)
{
    if (!timers_initialised) {
        /* lwip_init() will call this when initialising its own timers,
         * but the timer is not set up at this point so just return 0 */
        return -1;
    } else {
        uint64_t time_now = get_ticks();
        return time_now;
    }
}

void irq(sel4cp_channel ch)
{
    // uint32_t sr = gpt->sr;
    // gpt->sr = sr;

    // if (sr & (1 << 5)) {
    //     overflow_count++;
    // }

    // if (sr & 1) {
    //     gpt[IR] &= ~1;
    //     uint64_t abs_timeout = get_ticks() + (LWIP_TICK_MS * NS_IN_MS);
    //     gpt[OCR1] = abs_timeout;
    //     gpt[IR] |= 1;
        // sys_check_timeouts();
    // }
}

void gpt_init(void)
{
    // check shared usec counter
    struct tmrus_map *tmrus_map = (void *)(gpt_regs + TMRUS_OFFSET);

    print("cntr_1us = ");
    uint64_t num = tmrus_map->cntr_1us * NS_IN_US;
    puthex64(num);
    print("\n");



    // use TMR1 as gpt
    gpt = (volatile struct tmr_map *) (gpt_regs + TMR1_OFFSET);
    print("gpt = ");
    puthex64(gpt);
    print("\n");

    print("gpt CR = ");
    puthex64(gpt->cr);
    print("\n");

    gpt->cr = 0;

    // configure shared registers
    struct tmr_shared_map *tmr_shared_map = (volatile void *)( gpt_regs + TMR_SHARED_OFFSET);
    tmr_shared_map->TKEIE[1] = BIT(1);

    // configure usec counter
    // struct tmrus_map *tmrus_map = (void *)(gpt_regsreg_base + TMRUS_OFFSET);

    // Configure TMR1 (gpt)
    gpt->sr = (1 << 30); // clear interrupt

    uint32_t cr = (
        (1 << 31)  | // enable countdown
        (1 << 30) // | // Periodic
        // (1) // Enable
    );


    print("gpt CR addr = ");
    puthex64(&gpt->cr);
    print("\n");

    gpt->cr |= cr;

    print("gpt CR = ");
    puthex64(gpt->cr);
    print("\n");


    // gpt[IR] = ( 
    //     (1 << 5) // rollover interrupt
    // );

    // gpt[CSSR] = 0; // 1 microsecond downcount

    // set a timer! 
    // uint64_t abs_timeout = get_ticks() + (LWIP_TICK_MS * NS_IN_MS);
    // gpt[OCR1] = abs_timeout;
    // gpt[IR] |= 1;

    timers_initialised = 1;
}