#include "timer.h"

uintptr_t gpt_regs;

struct ttc_tmr_regs {
    /* Controls prescaler, selects clock input, edge */
    uint32_t clk_ctrl[3];   /* +0x00 */
    /* Enables counter, sets mode of operation, sets up/down
     * counting, enables matching, enables waveform output */
    uint32_t cnt_ctrl[3];   /* +0x0C */
    /* Returns current counter value */
    uint32_t cnt_val[3];    /* +0x18 */
    /* Sets interval value - If interval is enabled, this is the maximum value
     * that the counter will count up to or down from */
    uint32_t interval[3];   /* +0x24 */
    /* Sets match values, total 3 */
    uint32_t match[3][3];   /* +0x30 */
    /* Shows current interrupt status */
    uint32_t int_sts[3];    /* +0x54 */
    /* Enable interrupts */
    uint32_t int_en[3];     /* +0x60 */
    /* Enable event timer, stop timer, sets phrase */
    uint32_t event_ctrl[3]; /* +0x6C */
    /* Shows width of external pulse */
    uint32_t event[3];      /* +0x78 */
};



void gpt_init(void)
{
    gpt = (volatile uint32_t *) gpt_regs;

        uint32_t cr = (
        (1 << 9) | // Free run mode
        (1 << 6) | // Peripheral clocks
        (1) // Enable
    );

    gpt[CR] = cr;

    gpt[IR] = ( 
        (1 << 5) // rollover interrupt
    );

    // set a timer! 
    uint64_t abs_timeout = get_ticks() + (LWIP_TICK_MS * NS_IN_MS);
    gpt[OCR1] = abs_timeout;
    gpt[IR] |= 1;

    timers_initialised = 1;
}