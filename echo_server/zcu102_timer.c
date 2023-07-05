#include "timer.h"
#include "fence.h"

#define CLKCTRL_EXT_NEDGE           BIT(6)
#define CLKCTRL_EXT_SRC_EN          BIT(5)
#define CLKCTRL_PRESCALE_VAL(N)     (((N) & 0xf) << 1) /* rate = clk_src/[2^(N+1)] */
#define CLKCTRL_GET_PRESCALE_VAL(v) (((v) >> 1) & 0xf)
#define CLKCTRL_PRESCALE_EN         BIT(0)
#define CLKCTRL_PRESCALE_MASK       (CLKCTRL_PRESCALE_VAL(0xf) | CLKCTRL_PRESCALE_EN)

/* Waveform polarity: When this bit is high, the
 * waveform output goes from high to low on
 * Match_1 interrupt and returns high on overflow
 * or interval interrupt; when low, the waveform
 * goes from low to high on Match_1 interrupt and
 * returns low on overflow or interval interrupt */
#define CNTCTRL_WAVE_POL BIT(6)
/* Output waveform enable, active low. */
#define CNTCTRL_WAVE_EN  BIT(5)
/* Setting this bit high resets the counter value and
 * restarts counting; the RST bit is automatically
 * cleared on restart. */
#define CNTCTRL_RST      BIT(4)
/* Register Match mode: when Match is set, an
 * interrupt is generated when the count value
 * matches one of the three match registers and the
 * corresponding bit is set in the Interrupt Enable
 * register.
 */
#define CNTCTRL_MATCH    BIT(3)
/* Register Match mode: when Match is set, an
 * interrupt is generated when the count value
 * matches one of the three match registers and the
 * corresponding bit is set in the Interrupt Enable
 * register. */
#define CNTCTRL_DECR     BIT(2)
/* When this bit is high, the timer is in Interval
 * Mode, and the counter generates interrupts at
 * regular intervals; when low, the timer is in
 * overflow mode. */
#define CNTCTRL_INT      BIT(1)
/* Disable counter: when this bit is high, the counter
 * is stopped, holding its last value until reset,
 *  restarted or enabled again. */
#define CNTCTRL_STOP     BIT(0)

/* Event timer overflow interrupt */
#define INT_EVENT_OVR    BIT(5)
/* Counter overflow */
#define INT_CNT_OVR      BIT(4)
/* Match 3 interrupt */
#define INT_MATCH2       BIT(3)
/* Match 2 interrupt */
#define INT_MATCH1       BIT(2)
/* Match 1 interrupt */
#define INT_MATCH0       BIT(1)
/* Interval interrupt */
#define INT_INTERVAL     BIT(0)

/* Event Control Timer register: controls the behavior of the internal counter */

/* Specifies how to handle overflow at the internal counter (during the counting phase
 * of the external pulse)
 *
 * - When 0: Overflow causes E_En to be 0 (see E_En bit description)
 * - When 1: Overflow causes the internal counter to wrap around and continues incrementing
 */
#define EVCTRL_OVR       BIT(2)
/* Specifies the counting phase of the external pulse */
#define EVCTRL_LO        BIT(1)
/* When 0, immediately resets the internal counter to 0, and stops incrementing*/
#define EVCTRL_EN        BIT(0)

#define PRESCALE_MAX       0xf
#define PCLK_FREQ          111110000U

#ifdef CONFIG_PLAT_ZYNQMP
#define CNT_WIDTH 32
#define CNT_MAX ((1ULL << CNT_WIDTH) - 1)
#else
#define CNT_WIDTH 16
#define CNT_MAX (BIT(CNT_WIDTH) - 1)
#endif

/* Byte offsets into a field of ttc_tmr_regs_t for each ttc */
#define TTCX_TIMER1_OFFSET 0x0
#define TTCX_TIMER2_OFFSET 0x4
#define TTCX_TIMER3_OFFSET 0x8

#define TTCX_TIMER1_IRQ_POS 0
#define TTCX_TIMER2_IRQ_POS 1
#define TTCX_TIMER3_IRQ_POS 2

typedef uint64_t freq_t;

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

#define KHZ (1000)
#define MHZ (1000 * KHZ)
#define GHZ (1000 * MHZ)

#define NS_IN_MS 1000000ULL

#define MS_IN_S 1000ULL
#define US_IN_S 1000 * MS_IN_S
#define NS_IN_S 1000 * US_IN_S

#define FORCE_READ(address) \
    do { \
        typeof(*(address)) *_ptr = (address); \
        asm volatile ("" : "=m"(*_ptr) : "r"(*_ptr)); \
    } while (0)


static uint64_t hi_time;
int timers_initialised = 0;
freq_t clk_freq;

static inline bool _ttc_check_interrupt(volatile struct ttc_tmr_regs *regs)
{
    /* The int_sts register is being accessed through typedef ttc_tmr_regs_t
     * which is marked volatile, so the compiler will not elide this read.
     */
    uint32_t res = *regs->int_sts;
    /* There are no data dependencies within this function that imply that the
     * CPU cannot reorder this read in the pipeline. Use a CPU read barrier to
     * inform the CPU that it should stall reads until this read has completed.
     */
    THREAD_MEMORY_RELEASE();

    return !!res;
}


// TODO actually properly get frequency
static inline freq_t _ttc_get_freq()
{   
    return  clk_freq;
}

static inline uint64_t freq_cycles_and_hz_to_ns(uint64_t ncycles, freq_t hz)
{
    if (hz % GHZ == 0) {
        return ncycles / (hz / GHZ);
    } else if (hz % MHZ == 0) {
        return ncycles * MS_IN_S / (hz / MHZ);
    } else if (hz % KHZ == 0) {
        return ncycles * US_IN_S / (hz / KHZ);
    }

    return (ncycles * NS_IN_S) / hz;
}

uint64_t ttc_ticks_to_ns(uint32_t ticks)
{
    uint32_t fin = _ttc_get_freq();
    return freq_cycles_and_hz_to_ns(ticks, fin);
}

static uint64_t get_ticks(void) {
    volatile struct ttc_tmr_regs *gpt =  (void *) gpt_regs;

    uint32_t cnt = *gpt->cnt_val;
    
    // bool interrupt_pending = _ttc_check_interrupt(ttc);
    bool interrupt_pending = _ttc_check_interrupt(gpt);
    /* Check if there is an interrupt pending, i.e. counter overflowed */
    if (interrupt_pending) {
        /* Re-read the counter again */
        cnt = *gpt->cnt_val;
        /* Bump the hi_time counter now, as there may be latency in serving the interrupt */
        hi_time += ttc_ticks_to_ns(CNT_MAX);
    }
    uint32_t fin = _ttc_get_freq();
    return hi_time + freq_cycles_and_hz_to_ns(cnt, fin);}

u32_t sys_now(void)
{
    if (!timers_initialised) {
        /* lwip_init() will call this when initialising its own timers,
         * but the timer is not set up at this point so just return 0 */
        return 0;
    } else {
        uint64_t time_now = get_ticks();
        return time_now / NS_IN_MS;
    }
}

void irq(sel4cp_channel ch)
{   
    volatile struct ttc_tmr_regs *regs =  (void *) gpt_regs;
    sel4cp_dbg_puts("=====================>|irq|lwip timer IRQ\n");

    bool interrupt_pending = _ttc_check_interrupt(regs);

    // this timer is just a timestamp counter, so don't need to differentiate
    if (interrupt_pending) {
        /* Check if we already updated the timestamp when reading the time,
         * the interrupt status register should be empty if we did */
        hi_time += ttc_ticks_to_ns( CNT_MAX);
    } else {
        /* The MATCH0 interrupt is used in oneshot mode. It is enabled when a
         * oneshot function is called, and disabled here so only one interrupt
         * is triggered per call. */
        *regs->int_en &= ~INT_MATCH0;
    }
    sys_check_timeouts();
}

/* FPGA PL Clocks */
static freq_t _ttc_clk_get_freq(volatile struct ttc_tmr_regs *gpt) {
    freq_t fin, fout;

    // platsuppport doesn't  seem to have parent so use default freq
    fin = PCLK_FREQ;

    /* Calculate fout */
    uint32_t clk_ctrl = *gpt->clk_ctrl;
    if (clk_ctrl & CLKCTRL_PRESCALE_EN) {
        fout = fin >> (CLKCTRL_GET_PRESCALE_VAL(clk_ctrl) + 1);
    } else {
        fout = fin;
    }
    /* Return */
    return fout;
}


void gpt_init(void)
{
    volatile struct ttc_tmr_regs *gpt =  (void *) gpt_regs;

    clk_freq = _ttc_clk_get_freq(gpt);

    // disable interrupt
    *gpt->int_en = 0;
    FORCE_READ(gpt->int_sts); /* Clear on read */
    *gpt->cnt_ctrl = CNTCTRL_RST | CNTCTRL_STOP | CNTCTRL_INT | CNTCTRL_MATCH;
    *gpt->clk_ctrl = 0;
    *gpt->int_en = INT_INTERVAL;
    *gpt->interval = CNT_MAX;
    __sync_synchronize();
    
    // freerun momde 
    *gpt->cnt_ctrl = CNTCTRL_RST;
    *gpt->int_en = INT_EVENT_OVR | INT_CNT_OVR;

    __sync_synchronize();

    // start ttc
    *gpt->cnt_ctrl &= ~CNTCTRL_STOP;

    hi_time = 0;
    timers_initialised = 1;
}