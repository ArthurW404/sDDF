/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <ctype.h>

#define BIT(n) (1ul<<(n))

/* SR */
#define UART_SR_RTRIG           BIT( 0)
#define UART_SR_REMPTY          BIT( 1)
#define UART_SR_RFUL            BIT( 2)
#define UART_SR_TEMPTY          BIT( 3)
#define UART_SR_TFUL            BIT( 4)
#define UART_SR_RACTIVE         BIT(10)
#define UART_SR_TACTIVE         BIT(11)
#define UART_SR_FDELT           BIT(12)
#define UART_SR_TTRIG           BIT(13)
#define UART_SR_TNFUL           BIT(14)

struct zynq_uart_regs {
    uint32_t cr;            /* 0x00 Control Register */
    uint32_t mr;            /* 0x04 Mode Register */
    uint32_t ier;           /* 0x08 Interrupt Enable Register */
    uint32_t idr;           /* 0x0C Interrupt Disable Register */
    uint32_t imr;           /* 0x10 Interrupt Mask Register (read-only) */
    uint32_t isr;           /* 0x14 Channel Interrupt Status Register (write a 1 to clear) */
    uint32_t baudgen;       /* 0x18 Baud Rate Generator Register */
    uint32_t rxtout;        /* 0x1C Receiver Timeout Register */
    uint32_t rxwm;          /* 0x20 Receiver FIFO Trigger Level Register */
    uint32_t modemcr;       /* 0x24 Modem Control Register */
    uint32_t modemsr;       /* 0x28 Modem Status Register */
    uint32_t sr;            /* 0x2C Channel Status Register (read-only) */
    uint32_t fifo;          /* 0x30 Transmit and Receive FIFO */
    uint32_t bauddiv;       /* 0x34 Baud Rate Divider Register */
    uint32_t flowdel;       /* 0x38 Flow Control Delay Register */
    uint32_t pad[2];
    uint32_t txwm;          /* 0x44 Transmitter FIFO Trigger Level Register */
};

typedef volatile struct zynq_uart_regs zynq_uart_regs_t;

#define UART_REG(x) ((volatile uint32_t *)(UART_BASE + (x)))
#define UART_BASE 0x5000000 //0x30890000 in hardware on imx8mm. 
#define STAT 0x98
#define TRANSMIT 0x40
#define STAT_TDRE (1 << 14)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))

#ifdef __GNUC__
#define likely(x)   __builtin_expect(!!(x), 1)
#define unlikely(x) __builtin_expect(!!(x), 0)
#else
#define likely(x)   (!!(x))
#define unlikely(x) (!!(x))
#endif

void
dump_packet(void *packet_ptr, int len);

static void
putC(uint8_t ch)
{   
    zynq_uart_regs_t *regs = UART_BASE;

    // disable irq
    uint32_t imr = regs->imr;
    regs->idr = imr;

    // while (!(*UART_REG(STAT) & STAT_TDRE)) { }
    // *UART_REG(TRANSMIT) = ch;
    
    regs->fifo = ch;
    while ((regs->sr & (UART_SR_TEMPTY | UART_SR_TACTIVE)) != UART_SR_TEMPTY);

    // enable irq
    regs->ier = imr;
}

static int
print(const char *s)
{   
    int num_chars_put = 0;
    while (*s) {
        putC(*s);
        s++;
        num_chars_put++;
    }
    return num_chars_put;
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



#define __aligned(x) __attribute__((aligned(x)))
#define unlikely(x) __builtin_expect(!!(x), 0)

#define __always_inline inline __attribute__((always_inline))
#define  noinline   __attribute__((noinline))

#define __deprecated    __attribute__((deprecated))
#define __packed    __attribute__((packed))
#define __weak      __attribute__((weak))
#define __alias(symbol) __attribute__((alias(#symbol)))
#define __must_check        __attribute__((warn_unused_result))

#define MAX_PKT_SIZE    1536

#define BITS_PER_LONG 32

#define ENOTSUPP    524 /* Operation is not supported */

void udelay(uint32_t us);

unsigned long simple_strtoul(const char *cp, char **endp, unsigned int base);

#ifdef CONFIG_PHYS_64BIT
typedef unsigned long long phys_addr_t;
typedef unsigned long long phys_size_t;
#else
/* DMA addresses are 32-bits wide */
typedef unsigned long phys_addr_t;
typedef unsigned long phys_size_t;
#endif

typedef uint64_t u64;
typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef int64_t s64;
typedef int32_t s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef unsigned long ulong;
typedef unsigned short ushort;
typedef unsigned int  uint;
typedef unsigned char uchar;

typedef u64 __u64;
typedef u32 __u32;
typedef u16 __u16;
typedef u8  __u8;

// typedef u8 bool;

#define __bitwise /*__attribute__((bitwise))*/
#define __force /* __attribute__((force)) */

typedef s64 __bitwise __le64;
typedef s32 __bitwise __le32;
typedef s16 __bitwise __le16;
typedef s8  __bitwise __le8;

typedef s64 __bitwise __be64;
typedef s32 __bitwise __be32;
typedef s16 __bitwise __be16;
typedef s8  __bitwise __be8;

typedef unsigned __bitwise  gfp_t;

#define gpio_init()
#define WATCHDOG_RESET()

typedef struct bd_info {
    unsigned long   bi_memstart;    /* start of DRAM memory */
    phys_size_t bi_memsize; /* size  of DRAM memory in bytes */
    unsigned long   bi_flashstart;  /* start of FLASH memory */
    unsigned long   bi_flashsize;   /* size  of FLASH memory */
    unsigned long   bi_flashoffset; /* reserved area for startup monitor */
    unsigned long   bi_sramstart;   /* start of SRAM memory */
    unsigned long   bi_sramsize;    /* size  of SRAM memory */
#ifdef CONFIG_AVR32
    unsigned char   bi_phy_id[4];   /* PHY address for ATAG_ETHERNET */
    unsigned long   bi_board_number;/* ATAG_BOARDINFO */
#endif
#ifdef CONFIG_ARM
    unsigned long   bi_arm_freq; /* arm frequency */
    unsigned long   bi_dsp_freq; /* dsp core frequency */
    unsigned long   bi_ddr_freq; /* ddr frequency */
#endif
#if defined(CONFIG_5xx) || defined(CONFIG_8xx) || defined(CONFIG_MPC8260) \
    || defined(CONFIG_E500) || defined(CONFIG_MPC86xx)
    unsigned long   bi_immr_base;   /* base of IMMR register */
#endif
#if defined(CONFIG_MPC5xxx) || defined(CONFIG_M68K)
    unsigned long   bi_mbar_base;   /* base of internal registers */
#endif
#if defined(CONFIG_MPC83xx)
    unsigned long   bi_immrbar;
#endif
    unsigned long   bi_bootflags;   /* boot / reboot flag (Unused) */
    unsigned long   bi_ip_addr; /* IP Address */
    unsigned char   bi_enetaddr[6]; /* OLD: see README.enetaddr */
    unsigned short  bi_ethspeed;    /* Ethernet speed in Mbps */
    unsigned long   bi_intfreq; /* Internal Freq, in MHz */
    unsigned long   bi_busfreq; /* Bus Freq, in MHz */
#if defined(CONFIG_CPM2)
    unsigned long   bi_cpmfreq; /* CPM_CLK Freq, in MHz */
    unsigned long   bi_brgfreq; /* BRG_CLK Freq, in MHz */
    unsigned long   bi_sccfreq; /* SCC_CLK Freq, in MHz */
    unsigned long   bi_vco;     /* VCO Out from PLL, in MHz */
#endif
#if defined(CONFIG_MPC512X)
    unsigned long   bi_ipsfreq; /* IPS Bus Freq, in MHz */
#endif /* CONFIG_MPC512X */
#if defined(CONFIG_MPC5xxx) || defined(CONFIG_M68K)
    unsigned long   bi_ipbfreq; /* IPB Bus Freq, in MHz */
    unsigned long   bi_pcifreq; /* PCI Bus Freq, in MHz */
#endif
#if defined(CONFIG_EXTRA_CLOCK)
    unsigned long bi_inpfreq;   /* input Freq in MHz */
    unsigned long bi_vcofreq;   /* vco Freq in MHz */
    unsigned long bi_flbfreq;   /* Flexbus Freq in MHz */
#endif
#if defined(CONFIG_405)   || \
        defined(CONFIG_405GP) || \
        defined(CONFIG_405EP) || \
        defined(CONFIG_405EZ) || \
        defined(CONFIG_405EX) || \
        defined(CONFIG_440)
    unsigned char   bi_s_version[4];    /* Version of this structure */
    unsigned char   bi_r_version[32];   /* Version of the ROM (AMCC) */
    unsigned int    bi_procfreq;    /* CPU (Internal) Freq, in Hz */
    unsigned int    bi_plb_busfreq; /* PLB Bus speed, in Hz */
    unsigned int    bi_pci_busfreq; /* PCI Bus speed, in Hz */
    unsigned char   bi_pci_enetaddr[6]; /* PCI Ethernet MAC address */
#endif

#ifdef CONFIG_HAS_ETH1
    unsigned char   bi_enet1addr[6];    /* OLD: see README.enetaddr */
#endif
#ifdef CONFIG_HAS_ETH2
    unsigned char   bi_enet2addr[6];    /* OLD: see README.enetaddr */
#endif
#ifdef CONFIG_HAS_ETH3
    unsigned char   bi_enet3addr[6];    /* OLD: see README.enetaddr */
#endif
#ifdef CONFIG_HAS_ETH4
    unsigned char   bi_enet4addr[6];    /* OLD: see README.enetaddr */
#endif
#ifdef CONFIG_HAS_ETH5
    unsigned char   bi_enet5addr[6];    /* OLD: see README.enetaddr */
#endif

#if defined(CONFIG_405GP) || defined(CONFIG_405EP) || \
        defined(CONFIG_405EZ) || defined(CONFIG_440GX) || \
        defined(CONFIG_440EP) || defined(CONFIG_440GR) || \
        defined(CONFIG_440EPX) || defined(CONFIG_440GRX) || \
        defined(CONFIG_460EX) || defined(CONFIG_460GT)
    unsigned int    bi_opbfreq;     /* OPB clock in Hz */
    int     bi_iic_fast[2];     /* Use fast i2c mode */
#endif
#if defined(CONFIG_4xx)
#if defined(CONFIG_440GX) || \
        defined(CONFIG_460EX) || defined(CONFIG_460GT)
    int     bi_phynum[4];           /* Determines phy mapping */
    int     bi_phymode[4];          /* Determines phy mode */
#elif defined(CONFIG_405EP) || defined(CONFIG_405EX) || defined(CONFIG_440)
    int     bi_phynum[2];           /* Determines phy mapping */
    int     bi_phymode[2];          /* Determines phy mode */
#else
    int     bi_phynum[1];           /* Determines phy mapping */
    int     bi_phymode[1];          /* Determines phy mode */
#endif
#endif /* defined(CONFIG_4xx) */
    ulong           bi_arch_number; /* unique id for this board */
    ulong           bi_boot_params; /* where this board expects params */
#ifdef CONFIG_NR_DRAM_BANKS
    struct {            /* RAM configuration */
        phys_addr_t start;
        phys_size_t size;
    } bi_dram[CONFIG_NR_DRAM_BANKS];
#endif /* CONFIG_NR_DRAM_BANKS */
} bd_t;

