/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include <string.h>

#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"

#define IRQ_CH 1
#define TX_CH  2
#define RX_CH  2
#define INIT   4

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t hw_ring_buffer_vaddr;
uintptr_t hw_ring_buffer_paddr;
uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;
uintptr_t rx_cookies;
uintptr_t tx_cookies;
uintptr_t rx_free;
uintptr_t rx_used;
uintptr_t tx_free;
uintptr_t tx_used;
uintptr_t uart_base;

/* Make the minimum frame buffer 2k. This is a bit of a waste of memory, but ensures alignment */
#define PACKET_BUFFER_SIZE  2048
#define MAX_PACKET_SIZE     1536

#define RX_COUNT 256
#define TX_COUNT 256

_Static_assert((512 * 2) * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");

struct descriptor {
    uint16_t len;
    uint16_t stat;
    uint32_t addr;
};

typedef struct {
    unsigned int cnt;
    unsigned int remain;
    unsigned int tail;
    unsigned int head;
    volatile struct descriptor *descr;
    uintptr_t phys;
    void **cookies;
} ring_ctx_t;

ring_ctx_t rx;
ring_ctx_t tx;
unsigned int tx_lengths[TX_COUNT];

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;

static uint8_t mac[6];

volatile void *eth_base_reg = (void *)(uintptr_t)0x2000000;

struct eqos_mac_regs *mac_regs;
struct eqos_mtl_regs *mtl_regs;
struct eqos_dma_regs *dma_regs;
struct eqos_tegra186_regs *tegra186_regs;


static const struct eqos_config eqos_tegra186_config = {
    .reg_access_always_ok = false,
    .mdio_wait = 10,
    .swr_wait = 10,
    .config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
    .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_20_35,
};

static void get_mac_addr(volatile void *reg, uint8_t *mac)
{
    memcpy(mac, TX2_DEFAULT_MAC, 6);
}

static void set_mac(volatile void *mac_reg, uint8_t *mac)
{
    unsigned char enetaddr[ARP_HLEN];
    memcpy(enetaddr, TX2_DEFAULT_MAC, 6);
    uint32_t val1 = (enetaddr[5] << 8) | (enetaddr[4]);
    mac_regs->address0_high = val1;
    val1 = (enetaddr[3] << 24) | (enetaddr[2] << 16) |
           (enetaddr[1] << 8) | (enetaddr[0]);

    mac_regs->address0_low;
}

static void
dump_mac(uint8_t *mac)
{
    for (unsigned i = 0; i < 6; i++) {
        sel4cp_dbg_putc(hexchar((mac[i] >> 4) & 0xf));
        sel4cp_dbg_putc(hexchar(mac[i] & 0xf));
        if (i < 5) {
            sel4cp_dbg_putc(':');
        }
    }
}


void udelay(unsigned long us)
{
    volatile int i;
    for (; us > 0; us--) {
        for (i = 0; i < 1000; i++) {
        }
    }
}


static uintptr_t 
getPhysAddr(uintptr_t virtual)
{
   
}

static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint16_t len,
    uint16_t stat)
{
    volatile struct descriptor *d = &(ring->descr[idx]);
    d->addr = phys;
    d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    __sync_synchronize();

    d->stat = stat;
}

static inline void
enable_irqs(volatile void *eth, uint32_t mask)
{

}

static uintptr_t 
alloc_rx_buf(size_t buf_size, void **cookie)
{
   
}

static void fill_rx_bufs()
{
  
}

static void
handle_rx(volatile void *eth)
{
}

static void
complete_tx(volatile void *eth)
{

}

static void
raw_tx(volatile void *eth, unsigned int num, uintptr_t *phys,
                  unsigned int *len, void *cookie)
{
}

static void 
handle_eth(volatile void *eth)
{

}

static void 
handle_tx(volatile void *eth)
{

}

static void 
eth_setup(void)
{
    struct eqos_config *config = &eqos_tegra186_config;
    uint32_t *dma_ie;
    uint32_t val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;

    get_mac_addr(NULL, mac);
    sel4cp_dbg_puts("MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    // setup registers 
    mac_regs = (void *)(eth_base_reg + EQOS_MAC_REGS_BASE);
    mtl_regs = (void *)(eth_base_reg + EQOS_MTL_REGS_BASE);
    dma_regs = (void *)(eth_base_reg + EQOS_DMA_REGS_BASE);
    tegra186_regs = (void *)(eth_base_reg + EQOS_TEGRA186_REGS_BASE);

    /* set up descriptor rings */
    rx.cnt = RX_COUNT;
    rx.remain = rx.cnt - 2;
    rx.tail = 0;
    rx.head = 0;
    rx.phys = shared_dma_paddr;
    rx.cookies = (void **)rx_cookies;
    rx.descr = (volatile struct descriptor *)hw_ring_buffer_vaddr;

    tx.cnt = TX_COUNT;
    tx.remain = tx.cnt - 2;
    tx.tail = 0;
    tx.head = 0;
    tx.phys = shared_dma_paddr + (sizeof(struct descriptor) * RX_COUNT);
    tx.cookies = (void **)tx_cookies;
    tx.descr = (volatile struct descriptor *)(hw_ring_buffer_vaddr + (sizeof(struct descriptor) * RX_COUNT));

    // TX2 eqos device setup
    // ==== 
    // 
    // The device setup is based on eqos_start from linux and sel4 driver for ethernet
    // it currently skips starting clks, resets, phy, and link adjustment 
    // (some of these may be necessary for device to function)
    // 
    // ====

    /* Configure MTL */

    /* Flush TX queue */
    mtl_regs->txq0_operation_mode = (EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);

    while (*((uint32_t *)eth_base_reg + 0xd00));
    /* Enable Store and Forward mode for TX */
    mtl_regs->txq0_operation_mode = (EQOS_MTL_TXQ0_OPERATION_MODE_TSF);
    /* Program Tx operating mode */
    mtl_regs->txq0_operation_mode |= (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
                                            EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT);
    /* Transmit Queue weight */
    mtl_regs->txq0_quantum_weight = 0x10;

    /* Enable Store and Forward mode for RX, since no jumbo frame */
    mtl_regs->rxq0_operation_mode = (EQOS_MTL_RXQ0_OPERATION_MODE_RSF);

    /* Transmit/Receive queue fifo size; use all RAM for 1 queue */
    val = mac_regs->hw_feature1;
    tx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_SHIFT) &
                 EQOS_MAC_HW_FEATURE1_TXFIFOSIZE_MASK;
    rx_fifo_sz = (val >> EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_SHIFT) &
                 EQOS_MAC_HW_FEATURE1_RXFIFOSIZE_MASK;

    /*
     * r/tx_fifo_sz is encoded as log2(n / 128). Undo that by shifting.
     * r/tqs is encoded as (n / 256) - 1.
     */
    tqs = (128 << tx_fifo_sz) / 256 - 1;
    rqs = (128 << rx_fifo_sz) / 256 - 1;

    mtl_regs->txq0_operation_mode &= ~(EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
                                             EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    mtl_regs->txq0_operation_mode |=
        tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT;
    mtl_regs->rxq0_operation_mode &= ~(EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
                                             EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);
    mtl_regs->rxq0_operation_mode |=
        rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT;

    /* Flow control used only if each channel gets 4KB or more FIFO */
    if (rqs >= ((4096 / 256) - 1)) {
        uint32_t rfd, rfa;

        mtl_regs->rxq0_operation_mode |= (EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

        /*
         * Set Threshold for Activating Flow Contol space for min 2
         * frames ie, (1500 * 1) = 1500 bytes.
         *
         * Set Threshold for Deactivating Flow Contol for space of
         * min 1 frame (frame size 1500bytes) in receive fifo
         */
        if (rqs == ((4096 / 256) - 1)) {
            /*
             * This violates the above formula because of FIFO size
             * limit therefore overflow may occur inspite of this.
             */
            rfd = 0x3;  /* Full-3K */
            rfa = 0x1;  /* Full-1.5K */
        } else if (rqs == ((8192 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0xa;  /* Full-6K */
        } else if (rqs == ((16384 / 256) - 1)) {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x12; /* Full-10K */
        } else {
            rfd = 0x6;  /* Full-4K */
            rfa = 0x1E; /* Full-16K */
        }

        mtl_regs->rxq0_operation_mode &= ~((EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
                                                  EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                                                 (EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
                                                  EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
        mtl_regs->rxq0_operation_mode |= (rfd <<
                                                EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                                               (rfa <<
                                                EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT);
    }

    dma_ie = (uint32_t *)(eth_base_reg + 0xc30);
    *dma_ie = 0x3020100;

    /* Configure MAC, not sure if L4T is the same */
    mac_regs->rxq_ctrl0 =
        (config->config_mac <<
         EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Set TX flow control parameters */
    /* Set Pause Time */
    mac_regs->q0_tx_flow_ctrl = (0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
    /* Assign priority for RX flow control */
    mac_regs->rxq_ctrl2 = (1 << EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);

    /* Enable flow control */
    mac_regs->q0_tx_flow_ctrl |= (EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);

    mac_regs->rx_flow_ctrl = (EQOS_MAC_RX_FLOW_CTRL_RFE);

    mac_regs->configuration &=
        ~(EQOS_MAC_CONFIGURATION_GPSLCE |
          EQOS_MAC_CONFIGURATION_WD |
          EQOS_MAC_CONFIGURATION_JD |
          EQOS_MAC_CONFIGURATION_JE);

    /* PLSEN is set to 1 so that LPI is not initiated */
    // MAC_LPS_PLSEN_WR(1); << this macro below
    uint32_t v = mac_regs->unused_0ac[9];
    v = (v & (MAC_LPS_RES_WR_MASK_20)) | (((0) & (MAC_LPS_MASK_20)) << 20);
    v = (v & (MAC_LPS_RES_WR_MASK_10)) | (((0) & (MAC_LPS_MASK_10)) << 10);
    v = (v & (MAC_LPS_RES_WR_MASK_4)) | (((0) & (MAC_LPS_MASK_4)) << 4);
    v = ((v & MAC_LPS_PLSEN_WR_MASK) | ((1 & MAC_LPS_PLSEN_MASK) << 18));
    mac_regs->unused_0ac[9] = v;

    /* Update the MAC address */
    set_mac(mac_regs, TX2_DEFAULT_MAC);

    mac_regs->configuration &= 0xffcfff7c;
    mac_regs->configuration |=  DWCEQOS_MAC_CFG_TE | DWCEQOS_MAC_CFG_RE;

    /* Configure DMA */
    /* Enable OSP mode */
    dma_regs->ch0_tx_control = EQOS_DMA_CH0_TX_CONTROL_OSP;

    /* RX buffer size. Must be a multiple of bus width */
    dma_regs->ch0_rx_control = (EQOS_MAX_PACKET_SIZE << EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

    dma_regs->ch0_control = (EQOS_DMA_CH0_CONTROL_PBLX8);

    /*
     * Burst length must be < 1/2 FIFO size.
     * FIFO size in tqs is encoded as (n / 256) - 1.
     * Each burst is n * 8 (PBLX8) * 16 (AXI width) == 128 bytes.
     * Half of n * 256 is n * 128, so pbl == tqs, modulo the -1.
     */
    pbl = tqs + 1;
    if (pbl > 32) {
        pbl = 32;
    }
    dma_regs->ch0_tx_control &=
        ~(EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
          EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);
    dma_regs->ch0_tx_control |= (pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    dma_regs->ch0_rx_control &=
        ~(EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
          EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);
    dma_regs->ch0_rx_control |= (1 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

    /* DMA performance configuration */
    val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
          EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
          EQOS_DMA_SYSBUS_MODE_BLEN8;
    dma_regs->sysbus_mode = val;

    dma_regs->ch0_txdesc_list_haddress = 0;
    dma_regs->ch0_txdesc_list_address = tx.phys;
    dma_regs->ch0_txdesc_ring_length = TX_COUNT - 1;

    dma_regs->ch0_rxdesc_list_haddress = 0;
    dma_regs->ch0_rxdesc_list_address = rx.phys;
    dma_regs->ch0_rxdesc_ring_length = RX_COUNT - 1;

    dma_regs->ch0_dma_ie = 0;
    dma_regs->ch0_dma_ie = DWCEQOS_DMA_CH0_IE_RIE | DWCEQOS_DMA_CH0_IE_TIE |
                                 DWCEQOS_DMA_CH0_IE_NIE | DWCEQOS_DMA_CH0_IE_AIE |
                                 DWCEQOS_DMA_CH0_IE_FBEE | DWCEQOS_DMA_CH0_IE_RWTE;
    dma_regs->ch0_dma_rx_int_wd_timer = 120;
    udelay(100);

    dma_regs->ch0_tx_control = EQOS_DMA_CH0_TX_CONTROL_ST;
    dma_regs->ch0_rx_control = EQOS_DMA_CH0_RX_CONTROL_SR;

    // last_rx_desc = (rx.phys + ((EQOS_DESCRIPTORS_RX) * (uintptr_t)(sizeof(struct eqos_desc))));
    // last_tx_desc = (tx.phys + ((EQOS_DESCRIPTORS_TX) * (uintptr_t)(sizeof(struct eqos_desc))));

    /* Disable MMC event counters */
    *(uint32_t *)(eth_base_reg + REG_DWCEQOS_ETH_MMC_CONTROL) |= REG_DWCEQOS_MMC_CNTFREEZ;

    return;

}

void init_post()
{
    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, NULL, 0);

    fill_rx_bufs();
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": init complete -- waiting for interrupt\n");
    sel4cp_notify(INIT);

    /* Now take away our scheduling context. Uncomment this for a passive driver. */
    /* have_signal = true;
    msg = seL4_MessageInfo_new(0, 0, 0, 1);
    seL4_SetMR(0, 0);
    signal = (MONITOR_EP); */
}

void init(void)
{
    sel4cp_dbg_puts(sel4cp_name);
    sel4cp_dbg_puts(": elf PD init function running\n");

    eth_setup();
    /* Now wait for notification from lwip that buffers are initialised */
}

seL4_MessageInfo_t
protected(sel4cp_channel ch, sel4cp_msginfo msginfo)
{
    switch (ch) {
        case INIT:
            // return the MAC address. 
            // sel4cp_mr_set(0, eth->palr);
            // sel4cp_mr_set(1, eth->paur);
            return sel4cp_msginfo_new(0, 2);
        case TX_CH:
            // handle_tx(eth);
            break;
        default:
            sel4cp_dbg_puts("Received ppc on unexpected channel ");
            puthex64(ch);
            break;
    }
    return sel4cp_msginfo_new(0, 0);
}

void notified(sel4cp_channel ch)
{
    switch(ch) {
        case IRQ_CH:
            // handle_eth(eth);
            have_signal = true;
            signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
            signal = (BASE_IRQ_CAP + IRQ_CH);
            return;
        case INIT:
            init_post();
            break;
        case TX_CH:
            // handle_tx(eth);
            break;
        default:
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            break;
    }
}
