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

// IMX8mm descriptors
// struct descriptor {
//     uint16_t len;
//     uint16_t stat;
//     uint32_t addr;
// };


/* descriptor flags */
#define EQOS_DESC2_IOC      BIT(31)
#define EQOS_DESC3_OWN      BIT(31)
#define EQOS_DESC3_FD       BIT(29)
#define EQOS_DESC3_LD       BIT(28)
#define EQOS_DESC3_BUF1V    BIT(24)
#define DWCEQOS_DMA_RDES3_INTE    BIT(30)

/* descriptor 0, 1 and 2 need to be written to in order to trigger dma */
struct eqos_desc {
    uint32_t des0; /* address of the packet */
    uint32_t des1;
    uint32_t des2; /* length of packet */
    uint32_t des3; /* flags (interrupt, own, ld, etc) and length of packet */
};

typedef struct {
    unsigned int cnt;
    unsigned int remain;
    unsigned int tail;
    unsigned int head;
    volatile struct eqos_desc *descr;
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
volatile struct eqos_mac_regs *eth_mac = (void *)(0x2000000 + EQOS_MAC_REGS_BASE);

// struct for eqos registers and device metadata
struct eqos_priv eqos_dev;
struct eqos_priv *eqos = &eqos_dev;

// struct eqos_mac_regs *mac_regs;
// struct eqos_mtl_regs *mtl_regs;
// struct eqos_dma_regs *dma_regs;
// struct eqos_tegra186_regs *tegra186_regs;


static const struct eqos_config eqos_tegra186_config = {
    .reg_access_always_ok = false,
    .mdio_wait = 10,
    .swr_wait = 10,
    .config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
    .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_20_35,
};


static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint32_t len,
    uint32_t flags)
{
    volatile struct eqos_desc *d = &(ring->descr[idx]);
    // print("after getting d\n");
    d->des0 = phys;
    // print("Managed to save phys to des0\n");
    d->des1 = 0;
    d->des2 = len;
    d->des3 = flags;
}

void eqos_dma_disable_rxirq(struct eqos_priv *eqos)
{
    uint32_t regval;

    regval = eqos->dma_regs->ch0_dma_ie;
    regval &= ~DWCEQOS_DMA_CH0_IE_RIE;
    eqos->dma_regs->ch0_dma_ie = regval;
}

void eqos_dma_enable_rxirq(struct eqos_priv *eqos)
{
    uint32_t regval;
    
    print("rxirq eqos->dma_regs->ch0_dma_ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");

    regval = eqos->dma_regs->ch0_dma_ie;
    regval |= DWCEQOS_DMA_CH0_IE_RIE;

    print("rxirq regval = ");
    puthex64(regval);
    print("\n");

    eqos->dma_regs->ch0_dma_ie = regval;

    print("after: rxirq eqos->dma_regs->ch0_dma_ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");
}

void eqos_dma_disable_txirq(struct eqos_priv *eqos)
{
    uint32_t regval;

    regval = eqos->dma_regs->ch0_dma_ie;
    regval &= ~DWCEQOS_DMA_CH0_IE_TIE;
    eqos->dma_regs->ch0_dma_ie = regval;
}

void eqos_dma_enable_txirq(struct eqos_priv *eqos)
{
    uint32_t regval;

    print("txirq eqos->dma_regs->ch0_dma_ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");

    regval = eqos->dma_regs->ch0_dma_ie;
    regval |= DWCEQOS_DMA_CH0_IE_TIE;
    print("txirq regval = ");
    puthex64(regval);
    print("\n");
    eqos->dma_regs->ch0_dma_ie = regval;
}

void eqos_set_rx_tail_pointer(struct eqos_priv *eqos)
{
    uint32_t *dma_status = (uint32_t *)(eqos->regs + REG_DWCEQOS_DMA_CH0_STA);
    *dma_status |= DWCEQOS_DMA_CH0_IS_RI;
    // size_t num_buffers_in_ring = dev->rx_size - dev->rx_remain;

    // // if there is buffers in ring, set 
    // if (num_buffers_in_ring > 0) {
    //     uintptr_t last_rx_desc = (dev->rx_ring_phys + ((dev->rdh + num_buffers_in_ring) * sizeof(struct eqos_desc)));
    //     eqos->dma_regs->ch0_rxdesc_tail_pointer = last_rx_desc;
    // }

    // if there is buffers in ring, set tail 
    if (rx.remain != 0) {
        // calculate tail position
        uintptr_t last_rx_desc = rx.phys + rx.tail * sizeof(struct eqos_desc);
        eqos->dma_regs->ch0_rxdesc_tail_pointer = last_rx_desc;
    }

    
}


int eqos_send(struct eqos_priv *eqos, void *packet, int length)
{
    print("In eqos send\n");

    volatile struct eqos_desc *tx_desc;
    uint32_t ioc = 0;
    if (tx.tail % 32 == 0) {
        ioc = EQOS_DESC2_IOC;
    }

    print("Before update ring slot\n");




    tx_desc = &(tx.descr[tx.tail]);
    print("Something wrong with tx_desc\n");
    ring_ctx_t *ring = &tx;
    
    print("tx_desc (should equal tx.decr at the start) = ");
    puthex64(tx_desc);
    print("\n");

    print("tx.tail = ");
    puthex64(tx.tail);
    print("\n");
    
    // update_ring_slot(ring, tx.tail, (uintptr_t)packet, ioc | length, EQOS_DESC3_FD | EQOS_DESC3_LD | length);

    tx_desc->des0 = (uintptr_t)packet;
    tx_desc->des1 = 0;
    tx_desc->des2 = ioc | length;
    tx_desc->des3 = EQOS_DESC3_FD | EQOS_DESC3_LD | length;

    print("After update ring slot\n");


    __sync_synchronize();

    tx_desc->des3 |= EQOS_DESC3_OWN;

    print("right before dma_reg update\n");
    eqos->dma_regs->ch0_txdesc_tail_pointer = (uintptr_t)(&(tx.descr[tx.tail + 1])) +
                                              sizeof(struct eqos_desc);
    print("after dma_reg update\n");

    return 0;
}


static void get_mac_addr(struct eqos_priv *eqos, uint8_t *mac)
{
    //default one: 00:04:4b:c5:67:70
    __sync_synchronize();
    // memcpy(mac, TX2_DEFAULT_MAC, 6);
    uint32_t l, h;
    // l = eth_mac->address0_low;
    // h = eth_mac->address0_high;
    l = eqos->mac_regs->address0_low;
    h = eqos->mac_regs->address0_high;

    mac[0] = l >> 24;
    mac[1] = l >> 16 & 0xff;
    mac[2] = l >> 8 & 0xff;
    mac[3] = l & 0xff;
    mac[4] = h >> 24;
    mac[5] = h >> 16 & 0xff;
}

static void set_mac(struct eqos_priv *eqos, uint8_t *mac)
{
    unsigned char enetaddr[ARP_HLEN];
    memcpy(enetaddr, TX2_DEFAULT_MAC, 6);
    uint32_t val1 = (enetaddr[5] << 8) | (enetaddr[4]);
    eqos->mac_regs->address0_high = val1;
    val1 = (enetaddr[3] << 24) | (enetaddr[2] << 16) |
           (enetaddr[1] << 8) | (enetaddr[0]);

    eqos->mac_regs->address0_low = val1;
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
    uint64_t offset = virtual - shared_dma_vaddr;
    uintptr_t phys;

    if (offset < 0) {
        print("getPhysAddr: offset < 0");
        return 0;
    }

    phys = shared_dma_paddr + offset;
    return phys;
}

static inline void
enable_irqs(volatile void *eth, uint32_t mask)
{

}

static uintptr_t 
alloc_rx_buf(size_t buf_size, void **cookie)
{
    uintptr_t addr;
    unsigned int len;

    /* Try to grab a buffer from the available ring */
    if (driver_dequeue(rx_ring.free_ring, &addr, &len, cookie)) {
        print("RX Available ring is empty\n");
        return 0;
    }

    uintptr_t phys = getPhysAddr(addr);

    return getPhysAddr(addr);
}

static void fill_rx_bufs()
{
    ring_ctx_t *ring = &rx;
    __sync_synchronize();
    while (ring->remain > 0) {
        /* request a buffer */
        void *cookie = NULL;
        uintptr_t phys = alloc_rx_buf(MAX_PACKET_SIZE, &cookie);
        if (!phys) {
            break;
        }
        // uint16_t stat = RXD_EMPTY;
        int idx = ring->tail;
        int new_tail = idx + 1;
        
        // may need to handle wrap 
        // if (new_tail == ring->cnt) {
        //     new_tail = 0;
        //     stat |= WRAP;
        // }
        ring->cookies[idx] = cookie;
        update_ring_slot(ring, idx, phys, 0, EQOS_DESC3_OWN | EQOS_DESC3_BUF1V);

        ring->tail = new_tail;
        /* There is a race condition if add/remove is not synchronized. */
        ring->remain--;
    }
    __sync_synchronize();

    // if (dev->rx_remain != dev->rx_size) {
    // ^ linux version
    if (ring->tail != ring->head) {
        /* We've refilled some buffers, so set the tail pointer so that the DMA controller knows */
        eqos_set_rx_tail_pointer(eqos);
    }

    __sync_synchronize();

}

static void
handle_rx(volatile void *eth)
{
}

static void
complete_tx(struct eqos_priv *eqos)
{
    unsigned int cnt_org;
    void *cookie;
    ring_ctx_t *ring = &tx;
    unsigned int head = ring->head;
    unsigned int cnt = 0;
    volatile struct eqos_desc *tx_desc;


    while (head != ring->tail) {
        print("Looping through tx\n");
        if (0 == cnt) {
            cnt = tx_lengths[head];
            if ((0 == cnt) || (cnt > TX_COUNT)) {
                /* We are not supposed to read 0 here. */
                print("complete_tx with cnt=0 or max");
                return;
            }
            cnt_org = cnt;
            cookie = ring->cookies[head];
        }

        volatile struct eqos_desc *d = &(ring->descr[head]);

        /* If this buffer was not sent, we can't release any buffer. */
        // if (d->stat & TXD_READY) {
        //     /* give it another chance */
        //     if (!(eth->tdar & TDAR_TDAR)) {
        //         eth->tdar = TDAR_TDAR;
        //     }
        //     if (d->stat & TXD_READY) {
        //         break;
        //     }
        // }
        // uint32_t ring_pos = (i + ring->head) % TX_COUNT;
        // tx_desc = ring->descr[ring_pos];
        if ((d->des3 & EQOS_DESC3_OWN)) {
            /* not all parts complete */
            return;
        }

        /* Go to next buffer, handle roll-over. */
        if (++head == TX_COUNT) {
            head = 0;
        }

        if (0 == --cnt) {
            ring->head = head;
            /* race condition if add/remove is not synchronized. */
            ring->remain += cnt_org;
            /* give the buffer back */
            buff_desc_t *desc = (buff_desc_t *)cookie;

            enqueue_free(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
        }
    }
}

static void
raw_tx(struct eqos_priv *eqos, unsigned int num, uintptr_t *phys,
                  unsigned int *len, void *cookie)
{
    // assert(num == 1);
    int err;
    ring_ctx_t *ring = &tx;

    print("In raw_tx\n");
    /* Ensure we have room */
    if (ring->remain < num) {
        print("DOing complete_tx\n");
        /* not enough room, try to complete some and check again */
        complete_tx(eqos);
        unsigned int rem = ring->remain;
        if (rem < num) {
            print("TX queue lacks space");
            return;
        }
    }

    __sync_synchronize();

    unsigned int tail = ring->tail;
    unsigned int tail_new = tail;

    uint32_t i;
    for (i = 0; i < num; i++) {
        print("In send loop\n");
        // uint16_t stat = TXD_READY;
        // if (0 == i) {
        //     stat |= TXD_ADDCRC | TXD_LAST;
        // }

        unsigned int idx = tail_new;
        if (++tail_new == TX_COUNT) {
            tail_new = 0;
            // stat |= WRAP;
        }

        print("phys = ");
        puthex64(phys);
        print("\n");

        print("phys[0] = ");
        puthex64(phys[0]);
        print("\n");
        // update_ring_slot(ring, idx, *phys++, *len++, stat);

        print("len[0] = ");
        puthex64(len[0]);
        print("\n");

        err = eqos_send(eqos, (void *)phys[i], len[i]);
        // if (err == -ETIMEDOUT) {
        //     ZF_LOGF("send timed out");
        // }
        print("After send\n");

    }

    ring->cookies[tail] = cookie;
    tx_lengths[tail] = num;
    ring->tail = tail_new;
    /* There is a race condition here if add/remove is not synchronized. */
    ring->remain -= num;

    // __sync_synchronize();

    // if (!(eth->tdar & TDAR_TDAR)) {
    //     eth->tdar = TDAR_TDAR;
    // }

}

static void 
handle_eth(struct eqos_priv *eqos)
{
  
}

static void handle_tx(struct eqos_priv *eqos)
{
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = NULL;
    print("In handle_tx\n");
    // We need to put in an empty condition here. 
    while ((tx.remain > 1) && !driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        print("In driver_dequeue\n");
        uintptr_t phys = getPhysAddr(buffer);
        raw_tx(eqos, 1, &phys, &len, cookie);
    }
}

static void 
eth_setup(void)
{

    uint32_t *dma_ie;
    uint32_t val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;

    eqos->config = &eqos_tegra186_config;
    eqos->regs = eth_base_reg;

    // setup registers 
    eqos->mac_regs = (void *)(eqos->regs + EQOS_MAC_REGS_BASE);
    eqos->mtl_regs = (void *)(eqos->regs + EQOS_MTL_REGS_BASE);
    eqos->dma_regs = (void *)(eqos->regs + EQOS_DMA_REGS_BASE);
    eqos->tegra186_regs = (void *)(eqos->regs + EQOS_TEGRA186_REGS_BASE);

    print("MAC regs: ");
    puthex64(eqos->mac_regs);
    print("\n");

    print("mtl_regs regs: ");
    puthex64(eqos->mtl_regs);
    print("\n");
    
    print("dma_regs regs: ");
    puthex64(eqos->dma_regs);
    print("\n");

    get_mac_addr(eqos, mac);
    sel4cp_dbg_puts("MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");
    
    
    eqos->mac_regs->address0_low = 0;
    eqos->mac_regs->address0_high = 0;

    __sync_synchronize();

    sel4cp_dbg_puts("Updaated MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    volatile uint32_t *dma_status = (uint32_t *)(eqos->regs + REG_DWCEQOS_DMA_CH0_STA);
    
    print("dma_status: ");
    puthex64(*dma_status);
    print("\n");

    // *dma_status |= DWCEQOS_DMA_CH0_IS_RI;
    // *dma_status |= DWCEQOS_DMA_CH0_IS_RI;

    // print("updated dma_status: ");
    // puthex64(*dma_status);
    // print("\n");

    /* set up descriptor rings */
    rx.cnt = RX_COUNT;
    rx.remain = rx.cnt - 2;
    rx.tail = 0;
    rx.head = 0;
    rx.phys = shared_dma_paddr;
    rx.cookies = (void **)rx_cookies;
    rx.descr = (volatile struct eqos_desc *)hw_ring_buffer_vaddr;

    print("RX descr = ");
    puthex64(rx.descr);
    print("\n");

    tx.cnt = TX_COUNT;
    tx.remain = tx.cnt - 2;
    tx.tail = 0;
    tx.head = 0;
    tx.phys = shared_dma_paddr + (sizeof(struct eqos_desc) * RX_COUNT);
    tx.cookies = (void **)tx_cookies;
    tx.descr = (volatile struct eqos_desc *)(hw_ring_buffer_vaddr + (sizeof(struct eqos_desc) * RX_COUNT));
    
    print("TX descr = ");
    puthex64(tx.descr);
    print("\n");
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
    eqos->mtl_regs->txq0_operation_mode = (EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);

    while (*((uint32_t *)eqos->regs + 0xd00));
    /* Enable Store and Forward mode for TX */
    eqos->mtl_regs->txq0_operation_mode = (EQOS_MTL_TXQ0_OPERATION_MODE_TSF);
    /* Program Tx operating mode */
    eqos->mtl_regs->txq0_operation_mode |= (EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_ENABLED <<
                                            EQOS_MTL_TXQ0_OPERATION_MODE_TXQEN_SHIFT);
    /* Transmit Queue weight */
    eqos->mtl_regs->txq0_quantum_weight = 0x10;

    /* Enable Store and Forward mode for RX, since no jumbo frame */
    eqos->mtl_regs->rxq0_operation_mode = (EQOS_MTL_RXQ0_OPERATION_MODE_RSF);

    /* Transmit/Receive queue fifo size; use all RAM for 1 queue */
    val = eqos->mac_regs->hw_feature1;
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

    eqos->mtl_regs->txq0_operation_mode &= ~(EQOS_MTL_TXQ0_OPERATION_MODE_TQS_MASK <<
                                             EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT);
    eqos->mtl_regs->txq0_operation_mode |=
        tqs << EQOS_MTL_TXQ0_OPERATION_MODE_TQS_SHIFT;
    eqos->mtl_regs->rxq0_operation_mode &= ~(EQOS_MTL_RXQ0_OPERATION_MODE_RQS_MASK <<
                                             EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT);
    eqos->mtl_regs->rxq0_operation_mode |=
        rqs << EQOS_MTL_RXQ0_OPERATION_MODE_RQS_SHIFT;

    /* Flow control used only if each channel gets 4KB or more FIFO */
    if (rqs >= ((4096 / 256) - 1)) {
        uint32_t rfd, rfa;

        eqos->mtl_regs->rxq0_operation_mode |= (EQOS_MTL_RXQ0_OPERATION_MODE_EHFC);

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

        eqos->mtl_regs->rxq0_operation_mode &= ~((EQOS_MTL_RXQ0_OPERATION_MODE_RFD_MASK <<
                                                  EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                                                 (EQOS_MTL_RXQ0_OPERATION_MODE_RFA_MASK <<
                                                  EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT));
        eqos->mtl_regs->rxq0_operation_mode |= (rfd <<
                                                EQOS_MTL_RXQ0_OPERATION_MODE_RFD_SHIFT) |
                                               (rfa <<
                                                EQOS_MTL_RXQ0_OPERATION_MODE_RFA_SHIFT);
    }

    dma_ie = (uint32_t *)(eqos->regs + 0xc30);
    *dma_ie = 0x3020100;

    /* Configure MAC, not sure if L4T is the same */
    eqos->mac_regs->rxq_ctrl0 =
        (eqos->config->config_mac <<
         EQOS_MAC_RXQ_CTRL0_RXQ0EN_SHIFT);

    /* Set TX flow control parameters */
    /* Set Pause Time */
    eqos->mac_regs->q0_tx_flow_ctrl = (0xffff << EQOS_MAC_Q0_TX_FLOW_CTRL_PT_SHIFT);
    /* Assign priority for RX flow control */
    eqos->mac_regs->rxq_ctrl2 = (1 << EQOS_MAC_RXQ_CTRL2_PSRQ0_SHIFT);

    /* Enable flow control */
    eqos->mac_regs->q0_tx_flow_ctrl |= (EQOS_MAC_Q0_TX_FLOW_CTRL_TFE);

    eqos->mac_regs->rx_flow_ctrl = (EQOS_MAC_RX_FLOW_CTRL_RFE);

    eqos->mac_regs->configuration &=
        ~(EQOS_MAC_CONFIGURATION_GPSLCE |
          EQOS_MAC_CONFIGURATION_WD |
          EQOS_MAC_CONFIGURATION_JD |
          EQOS_MAC_CONFIGURATION_JE);

    /* PLSEN is set to 1 so that LPI is not initiated */
    // MAC_LPS_PLSEN_WR(1); << this macro below
    uint32_t v = eqos->mac_regs->unused_0ac[9];
    v = (v & (MAC_LPS_RES_WR_MASK_20)) | (((0) & (MAC_LPS_MASK_20)) << 20);
    v = (v & (MAC_LPS_RES_WR_MASK_10)) | (((0) & (MAC_LPS_MASK_10)) << 10);
    v = (v & (MAC_LPS_RES_WR_MASK_4)) | (((0) & (MAC_LPS_MASK_4)) << 4);
    v = ((v & MAC_LPS_PLSEN_WR_MASK) | ((1 & MAC_LPS_PLSEN_MASK) << 18));
    eqos->mac_regs->unused_0ac[9] = v;

    /* Update the MAC address */
    set_mac(eqos, TX2_DEFAULT_MAC);

    eqos->mac_regs->configuration &= 0xffcfff7c;
    eqos->mac_regs->configuration |=  DWCEQOS_MAC_CFG_TE | DWCEQOS_MAC_CFG_RE;

    /* Configure DMA */
    /* Enable OSP mode */
    eqos->dma_regs->ch0_tx_control = EQOS_DMA_CH0_TX_CONTROL_OSP;

    /* RX buffer size. Must be a multiple of bus width */
    eqos->dma_regs->ch0_rx_control = (EQOS_MAX_PACKET_SIZE << EQOS_DMA_CH0_RX_CONTROL_RBSZ_SHIFT);

    eqos->dma_regs->ch0_control = (EQOS_DMA_CH0_CONTROL_PBLX8);

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
    eqos->dma_regs->ch0_tx_control &=
        ~(EQOS_DMA_CH0_TX_CONTROL_TXPBL_MASK <<
          EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);
    eqos->dma_regs->ch0_tx_control |= (pbl << EQOS_DMA_CH0_TX_CONTROL_TXPBL_SHIFT);

    eqos->dma_regs->ch0_rx_control &=
        ~(EQOS_DMA_CH0_RX_CONTROL_RXPBL_MASK <<
          EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);
    eqos->dma_regs->ch0_rx_control |= (1 << EQOS_DMA_CH0_RX_CONTROL_RXPBL_SHIFT);

    /* DMA performance configuration */
    val = (2 << EQOS_DMA_SYSBUS_MODE_RD_OSR_LMT_SHIFT) |
          EQOS_DMA_SYSBUS_MODE_EAME | EQOS_DMA_SYSBUS_MODE_BLEN16 |
          EQOS_DMA_SYSBUS_MODE_BLEN8;
    eqos->dma_regs->sysbus_mode = val;

    eqos->dma_regs->ch0_txdesc_list_haddress = 0;
    eqos->dma_regs->ch0_txdesc_list_address = tx.phys;
    eqos->dma_regs->ch0_txdesc_ring_length = TX_COUNT - 1;

    eqos->dma_regs->ch0_rxdesc_list_haddress = 0;
    eqos->dma_regs->ch0_rxdesc_list_address = rx.phys;
    eqos->dma_regs->ch0_rxdesc_ring_length = RX_COUNT - 1;
    
    print("init: ch dma ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");    
    
    eqos->dma_regs->ch0_dma_ie = 0;

    print("init: ch dma ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");    

    eqos->dma_regs->ch0_dma_ie = DWCEQOS_DMA_CH0_IE_RIE | DWCEQOS_DMA_CH0_IE_TIE |
                                 DWCEQOS_DMA_CH0_IE_NIE | DWCEQOS_DMA_CH0_IE_AIE |
                                 DWCEQOS_DMA_CH0_IE_FBEE | DWCEQOS_DMA_CH0_IE_RWTE;
    
    print("init: ch dma ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");    

    eqos->dma_regs->ch0_dma_rx_int_wd_timer = 120;
    udelay(100);

    eqos->dma_regs->ch0_tx_control = EQOS_DMA_CH0_TX_CONTROL_ST;
    eqos->dma_regs->ch0_rx_control = EQOS_DMA_CH0_RX_CONTROL_SR;

    // last_rx_desc = (rx.phys + ((EQOS_DESCRIPTORS_RX) * (uintptr_t)(sizeof(struct eqos_desc))));
    // last_tx_desc = (tx.phys + ((EQOS_DESCRIPTORS_TX) * (uintptr_t)(sizeof(struct eqos_desc))));

    /* Disable MMC event counters */
    *(uint32_t *)(eqos->regs + REG_DWCEQOS_ETH_MMC_CONTROL) |= REG_DWCEQOS_MMC_CNTFREEZ;

    return;

}

void init_post()
{
    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, NULL, 0);

    fill_rx_bufs();

    /* enable events */
    eqos_dma_enable_rxirq(eqos);

    // do we need tx?
    eqos_dma_enable_txirq(eqos);


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
    udelay(100000);
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
            sel4cp_mr_set(0, eqos->mac_regs->address0_low);
            sel4cp_mr_set(1, eqos->mac_regs->address0_high);
            return sel4cp_msginfo_new(0, 2);
        case TX_CH:
            sel4cp_dbg_puts("In protected send\n");
            handle_tx(eqos);
            sel4cp_dbg_puts("After: In protected send\n");

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
            sel4cp_dbg_puts("===> RX irq invoked\n");
            handle_eth(eqos);


            have_signal = true;
            signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
            signal = (BASE_IRQ_CAP + IRQ_CH);
            return;
        case INIT:
            init_post();
            break;
        case TX_CH:
            sel4cp_dbg_puts("===> In notified send\n");
            handle_tx(eqos);            
            sel4cp_dbg_puts("===> After: In notified send\n");
            break;
        default:
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            print("ch =");
            puthex64(ch);
            print("\n");
            break;
    }
}
