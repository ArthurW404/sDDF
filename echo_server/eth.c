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
#include "dwc_eth_qos.h"
#include "shared_ringbuffer.h"
#include "util.h"
#include "clock.h"
#include "gpio.h"
#include "reset.h"
#include "reset-bindings.h"
#include "wait_bit.h"
#include "phy.h"
#include "miiphy.h"
#include "io.h"

#define IRQ_CH 1
#define TX_CH  2
#define RX_CH  2
#define INIT   4

#define MDC_FREQ    20000000UL

/* Memory regions. These all have to be here to keep compiler happy */
uintptr_t hw_ring_buffer_vaddr;
// uintptr_t hw_ring_buffer_paddr;
uintptr_t shared_dma_vaddr;
uintptr_t shared_dma_paddr;
uintptr_t rx_cookies;
uintptr_t tx_cookies;
uintptr_t rx_free;
uintptr_t rx_used;
uintptr_t tx_free;
uintptr_t tx_used;
uintptr_t uart_base;

ring_ctx_t rx;
ring_ctx_t tx;
unsigned int tx_lengths[TX_COUNT];

/* Pointers to shared_ringbuffers */
ring_handle_t rx_ring;
ring_handle_t tx_ring;


volatile void *eth_base_reg = (void *)(uintptr_t)0x2000000;
volatile struct eqos_mac_regs *eth_mac = (void *)(0x2000000 + EQOS_MAC_REGS_BASE);

// struct for eqos registers and device metadata
struct eqos_priv eqos_dev;
struct eqos_priv *eqos = &eqos_dev;

static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint32_t len,
    uint32_t flags)
{
    // sel4cp_dbg_puts("|update_ring_slot| phys =");
    // puthex64(phys);
    // sel4cp_dbg_puts("\n");
    volatile struct eqos_desc *d = &(ring->descr[idx]);
    // print("after getting d\n");
    d->des0 = phys;
    // print("Managed to save phys to des0\n");
    d->des1 = 0;
    d->des2 = len;
    d->des3 = flags;
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

        err = eqos_send(eqos, *phys++, *len++);
        // if (err == -ETIMEDOUT) {
        //     print("send timed out");
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


static void complete_rx(struct eqos_priv *eqos)
{
    // struct tx2_eth_data *dev = (struct tx2_eth_data *)eth_driver->eth_data;
    
    // unsigned int num_in_ring = dev->rx_size - dev->rx_remain;

    // for (int i = 0; i < num_in_ring; i++) {
    //     unsigned int status = dev->rx_ring[dev->rdh].des3;

    //     /* Ensure no memory references get ordered before we checked the descriptor was written back */
    //     __sync_synchronize();
    //     if (status & EQOS_DESC3_OWN) {
    //         /* not complete yet */
    //         break;
    //     }

    //     /* TBD: Need to handle multiple buffers for single frame? */
    //     void *cookie = dev->rx_cookies[dev->rdh];
    //     dev->rx_cookies[dev->rdh] = 0;
    //     unsigned int len = status & 0x7fff;

    //     dev->rx_remain++;
    //     /* update rdh */
    //     dev->rdh = (dev->rdh + 1) % dev->rx_size;

    //     /* Give the buffers back */
    //     eth_driver->i_cb.rx_complete(eth_driver->cb_cookie, 1, &cookie, &len);
    // }
}


static void 
handle_eth(struct eqos_priv *eqos)
{
    // uint32_t val = eqos_handle_irq(eqos, irq);

    // if (val & TX_IRQ) {
    //     eqos_dma_disable_txirq(eth_data);
    //     complete_tx(driver);
    //     eqos_dma_enable_txirq(eth_data);
    // }

    // if (val & RX_IRQ) {
    //     eqos_dma_disable_rxirq(eth_data);
    //     complete_rx(driver);
    //     fill_rx_bufs(driver);
    //     /*
    //      * RX IRQ is was disabled when checking the IRQ, and thus need to be
    //      * re-enabled
    //      */
    //     eqos_dma_enable_rxirq(eth_data);
    // }

    // if (val == 0) {
    //     ZF_LOGD("No TX or RX IRQ, ignoring this interrupt");
    // }  
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

        print("|handle_tx| Buffer = ");
        printn(buffer, len);
        print("\n");

        uintptr_t phys = getPhysAddr(buffer);

        print("|handle_tx| phys = ");
        puthex64(phys);
        print("\n");

        raw_tx(eqos, 1, &phys, &len, cookie);
    }
}


// similar to ethif_tx2_init
static void 
eth_setup(void)
{
    print("|eth_setup| called\n");
    /* set up descriptor rings */
    rx.cnt = RX_COUNT;
    rx.remain = rx.cnt - 2;
    rx.tail = 0;
    rx.head = 0;
    rx.phys = shared_dma_paddr;
    rx.cookies = (void **)rx_cookies;
    rx.descr = (volatile struct eqos_desc *)hw_ring_buffer_vaddr;

    // print("RX descr = ");
    // puthex64(rx.descr);
    // print("\n");

    tx.cnt = TX_COUNT;
    tx.remain = tx.cnt - 2;
    tx.tail = 0;
    tx.head = 0;
    tx.phys = shared_dma_paddr + (sizeof(struct eqos_desc) * RX_COUNT);
    tx.cookies = (void **)tx_cookies;
    tx.descr = (volatile struct eqos_desc *)(hw_ring_buffer_vaddr + (sizeof(struct eqos_desc) * RX_COUNT));
    
    eqos->rx = &rx;
    eqos->tx = &tx;

    // print("TX descr = ");
    // puthex64(tx.descr);
    // print("\n");

    // print("hw_ring_buffer_paddr = ");
    // puthex64(hw_ring_buffer_paddr);
    // print("\n");

    tx2_initialise(eqos, eth_base_reg);

    return;
}

void init_post()
{
    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, NULL, 0);

    fill_rx_bufs();

    eqos_start(eqos);

    // /* enable events */
    // eqos_dma_enable_rxirq(eqos);

    // // do we need tx?
    // eqos_dma_enable_txirq(eqos);

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
        case 0x11:
            eqos_handle_irq(eqos, 0x11);
            have_signal = true;
            signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
            // signal = (BASE_IRQ_CAP + 0x11);
            break;
        default:
            
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            print("ch =");
            puthex64(ch);
            print("\n");
            break;
    }
}
