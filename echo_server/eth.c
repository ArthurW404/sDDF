/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include "eth.h"
#include "shared_ringbuffer.h"
#include "util.h"
#include "io.h"

#include "timer.h"
#include "miiphy.h"
#include "zynq_gem.h"

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

#define RX_COUNT 128
#define TX_COUNT 128
// #define RX_COUNT 256
// #define TX_COUNT 256

_Static_assert((512 * 2) * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");

typedef struct {
    unsigned int cnt;
    unsigned int remain;
    unsigned int tail;
    unsigned int head;
    volatile struct emac_bd *descr;
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

volatile struct zynq_gem_regs *eth = (void *)(uintptr_t)0x2000000;

static void get_mac_addr(volatile struct zynq_gem_regs *reg, uint8_t *mac)
{
    /* fill enetaddr */
    u32 maclow = readl(&reg->laddr[0][LADDR_LOW]);
    u32 machigh = readl(&reg->laddr[0][LADDR_HIGH]);

    mac[0] = maclow;
    mac[1] = maclow >> 8;
    mac[2] = maclow >> 16;
    mac[3] = maclow >> 24;

    mac[4] = machigh;
    mac[5] = machigh >> 8;
}

static void set_mac(volatile struct zynq_gem_regs *reg, uint8_t *mac)
{
    // reg->palr = (mac[0] << 24) | (mac[1] << 16) | (mac[2] << 8) | (mac[3]);
    // reg->paur = (mac[4] << 24) | (mac[5] << 16);
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

static void update_ring_slot(
    ring_ctx_t *ring,
    unsigned int idx,
    uintptr_t phys,
    uint16_t len,
    uint16_t stat)
{
    volatile struct emac_bd *d = &(ring->descr[idx]);
    d->addr = phys;
    // d->len = len;

    /* Ensure all writes to the descriptor complete, before we set the flags
     * that makes hardware aware of this slot.
     */
    __sync_synchronize();

    d->status = stat;
}

static inline void
enable_irqs(volatile struct zynq_gem_regs *eth, uint32_t mask)
{
    // eth->eimr = mask;
}

static uintptr_t 
alloc_rx_buf(size_t buf_size, void **cookie)
{
    uintptr_t addr;
    unsigned int len;

    /* Try to grab a buffer from the free ring */
    if (driver_dequeue(rx_ring.free_ring, &addr, &len, cookie)) {
        print("RX Free ring is empty\n");
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
        uint32_t stat = 0;
        int idx = ring->tail;
        int new_tail = idx + 1;
        if (new_tail == ring->cnt) {
            new_tail = 0;
            stat |= ZYNQ_GEM_TXBUF_WRAP_MASK;
        }
        ring->cookies[idx] = cookie;
        // update_ring_slot(ring, idx, phys, 0, stat);
        ring->descr[idx].status = stat;

        ring->descr[idx].addr &=  ~(ZYNQ_GEM_RXBUF_NEW_MASK | ZYNQ_GEM_RXBUF_ADD_MASK);

        ring->descr[idx].addr |= (phys & ZYNQ_GEM_RXBUF_ADD_MASK);
        
        __sync_synchronize();

        ring->tail = new_tail;
        /* There is a race condition if add/remove is not synchronized. */
        ring->remain--;
    }
    __sync_synchronize();

    if (ring->tail != ring->head  && !zynq_gem_recv_enabled(eth)) {
        /* Make sure rx is enabled */
        zynq_gem_recv_enable(eth);
    }
}

static void
handle_rx(volatile struct zynq_gem_regs *eth)
{
    ring_ctx_t *ring = &rx;
    unsigned int head = ring->head;

    int num = 1;
    int was_empty = ring_empty(rx_ring.used_ring);

    // we don't want to dequeue packets if we have nothing to replace it with
    while (head != ring->tail && (ring_size(rx_ring.free_ring) > num)) {
        volatile struct emac_bd *d = &(ring->descr[head]);

        unsigned int status = d->status;
        unsigned int addr = d->addr;

        /* Ensure no memory references get ordered before we checked the descriptor was written back */
        __sync_synchronize();
        if (!(addr & ZYNQ_GEM_RXBUF_NEW_MASK)) {
            /* not complete yet */
            break;
        }

        void *cookie = ring->cookies[head];        
        unsigned int len = status & ZYNQ_GEM_RXBUF_LEN_MASK;

        /* Go to next buffer, handle roll-over. */
        if (++head == ring->cnt) {
            head = 0;
        }
        ring->head = head;

        /* There is a race condition here if add/remove is not synchronized. */
        ring->remain++;

        buff_desc_t *desc = (buff_desc_t *)cookie;

        enqueue_used(&rx_ring, desc->encoded_addr, len, desc->cookie);
        num++;
    }

    /* Notify client (only if we have actually processed a packet and 
    the client hasn't already been notified!) */
    if (num > 1 && was_empty) {
        sel4cp_notify(RX_CH);
    } 

    if (ring->tail != ring->head && !zynq_gem_recv_enabled(eth)) {
        zynq_gem_recv_enable(eth);
    }
}

static void
complete_tx(volatile struct zynq_gem_regs *eth)
{
    // unsigned int cnt_org;
    // void *cookie;
    // ring_ctx_t *ring = &tx;
    // unsigned int head = ring->head;
    // unsigned int cnt = 0;

    // while (head != ring->tail) {
    //     if (0 == cnt) {
    //         cnt = tx_lengths[head];
    //         if ((0 == cnt) || (cnt > TX_COUNT)) {
    //             /* We are not supposed to read 0 here. */
    //             print("complete_tx with cnt=0 or max");
    //             return;
    //         }
    //         cnt_org = cnt;
    //         cookie = ring->cookies[head];
    //     }

    //     volatile struct emac_bd *d = &(ring->descr[head]);

    //     /* If this buffer was not sent, we can't release any buffer. */
    //     if (d->stat & TXD_READY) {
    //         /* give it another chance */
    //         if (!(eth->tdar & TDAR_TDAR)) {
    //             eth->tdar = TDAR_TDAR;
    //         }
    //         if (d->stat & TXD_READY) {
    //             break;
    //         }
    //     }

    //     /* Go to next buffer, handle roll-over. */
    //     if (++head == TX_COUNT) {
    //         head = 0;
    //     }

    //     if (0 == --cnt) {
    //         ring->head = head;
    //         /* race condition if add/remove is not synchronized. */
    //         ring->remain += cnt_org;
    //         /* give the buffer back */
    //         buff_desc_t *desc = (buff_desc_t *)cookie;

    //         enqueue_free(&tx_ring, desc->encoded_addr, desc->len, desc->cookie);
    //     }
    // }
}

static void
raw_tx(volatile struct zynq_gem_regs *eth, unsigned int num, uintptr_t *phys,
                  unsigned int *len, void *cookie)
{
    ring_ctx_t *ring = &tx;

    /* Ensure we have room */
    if (ring->remain < num) {
        /* not enough room, try to complete some and check again */
        sel4cp_dbg_puts("not enough room in raw_tx\n");
        complete_tx(eth);
        unsigned int rem = ring->remain;
        if (rem < num) {
            print("TX queue lacks space");
            return;
        }
    }

    __sync_synchronize();

    uintptr_t txbase = ring->phys + (uintptr_t)(ring->tail * sizeof(struct emac_bd));
    
    unsigned int tail = ring->tail;
    unsigned int tail_new = tail;

    unsigned int i = num;
    while (i-- > 0) {
        // uint16_t stat = TXD_READY;
        // if (0 == i) {
        //     stat |= TXD_ADDCRC | TXD_LAST;
        // }

        // unsigned int idx = tail_new;
        // if (++tail_new == TX_COUNT) {
        //     tail_new = 0;
        //     stat |= WRAP;
        // }
        // update_ring_slot(ring, idx, *phys++, *len++, stat);
        unsigned int ring = (tx.tail + i) % tx.cnt;
        tx.descr[ring].addr = phys[i];
        tx.descr[ring].status &= ~(ZYNQ_GEM_TXBUF_USED_MASK | ZYNQ_GEM_TXBUF_FRMLEN_MASK | ZYNQ_GEM_TXBUF_LAST_MASK);
        tx.descr[ring].status |= (len[i] & ZYNQ_GEM_TXBUF_FRMLEN_MASK);
        if (i == (num - 1)) {
            tx.descr[ring].status |= ZYNQ_GEM_TXBUF_LAST_MASK;
        }
    }

    ring->cookies[tail] = cookie;
    tx_lengths[tail] = num;
    ring->tail = tail_new;
    /* There is a race condition here if add/remove is not synchronized. */
    ring->remain -= num;

    __sync_synchronize();

    // if (!(eth->tdar & TDAR_TDAR)) {
    //     eth->tdar = TDAR_TDAR;
    // }
    zynq_gem_start_send(eth, txbase);
    print("Finished sending\n");
}

static void 
handle_eth(volatile struct zynq_gem_regs *eth)
{
    // uint32_t e = eth->eir & IRQ_MASK;
    // /* write to clear events */
    // eth->eir = e;

    // Clear Interrupts
    u32 isr = readl(&eth->isr);
    writel(isr, &eth->isr);
    
    // while (e & IRQ_MASK) {
    if (isr & ZYNQ_GEM_IXR_TXCOMPLETE) {
        sel4cp_dbg_puts("IRQ is a TXCOMPLETE\n");
        /* Clear TX Status register */
        u32 val = readl(&eth->txsr);
        writel(val, &eth->txsr);

        complete_tx(eth);
    }

    if (isr & ZYNQ_GEM_IXR_FRAMERX) {
        sel4cp_dbg_puts("IRQ is a RXCOMPLETE\n");
        /* Clear RX Status register */
        u32 val = readl(&eth->rxsr);
        writel(val, &eth->rxsr);

        handle_rx(eth);
        fill_rx_bufs(eth);
    }

        // do I need this last section for zcu102
        // e = eth->eir & IRQ_MASK;
        // eth->eir = e;
    // }
}

static void 
handle_tx(volatile struct zynq_gem_regs *eth)
{
    uintptr_t buffer = 0;
    unsigned int len = 0;
    void *cookie = NULL;

    // We need to put in an empty condition here. 
    while ((tx.remain > 1) && !driver_dequeue(tx_ring.used_ring, &buffer, &len, &cookie)) {
        
        uintptr_t phys = getPhysAddr(buffer);
        raw_tx(eth, 1, &phys, &len, cookie);
    }
}


struct mii_dev mii_bus;
struct mii_dev *bus = &mii_bus;
char *device_name = "Gem.2000000";

static void 
eth_setup(void)
{
    u32 i;
    int ret;
    struct phy_device *phydev;
    unsigned long clk_rate = 0;
    struct zynq_gem_regs *regs = eth;
    // struct zynq_gem_priv *priv = dev->priv;
    // struct clock *gem_clk;
    const u32 supported = SUPPORTED_10baseT_Half |
                          SUPPORTED_10baseT_Full |
                          SUPPORTED_100baseT_Half |
                          SUPPORTED_100baseT_Full |
                          SUPPORTED_1000baseT_Half |
                          SUPPORTED_1000baseT_Full;


    get_mac_addr(eth, mac);
    sel4cp_dbg_puts("MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    /* set up descriptor rings */
    rx.cnt = RX_COUNT;
    rx.remain = rx.cnt - 2;
    rx.tail = 0;
    rx.head = 0;
    rx.phys = shared_dma_paddr;
    rx.cookies = (void **)rx_cookies;
    rx.descr = (volatile struct emac_bd *)hw_ring_buffer_vaddr;

    for (unsigned int i = 0; i < rx.cnt; i++) {
        rx.descr[i] = (struct emac_bd) {
            .addr = ZYNQ_GEM_RXBUF_NEW_MASK,
            .status = 0
        };
    }

    rx.descr[rx.cnt - 1].addr |= ZYNQ_GEM_RXBUF_WRAP_MASK;

    tx.cnt = TX_COUNT;
    tx.remain = tx.cnt - 2;
    tx.tail = 0;
    tx.head = 0;
    tx.phys = shared_dma_paddr + (sizeof(struct emac_bd) * RX_COUNT);
    tx.cookies = (void **)tx_cookies;
    tx.descr = (volatile struct emac_bd *)(hw_ring_buffer_vaddr + (sizeof(struct emac_bd) * RX_COUNT));

    for (unsigned int i = 0; i < tx.cnt; i++) {
        tx.descr[i] = (struct emac_bd) {
            .addr = 0,
            .status = ZYNQ_GEM_TXBUF_USED_MASK
        };
    }

    tx.descr[tx.cnt - 1].status |= ZYNQ_GEM_TXBUF_WRAP_MASK;

    __sync_synchronize();

    miiphy_init();
    print("|miiphy_init| called\n");

    phy_init();
    print("|phy_init| called\n");

    miiphy_register(device_name, zynq_gem_miiphyread, zynq_gem_miiphy_write);

    bus = miiphy_get_dev_by_name(device_name);

    /* Initialize the buffer descriptor registers */
    writel(lower_32_bits((hw_ring_buffer_paddr + (sizeof(struct emac_bd) * RX_COUNT))), &regs->txqbase);
#if defined(CONFIG_PHYS_64BIT)
    writel(upper_32_bits((hw_ring_buffer_paddr + (sizeof(struct emac_bd) * RX_COUNT))), &regs->upper_txqbase);
#endif
    writel(lower_32_bits(hw_ring_buffer_paddr), &regs->rxqbase);
#if defined(CONFIG_PHYS_64BIT)
    writel(upper_32_bits(hw_ring_buffer_paddr), &regs->upper_rxqbase);
#endif

    print("zynq_gem_init: Start\n");
    bool dma_64bit; 
    if (readl(&regs->dcfg6) & ZYNQ_GEM_DCFG_DBG6_DMA_64B) {
        dma_64bit = true;
    } else {
        dma_64bit = false;
    }

    if (!dma_64bit) {
        print("ERR: %s: Using 64-bit DMA but HW doesn't support it\n");
        return;
    }

    // /* Disable all interrupts */
    // writel(0xFFFFFFFF, &regs->idr);

    // numbering based on trm
    // 1. initialise the controller

    /*  Disable the receiver & transmitter */
    
    // 1.1 clear network control register
    writel(0, &regs->nwctrl);
    
    writel(0xFFFFFFFF, &regs->txsr);
    writel(0xFFFFFFFF, &regs->rxsr);
    writel(0, &regs->phymntnc);


    /* 1.4 Disable all interrupts */
    writel(0x7FFFFFF, &regs->idr);

    // 1.5 clear gem.receive_q{,1}_ptr and  tx regs
    writel(0, &regs->transmit_q1_ptr);
    writel(0, &regs->receive_q1_ptr);

    /* Clear the Hash registers for the mac address
        * pointed by AddressPtr
        */
    writel(0x0, &regs->hashl);
    /* Write bits [63:32] in TOP */
    writel(0x0, &regs->hashh);

    /* Clear all counters */
    for (i = 0; i < STAT_SIZE; i++) {
        readl(&regs->stat[i]);
    }

    /* Setup for DMA Configuration register */
    writel(ZYNQ_GEM_DMACR_INIT, &regs->dmacr);

    /* Setup for Network Control register, MDIO, Rx and Tx enable */
    setbits_le32(&regs->nwctrl, ZYNQ_GEM_NWCTRL_MDEN_MASK);

    // 3. IO config

    // 4. configure PHY

    int phyaddr = -1;
    ret = phy_detection(&phyaddr);
    if (ret) {
        print("GEM PHY init failed\n");
        return ret;
    }

    // printf_("printf: phyaddr = %d\n", phyaddr);

    print("phyaddr = ");
    puthex64(phyaddr);
    print("\n");

    phy_interface_t interface = PHY_INTERFACE_MODE_MII;

    /* interface - look at tsec */
    phydev = phy_connect(bus, phyaddr, NULL,
                         interface);


    phydev->supported = supported | ADVERTISED_Pause |
                        ADVERTISED_Asym_Pause;
    phydev->advertising = phydev->supported;
    // priv->phydev = phydev;
    phy_config(phydev);

    ret = phy_startup(phydev);
    if (ret) {
        print("phy_startup failed!\n");
        return ret;
    }

    if (!phydev->link) {
        print("%s: No link.\n");
        // print("%s: No link.\n", phydev->dev->name);
        return -1;
    }

    switch (phydev->speed) {
    case SPEED_1000:
        print("SPEED_1000\n");
        writel(ZYNQ_GEM_NWCFG_INIT | ZYNQ_GEM_NWCFG_SPEED1000,
               &regs->nwcfg);
        clk_rate = ZYNQ_GEM_FREQUENCY_1000;
        break;
    case SPEED_100:
        print("SPEED_100\n");
        writel(ZYNQ_GEM_NWCFG_INIT | ZYNQ_GEM_NWCFG_SPEED100,
               &regs->nwcfg);
        clk_rate = ZYNQ_GEM_FREQUENCY_100;
        break;
    case SPEED_10:
        clk_rate = ZYNQ_GEM_FREQUENCY_10;
        break;
    }

    /* Note:
     * This is the place to configure the the GEM clock if not using the EMIO interface
     * (!priv->emio). Clock configuration on the ZynqMP is different from the Zynq7000, making
     * porting non-trivial. For now, assume that the bootloader has set appropriate clock settings.
     */

    // 5. configure buffer descriptor

    // 6. interrupts
    // /* Enable IRQs */
    writel((ZYNQ_GEM_IXR_FRAMERX | ZYNQ_GEM_IXR_TXCOMPLETE), &regs->ier);

    // 7. Enable controller 
    // 7.1 enable transmit
    setbits_le32(&regs->nwctrl, ZYNQ_GEM_NWCTRL_TXEN_MASK);
    // enable receiver done in fill_rx_buf (in camkes)

    // prom mode enabled:
    /* Read Current Value and Set CopyAll bit */
    uint32_t status = readl(&regs->nwcfg);
    writel(status | ZYNQ_GEM_NWCFG_COPY_ALL, &regs->nwcfg);

    uint8_t new_mac[6];
    get_mac_addr(eth, new_mac);
    sel4cp_dbg_puts("post init MAC: ");
    dump_mac(new_mac);
    sel4cp_dbg_puts("\n");
}

void init_post()
{
    struct zynq_gem_regs *regs = eth;

    /* Set up shared memory regions */
    ring_init(&rx_ring, (ring_buffer_t *)rx_free, (ring_buffer_t *)rx_used, NULL, 0);
    ring_init(&tx_ring, (ring_buffer_t *)tx_free, (ring_buffer_t *)tx_used, NULL, 0);

    fill_rx_bufs();

    /* Enable IRQs */
    // writel((ZYNQ_GEM_IXR_FRAMERX | ZYNQ_GEM_IXR_TXCOMPLETE), &regs->ier);
    // setbits_le32(&regs->nwctrl, ZYNQ_GEM_NWCTRL_TXEN_MASK);

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
            sel4cp_mr_set(0, readl(&eth->laddr[0][LADDR_LOW]));
            sel4cp_mr_set(1, readl(&eth->laddr[0][LADDR_HIGH]));
            return sel4cp_msginfo_new(0, 2);
        case TX_CH:
            print("In TX_CH protected\n");
            handle_tx(eth);
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
            print("In IRQ_CH\n");
            handle_eth(eth);
            have_signal = true;
            signal_msg = seL4_MessageInfo_new(IRQAckIRQ, 0, 0, 0);
            signal = (BASE_IRQ_CAP + IRQ_CH);
            return;
        case INIT:
            init_post();
            break;
        case TX_CH:
            print("In TX_CH notified\n");

            handle_tx(eth);
            break;
        default:
            sel4cp_dbg_puts("eth driver: received notification on unexpected channel\n");
            print("ch =");
            puthex64(ch);
            print("\n");
            break;
    }
}
