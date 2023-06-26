/*
 * Copyright 2022, UNSW
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <sel4cp.h>
#include <sel4/sel4.h>
#include <string.h>

#include "clock.h"
#include "gpio.h"
#include "reset.h"
#include "phy.h"
#include "shared_ringbuffer.h"

#define BIT(n) (1ul<<(n))

/* Make the minimum frame buffer 2k. This is a bit of a waste of memory, but ensures alignment */
#define PACKET_BUFFER_SIZE  2048
#define MAX_PACKET_SIZE     1536

#define RX_COUNT 256
#define TX_COUNT 256

_Static_assert((512 * 2) * PACKET_BUFFER_SIZE <= 0x200000, "Expect rx+tx buffers to fit in single 2MB page");
_Static_assert(sizeof(ring_buffer_t) <= 0x200000, "Expect ring buffer ring to fit in single 2MB page");

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

#include "dwc_eth_qos.h"

/* descriptor flags */
#define EQOS_DESC2_IOC      BIT(31)
#define EQOS_DESC3_OWN      BIT(31)
#define EQOS_DESC3_FD       BIT(29)
#define EQOS_DESC3_LD       BIT(28)
#define EQOS_DESC3_BUF1V    BIT(24)
#define DWCEQOS_DMA_RDES3_INTE    BIT(30)


#define TX2_DEFAULT_MAC "\x00\x04\x4b\xc5\x67\x70"

#define CONFIG_SYS_CACHELINE_SIZE 64

#ifdef CONFIG_SYS_CACHELINE_SIZE
#define ARCH_DMA_MINALIGN   CONFIG_SYS_CACHELINE_SIZE
#else
#define ARCH_DMA_MINALIGN   16
#endif


#define __ALIGN_MASK(x,mask)    (((x)+(mask))&~(mask))

#define EQOS_ALIGN(x,a)      __ALIGN_MASK((x),(typeof(x))(a)-1)
#define EQOS_MAX_PACKET_SIZE    EQOS_ALIGN(1568, ARCH_DMA_MINALIGN)

// struct tx2_eth_data *dev

void eqos_dma_disable_rxirq(struct eqos_priv *eqos);

void eqos_dma_enable_rxirq(struct eqos_priv *eqos);

void eqos_dma_disable_txirq(struct eqos_priv *eqos);

void eqos_dma_enable_txirq(struct eqos_priv *eqos);

void eqos_stop(struct eqos_priv *eqos);

int eqos_start(struct eqos_priv *eqos);

int eqos_send(struct eqos_priv *eqos, void *packet, int length);

int eqos_handle_irq(struct eqos_priv *eqos, int irq);

int eqos_recv(struct eqos_priv *eqos, uintptr_t packetp);

void *tx2_initialise(struct eqos_priv *eqos, uintptr_t base_addr);

void eqos_set_rx_tail_pointer(struct eqos_priv *eqos);

static void get_mac_addr(struct eqos_priv *eqos, uint8_t *mac)
{
    //default one: 00:04:4b:c5:67:70
    // __sync_synchronize();
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

/* MAC HW ADDR regs */
// from linux dwmac4.h
#define GMAC_HI_REG_AE			BIT(31)


static void set_mac(struct eqos_priv *eqos, uint8_t *mac)
{
    // using tx2a mac address since
    unsigned char enetaddr[ARP_HLEN];
    memcpy(enetaddr, TX2_DEFAULT_MAC, 6);
    uint32_t val1 = (enetaddr[5] << 8) | (enetaddr[4]);

    /* For MAC Addr registers se have to set the Address Enaeqos_handle_irqle (AE)
	 * bit that has no effect on the High Reg 0 where the bit 31 (MO)
	 * is RO.
	 */
    eqos->mac_regs->address0_high = val1  | GMAC_HI_REG_AE;
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