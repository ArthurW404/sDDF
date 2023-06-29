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

struct eqos_mac_regs;
struct eqos_mtl_regs;
struct eqos_dma_regs;
struct eqos_tegra186_regs;

struct eqos_priv {
    const struct eqos_config *config;
    uintptr_t regs;
    struct eqos_mac_regs *mac_regs;
    struct eqos_mtl_regs *mtl_regs;
    struct eqos_dma_regs *dma_regs;
    struct eqos_tegra186_regs *tegra186_regs;
    struct clock *clk_master_bus;
    struct clock *clk_rx;
    struct clock *clk_ptp_ref;
    struct clock *clk_tx;
    struct clock *clk_slave_bus;
    struct mii_dev *mii;
    struct phy_device *phy;
    // uintptr_t last_rx_desc;
    // uintptr_t last_tx_desc;
    // unsigned char enetaddr[ARP_HLEN];
    // bool reg_access_ok;
    // ps_io_ops_t *tx2_io_ops;
    gpio_sys_t *gpio_sys;
    gpio_t gpio;
    reset_sys_t *reset_sys;
    clock_sys_t *clock_sys;


    // sddf attributes
    ring_ctx_t *rx;
    ring_ctx_t *tx;
};
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


/* MAC HW ADDR regs */
// from linux dwmac4.h
#define GMAC_HI_REG_AE			BIT(31)

