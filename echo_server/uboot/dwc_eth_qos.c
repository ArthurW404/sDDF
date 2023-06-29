/*
 * SPDX-License-Identifier: GPL-2.0-only
 */

/*
 * Copyright (c) 2016, NVIDIA CORPORATION.
 *
 * Portions based on U-Boot's rtl8169.c.
 */

/*
 * This driver supports the Synopsys Designware Ethernet QOS (Quality Of
 * Service) IP block. The IP supports multiple options for bus type, clocking/
 * reset structure, and feature list.
 *
 * The driver is written such that generic core logic is kept separate from
 * configuration-specific logic. Code that interacts with configuration-
 * specific resources is split out into separate functions to avoid polluting
 * common code. If/when this driver is enhanced to support multiple
 * configurations, the core code should be adapted to call all configuration-
 * specific functions through function pointers, with the definition of those
 * function pointers being supplied by struct udevice_id eqos_ids[]'s .data
 * field.
 *
 * The following configurations are currently supported:
 * tegra186:
 *    NVIDIA's Tegra186 chip. This configuration uses an AXI master/DMA bus, an
 *    AHB slave/register bus, contains the DMA, MTL, and MAC sub-blocks, and
 *    supports a single RGMII PHY. This configuration also has SW control over
 *    all clock and reset signals to the HW block.
 */

#include "miiphy.h"
#include "net.h"
#include "phy.h"

#include "wait_bit.h"
#include "io.h"
#include "dwc_eth_qos.h"
#include "tx2_configs.h"
#include "util.h"
#include "eth.h"
#include "reset.h"
#include "clock.h"
#include "gpio.h"

#include <string.h>
// #include "dwc_eth_qos.h"

#define UNUSED       __attribute__((__unused__))

uintptr_t hw_ring_buffer_paddr;

// statically assign memory
static clock_sys_t clock_sys = {0};
static tx2_clk_t tx2_clk = {0};
static gpio_sys_t gpio_sys = {0};
static reset_sys_t reset_sys = {0};
static tx2_reset_t tx2_reset = {0};
static clk_t clk_slave_bus = {0};
static clk_t clk_rx = {0};
static clk_t clk_ptp_ref = {0};
static clk_t clk_tx = {0};

static uint8_t mac[6];

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

    __sync_synchronize();

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

    __sync_synchronize();

    print("after: txirq eqos->dma_regs->ch0_dma_ie = ");
    puthex64(eqos->dma_regs->ch0_dma_ie);
    print("\n");

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
    if (eqos->rx->remain != 0) {
        // calculate tail position
        uintptr_t last_rx_desc = eqos->rx->phys + eqos->rx->tail * sizeof(struct eqos_desc);
        eqos->dma_regs->ch0_rxdesc_tail_pointer = last_rx_desc;
    }
}


#define TX_IRQ BIT(0)
#define RX_IRQ BIT(1)

int eqos_handle_irq(struct eqos_priv *eqos, int irq)
{
    // struct eqos_priv *eqos = (struct eqos_priv *)dev->eth_dev;

    uint32_t cause = eqos->dma_regs->dma_control[0];
    uint32_t *dma_status;
    int ret = 0;

    if (cause & DWCEQOS_DMA_IS_DC0IS) {
        dma_status = (uint32_t *)(eqos->regs + REG_DWCEQOS_DMA_CH0_STA);
        print("-->In DWCEQOS_DMA_IS_DC0IS\n");
        /* Transmit Interrupt */
        if (*dma_status & DWCEQOS_DMA_CH0_IS_TI) {
            print("-->is TX interrupt\n");
            ret |= TX_IRQ;
        }

        /* Receive Interrupt */
        if (*dma_status & DWCEQOS_DMA_CH0_IS_RI) {
            print("-->is RX interrupt\n");
            ret |= RX_IRQ;
        }

        /* Ack */
        *dma_status = *dma_status;
    }

    return ret;
}

static int eqos_mdio_wait_idle(struct eqos_priv *eqos)
{
    return wait_for_bit_le32(&eqos->mac_regs->mdio_address,
                             EQOS_MAC_MDIO_ADDRESS_GB, false,
                             1000000, true);
}

static int eqos_mdio_read(struct mii_dev *bus, int mdio_addr, int mdio_devad,
                          int mdio_reg)
{
    struct eqos_priv *eqos = bus->priv;
    uint32_t val;
    int ret;

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        print("MDIO not idle at entry");
        return ret;
    }
    
    val = eqos->mac_regs->mdio_address;
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
           EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
           (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
           (eqos->config->config_mac_mdio <<
            EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_GOC_READ <<
            EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
           EQOS_MAC_MDIO_ADDRESS_GB;
    eqos->mac_regs->mdio_address = val;

    udelay(eqos->config->mdio_wait);

    __sync_synchronize();
    ret = eqos_mdio_wait_idle(eqos);

    
    if (ret) {
        print("MDIO read didn't complete");
        return ret;
    }

    val = eqos->mac_regs->mdio_data;
    val &= EQOS_MAC_MDIO_DATA_GD_MASK;

    return val;
}

static int eqos_mdio_write(struct mii_dev *bus, int mdio_addr, int mdio_devad,
                           int mdio_reg, u16 mdio_val)
{
    struct eqos_priv *eqos = bus->priv;
    u32 val;
    int ret;

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        print("MDIO not idle at entry");
        return ret;
    }

    writel(mdio_val, &eqos->mac_regs->mdio_data);

    val = readl(&eqos->mac_regs->mdio_address);
    val &= EQOS_MAC_MDIO_ADDRESS_SKAP |
           EQOS_MAC_MDIO_ADDRESS_C45E;
    val |= (mdio_addr << EQOS_MAC_MDIO_ADDRESS_PA_SHIFT) |
           (mdio_reg << EQOS_MAC_MDIO_ADDRESS_RDA_SHIFT) |
           (eqos->config->config_mac_mdio <<
            EQOS_MAC_MDIO_ADDRESS_CR_SHIFT) |
           (EQOS_MAC_MDIO_ADDRESS_GOC_WRITE <<
            EQOS_MAC_MDIO_ADDRESS_GOC_SHIFT) |
           EQOS_MAC_MDIO_ADDRESS_GB;
    writel(val, &eqos->mac_regs->mdio_address);

    udelay(eqos->config->mdio_wait);

    __sync_synchronize();

    ret = eqos_mdio_wait_idle(eqos);
    if (ret) {
        print("MDIO read didn't complete");
        return ret;
    }

    return 0;
}

static int eqos_start_clks_tegra186(struct eqos_priv *eqos)
{
    int ret;
    sel4cp_dbg_puts("==clk==> In eqos_start_clks_tegra186\n");
    // assert(clock_sys_valid(eqos->clock_sys));

    eqos->clk_slave_bus = tx2_car_get_clock(eqos->clock_sys, CLK_AXI_CBB, &clk_slave_bus);
    if (eqos->clk_slave_bus == NULL) {
        sel4cp_dbg_puts("tx2_car_get_clock failed CLK_SLAVE_BUS");
        return -ENODEV;
    }

    sel4cp_dbg_puts("|eqos_start_clks_tegra186| Before gate enable\n");

    print("eqos->clock_sys =");
    puthex64(eqos->clock_sys);
    print("\n");


    ret = tx2_car_gate_enable(eqos->clock_sys, CLK_GATE_AXI_CBB, CLKGATE_ON);
    if (ret) {
        sel4cp_dbg_puts("Failed to enable CLK_GATE_AXI_CBB") ;
        return -EIO;
    }

    ret = tx2_car_gate_enable(eqos->clock_sys, CLK_GATE_EQOS_AXI, CLKGATE_ON);
    if (ret) {
        sel4cp_dbg_puts("Failed to enable CLK_GATE_EQOS_AXI");
        return -EIO;
    }

    eqos->clk_rx = tx2_car_get_clock(eqos->clock_sys, CLK_EQOS_RX_INPUT, &clk_rx);
    if (eqos->clk_rx == NULL) {
        sel4cp_dbg_puts("tx2_car_get_clock failed CLK_RX");
        return -ENODEV;
    }
    freq_t clk_rx_freq = eqos->clk_rx->get_freq(eqos->clk_rx);

    print("clk_rx_freq = ");
    puthex64(clk_rx_freq);
    print("\n");

    ret = tx2_car_gate_enable(eqos->clock_sys, CLK_GATE_EQOS_RX, CLKGATE_ON);
    if (ret) {
        sel4cp_dbg_puts("Failed to enable CLK_GATE_EQOS_RX");
        return -EIO;
    }

    eqos->clk_ptp_ref = tx2_car_get_clock(eqos->clock_sys, CLK_EQOS_PTP_REF, &clk_ptp_ref);
    if (eqos->clk_ptp_ref == NULL) {
        sel4cp_dbg_puts("tx2_car_get_clock failed CLK_EQOS_PTP_REF");
        return -ENODEV;
    }


    ret = tx2_car_gate_enable(eqos->clock_sys, CLK_GATE_EQOS_PTP_REF, CLKGATE_ON);
    if (ret) {
        sel4cp_dbg_puts("Failed to enable CLK_GATE_EQOS_PTP_REF");
        return -EIO;
    }


    eqos->clk_tx = tx2_car_get_clock(eqos->clock_sys, CLK_EQOS_TX, &clk_tx);
    if (eqos->clk_tx == NULL) {
        sel4cp_dbg_puts("tx2_car_get_clock failed CLK_TX");
        return -ENODEV;
    }

    freq_t clk_tx_freq = eqos->clk_tx->get_freq(eqos->clk_tx);

    print("clk_tx_freq = ");
    puthex64(clk_tx_freq);
    print("\n");


    ret = tx2_car_gate_enable(eqos->clock_sys, CLK_GATE_EQOS_TX, CLKGATE_ON);
    if (ret) {
        sel4cp_dbg_puts("Failed to enable CLK_GATE_EQOS_TX");
        return -EIO;
    }

    return 0;
}


static int eqos_calibrate_pads_tegra186(struct eqos_priv *eqos)
{
    int ret;

    print("initial eqos_calibrate_pads_tegra186: eqos->tegra186_regs->auto_cal_status = ");
    puthex64(eqos->tegra186_regs->auto_cal_status);
    print("\n");

    eqos->tegra186_regs->sdmemcomppadctrl |= (EQOS_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD);

    udelay(1);

    print("before cal_config: eqos->tegra186_regs->auto_cal_status = ");
    puthex64(eqos->tegra186_regs->auto_cal_status);
    print("\n");
    

    eqos->tegra186_regs->auto_cal_config |= (EQOS_AUTO_CAL_CONFIG_START | EQOS_AUTO_CAL_CONFIG_ENABLE);

    ret = wait_for_bit_le32(&eqos->tegra186_regs->auto_cal_status,
                            EQOS_AUTO_CAL_STATUS_ACTIVE, true, 10, false);

    print("after wait: eqos->tegra186_regs->auto_cal_status = ");
    puthex64(eqos->tegra186_regs->auto_cal_status);
    print("\n");
    
    if (ret) {
        sel4cp_dbg_puts("calibrate didn't start\n");
        goto failed;
    } else {
        sel4cp_dbg_puts("calibrate started!!!\n");
    }
    
    ret = wait_for_bit_le32(&eqos->tegra186_regs->auto_cal_status,
                            EQOS_AUTO_CAL_STATUS_ACTIVE, false, 100, false);
    if (ret) {
        sel4cp_dbg_puts("calibrate didn't finish\n");
        goto failed;
    } else {
        sel4cp_dbg_puts("calibrate finished!!!\n");
    }

    ret = 0;

failed:
    eqos->tegra186_regs->sdmemcomppadctrl &= ~(EQOS_SDMEMCOMPPADCTRL_PAD_E_INPUT_OR_E_PWRD);

    return ret;
}

static int eqos_disable_calibration_tegra186(struct eqos_priv *eqos)
{

    eqos->tegra186_regs->auto_cal_config &= ~(EQOS_AUTO_CAL_CONFIG_ENABLE);

    return 0;
}

static freq_t eqos_get_tick_clk_rate_tegra186(struct eqos_priv *eqos)
{
    return eqos->clk_rx->get_freq(eqos->clk_slave_bus);
}


static int eqos_set_full_duplex(struct eqos_priv *eqos)
{

    eqos->mac_regs->configuration |= (EQOS_MAC_CONFIGURATION_DM);

    return 0;
}

static int eqos_set_half_duplex(struct eqos_priv *eqos)
{

    eqos->mac_regs->configuration &= ~(EQOS_MAC_CONFIGURATION_DM);

    /* WAR: Flush TX queue when switching to half-duplex */
    eqos->mtl_regs->txq0_operation_mode |= (EQOS_MTL_TXQ0_OPERATION_MODE_FTQ);

    return 0;
}

static int eqos_set_gmii_speed(struct eqos_priv *eqos)
{

    eqos->mac_regs->configuration &= ~(EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_100(struct eqos_priv *eqos)
{

    eqos->mac_regs->configuration |=
        (EQOS_MAC_CONFIGURATION_PS | EQOS_MAC_CONFIGURATION_FES);

    return 0;
}

static int eqos_set_mii_speed_10(struct eqos_priv *eqos)
{

    eqos->mac_regs->configuration &= ~(EQOS_MAC_CONFIGURATION_FES);
    eqos->mac_regs->configuration |= (EQOS_MAC_CONFIGURATION_PS);

    return 0;
}

static int eqos_set_tx_clk_speed_tegra186(struct eqos_priv *eqos)
{
    ulong rate;
    int ret;

    switch (eqos->phy->speed) {
    case SPEED_1000:
        rate = 125 * 1000 * 1000;
        break;
    case SPEED_100:
        rate = 25 * 1000 * 1000;
        break;
    case SPEED_10:
        rate = 2.5 * 1000 * 1000;
        break;
    default:
        // print("invalid speed %d", eqos->phy->speed);
        print("invalid speed ");
        puthex64(eqos->phy->speed);
        print("\n");
        return -EINVAL;
    }
    
    ret = eqos->clk_tx->set_freq(eqos->clk_tx, rate);

    if (ret < 0) {
        print("clk_set_rate(tx_clk, %lu) failed: %d\n");
        // print("clk_set_rate(tx_clk, %lu) failed: %d", rate, ret);
        return ret;
    }

    return 0;
}

static int eqos_adjust_link(struct eqos_priv *eqos)
{
    int ret;
    bool en_calibration;


    if (eqos->phy->duplex) {
        ret = eqos_set_full_duplex(eqos);
    } else {
        ret = eqos_set_half_duplex(eqos);
    }
    if (ret < 0) {
        return ret;
    }

    switch (eqos->phy->speed) {
    case SPEED_1000:
        print("eth Speed 1000\n");
        en_calibration = true;
        ret = eqos_set_gmii_speed(eqos);
        break;
    case SPEED_100:
        print("eth Speed 100\n");
        en_calibration = true;
        ret = eqos_set_mii_speed_100(eqos);
        break;
    case SPEED_10:

        en_calibration = false;
        ret = eqos_set_mii_speed_10(eqos);
        break;
    default:
        return -EINVAL;
    }
    if (ret < 0) {
        return ret;
    }

    print("|eqos_adjust_link|before eqos_set_tx_clk_speed_tegra186\n");
    ret = eqos_set_tx_clk_speed_tegra186(eqos);
    if (ret < 0) {
        print("eqos_set_tx_clk_speed() failed: %d\n");
        return ret;
    }

    if (en_calibration) {
        print("|eqos_adjust_link|before eqos_calibrate_pads_tegra186\n");
        ret = eqos_calibrate_pads_tegra186(eqos);
        if (ret < 0) {
            print("eqos_calibrate_pads() failed: %d\n");
            return ret;
        }
    } else {
        ret = eqos_disable_calibration_tegra186(eqos);
        if (ret < 0) {
            return ret;
        }
    }


    return 0;
}

int eqos_send(struct eqos_priv *eqos, void *packet, int length)
{
    print("In eqos send\n");

    volatile struct eqos_desc *tx_desc;
    // uint32_t ioc = 0;
    // if (eqos->tx->tail % 32 == 0) {
    //     ioc = EQOS_DESC2_IOC;
    // }

    print("Before update ring slot\n");

    tx_desc = &(eqos->tx->descr[eqos->tx->tail]);
    ring_ctx_t *ring = eqos->tx;
    
    print("tx_desc (should equal tx.decr at the start) = ");
    puthex64(tx_desc);
    print("\n");

    print("eqos->tx->tail = ");
    puthex64(eqos->tx->tail);
    print("\n");

    print("|eqos_send|packet =");
    puthex64(packet);
    print("\n");
    // print("packet = ");
    // printn(packet, length);
    // print("\n");
    // update_ring_slot(ring, tx.tail, (uintptr_t)packet, ioc | length, EQOS_DESC3_FD | EQOS_DESC3_LD | length);

    tx_desc->des0 = (uintptr_t)packet;
    tx_desc->des1 = 0;
    // tx_desc->des2 = ioc | length;
    tx_desc->des2 = length;

    print("After update ring slot\n");

    __sync_synchronize();

    tx_desc->des3 = EQOS_DESC3_FD | EQOS_DESC3_LD | length;
    tx_desc->des3 |= EQOS_DESC3_OWN;

    print("right before dma_reg update\n");
    // eqos->dma_regs->ch0_txdesc_tail_pointer = (uintptr_t)(&(eqos->tx->descr[eqos->tx->tail + 1])) +
    //                                           sizeof(struct eqos_desc);
    eqos->dma_regs->ch0_txdesc_tail_pointer = (uintptr_t)(tx_desc + 1);
    
    print("after dma_reg update\n");

    print("&eqos->dma_regs->ch0_txdesc_tail_pointer =");
    puthex64(&eqos->dma_regs->ch0_txdesc_tail_pointer);
    print("\n");

    print("eqos->dma_regs->ch0_txdesc_list_address = ");
    puthex64(eqos->dma_regs->ch0_txdesc_list_address);
    print("\n");


    print("Updated tail pointer =");
    puthex64(eqos->dma_regs->ch0_txdesc_tail_pointer);
    print("\n");

    for (int i = 0; i < 1000000; i++) {
		// eqos->config->ops->eqos_inval_desc(tx_desc);
		if (!(tx_desc->des3 & EQOS_DESC3_OWN)) {
            sel4cp_dbg_puts("Something happened!\n");
			return 0;
        }
		udelay(1);
	}

    print("Eqos send timedout\n");

    return 0;
}

static const struct eqos_config eqos_tegra186_config = {
    .reg_access_always_ok = false,
    .mdio_wait = 10,
    .swr_wait = 10,
    .config_mac = EQOS_MAC_RXQ_CTRL0_RXQ0EN_ENABLED_DCB,
    .config_mac_mdio = EQOS_MAC_MDIO_ADDRESS_CR_20_35,
};

static int eqos_start_resets_tegra186(struct eqos_priv *eqos)
{
    int ret;

    ret = gpio_set(&eqos->gpio);
    if (ret < 0) {
        // print("dm_gpio_set_value(phy_reset, assert) failed: %d", ret);
        print("dm_gpio_set_value(phy_reset, assert) failed: %d");
        return ret;
    }

    udelay(2);

    ret = gpio_clr(&eqos->gpio);
    if (ret < 0) {
        // print("dm_gpio_set_value(phy_reset, deassert) failed: %d", ret);
        print("dm_gpio_set_value(phy_reset, deassert) failed: %d");
        return ret;
    }

    ret = reset_sys_assert(eqos->reset_sys, RESET_EQOS);
    if (ret < 0) {
        // print("reset_assert() failed: %d", ret);
        print("reset_assert() failed: %d\n");
        return ret;
    }

    udelay(2);

    ret = reset_sys_deassert(eqos->reset_sys, RESET_EQOS);
    if (ret < 0) {
        // print("reset_deassert() failed: %d", ret);
        print("reset_deassert() failed: %d\n");
        return ret;
    }
    return 0;
}

int eqos_start(struct eqos_priv *eqos)
{
    uint32_t *dma_ie;
    uint32_t ret, val, tx_fifo_sz, rx_fifo_sz, tqs, rqs, pbl;
    freq_t rate;

    ret = eqos_start_clks_tegra186(eqos);
    if (ret) {
        sel4cp_dbg_puts("eqos_start_clks_tegra186 failed");
        // goto err;
        return -1;
    }

    get_mac_addr(eqos, mac);
    sel4cp_dbg_puts("MAC before reset: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    ret = eqos_start_resets_tegra186(eqos);
    if (ret) {
        sel4cp_dbg_puts("eqos_start_resets_tegra186 failed");
        // goto err_stop_clks;
        return -1;
    }

    udelay(10);

    // /*
	//  * Assert the SWR first, the actually reset the MAC and to latch in
	//  * e.g. i.MX8M Plus GPR[1] content, which selects interface mode.
	//  */
    // eqos->dma_regs->mode = (EQOS_DMA_MODE_SWR);

    ret = wait_for_bit_le32(&eqos->dma_regs->mode,
                            EQOS_DMA_MODE_SWR, false,
                            eqos->config->swr_wait, false);

    if (ret) {
        sel4cp_dbg_puts("EQOS_DMA_MODE_SWR stuck");
        // goto err_stop_resets;
        return -1;
    }

    // udelay(10000);

    ret = eqos_calibrate_pads_tegra186(eqos);
    if (ret < 0) {
        sel4cp_dbg_puts("eqos_calibrate_pads() failed: %d");
        // goto err_stop_resets;
        return -1;
    }

    sel4cp_dbg_puts("After calibrate pads\n");

	// rate = eqos_get_tick_clk_rate_tegra186(eqos);
	// val = (rate / 1000000) - 1;
	// writel(val, &eqos->mac_regs->us_tic_counter);

    // __sync_synchronize();

    /*
     * if PHY was already connected and configured,
     * don't need to reconnect/reconfigure again
     */
    if (!eqos->phy) {
        eqos->phy = phy_connect(eqos->mii, 0, NULL, PHY_INTERFACE_MODE_MII);
        
        if (!eqos->phy) {
            print("phy_connect() failed");
            // goto err_stop_resets;
            return -1;
        }

        ret = phy_config(eqos->phy);
        if (ret < 0) {
            // print("phy_config() failed: %d", ret);
            print("phy_config() failed: %d\n");
            return -1;
        }
    }

    if (!eqos->phy)
        sel4cp_dbg_puts("For some reason the phy is not on????\n");
    
    ret = phy_startup(eqos->phy);
    if (ret < 0) {
        // print("phy_startup() failed: %d", ret);
        print("phy_startup() failed: %d\n");
        return -1;
    }

    if (!eqos->phy->link) {
        print("No link");
        return -1;
    }

    ret = eqos_adjust_link(eqos);
    if (ret < 0) {
        print("eqos_adjust_link() failed: %d\n");

        // goto err_shutdown_phy;
        return -1;
    }



    get_mac_addr(eqos, mac);
    sel4cp_dbg_puts("MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    udelay(10);

    ret = wait_for_bit_le32(&eqos->dma_regs->mode,
                            EQOS_DMA_MODE_SWR, false,
                            eqos->config->swr_wait, false);

    volatile uint32_t *dma_status = (uint32_t *)(eqos->regs + REG_DWCEQOS_DMA_CH0_STA);
    
    print("dma_status: ");
    puthex64(*dma_status);
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
        u32 rfd, rfa;

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


    // /* Update the MAC address */
    // memcpy(eqos->enetaddr, TX2_DEFAULT_MAC, 6);
    // uint32_t val1 = (eqos->enetaddr[5] << 8) | (eqos->enetaddr[4]);
    // eqos->mac_regs->address0_high = val1;
    // val1 = (eqos->enetaddr[3] << 24) | (eqos->enetaddr[2] << 16) |
    //        (eqos->enetaddr[1] << 8) | (eqos->enetaddr[0]);
    // eqos->mac_regs->address0_low = val1;


    sel4cp_dbg_puts("Orig MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

    // /* Update the MAC address */
    // set_mac(eqos, TX2_DEFAULT_MAC);

    unsigned char enetaddr[ARP_HLEN];
    memcpy(enetaddr, TX2_DEFAULT_MAC, 6);
    uint32_t val1 = (enetaddr[5] << 8) | (enetaddr[4]);
    eqos->mac_regs->address0_high = val1;
    val1 = (enetaddr[3] << 24) | (enetaddr[2] << 16) |
           (enetaddr[1] << 8) | (enetaddr[0]);
    eqos->mac_regs->address0_low = val1;

    __sync_synchronize();

    sel4cp_dbg_puts("Updated MAC: ");
    dump_mac(mac);
    sel4cp_dbg_puts("\n");

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
    eqos->dma_regs->ch0_txdesc_list_address = hw_ring_buffer_paddr + (sizeof(struct eqos_desc) * RX_COUNT);
    eqos->dma_regs->ch0_txdesc_ring_length = TX_COUNT - 1;

    eqos->dma_regs->ch0_rxdesc_list_haddress = 0;
    eqos->dma_regs->ch0_rxdesc_list_address = hw_ring_buffer_paddr;
    eqos->dma_regs->ch0_rxdesc_ring_length = RX_COUNT - 1;
  
    
    eqos->dma_regs->ch0_dma_ie = 0;
    eqos->dma_regs->ch0_dma_ie = DWCEQOS_DMA_CH0_IE_RIE | DWCEQOS_DMA_CH0_IE_TIE |
                                 DWCEQOS_DMA_CH0_IE_NIE | DWCEQOS_DMA_CH0_IE_AIE |
                                 DWCEQOS_DMA_CH0_IE_FBEE | DWCEQOS_DMA_CH0_IE_RWTE;
    eqos->dma_regs->ch0_dma_rx_int_wd_timer = 120;
    udelay(100);

    eqos->dma_regs->ch0_tx_control = EQOS_DMA_CH0_TX_CONTROL_ST;
    eqos->dma_regs->ch0_rx_control = EQOS_DMA_CH0_RX_CONTROL_SR;

    // eqos->last_rx_desc = (d->rx_ring_phys + ((EQOS_DESCRIPTORS_RX) * (uintptr_t)(sizeof(struct eqos_desc))));
    // eqos->last_tx_desc = (d->tx_ring_phys + ((EQOS_DESCRIPTORS_TX) * (uintptr_t)(sizeof(struct eqos_desc))));

    /* Disable MMC event counters */
    *(uint32_t *)(eqos->regs + REG_DWCEQOS_ETH_MMC_CONTROL) |= REG_DWCEQOS_MMC_CNTFREEZ;

    // eqos->dma_regs->ch0_tx_control |= EQOS_DMA_CH0_TX_CONTROL_ST;
    // eqos->dma_regs->ch0_rx_control |= EQOS_DMA_CH0_RX_CONTROL_SR;
    // eqos->mac_regs->configuration |= EQOS_MAC_CONFIGURATION_TE | EQOS_MAC_CONFIGURATION_RE;
    // // last_rx_desc = (eqos->rx->phys + ((EQOS_DESCRIPTORS_RX) * (uintptr_t)(sizeof(struct eqos_desc))));
    // // last_tx_desc = (tx.phys + ((EQOS_DESCRIPTORS_TX) * (uintptr_t)(sizeof(struct eqos_desc))));

    // /* Disable MMC event counters */
    // *(uint32_t *)(eqos->regs + REG_DWCEQOS_ETH_MMC_CONTROL) |= REG_DWCEQOS_MMC_CNTFREEZ;
    return 0;
}

static int tx2_initialise_hardware(struct eqos_priv *eqos)
{
    print("|tx2_initialise_hardware| called\n");
    uint32_t ret;

    // seems to just initialise bpmp and assign clock register maps and clock
    eqos->clock_sys = &clock_sys;
    eqos->clock_sys->priv = &tx2_clk;
    ret = clock_sys_init(eqos->clock_sys);
    if (ret) {
        sel4cp_dbg_puts("eqos_start_clks_tegra186 failed");
        return -1;
    }
    tx2_clk_t *clk = &tx2_clk; 

    eqos->reset_sys = &reset_sys;
    tx2_reset_t *reset = &tx2_reset;
    eqos->reset_sys->data = reset;
    // use clock's bpmp 
    ret = reset_sys_init(clk->bpmp, eqos->reset_sys);
    if (ret) {
        print("failed reset sys init\n");
        return -1;
    }

    eqos->gpio_sys = &gpio_sys;
    ret = gpio_sys_init(eqos->gpio_sys);
    if (ret) {
        // goto fail;
        print("failed gpio sys init\n");
        return -1;
    }
    return 0;
}

void *tx2_initialise(struct eqos_priv *eqos,uintptr_t base_addr)
{
    print("|tx2_initialise| called\n");

    int ret;

    /* initialise miiphy */
    miiphy_init();

    /* initialise phy */
    ret = phy_init();
    if (ret != 0) {
        print("failed to initialise phy");
    }


    ret = tx2_initialise_hardware(eqos);
    if (ret < 0) {
        sel4cp_dbg_puts("tx2_initialise_hardware() failed");
        // goto err_stop_resets;
        return NULL;
    }

    ret = eqos->gpio_sys->init(eqos->gpio_sys, GPIO_PM4, GPIO_DIR_OUT, &eqos->gpio);
    if (ret != 0) {
        print("failed to init phy reset gpio pin\n");
        return NULL;
    }

    eqos->config = &eqos_tegra186_config;
    // eqos->regs = eth_base_reg;
    eqos->regs = base_addr;

    // setup registers 
    eqos->mac_regs = (void *)(eqos->regs + EQOS_MAC_REGS_BASE);
    eqos->mtl_regs = (void *)(eqos->regs + EQOS_MTL_REGS_BASE);
    eqos->dma_regs = (void *)(eqos->regs + EQOS_DMA_REGS_BASE);
    eqos->tegra186_regs = (void *)(eqos->regs + EQOS_TEGRA186_REGS_BASE);
    
    eqos->mii = mdio_alloc();

    print("eqos->mii = ");
    puthex64(eqos->mii);
    print("\n");

    if (!eqos->mii) {
        print("Mdio alloc failed\n");
        // goto err;
        return NULL;

    }

    eqos->mii->read = eqos_mdio_read;
    eqos->mii->write = eqos_mdio_write;
    eqos->mii->priv = eqos;
    strcpy(eqos->mii->name, "mii\0");

    ret = mdio_register(eqos->mii);
    if (ret < 0) {
        print("Mdio register failed\n");
        // goto err_free_mdio;
        return NULL;
    }

    return (void *)eqos;
err_free_mdio:
    mdio_free(eqos->mii);
err:
    print("Tx2 initialise failed");
    return NULL;

    // return eqos;
}
