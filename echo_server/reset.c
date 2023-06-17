/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <errno.h>

// #include <platsupport/reset.h>
// #include <platsupport/plat/reset.h>
#include "reset.h"
#include "reset-bindings.h"
/* NVIDIA interface */
#include "bpmp.h" /* struct mrq_reset_request */
#include "util.h"

// extern uint32_t mrq_reset_id_map[];

/* This maps the reset_ids to the tx2 BPMP reset IDs */

uint32_t mrq_reset_id_map[] = {
    [RESET_TOP_GTE] = TEGRA186_RESET_TOP_GTE,
    [RESET_SHSP] = TEGRA186_RESET_SHSP,
    [RESET_KFUSE] = TEGRA186_RESET_KFUSE,
    [RESET_GPU] = TEGRA186_RESET_GPU,
    [RESET_UPHY] = TEGRA186_RESET_UPHY,
    [RESET_SPI3] = TEGRA186_RESET_SPI3,
    [RESET_I2C1] = TEGRA186_RESET_I2C1,
    [RESET_I2C5] = TEGRA186_RESET_I2C5,
    [RESET_SPI1] = TEGRA186_RESET_SPI1,
    [RESET_ISP] = TEGRA186_RESET_ISP,
    [RESET_VI] = TEGRA186_RESET_VI,
    [RESET_TSCTNVI] = TEGRA186_RESET_TSCTNVI,
    [RESET_SDMMC1] = TEGRA186_RESET_SDMMC1,
    [RESET_SDMMC2] = TEGRA186_RESET_SDMMC2,
    [RESET_SDMMC3] = TEGRA186_RESET_SDMMC3,
    [RESET_SDMMC4] = TEGRA186_RESET_SDMMC4,
    [RESET_UARTA] = TEGRA186_RESET_UARTA,
    [RESET_UARTB] = TEGRA186_RESET_UARTB,
    [RESET_HOST1X] = TEGRA186_RESET_HOST1X,
    [RESET_EXTPERIPH4] = TEGRA186_RESET_EXTPERIPH4,
    [RESET_SPI4] = TEGRA186_RESET_SPI4,
    [RESET_I2C3] = TEGRA186_RESET_I2C3,
    [RESET_UARTD] = TEGRA186_RESET_UARTD,
    [RESET_CSITE] = TEGRA186_RESET_CSITE,
    [RESET_DTV] = TEGRA186_RESET_DTV,
    [RESET_TSEC] = TEGRA186_RESET_TSEC,
    [RESET_I2C4] = TEGRA186_RESET_I2C4,
    [RESET_HDA2CODEC_2X] = TEGRA186_RESET_HDA2CODEC_2X,
    [RESET_EXTPERIPH1] = TEGRA186_RESET_EXTPERIPH1,
    [RESET_EXTPERIPH2] = TEGRA186_RESET_EXTPERIPH2,
    [RESET_EXTPERIPH3] = TEGRA186_RESET_EXTPERIPH3,
    [RESET_SOR0] = TEGRA186_RESET_SOR0,
    [RESET_SOR1] = TEGRA186_RESET_SOR1,
    [RESET_DPAUX] = TEGRA186_RESET_DPAUX,
    [RESET_CEC] = TEGRA186_RESET_CEC,
    [RESET_HDA] = TEGRA186_RESET_HDA,
    [RESET_APE] = TEGRA186_RESET_APE,
    [RESET_DSIPADCTL] = TEGRA186_RESET_DSIPADCTL,
    [RESET_DSI] = TEGRA186_RESET_DSI,
    [RESET_MIPI_CAL] = TEGRA186_RESET_MIPI_CAL,
    [RESET_DVFS] = TEGRA186_RESET_DVFS,
    [RESET_AUD_MCLK] = TEGRA186_RESET_AUD_MCLK,
    [RESET_I2C6] = TEGRA186_RESET_I2C6,
    [RESET_NVDEC] = TEGRA186_RESET_NVDEC,
    [RESET_NVJPG] = TEGRA186_RESET_NVJPG,
    [RESET_NVENC] = TEGRA186_RESET_NVENC,
    [RESET_QSPI] = TEGRA186_RESET_QSPI,
    [RESET_VI_I2C] = TEGRA186_RESET_VI_I2C,
    [RESET_TSECB] = TEGRA186_RESET_TSECB,
    [RESET_GPIO_CTL0] = TEGRA186_RESET_GPIO_CTL0,
    [RESET_GPIO_CTL1] = TEGRA186_RESET_GPIO_CTL1,
    [RESET_GPIO_CTL2] = TEGRA186_RESET_GPIO_CTL2,
    [RESET_GPIO_CTL3] = TEGRA186_RESET_GPIO_CTL3,
    [RESET_GPIO_CTL4] = TEGRA186_RESET_GPIO_CTL4,
    [RESET_TACH] = TEGRA186_RESET_TACH,
    [RESET_I2C7] = TEGRA186_RESET_I2C7,
    [RESET_I2C9] = TEGRA186_RESET_I2C9,
    [RESET_I2C12] = TEGRA186_RESET_I2C12,
    [RESET_I2C13] = TEGRA186_RESET_I2C13,
    [RESET_I2C14] = TEGRA186_RESET_I2C14,
    [RESET_PWM1] = TEGRA186_RESET_PWM1,
    [RESET_PWM2] = TEGRA186_RESET_PWM2,
    [RESET_PWM3] = TEGRA186_RESET_PWM3,
    [RESET_PWM5] = TEGRA186_RESET_PWM5,
    [RESET_PWM6] = TEGRA186_RESET_PWM6,
    [RESET_PWM7] = TEGRA186_RESET_PWM7,
    [RESET_PWM8] = TEGRA186_RESET_PWM8,
    [RESET_UARTE] = TEGRA186_RESET_UARTE,
    [RESET_UARTF] = TEGRA186_RESET_UARTF,
    [RESET_BPMP_PM] = TEGRA186_RESET_BPMP_PM,
    [RESET_BPMP_CVC] = TEGRA186_RESET_BPMP_CVC,
    [RESET_BPMP_DMA] = TEGRA186_RESET_BPMP_DMA,
    [RESET_BPMP_HSP] = TEGRA186_RESET_BPMP_HSP,
    [RESET_TSCTNBPMP] = TEGRA186_RESET_TSCTNBPMP,
    [RESET_BPMP_TKE] = TEGRA186_RESET_BPMP_TKE,
    [RESET_BPMP_GTE] = TEGRA186_RESET_BPMP_GTE,
    [RESET_BPMP_APB] = TEGRA186_RESET_BPMP_APB,
    [RESET_SOC_THERM] = TEGRA186_RESET_SOC_THERM,
    [RESET_AON_ACTMON] = TEGRA186_RESET_AON_ACTMON,
    [RESET_AOPM] = TEGRA186_RESET_AOPM,
    [RESET_AOVC] = TEGRA186_RESET_AOVC,
    [RESET_AON_DMA] = TEGRA186_RESET_AON_DMA,
    [RESET_AON_GPIO] = TEGRA186_RESET_AON_GPIO,
    [RESET_AON_HSP] = TEGRA186_RESET_AON_HSP,
    [RESET_CAN1] = TEGRA186_RESET_CAN1,
    [RESET_AON_APB] = TEGRA186_RESET_AON_APB,
    [RESET_UARTG] = TEGRA186_RESET_UARTG,
    [RESET_I2C2] = TEGRA186_RESET_I2C2,
    [RESET_I2C8] = TEGRA186_RESET_I2C8,
    [RESET_I2C10] = TEGRA186_RESET_I2C10,
    [RESET_SPI2] = TEGRA186_RESET_SPI2,
    [RESET_DMIC5] = TEGRA186_RESET_DMIC5,
    [RESET_PWM4] = TEGRA186_RESET_PWM4,
    [RESET_TSCTNAON] = TEGRA186_RESET_TSCTNAON,
    [RESET_AON_TKE] = TEGRA186_RESET_AON_TKE,
    [RESET_AON_GTE] = TEGRA186_RESET_AON_GTE,
    [RESET_SCE_ACTMON] = TEGRA186_RESET_SCE_ACTMON,
    [RESET_SCE_PM] = TEGRA186_RESET_SCE_PM,
    [RESET_SCE_DMA] = TEGRA186_RESET_SCE_DMA,
    [RESET_SCE_HSP] = TEGRA186_RESET_SCE_HSP,
    [RESET_TSCTNSCE] = TEGRA186_RESET_TSCTNSCE,
    [RESET_SCE_TKE] = TEGRA186_RESET_SCE_TKE,
    [RESET_SCE_GTE] = TEGRA186_RESET_SCE_GTE,
    [RESET_SCE_CFG] = TEGRA186_RESET_SCE_CFG,
    [RESET_SCE_APB] = TEGRA186_RESET_SCE_APB,
    [RESET_DSIC] = TEGRA186_RESET_DSIC,
    [RESET_DSID] = TEGRA186_RESET_DSID,
    [RESET_GPIO_CTL5] = TEGRA186_RESET_GPIO_CTL5,
    [RESET_PEX_USB_UPHY_L5] = TEGRA186_RESET_PEX_USB_UPHY_L5,
    [RESET_PEX_USB_UPHY_L4] = TEGRA186_RESET_PEX_USB_UPHY_L4,
    [RESET_PEX_USB_UPHY_L3] = TEGRA186_RESET_PEX_USB_UPHY_L3,
    [RESET_PEX_USB_UPHY_L2] = TEGRA186_RESET_PEX_USB_UPHY_L2,
    [RESET_PEX_USB_UPHY_L1] = TEGRA186_RESET_PEX_USB_UPHY_L1,
    [RESET_PEX_USB_UPHY_L0] = TEGRA186_RESET_PEX_USB_UPHY_L0,
    [RESET_PEX_USB_UPHY_PLL1] = TEGRA186_RESET_PEX_USB_UPHY_PLL1,
    [RESET_PEX_USB_UPHY_PLL0] = TEGRA186_RESET_PEX_USB_UPHY_PLL0,
    [RESET_PEX_USB_UPHY] = TEGRA186_RESET_PEX_USB_UPHY,
    [RESET_PCIEXCLK] = TEGRA186_RESET_PCIEXCLK,
    [RESET_AFI] = TEGRA186_RESET_AFI,
    [RESET_PCIE] = TEGRA186_RESET_PCIE,
    [RESET_ADSPNEON] = TEGRA186_RESET_ADSPNEON,
    [RESET_ADSPSCU] = TEGRA186_RESET_ADSPSCU,
    [RESET_ADSPWDT] = TEGRA186_RESET_ADSPWDT,
    [RESET_ADSPDBG] = TEGRA186_RESET_ADSPDBG,
    [RESET_ADSPPERIPH] = TEGRA186_RESET_ADSPPERIPH,
    [RESET_ADSPINTF] = TEGRA186_RESET_ADSPINTF,
    [RESET_ADSP] = TEGRA186_RESET_ADSP,
    [RESET_EMC_MEM] = TEGRA186_RESET_EMC_MEM,
    [RESET_EMC_EMC] = TEGRA186_RESET_EMC_EMC,
    [RESET_SATACOLD] = TEGRA186_RESET_SATACOLD,
    [RESET_SATA] = TEGRA186_RESET_SATA,
    [RESET_XUSB_SS] = TEGRA186_RESET_XUSB_SS,
    [RESET_XUSB_PADCTL] = TEGRA186_RESET_XUSB_PADCTL,
    [RESET_XUSB_DEV] = TEGRA186_RESET_XUSB_DEV,
    [RESET_XUSB_HOST] = TEGRA186_RESET_XUSB_HOST,
    [RESET_DSIB] = TEGRA186_RESET_DSIB,
    [RESET_MPHY_IOBIST] = TEGRA186_RESET_MPHY_IOBIST,
    [RESET_MPHY_CLK_CTL] = TEGRA186_RESET_MPHY_CLK_CTL,
    [RESET_MPHY_L1_RX] = TEGRA186_RESET_MPHY_L1_RX,
    [RESET_MPHY_L1_TX] = TEGRA186_RESET_MPHY_L1_TX,
    [RESET_MPHY_L0_RX] = TEGRA186_RESET_MPHY_L0_RX,
    [RESET_MPHY_L0_TX] = TEGRA186_RESET_MPHY_L0_TX,
    [RESET_JTAG2AXI] = TEGRA186_RESET_JTAG2AXI,
    [RESET_GPCDMA] = TEGRA186_RESET_GPCDMA,
    [RESET_NVDISPLAY0_MISC] = TEGRA186_RESET_NVDISPLAY0_MISC,
    [RESET_NVDISPLAY0_WGRP5] = TEGRA186_RESET_NVDISPLAY0_WGRP5,
    [RESET_NVDISPLAY0_WGRP4] = TEGRA186_RESET_NVDISPLAY0_WGRP4,
    [RESET_NVDISPLAY0_WGRP3] = TEGRA186_RESET_NVDISPLAY0_WGRP3,
    [RESET_NVDISPLAY0_WGRP2] = TEGRA186_RESET_NVDISPLAY0_WGRP2,
    [RESET_NVDISPLAY0_WGRP1] = TEGRA186_RESET_NVDISPLAY0_WGRP1,
    [RESET_NVDISPLAY0_WGRP0] = TEGRA186_RESET_NVDISPLAY0_WGRP0,
    [RESET_NVDISPLAY0_HEAD2] = TEGRA186_RESET_NVDISPLAY0_HEAD2,
    [RESET_NVDISPLAY0_HEAD1] = TEGRA186_RESET_NVDISPLAY0_HEAD1,
    [RESET_NVDISPLAY0_HEAD0] = TEGRA186_RESET_NVDISPLAY0_HEAD0,
    [RESET_EMCSB_MEM] = TEGRA186_RESET_EMCSB_MEM,
    [RESET_EMCSB_EMC] = TEGRA186_RESET_EMCSB_EMC,
    [RESET_UFSHC_AXI_M] = TEGRA186_RESET_UFSHC_AXI_M,
    [RESET_UFSHC_LP] = TEGRA186_RESET_UFSHC_LP,
    [RESET_BPMP_NIC] = TEGRA186_RESET_BPMP_NIC,
    [RESET_BPMP_NSYSPORESET] = TEGRA186_RESET_BPMP_NSYSPORESET,
    [RESET_BPMP_NRESET] = TEGRA186_RESET_BPMP_NRESET,
    [RESET_BPMP_DBGRESETN] = TEGRA186_RESET_BPMP_DBGRESETN,
    [RESET_BPMP_PRESETDBGN] = TEGRA186_RESET_BPMP_PRESETDBGN,
    [RESET_BPMP_PM_ACTMON] = TEGRA186_RESET_BPMP_PM_ACTMON,
    [RESET_ACTMON] = TEGRA186_RESET_ACTMON,
    [RESET_AON_NIC] = TEGRA186_RESET_AON_NIC,
    [RESET_AON_NSYSPORESET] = TEGRA186_RESET_AON_NSYSPORESET,
    [RESET_AON_NRESET] = TEGRA186_RESET_AON_NRESET,
    [RESET_AON_DBGRESETN] = TEGRA186_RESET_AON_DBGRESETN,
    [RESET_AON_PRESETDBGN] = TEGRA186_RESET_AON_PRESETDBGN,
    [RESET_SCE_NIC] = TEGRA186_RESET_SCE_NIC,
    [RESET_SCE_NSYSPORESET] = TEGRA186_RESET_SCE_NSYSPORESET,
    [RESET_SCE_NRESET] = TEGRA186_RESET_SCE_NRESET,
    [RESET_SCE_DBGRESETN] = TEGRA186_RESET_SCE_DBGRESETN,
    [RESET_SCE_PRESETDBGN] = TEGRA186_RESET_SCE_PRESETDBGN,
    [RESET_EQOS] = TEGRA186_RESET_EQOS,
};


// typedef struct tx2_reset {
//     ps_io_ops_t *io_ops;
//     struct tx2_bpmp *bpmp;
// } tx2_reset_t;

static inline bool check_valid_reset(reset_id_t id)
{
    return (RESET_TOP_GTE <= id && id < NRESETS);
}

static int tx2_reset_common(void *data, reset_id_t id, bool assert)
{
    if (!check_valid_reset(id)) {
        print("Invalid reset ID");
        return -EINVAL;
    }

    tx2_reset_t *reset = data;
    uint32_t bpmp_reset_id = mrq_reset_id_map[id];

    /* Setup a message and make a call to BPMP */
    struct mrq_reset_request req = { .reset_id = bpmp_reset_id };
    req.cmd = (assert) ? CMD_RESET_ASSERT : CMD_RESET_DEASSERT;

    int bytes_recvd = tx2_bpmp_call(reset->bpmp, MRQ_RESET, &req, sizeof(req), NULL, 0);
    if (bytes_recvd < 0) {
        return -EIO;
    }

    return 0;
}

static int tx2_reset_assert(void *data, reset_id_t id)
{
    return tx2_reset_common(data, id, true);
}

static int tx2_reset_deassert(void *data, reset_id_t id)
{
    return tx2_reset_common(data, id, false);
}

// static int interface_search_handler(void *handler_data, void *interface_instance, char **properties)
// {
//     /* Select the first one that is registered */
//     tx2_reset_t *reset = handler_data;
//     reset->bpmp = (struct tx2_bpmp *) interface_instance;
//     return PS_INTERFACE_FOUND_MATCH;
// }

int reset_sys_init( void *dependencies, reset_sys_t *reset_sys)
{
    if (!reset_sys) {
        print("null reset_sys argument");
    }

    int error = 0;
    tx2_reset_t *reset = reset_sys->data;

    if (dependencies) {
        reset->bpmp = (struct tx2_bpmp *) dependencies;
    } else {
        // no longer handling this case
        goto fail;
        // /* See if there's a registered interface for the BPMP, if not, then we
        //  * initialise one ourselves. */
        // error = ps_interface_find(&io_ops->interface_registration_ops, TX2_BPMP_INTERFACE,
        //                           interface_search_handler, reset);
        // if (error) {
        //     error = ps_calloc(&io_ops->malloc_ops, 1, sizeof(struct tx2_bpmp), (void **) &reset->bpmp);
        //     if (error) {
        //         print("Failed to allocate memory for the BPMP structure to be initialised");
        //         goto fail;
        //     }

        //     error = tx2_bpmp_init(io_ops, reset->bpmp);
        //     if (error) {
        //         print("Failed to initialise the BPMP");
        //         goto fail;
        //     }
        // }
    }

    reset_sys->reset_assert = &tx2_reset_assert;
    reset_sys->reset_deassert = &tx2_reset_deassert;

    return 0;

fail:

    if (reset_sys->data) {
        print("|reset_sys_init| Failed\n");
        // if (reset->bpmp) {
        //     ZF_LOGF_IF(ps_free(&io_ops->malloc_ops, sizeof(struct tx2_bpmp), (void *) reset->bpmp),
        //                "Failed to free the BPMP structure after a failed reset subsystem initialisation");
        // }
        // ZF_LOGF_IF(ps_free(&io_ops->malloc_ops, sizeof(tx2_reset_t), (void *) reset_sys->data),
        //            "Failed to free the reset private data after a failed reset subsystem initialisation");
    }

    return error;
}
