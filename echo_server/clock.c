#include <stdint.h>
#include <errno.h>
#include <string.h>

#include "bpmp.h"
#include "clock_bindings.h"

#include "clock.h"

uint32_t mrq_clk_id_map[] = {
    [CLK_PLLC_OUT_ISP] = TEGRA186_CLK_PLLC_OUT_ISP,
    [CLK_PLLC_OUT_VE] = TEGRA186_CLK_PLLC_OUT_VE,
    [CLK_PLLC_OUT_AON] = TEGRA186_CLK_PLLC_OUT_AON,
    [CLK_SOR_SAFE] = TEGRA186_CLK_SOR_SAFE,
    [CLK_I2S2] = TEGRA186_CLK_I2S2,
    [CLK_I2S3] = TEGRA186_CLK_I2S3,
    [CLK_SPDIF_DOUBLER] = TEGRA186_CLK_SPDIF_DOUBLER,
    [CLK_SPI3] = TEGRA186_CLK_SPI3,
    [CLK_I2C1] = TEGRA186_CLK_I2C1,
    [CLK_I2C5] = TEGRA186_CLK_I2C5,
    [CLK_SPI1] = TEGRA186_CLK_SPI1,
    [CLK_VI] = TEGRA186_CLK_VI,
    [CLK_SDMMC1] = TEGRA186_CLK_SDMMC1,
    [CLK_SDMMC2] = TEGRA186_CLK_SDMMC2,
    [CLK_SDMMC4] = TEGRA186_CLK_SDMMC4,
    [CLK_UARTA] = TEGRA186_CLK_UARTA,
    [CLK_UARTB] = TEGRA186_CLK_UARTB,
    [CLK_HOST1X] = TEGRA186_CLK_HOST1X,
    [CLK_EMC] = TEGRA186_CLK_EMC,
    [CLK_EXTPERIPH4] = TEGRA186_CLK_EXTPERIPH4,
    [CLK_SPI4] = TEGRA186_CLK_SPI4,
    [CLK_I2C3] = TEGRA186_CLK_I2C3,
    [CLK_SDMMC3] = TEGRA186_CLK_SDMMC3,
    [CLK_UARTD] = TEGRA186_CLK_UARTD,
    [CLK_I2S1] = TEGRA186_CLK_I2S1,
    [CLK_TSEC] = TEGRA186_CLK_TSEC,
    [CLK_I2S4] = TEGRA186_CLK_I2S4,
    [CLK_I2S5] = TEGRA186_CLK_I2S5,
    [CLK_I2C4] = TEGRA186_CLK_I2C4,
    [CLK_AHUB] = TEGRA186_CLK_AHUB,
    [CLK_HDA2CODEC_2X] = TEGRA186_CLK_HDA2CODEC_2X,
    [CLK_EXTPERIPH1] = TEGRA186_CLK_EXTPERIPH1,
    [CLK_EXTPERIPH2] = TEGRA186_CLK_EXTPERIPH2,
    [CLK_EXTPERIPH3] = TEGRA186_CLK_EXTPERIPH3,
    [CLK_I2C_SLOW] = TEGRA186_CLK_I2C_SLOW,
    [CLK_SOR1] = TEGRA186_CLK_SOR1,
    [CLK_SOR0] = TEGRA186_CLK_SOR0,
    [CLK_SATA] = TEGRA186_CLK_SATA,
    [CLK_HDA] = TEGRA186_CLK_HDA,
    [CLK_PLLREFE_OUT] = TEGRA186_CLK_PLLREFE_OUT,
    [CLK_PLLC4_OUT] = TEGRA186_CLK_PLLC4_OUT,
    [CLK_XUSB] = TEGRA186_CLK_XUSB,
    [CLK_XUSB_DEV] = TEGRA186_CLK_XUSB_DEV,
    [CLK_XUSB_HOST] = TEGRA186_CLK_XUSB_HOST,
    [CLK_DSIA_LP] = TEGRA186_CLK_DSIA_LP,
    [CLK_DSIB_LP] = TEGRA186_CLK_DSIB_LP,
    [CLK_DMIC1] = TEGRA186_CLK_DMIC1,
    [CLK_DMIC2] = TEGRA186_CLK_DMIC2,
    [CLK_AUD_MCLK] = TEGRA186_CLK_AUD_MCLK,
    [CLK_I2C6] = TEGRA186_CLK_I2C6,
    [CLK_UART_FST_MIPI_CAL] = TEGRA186_CLK_UART_FST_MIPI_CAL,
    [CLK_VIC] = TEGRA186_CLK_VIC,
    [CLK_SDMMC_LEGACY_TM] = TEGRA186_CLK_SDMMC_LEGACY_TM,
    [CLK_NVDEC] = TEGRA186_CLK_NVDEC,
    [CLK_NVJPG] = TEGRA186_CLK_NVJPG,
    [CLK_NVENC] = TEGRA186_CLK_NVENC,
    [CLK_QSPI] = TEGRA186_CLK_QSPI,
    [CLK_VI_I2C] = TEGRA186_CLK_VI_I2C,
    [CLK_MAUD] = TEGRA186_CLK_MAUD,
    [CLK_TSECB] = TEGRA186_CLK_TSECB,
    [CLK_MPHY_L0_RX_SYMB] = TEGRA186_CLK_MPHY_L0_RX_SYMB,
    [CLK_MPHY_L0_TX_SYMB] = TEGRA186_CLK_MPHY_L0_TX_SYMB,
    [CLK_MPHY_IOBIST] = TEGRA186_CLK_MPHY_IOBIST,
    [CLK_MPHY_TX_1MHZ_REF] = TEGRA186_CLK_MPHY_TX_1MHZ_REF,
    [CLK_MPHY_CORE_PLL_FIXED] = TEGRA186_CLK_MPHY_CORE_PLL_FIXED,
    [CLK_AXI_CBB] = TEGRA186_CLK_AXI_CBB,
    [CLK_DMIC3] = TEGRA186_CLK_DMIC3,
    [CLK_DMIC4] = TEGRA186_CLK_DMIC4,
    [CLK_DSPK1] = TEGRA186_CLK_DSPK1,
    [CLK_DSPK2] = TEGRA186_CLK_DSPK2,
    [CLK_I2S6] = TEGRA186_CLK_I2S6,
    [CLK_NVDISPLAY_P0] = TEGRA186_CLK_NVDISPLAY_P0,
    [CLK_NVDISPLAY_DISP] = TEGRA186_CLK_NVDISPLAY_DISP,
    [CLK_NVDISPLAYHUB] = TEGRA186_CLK_NVDISPLAYHUB,
    [CLK_NVDISPLAY_P1] = TEGRA186_CLK_NVDISPLAY_P1,
    [CLK_NVDISPLAY_P2] = TEGRA186_CLK_NVDISPLAY_P2,
    [CLK_TACH] = TEGRA186_CLK_TACH,
    [CLK_UFSHC] = TEGRA186_CLK_UFSHC,
    [CLK_UFSDEV_REF] = TEGRA186_CLK_UFSDEV_REF,
    [CLK_NVCSI] = TEGRA186_CLK_NVCSI,
    [CLK_NVCSILP] = TEGRA186_CLK_NVCSILP,
    [CLK_I2C7] = TEGRA186_CLK_I2C7,
    [CLK_I2C9] = TEGRA186_CLK_I2C9,
    [CLK_I2C12] = TEGRA186_CLK_I2C12,
    [CLK_I2C13] = TEGRA186_CLK_I2C13,
    [CLK_I2C14] = TEGRA186_CLK_I2C14,
    [CLK_PWM1] = TEGRA186_CLK_PWM1,
    [CLK_PWM2] = TEGRA186_CLK_PWM2,
    [CLK_PWM3] = TEGRA186_CLK_PWM3,
    [CLK_PWM5] = TEGRA186_CLK_PWM5,
    [CLK_PWM6] = TEGRA186_CLK_PWM6,
    [CLK_PWM7] = TEGRA186_CLK_PWM7,
    [CLK_PWM8] = TEGRA186_CLK_PWM8,
    [CLK_UARTE] = TEGRA186_CLK_UARTE,
    [CLK_UARTF] = TEGRA186_CLK_UARTF,
    [CLK_DBGAPB] = TEGRA186_CLK_DBGAPB,
    [CLK_BPMP_CPU_NIC] = TEGRA186_CLK_BPMP_CPU_NIC,
    [CLK_BPMP_APB] = TEGRA186_CLK_BPMP_APB,
    [CLK_ACTMON] = TEGRA186_CLK_ACTMON,
    [CLK_AON_CPU_NIC] = TEGRA186_CLK_AON_CPU_NIC,
    [CLK_CAN1] = TEGRA186_CLK_CAN1,
    [CLK_CAN1_HOST] = TEGRA186_CLK_CAN1_HOST,
    [CLK_CAN2] = TEGRA186_CLK_CAN2,
    [CLK_CAN2_HOST] = TEGRA186_CLK_CAN2_HOST,
    [CLK_AON_APB] = TEGRA186_CLK_AON_APB,
    [CLK_UARTC] = TEGRA186_CLK_UARTC,
    [CLK_UARTG] = TEGRA186_CLK_UARTG,
    [CLK_AON_UART_FST_MIPI_CAL] = TEGRA186_CLK_AON_UART_FST_MIPI_CAL,
    [CLK_I2C2] = TEGRA186_CLK_I2C2,
    [CLK_I2C8] = TEGRA186_CLK_I2C8,
    [CLK_I2C10] = TEGRA186_CLK_I2C10,
    [CLK_AON_I2C_SLOW] = TEGRA186_CLK_AON_I2C_SLOW,
    [CLK_SPI2] = TEGRA186_CLK_SPI2,
    [CLK_DMIC5] = TEGRA186_CLK_DMIC5,
    [CLK_AON_TOUCH] = TEGRA186_CLK_AON_TOUCH,
    [CLK_PWM4] = TEGRA186_CLK_PWM4,
    [CLK_TSC] = TEGRA186_CLK_TSC,
    [CLK_MSS_ENCRYPT] = TEGRA186_CLK_MSS_ENCRYPT,
    [CLK_SCE_CPU_NIC] = TEGRA186_CLK_SCE_CPU_NIC,
    [CLK_SCE_APB] = TEGRA186_CLK_SCE_APB,
    [CLK_DSIC] = TEGRA186_CLK_DSIC,
    [CLK_DSIC_LP] = TEGRA186_CLK_DSIC_LP,
    [CLK_DSID] = TEGRA186_CLK_DSID,
    [CLK_DSID_LP] = TEGRA186_CLK_DSID_LP,
    [CLK_PEX_SATA_USB_RX_BYP] = TEGRA186_CLK_PEX_SATA_USB_RX_BYP,
    [CLK_SPDIF_OUT] = TEGRA186_CLK_SPDIF_OUT,
    [CLK_EQOS_PTP_REF] = TEGRA186_CLK_EQOS_PTP_REF,
    [CLK_EQOS_TX] = TEGRA186_CLK_EQOS_TX,
    [CLK_USB2_HSIC_TRK] = TEGRA186_CLK_USB2_HSIC_TRK,
    [CLK_XUSB_CORE_SS] = TEGRA186_CLK_XUSB_CORE_SS,
    [CLK_XUSB_CORE_DEV] = TEGRA186_CLK_XUSB_CORE_DEV,
    [CLK_XUSB_FALCON] = TEGRA186_CLK_XUSB_FALCON,
    [CLK_XUSB_FS] = TEGRA186_CLK_XUSB_FS,
    [CLK_PLL_A_OUT0] = TEGRA186_CLK_PLL_A_OUT0,
    [CLK_SYNC_I2S1] = TEGRA186_CLK_SYNC_I2S1,
    [CLK_SYNC_I2S2] = TEGRA186_CLK_SYNC_I2S2,
    [CLK_SYNC_I2S3] = TEGRA186_CLK_SYNC_I2S3,
    [CLK_SYNC_I2S4] = TEGRA186_CLK_SYNC_I2S4,
    [CLK_SYNC_I2S5] = TEGRA186_CLK_SYNC_I2S5,
    [CLK_SYNC_I2S6] = TEGRA186_CLK_SYNC_I2S6,
    [CLK_SYNC_DSPK1] = TEGRA186_CLK_SYNC_DSPK1,
    [CLK_SYNC_DSPK2] = TEGRA186_CLK_SYNC_DSPK2,
    [CLK_SYNC_DMIC1] = TEGRA186_CLK_SYNC_DMIC1,
    [CLK_SYNC_DMIC2] = TEGRA186_CLK_SYNC_DMIC2,
    [CLK_SYNC_DMIC3] = TEGRA186_CLK_SYNC_DMIC3,
    [CLK_SYNC_DMIC4] = TEGRA186_CLK_SYNC_DMIC4,
    [CLK_SYNC_SPDIF] = TEGRA186_CLK_SYNC_SPDIF,
    [CLK_PLLREFE_OUT_GATED] = TEGRA186_CLK_PLLREFE_OUT_GATED,
    [CLK_PLLREFE_OUT1] = TEGRA186_CLK_PLLREFE_OUT1,
    [CLK_PLLD_OUT1] = TEGRA186_CLK_PLLD_OUT1,
    [CLK_PLLP_OUT0] = TEGRA186_CLK_PLLP_OUT0,
    [CLK_PLLP_OUT5] = TEGRA186_CLK_PLLP_OUT5,
    [CLK_PLLA] = TEGRA186_CLK_PLLA,
    [CLK_ACLK] = TEGRA186_CLK_ACLK,
    [CLK_PLL_U_48M] = TEGRA186_CLK_PLL_U_48M,
    [CLK_PLL_U_480M] = TEGRA186_CLK_PLL_U_480M,
    [CLK_PLLC4_OUT0] = TEGRA186_CLK_PLLC4_OUT0,
    [CLK_PLLC4_OUT1] = TEGRA186_CLK_PLLC4_OUT1,
    [CLK_PLLC4_OUT2] = TEGRA186_CLK_PLLC4_OUT2,
    [CLK_PLLC4_OUT_MUX] = TEGRA186_CLK_PLLC4_OUT_MUX,
    [CLK_DFLLDISP_DIV] = TEGRA186_CLK_DFLLDISP_DIV,
    [CLK_PLLDISPHUB_DIV] = TEGRA186_CLK_PLLDISPHUB_DIV,
    [CLK_PLLP_DIV8] = TEGRA186_CLK_PLLP_DIV8,
    [CLK_BPMP_NIC] = TEGRA186_CLK_BPMP_NIC,
    [CLK_PLL_A_OUT1] = TEGRA186_CLK_PLL_A_OUT1,
    [CLK_GPC2CLK] = TEGRA186_CLK_GPC2CLK,
    [CLK_PLLE_PWRSEQ] = TEGRA186_CLK_PLLE_PWRSEQ,
    [CLK_PLLREFE_REF] = TEGRA186_CLK_PLLREFE_REF,
    [CLK_SOR0_OUT] = TEGRA186_CLK_SOR0_OUT,
    [CLK_SOR1_OUT] = TEGRA186_CLK_SOR1_OUT,
    [CLK_PLLREFE_OUT1_DIV5] = TEGRA186_CLK_PLLREFE_OUT1_DIV5,
    [CLK_UTMIP_PLL_PWRSEQ] = TEGRA186_CLK_UTMIP_PLL_PWRSEQ,
    [CLK_PEX_USB_PAD0_MGMT] = TEGRA186_CLK_PEX_USB_PAD0_MGMT,
    [CLK_PEX_USB_PAD1_MGMT] = TEGRA186_CLK_PEX_USB_PAD1_MGMT,
    [CLK_UPHY_PLL0_PWRSEQ] = TEGRA186_CLK_UPHY_PLL0_PWRSEQ,
    [CLK_UPHY_PLL1_PWRSEQ] = TEGRA186_CLK_UPHY_PLL1_PWRSEQ,
    [CLK_PLLREFE_PLLE_PASSTHROUGH] = TEGRA186_CLK_PLLREFE_PLLE_PASSTHROUGH,
    [CLK_PLLREFE_PEX] = TEGRA186_CLK_PLLREFE_PEX,
    [CLK_PLLREFE_IDDQ] = TEGRA186_CLK_PLLREFE_IDDQ,
    [CLK_QSPI_OUT] = TEGRA186_CLK_QSPI_OUT,
    [CLK_GPCCLK] = TEGRA186_CLK_GPCCLK,
    [CLK_AON_NIC] = TEGRA186_CLK_AON_NIC,
    [CLK_SCE_NIC] = TEGRA186_CLK_SCE_NIC,
    [CLK_PLLE] = TEGRA186_CLK_PLLE,
    [CLK_PLLC] = TEGRA186_CLK_PLLC,
    [CLK_PLLP] = TEGRA186_CLK_PLLP,
    [CLK_PLLD] = TEGRA186_CLK_PLLD,
    [CLK_PLLD2] = TEGRA186_CLK_PLLD2,
    [CLK_PLLREFE_VCO] = TEGRA186_CLK_PLLREFE_VCO,
    [CLK_PLLC2] = TEGRA186_CLK_PLLC2,
    [CLK_PLLC3] = TEGRA186_CLK_PLLC3,
    [CLK_PLLDP] = TEGRA186_CLK_PLLDP,
    [CLK_PLLC4_VCO] = TEGRA186_CLK_PLLC4_VCO,
    [CLK_PLLA1] = TEGRA186_CLK_PLLA1,
    [CLK_PLLNVCSI] = TEGRA186_CLK_PLLNVCSI,
    [CLK_PLLDISPHUB] = TEGRA186_CLK_PLLDISPHUB,
    [CLK_PLLD3] = TEGRA186_CLK_PLLD3,
    [CLK_PLLBPMPCAM] = TEGRA186_CLK_PLLBPMPCAM,
    [CLK_PLLAON] = TEGRA186_CLK_PLLAON,
    [CLK_PLLU] = TEGRA186_CLK_PLLU,
    [CLK_PLLC4_VCO_DIV2] = TEGRA186_CLK_PLLC4_VCO_DIV2,
    [CLK_NAFLL_AXI_CBB] = TEGRA186_CLK_NAFLL_AXI_CBB,
    [CLK_NAFLL_BPMP] = TEGRA186_CLK_NAFLL_BPMP,
    [CLK_NAFLL_ISP] = TEGRA186_CLK_NAFLL_ISP,
    [CLK_NAFLL_NVDEC] = TEGRA186_CLK_NAFLL_NVDEC,
    [CLK_NAFLL_NVENC] = TEGRA186_CLK_NAFLL_NVENC,
    [CLK_NAFLL_NVJPG] = TEGRA186_CLK_NAFLL_NVJPG,
    [CLK_NAFLL_SCE] = TEGRA186_CLK_NAFLL_SCE,
    [CLK_NAFLL_SE] = TEGRA186_CLK_NAFLL_SE,
    [CLK_NAFLL_TSEC] = TEGRA186_CLK_NAFLL_TSEC,
    [CLK_NAFLL_TSECB] = TEGRA186_CLK_NAFLL_TSECB,
    [CLK_NAFLL_VI] = TEGRA186_CLK_NAFLL_VI,
    [CLK_NAFLL_VIC] = TEGRA186_CLK_NAFLL_VIC,
    [CLK_NAFLL_DISP] = TEGRA186_CLK_NAFLL_DISP,
    [CLK_NAFLL_GPU] = TEGRA186_CLK_NAFLL_GPU,
    [CLK_NAFLL_MCPU] = TEGRA186_CLK_NAFLL_MCPU,
    [CLK_NAFLL_BCPU] = TEGRA186_CLK_NAFLL_BCPU,
    [CLK_PLL_REF] = TEGRA186_CLK_PLL_REF,
    [CLK_OSC] = TEGRA186_CLK_OSC,
    [CLK_EQOS_RX_INPUT] = TEGRA186_CLK_EQOS_RX_INPUT,
    [CLK_DTV_INPUT] = TEGRA186_CLK_DTV_INPUT,
    [CLK_SOR0_PAD_CLKOUT] = TEGRA186_CLK_SOR0_PAD_CLKOUT,
    [CLK_SOR1_PAD_CLKOUT] = TEGRA186_CLK_SOR1_PAD_CLKOUT,
    [CLK_I2S1_SYNC_INPUT] = TEGRA186_CLK_I2S1_SYNC_INPUT,
    [CLK_I2S2_SYNC_INPUT] = TEGRA186_CLK_I2S2_SYNC_INPUT,
    [CLK_I2S3_SYNC_INPUT] = TEGRA186_CLK_I2S3_SYNC_INPUT,
    [CLK_I2S4_SYNC_INPUT] = TEGRA186_CLK_I2S4_SYNC_INPUT,
    [CLK_I2S5_SYNC_INPUT] = TEGRA186_CLK_I2S5_SYNC_INPUT,
    [CLK_I2S6_SYNC_INPUT] = TEGRA186_CLK_I2S6_SYNC_INPUT,
    [CLK_SPDIFIN_SYNC_INPUT] = TEGRA186_CLK_SPDIFIN_SYNC_INPUT,
};

uint32_t mrq_gate_id_map[] = {
    [CLK_GATE_FUSE] = TEGRA186_CLK_FUSE,
    [CLK_GATE_GPU] = TEGRA186_CLK_GPU,
    [CLK_GATE_PCIE] = TEGRA186_CLK_PCIE,
    [CLK_GATE_AFI] = TEGRA186_CLK_AFI,
    [CLK_GATE_PCIE2_IOBIST] = TEGRA186_CLK_PCIE2_IOBIST,
    [CLK_GATE_PCIERX0] = TEGRA186_CLK_PCIERX0,
    [CLK_GATE_PCIERX1] = TEGRA186_CLK_PCIERX1,
    [CLK_GATE_PCIERX2] = TEGRA186_CLK_PCIERX2,
    [CLK_GATE_PCIERX3] = TEGRA186_CLK_PCIERX3,
    [CLK_GATE_PCIERX4] = TEGRA186_CLK_PCIERX4,
    [CLK_GATE_PLLC_OUT_ISP] = TEGRA186_CLK_PLLC_OUT_ISP,
    [CLK_GATE_PLLC_OUT_VE] = TEGRA186_CLK_PLLC_OUT_VE,
    [CLK_GATE_PLLC_OUT_AON] = TEGRA186_CLK_PLLC_OUT_AON,
    [CLK_GATE_SOR_SAFE] = TEGRA186_CLK_SOR_SAFE,
    [CLK_GATE_I2S2] = TEGRA186_CLK_I2S2,
    [CLK_GATE_I2S3] = TEGRA186_CLK_I2S3,
    [CLK_GATE_SPDIF_IN] = TEGRA186_CLK_SPDIF_IN,
    [CLK_GATE_SPDIF_DOUBLER] = TEGRA186_CLK_SPDIF_DOUBLER,
    [CLK_GATE_SPI3] = TEGRA186_CLK_SPI3,
    [CLK_GATE_I2C1] = TEGRA186_CLK_I2C1,
    [CLK_GATE_I2C5] = TEGRA186_CLK_I2C5,
    [CLK_GATE_SPI1] = TEGRA186_CLK_SPI1,
    [CLK_GATE_ISP] = TEGRA186_CLK_ISP,
    [CLK_GATE_VI] = TEGRA186_CLK_VI,
    [CLK_GATE_SDMMC1] = TEGRA186_CLK_SDMMC1,
    [CLK_GATE_SDMMC2] = TEGRA186_CLK_SDMMC2,
    [CLK_GATE_SDMMC4] = TEGRA186_CLK_SDMMC4,
    [CLK_GATE_UARTA] = TEGRA186_CLK_UARTA,
    [CLK_GATE_UARTB] = TEGRA186_CLK_UARTB,
    [CLK_GATE_HOST1X] = TEGRA186_CLK_HOST1X,
    [CLK_GATE_EMC] = TEGRA186_CLK_EMC,
    [CLK_GATE_EXTPERIPH4] = TEGRA186_CLK_EXTPERIPH4,
    [CLK_GATE_SPI4] = TEGRA186_CLK_SPI4,
    [CLK_GATE_I2C3] = TEGRA186_CLK_I2C3,
    [CLK_GATE_SDMMC3] = TEGRA186_CLK_SDMMC3,
    [CLK_GATE_UARTD] = TEGRA186_CLK_UARTD,
    [CLK_GATE_I2S1] = TEGRA186_CLK_I2S1,
    [CLK_GATE_DTV] = TEGRA186_CLK_DTV,
    [CLK_GATE_TSEC] = TEGRA186_CLK_TSEC,
    [CLK_GATE_DP2] = TEGRA186_CLK_DP2,
    [CLK_GATE_I2S4] = TEGRA186_CLK_I2S4,
    [CLK_GATE_I2S5] = TEGRA186_CLK_I2S5,
    [CLK_GATE_I2C4] = TEGRA186_CLK_I2C4,
    [CLK_GATE_AHUB] = TEGRA186_CLK_AHUB,
    [CLK_GATE_HDA2CODEC_2X] = TEGRA186_CLK_HDA2CODEC_2X,
    [CLK_GATE_EXTPERIPH1] = TEGRA186_CLK_EXTPERIPH1,
    [CLK_GATE_EXTPERIPH2] = TEGRA186_CLK_EXTPERIPH2,
    [CLK_GATE_EXTPERIPH3] = TEGRA186_CLK_EXTPERIPH3,
    [CLK_GATE_I2C_SLOW] = TEGRA186_CLK_I2C_SLOW,
    [CLK_GATE_SOR1] = TEGRA186_CLK_SOR1,
    [CLK_GATE_CEC] = TEGRA186_CLK_CEC,
    [CLK_GATE_DPAUX1] = TEGRA186_CLK_DPAUX1,
    [CLK_GATE_DPAUX] = TEGRA186_CLK_DPAUX,
    [CLK_GATE_SOR0] = TEGRA186_CLK_SOR0,
    [CLK_GATE_HDA2HDMICODEC] = TEGRA186_CLK_HDA2HDMICODEC,
    [CLK_GATE_SATA] = TEGRA186_CLK_SATA,
    [CLK_GATE_SATA_OOB] = TEGRA186_CLK_SATA_OOB,
    [CLK_GATE_SATA_IOBIST] = TEGRA186_CLK_SATA_IOBIST,
    [CLK_GATE_HDA] = TEGRA186_CLK_HDA,
    [CLK_GATE_SE] = TEGRA186_CLK_SE,
    [CLK_GATE_APB2APE] = TEGRA186_CLK_APB2APE,
    [CLK_GATE_APE] = TEGRA186_CLK_APE,
    [CLK_GATE_IQC1] = TEGRA186_CLK_IQC1,
    [CLK_GATE_IQC2] = TEGRA186_CLK_IQC2,
    [CLK_GATE_PLLREFE_OUT] = TEGRA186_CLK_PLLREFE_OUT,
    [CLK_GATE_PLLREFE_PLL_REF] = TEGRA186_CLK_PLLREFE_PLL_REF,
    [CLK_GATE_PLLC4_OUT] = TEGRA186_CLK_PLLC4_OUT,
    [CLK_GATE_XUSB] = TEGRA186_CLK_XUSB,
    [CLK_GATE_XUSB_DEV] = TEGRA186_CLK_XUSB_DEV,
    [CLK_GATE_XUSB_HOST] = TEGRA186_CLK_XUSB_HOST,
    [CLK_GATE_XUSB_SS] = TEGRA186_CLK_XUSB_SS,
    [CLK_GATE_DSI] = TEGRA186_CLK_DSI,
    [CLK_GATE_MIPI_CAL] = TEGRA186_CLK_MIPI_CAL,
    [CLK_GATE_DSIA_LP] = TEGRA186_CLK_DSIA_LP,
    [CLK_GATE_DSIB] = TEGRA186_CLK_DSIB,
    [CLK_GATE_DSIB_LP] = TEGRA186_CLK_DSIB_LP,
    [CLK_GATE_DMIC1] = TEGRA186_CLK_DMIC1,
    [CLK_GATE_DMIC2] = TEGRA186_CLK_DMIC2,
    [CLK_GATE_AUD_MCLK] = TEGRA186_CLK_AUD_MCLK,
    [CLK_GATE_I2C6] = TEGRA186_CLK_I2C6,
    [CLK_GATE_UART_FST_MIPI_CAL] = TEGRA186_CLK_UART_FST_MIPI_CAL,
    [CLK_GATE_VIC] = TEGRA186_CLK_VIC,
    [CLK_GATE_SDMMC_LEGACY_TM] = TEGRA186_CLK_SDMMC_LEGACY_TM,
    [CLK_GATE_NVDEC] = TEGRA186_CLK_NVDEC,
    [CLK_GATE_NVJPG] = TEGRA186_CLK_NVJPG,
    [CLK_GATE_NVENC] = TEGRA186_CLK_NVENC,
    [CLK_GATE_QSPI] = TEGRA186_CLK_QSPI,
    [CLK_GATE_VI_I2C] = TEGRA186_CLK_VI_I2C,
    [CLK_GATE_HSIC_TRK] = TEGRA186_CLK_HSIC_TRK,
    [CLK_GATE_USB2_TRK] = TEGRA186_CLK_USB2_TRK,
    [CLK_GATE_MAUD] = TEGRA186_CLK_MAUD,
    [CLK_GATE_TSECB] = TEGRA186_CLK_TSECB,
    [CLK_GATE_ADSP] = TEGRA186_CLK_ADSP,
    [CLK_GATE_ADSPNEON] = TEGRA186_CLK_ADSPNEON,
    [CLK_GATE_MPHY_L0_RX_SYMB] = TEGRA186_CLK_MPHY_L0_RX_SYMB,
    [CLK_GATE_MPHY_L0_RX_LS_BIT] = TEGRA186_CLK_MPHY_L0_RX_LS_BIT,
    [CLK_GATE_MPHY_L0_TX_SYMB] = TEGRA186_CLK_MPHY_L0_TX_SYMB,
    [CLK_GATE_MPHY_L0_TX_LS_3XBIT] = TEGRA186_CLK_MPHY_L0_TX_LS_3XBIT,
    [CLK_GATE_MPHY_L0_RX_ANA] = TEGRA186_CLK_MPHY_L0_RX_ANA,
    [CLK_GATE_MPHY_L1_RX_ANA] = TEGRA186_CLK_MPHY_L1_RX_ANA,
    [CLK_GATE_MPHY_IOBIST] = TEGRA186_CLK_MPHY_IOBIST,
    [CLK_GATE_MPHY_TX_1MHZ_REF] = TEGRA186_CLK_MPHY_TX_1MHZ_REF,
    [CLK_GATE_MPHY_CORE_PLL_FIXED] = TEGRA186_CLK_MPHY_CORE_PLL_FIXED,
    [CLK_GATE_AXI_CBB] = TEGRA186_CLK_AXI_CBB,
    [CLK_GATE_DMIC3] = TEGRA186_CLK_DMIC3,
    [CLK_GATE_DMIC4] = TEGRA186_CLK_DMIC4,
    [CLK_GATE_DSPK1] = TEGRA186_CLK_DSPK1,
    [CLK_GATE_DSPK2] = TEGRA186_CLK_DSPK2,
    [CLK_GATE_I2S6] = TEGRA186_CLK_I2S6,
    [CLK_GATE_NVDISPLAY_P0] = TEGRA186_CLK_NVDISPLAY_P0,
    [CLK_GATE_NVDISPLAY_DISP] = TEGRA186_CLK_NVDISPLAY_DISP,
    [CLK_GATE_NVDISPLAY_DSC] = TEGRA186_CLK_NVDISPLAY_DSC,
    [CLK_GATE_NVDISPLAYHUB] = TEGRA186_CLK_NVDISPLAYHUB,
    [CLK_GATE_NVDISPLAY_P1] = TEGRA186_CLK_NVDISPLAY_P1,
    [CLK_GATE_NVDISPLAY_P2] = TEGRA186_CLK_NVDISPLAY_P2,
    [CLK_GATE_TACH] = TEGRA186_CLK_TACH,
    [CLK_GATE_EQOS_AXI] = TEGRA186_CLK_EQOS_AXI,
    [CLK_GATE_EQOS_RX] = TEGRA186_CLK_EQOS_RX,
    [CLK_GATE_UFSHC] = TEGRA186_CLK_UFSHC,
    [CLK_GATE_UFSDEV_REF] = TEGRA186_CLK_UFSDEV_REF,
    [CLK_GATE_NVCSI] = TEGRA186_CLK_NVCSI,
    [CLK_GATE_NVCSILP] = TEGRA186_CLK_NVCSILP,
    [CLK_GATE_I2C7] = TEGRA186_CLK_I2C7,
    [CLK_GATE_I2C9] = TEGRA186_CLK_I2C9,
    [CLK_GATE_I2C12] = TEGRA186_CLK_I2C12,
    [CLK_GATE_I2C13] = TEGRA186_CLK_I2C13,
    [CLK_GATE_I2C14] = TEGRA186_CLK_I2C14,
    [CLK_GATE_PWM1] = TEGRA186_CLK_PWM1,
    [CLK_GATE_PWM2] = TEGRA186_CLK_PWM2,
    [CLK_GATE_PWM3] = TEGRA186_CLK_PWM3,
    [CLK_GATE_PWM5] = TEGRA186_CLK_PWM5,
    [CLK_GATE_PWM6] = TEGRA186_CLK_PWM6,
    [CLK_GATE_PWM7] = TEGRA186_CLK_PWM7,
    [CLK_GATE_PWM8] = TEGRA186_CLK_PWM8,
    [CLK_GATE_UARTE] = TEGRA186_CLK_UARTE,
    [CLK_GATE_UARTF] = TEGRA186_CLK_UARTF,
    [CLK_GATE_DBGAPB] = TEGRA186_CLK_DBGAPB,
    [CLK_GATE_BPMP_CPU_NIC] = TEGRA186_CLK_BPMP_CPU_NIC,
    [CLK_GATE_BPMP_APB] = TEGRA186_CLK_BPMP_APB,
    [CLK_GATE_ACTMON] = TEGRA186_CLK_ACTMON,
    [CLK_GATE_AON_CPU_NIC] = TEGRA186_CLK_AON_CPU_NIC,
    [CLK_GATE_CAN1] = TEGRA186_CLK_CAN1,
    [CLK_GATE_CAN1_HOST] = TEGRA186_CLK_CAN1_HOST,
    [CLK_GATE_CAN2] = TEGRA186_CLK_CAN2,
    [CLK_GATE_CAN2_HOST] = TEGRA186_CLK_CAN2_HOST,
    [CLK_GATE_AON_APB] = TEGRA186_CLK_AON_APB,
    [CLK_GATE_UARTC] = TEGRA186_CLK_UARTC,
    [CLK_GATE_UARTG] = TEGRA186_CLK_UARTG,
    [CLK_GATE_AON_UART_FST_MIPI_CAL] = TEGRA186_CLK_AON_UART_FST_MIPI_CAL,
    [CLK_GATE_I2C2] = TEGRA186_CLK_I2C2,
    [CLK_GATE_I2C8] = TEGRA186_CLK_I2C8,
    [CLK_GATE_I2C10] = TEGRA186_CLK_I2C10,
    [CLK_GATE_AON_I2C_SLOW] = TEGRA186_CLK_AON_I2C_SLOW,
    [CLK_GATE_SPI2] = TEGRA186_CLK_SPI2,
    [CLK_GATE_DMIC5] = TEGRA186_CLK_DMIC5,
    [CLK_GATE_AON_TOUCH] = TEGRA186_CLK_AON_TOUCH,
    [CLK_GATE_PWM4] = TEGRA186_CLK_PWM4,
    [CLK_GATE_TSC] = TEGRA186_CLK_TSC,
    [CLK_GATE_MSS_ENCRYPT] = TEGRA186_CLK_MSS_ENCRYPT,
    [CLK_GATE_SCE_CPU_NIC] = TEGRA186_CLK_SCE_CPU_NIC,
    [CLK_GATE_SCE_APB] = TEGRA186_CLK_SCE_APB,
    [CLK_GATE_DSIC] = TEGRA186_CLK_DSIC,
    [CLK_GATE_DSIC_LP] = TEGRA186_CLK_DSIC_LP,
    [CLK_GATE_DSID] = TEGRA186_CLK_DSID,
    [CLK_GATE_DSID_LP] = TEGRA186_CLK_DSID_LP,
    [CLK_GATE_PEX_SATA_USB_RX_BYP] = TEGRA186_CLK_PEX_SATA_USB_RX_BYP,
    [CLK_GATE_SPDIF_OUT] = TEGRA186_CLK_SPDIF_OUT,
    [CLK_GATE_EQOS_PTP_REF] = TEGRA186_CLK_EQOS_PTP_REF,
    [CLK_GATE_EQOS_TX] = TEGRA186_CLK_EQOS_TX,
    [CLK_GATE_USB2_HSIC_TRK] = TEGRA186_CLK_USB2_HSIC_TRK,
    [CLK_GATE_XUSB_CORE_SS] = TEGRA186_CLK_XUSB_CORE_SS,
    [CLK_GATE_XUSB_CORE_DEV] = TEGRA186_CLK_XUSB_CORE_DEV,
    [CLK_GATE_XUSB_FALCON] = TEGRA186_CLK_XUSB_FALCON,
    [CLK_GATE_XUSB_FS] = TEGRA186_CLK_XUSB_FS,
    [CLK_GATE_PLL_A_OUT0] = TEGRA186_CLK_PLL_A_OUT0,
    [CLK_GATE_SYNC_I2S1] = TEGRA186_CLK_SYNC_I2S1,
    [CLK_GATE_SYNC_I2S2] = TEGRA186_CLK_SYNC_I2S2,
    [CLK_GATE_SYNC_I2S3] = TEGRA186_CLK_SYNC_I2S3,
    [CLK_GATE_SYNC_I2S4] = TEGRA186_CLK_SYNC_I2S4,
    [CLK_GATE_SYNC_I2S5] = TEGRA186_CLK_SYNC_I2S5,
    [CLK_GATE_SYNC_I2S6] = TEGRA186_CLK_SYNC_I2S6,
    [CLK_GATE_SYNC_DSPK1] = TEGRA186_CLK_SYNC_DSPK1,
    [CLK_GATE_SYNC_DSPK2] = TEGRA186_CLK_SYNC_DSPK2,
    [CLK_GATE_SYNC_DMIC1] = TEGRA186_CLK_SYNC_DMIC1,
    [CLK_GATE_SYNC_DMIC2] = TEGRA186_CLK_SYNC_DMIC2,
    [CLK_GATE_SYNC_DMIC3] = TEGRA186_CLK_SYNC_DMIC3,
    [CLK_GATE_SYNC_DMIC4] = TEGRA186_CLK_SYNC_DMIC4,
    [CLK_GATE_SYNC_SPDIF] = TEGRA186_CLK_SYNC_SPDIF,
    [CLK_GATE_PLLREFE_OUT_GATED] = TEGRA186_CLK_PLLREFE_OUT_GATED,
    [CLK_GATE_PLLREFE_OUT1] = TEGRA186_CLK_PLLREFE_OUT1,
    [CLK_GATE_PLLD_OUT1] = TEGRA186_CLK_PLLD_OUT1,
    [CLK_GATE_PLLP_OUT0] = TEGRA186_CLK_PLLP_OUT0,
    [CLK_GATE_PLLP_OUT5] = TEGRA186_CLK_PLLP_OUT5,
    [CLK_GATE_PLLA] = TEGRA186_CLK_PLLA,
    [CLK_GATE_ACLK] = TEGRA186_CLK_ACLK,
    [CLK_GATE_PLL_U_48M] = TEGRA186_CLK_PLL_U_48M,
    [CLK_GATE_PLL_U_480M] = TEGRA186_CLK_PLL_U_480M,
    [CLK_GATE_PLLC4_OUT0] = TEGRA186_CLK_PLLC4_OUT0,
    [CLK_GATE_PLLC4_OUT1] = TEGRA186_CLK_PLLC4_OUT1,
    [CLK_GATE_PLLC4_OUT2] = TEGRA186_CLK_PLLC4_OUT2,
    [CLK_GATE_PLLC4_OUT_MUX] = TEGRA186_CLK_PLLC4_OUT_MUX,
    [CLK_GATE_DFLLDISP_DIV] = TEGRA186_CLK_DFLLDISP_DIV,
    [CLK_GATE_PLLDISPHUB_DIV] = TEGRA186_CLK_PLLDISPHUB_DIV,
    [CLK_GATE_PLLP_DIV8] = TEGRA186_CLK_PLLP_DIV8,
    [CLK_GATE_BPMP_NIC] = TEGRA186_CLK_BPMP_NIC,
    [CLK_GATE_PLL_A_OUT1] = TEGRA186_CLK_PLL_A_OUT1,
    [CLK_GATE_GPC2CLK] = TEGRA186_CLK_GPC2CLK,
    [CLK_GATE_KFUSE] = TEGRA186_CLK_KFUSE,
    [CLK_GATE_PLLE_PWRSEQ] = TEGRA186_CLK_PLLE_PWRSEQ,
    [CLK_GATE_PLLREFE_REF] = TEGRA186_CLK_PLLREFE_REF,
    [CLK_GATE_SOR0_OUT] = TEGRA186_CLK_SOR0_OUT,
    [CLK_GATE_SOR1_OUT] = TEGRA186_CLK_SOR1_OUT,
    [CLK_GATE_PLLREFE_OUT1_DIV5] = TEGRA186_CLK_PLLREFE_OUT1_DIV5,
    [CLK_GATE_UTMIP_PLL_PWRSEQ] = TEGRA186_CLK_UTMIP_PLL_PWRSEQ,
    [CLK_GATE_PEX_USB_PAD0_MGMT] = TEGRA186_CLK_PEX_USB_PAD0_MGMT,
    [CLK_GATE_PEX_USB_PAD1_MGMT] = TEGRA186_CLK_PEX_USB_PAD1_MGMT,
    [CLK_GATE_UPHY_PLL0_PWRSEQ] = TEGRA186_CLK_UPHY_PLL0_PWRSEQ,
    [CLK_GATE_UPHY_PLL1_PWRSEQ] = TEGRA186_CLK_UPHY_PLL1_PWRSEQ,
    [CLK_GATE_PLLREFE_PLLE_PASSTHROUGH] = TEGRA186_CLK_PLLREFE_PLLE_PASSTHROUGH,
    [CLK_GATE_PLLREFE_PEX] = TEGRA186_CLK_PLLREFE_PEX,
    [CLK_GATE_PLLREFE_IDDQ] = TEGRA186_CLK_PLLREFE_IDDQ,
    [CLK_GATE_QSPI_OUT] = TEGRA186_CLK_QSPI_OUT,
    [CLK_GATE_GPCCLK] = TEGRA186_CLK_GPCCLK,
    [CLK_GATE_AON_NIC] = TEGRA186_CLK_AON_NIC,
    [CLK_GATE_SCE_NIC] = TEGRA186_CLK_SCE_NIC,
    [CLK_GATE_PLLE] = TEGRA186_CLK_PLLE,
    [CLK_GATE_PLLC] = TEGRA186_CLK_PLLC,
    [CLK_GATE_PLLP] = TEGRA186_CLK_PLLP,
    [CLK_GATE_PLLD] = TEGRA186_CLK_PLLD,
    [CLK_GATE_PLLD2] = TEGRA186_CLK_PLLD2,
    [CLK_GATE_PLLREFE_VCO] = TEGRA186_CLK_PLLREFE_VCO,
    [CLK_GATE_PLLC2] = TEGRA186_CLK_PLLC2,
    [CLK_GATE_PLLC3] = TEGRA186_CLK_PLLC3,
    [CLK_GATE_PLLDP] = TEGRA186_CLK_PLLDP,
    [CLK_GATE_PLLC4_VCO] = TEGRA186_CLK_PLLC4_VCO,
    [CLK_GATE_PLLA1] = TEGRA186_CLK_PLLA1,
    [CLK_GATE_PLLNVCSI] = TEGRA186_CLK_PLLNVCSI,
    [CLK_GATE_PLLDISPHUB] = TEGRA186_CLK_PLLDISPHUB,
    [CLK_GATE_PLLD3] = TEGRA186_CLK_PLLD3,
    [CLK_GATE_PLLBPMPCAM] = TEGRA186_CLK_PLLBPMPCAM,
    [CLK_GATE_PLLAON] = TEGRA186_CLK_PLLAON,
    [CLK_GATE_PLLU] = TEGRA186_CLK_PLLU,
    [CLK_GATE_PLLC4_VCO_DIV2] = TEGRA186_CLK_PLLC4_VCO_DIV2,
    [CLK_GATE_NAFLL_AXI_CBB] = TEGRA186_CLK_NAFLL_AXI_CBB,
    [CLK_GATE_NAFLL_BPMP] = TEGRA186_CLK_NAFLL_BPMP,
    [CLK_GATE_NAFLL_ISP] = TEGRA186_CLK_NAFLL_ISP,
    [CLK_GATE_NAFLL_NVDEC] = TEGRA186_CLK_NAFLL_NVDEC,
    [CLK_GATE_NAFLL_NVENC] = TEGRA186_CLK_NAFLL_NVENC,
    [CLK_GATE_NAFLL_NVJPG] = TEGRA186_CLK_NAFLL_NVJPG,
    [CLK_GATE_NAFLL_SCE] = TEGRA186_CLK_NAFLL_SCE,
    [CLK_GATE_NAFLL_SE] = TEGRA186_CLK_NAFLL_SE,
    [CLK_GATE_NAFLL_TSEC] = TEGRA186_CLK_NAFLL_TSEC,
    [CLK_GATE_NAFLL_TSECB] = TEGRA186_CLK_NAFLL_TSECB,
    [CLK_GATE_NAFLL_VI] = TEGRA186_CLK_NAFLL_VI,
    [CLK_GATE_NAFLL_VIC] = TEGRA186_CLK_NAFLL_VIC,
    [CLK_GATE_NAFLL_DISP] = TEGRA186_CLK_NAFLL_DISP,
    [CLK_GATE_NAFLL_GPU] = TEGRA186_CLK_NAFLL_GPU,
    [CLK_GATE_NAFLL_MCPU] = TEGRA186_CLK_NAFLL_MCPU,
    [CLK_GATE_NAFLL_BCPU] = TEGRA186_CLK_NAFLL_BCPU,
    [CLK_GATE_32K] = TEGRA186_CLK_CLK_32K,
    [CLK_GATE_M] = TEGRA186_CLK_CLK_M,
    [CLK_GATE_PLL_REF] = TEGRA186_CLK_PLL_REF,
    [CLK_GATE_OSC] = TEGRA186_CLK_OSC,
    [CLK_GATE_EQOS_RX_INPUT] = TEGRA186_CLK_EQOS_RX_INPUT,
    [CLK_GATE_DTV_INPUT] = TEGRA186_CLK_DTV_INPUT,
    [CLK_GATE_SOR0_PAD_CLKOUT] = TEGRA186_CLK_SOR0_PAD_CLKOUT,
    [CLK_GATE_SOR1_PAD_CLKOUT] = TEGRA186_CLK_SOR1_PAD_CLKOUT,
    [CLK_GATE_I2S1_SYNC_INPUT] = TEGRA186_CLK_I2S1_SYNC_INPUT,
    [CLK_GATE_I2S2_SYNC_INPUT] = TEGRA186_CLK_I2S2_SYNC_INPUT,
    [CLK_GATE_I2S3_SYNC_INPUT] = TEGRA186_CLK_I2S3_SYNC_INPUT,
    [CLK_GATE_I2S4_SYNC_INPUT] = TEGRA186_CLK_I2S4_SYNC_INPUT,
    [CLK_GATE_I2S5_SYNC_INPUT] = TEGRA186_CLK_I2S5_SYNC_INPUT,
    [CLK_GATE_I2S6_SYNC_INPUT] = TEGRA186_CLK_I2S6_SYNC_INPUT,
    [CLK_GATE_SPDIFIN_SYNC_INPUT] = TEGRA186_CLK_SPDIFIN_SYNC_INPUT,
};

// memory 
uintptr_t car_base;

struct tx2_bpmp bpmp;



static freq_t tx2_car_get_freq(clk_t *clk)
{
    struct mrq_clk_request req = { .cmd_and_id = (CMD_CLK_GET_RATE << 24) | clk->id };
    struct mrq_clk_response res = {0};
    tx2_clk_t *tx2_clk = clk->clk_sys->priv;

    int bytes_recvd = tx2_bpmp_call(tx2_clk->bpmp, MRQ_CLK, &req, sizeof(req), &res, sizeof(&res));
    if (bytes_recvd < 0) {
        return 0;
    }

    return (freq_t) res.clk_get_rate.rate;
}

static freq_t tx2_car_set_freq(clk_t *clk, freq_t hz)
{
    struct mrq_clk_request req = { .cmd_and_id = (CMD_CLK_SET_RATE << 24) | clk->id };
    req.clk_set_rate.rate = hz;
    struct mrq_clk_response res = {0};
    tx2_clk_t *tx2_clk = clk->clk_sys->priv;

    int bytes_recvd = tx2_bpmp_call(tx2_clk->bpmp, MRQ_CLK, &req, sizeof(req), &res, sizeof(&res));
    if (bytes_recvd < 0) {
        return 0;
    }

    clk->req_freq = hz;

    return (freq_t) res.clk_set_rate.rate;
}

/**
 * Initialise and acquire a system clock
 * @param[in] clock_sys  A handle to the clock subsystem
 * @param[in] id         The ID of the clock to acquire
 * @param[in] ret_clk    (since system has no memory allocator, memory is provided statically by parent)
 * @return               On success, a handle to the acquired clock.
 *                       Otherwise, NULL.
 */
clk_t *tx2_car_get_clock(clock_sys_t *clock_sys, enum clk_id id, clk_t *ret_clk)
{
    print("|tx2_car_get_clock| called\n");
    int error;
    if (!check_valid_clk_id(id)) {
        sel4cp_dbg_puts("Invalid clock ID");
        return NULL;
    }

    // clk_t *ret_clk = NULL;
    tx2_clk_t *tx2_clk = clock_sys->priv;
    size_t clk_name_len = 0;
    // int error = ps_calloc(&tx2_clk->io_ops->malloc_ops, 1, sizeof(*ret_clk), (void **) &ret_clk);
    // if (error) {
    //     sel4cp_dbg_puts("Failed to allocate memory for the clock structure");
    //     return NULL;
    // }

    bool clock_initialised = false;

    /* Enable the clock while we're at it, clk_id is also a substitute for clock_gate */
    error = tx2_car_gate_enable(clock_sys, id, CLKGATE_ON);
    if (error) {
        goto fail;
    }

    clock_initialised = true;

    uint32_t bpmp_clk_id = mrq_clk_id_map[id];

    /* Get info about this clock so we can fill it in */
    struct mrq_clk_request req = { .cmd_and_id = (CMD_CLK_GET_ALL_INFO << 24) | bpmp_clk_id };
    struct mrq_clk_response res = {0};
    // char *clock_name = NULL;
    int bytes_recvd = tx2_bpmp_call(tx2_clk->bpmp, MRQ_CLK, &req, sizeof(req), &res, sizeof(res));
    if (bytes_recvd < 0) {
        sel4cp_dbg_puts("Failed to initialise the clock");
        goto fail;
    }
    clk_name_len = strlen((char *) res.clk_get_all_info.name) + 1;
    // error = ps_calloc(&tx2_clk->io_ops->malloc_ops, 1, sizeof(char) * clk_name_len, (void **) &clock_name);
    char clock_name[1000];
    if (error) {
        sel4cp_dbg_puts("Failed to allocate memory for the name of the clock");
        goto fail;
    }
    strncpy(clock_name, (char *) res.clk_get_all_info.name, clk_name_len);

    // since its stack memory, can't save it, just print
    sel4cp_dbg_puts("-->||clock||Clock name is =");
    sel4cp_dbg_puts(clock_name);
    sel4cp_dbg_puts("\n");
    // ret_clk->name = (const char *) clock_name;

    /* There's no need for the init nor the recal functions as we're already
     * doing it now and that the BPMP handles the recalibration for us */
    ret_clk->get_freq = tx2_car_get_freq;
    ret_clk->set_freq = tx2_car_set_freq;

    ret_clk->id = id;
    ret_clk->clk_sys = clock_sys;

    return ret_clk;

fail:
    if (ret_clk) {
        sel4cp_dbg_puts("In fail, probably doesn't need to free stuff\n");
        // if (ret_clk->name) {
        //     ps_free(&tx2_clk->io_ops->malloc_ops, sizeof(char) * clk_name_len, (void *) ret_clk->name);
        // }

        // ps_free(&tx2_clk->io_ops->malloc_ops, sizeof(*ret_clk), (void *) ret_clk);
    }

    if (clock_initialised) {
        tx2_car_gate_enable(clock_sys, id, CLKGATE_OFF);
        sel4cp_dbg_puts("Failed to disable clock following failed clock initialisation operation\n");
        // ZF_LOGF_IF(tx2_car_gate_enable(clock_sys, id, CLKGATE_OFF),
        //            "Failed to disable clock following failed clock initialisation operation");
    }

    return NULL;
};

/**
 * Configure the gating mode of a clock
 * @param[in] clock_sys  A handle to the clock subsystem
 * @param[in] gate       The ID of the gate to control
 * @param[in] mode       The mode at which the clock should be gated
 * @return               0 on success;

 */
int tx2_car_gate_enable(clock_sys_t *clock_sys, enum clock_gate gate, enum clock_gate_mode mode)
{
    // sel4cp_dbg_puts("|tx2_car_gate_enable| Called\n");

    if (!check_valid_gate(gate)) {
        sel4cp_dbg_puts("Invalid clock gate!\n");
        return -EINVAL;
    }

    if (mode == CLKGATE_IDLE || mode == CLKGATE_SLEEP) {
        sel4cp_dbg_puts("Idle and sleep gate modes are not supported");
        return -EINVAL;
    }

    // sel4cp_dbg_puts("|tx2_car_gate_enable| Before command\n");
    uint32_t command = (mode == CLKGATE_ON ? CMD_CLK_ENABLE : CMD_CLK_DISABLE);
    
    // sel4cp_dbg_puts("|tx2_car_gate_enable| Before gateid\n");
    uint32_t bpmp_gate_id = mrq_gate_id_map[gate];
    // sel4cp_dbg_puts("|tx2_car_gate_enable| Got gate_id!\n");

    /* Setup the message and make a call to BPMP */
    struct mrq_clk_request req = { .cmd_and_id = (command << 24) | bpmp_gate_id };
    struct mrq_clk_response res = {0};
    tx2_clk_t *clk = clock_sys->priv;

    // print("sizeof(struct mrq_clk_request) = ");
    // puthex64(sizeof(struct mrq_clk_request));
    // print("\n");

    // print("sizeof(struct mrq_clk_response) = ");
    // puthex64(sizeof(struct mrq_clk_response));
    // print("\n");

    // sel4cp_dbg_puts("|tx2_car_gate_enable| Before bpmp call\n");
    int bytes_recvd = tx2_bpmp_call(clk->bpmp, MRQ_CLK, &req, sizeof(req), &res, sizeof(res));
    if (bytes_recvd < 0) {
        return -EIO;
    }

    return 0;
}


int clock_sys_init(clock_sys_t *clock_sys)
{
    // initialising bpmp 

    if (!clock_sys) {
        if (!clock_sys) {
            sel4cp_dbg_puts("null clock_sys argument");
        }

        return -EINVAL;
    }

    int error = 0;
    tx2_clk_t *clk = NULL;
    void *car_vaddr = (void *)car_base;

    clk = clock_sys->priv;

    clk->car_vaddr =  (volatile void *) car_vaddr;

    clk->bpmp = &bpmp;
    error = tx2_bpmp_init(clk->bpmp);

    // clock_sys->gate_enable = &tx2_car_gate_enable;
    // clock_sys->get_clock = &tx2_car_get_clock;

    return 0;
}


