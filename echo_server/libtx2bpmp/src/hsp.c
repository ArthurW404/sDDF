/*
 * Copyright (c) 2016, NVIDIA CORPORATION.
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

// #include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <sel4cp.h>

// #include <platsupport/pmem.h>
// #include <platsupport/fdt.h>
#include "hsp.h"
#include "util.h"
// #include <utils/util.h>

#define BIT(n) (1ul<<(n))

/* Register holds information about the number of
 * shared mailboxes, shared semaphores etc. */
#define HSP_INT_DIMENSION_OFFSET 0x380
#define HSP_INT_DIMENSION_SM_SHIFT 0
#define HSP_INT_DIMENSION_SS_SHIFT 4
#define HSP_INT_DIMENSION_AS_SHIFT 8
#define HSP_INT_DIMENSION_NUM_MASK 0xf

#define HSP_DOORBELL_BLOCK_STRIDE 0x100

#define HSP_BITMAP_TZ_SECURE_SHFIT 0
#define HSP_BITMAP_TZ_NONSECURE_SHIFT 16

uintptr_t hsp_regs;

typedef struct tx2_hsp_priv {
    // ps_io_ops_t *io_ops;
    volatile void *hsp_base;
    void *doorbell_base;
    uintptr_t tx2_hsp_region;
} tx2_hsp_priv_t;

tx2_hsp_priv_t hsp_priv_mem;

enum dbell_reg_offset {
    DBELL_TRIGGER = 0x0,
    DBELL_ENABLE = 0x4,
    DBELL_RAW = 0x8,
    DBELL_PENDING = 0xc
};

enum dbell_bitmap_offset {
    CCPLEX_BIT = BIT(1),
    DPMU_BIT = BIT(2),
    BPMP_BIT = BIT(3),
    SPE_BIT = BIT(4),
    CPE_BIT = BIT(5),
    SCE_BIT = CPE_BIT,
    DMA_BIT = BIT(6),
    TSECA_BIT = BIT(7),
    TSECB_BIT = BIT(8),
    JTAGM_BIT = BIT(9),
    CSITE_BIT = BIT(10),
    APE_BIT = BIT(11)
};


static bool check_doorbell_id_is_valid(enum tx2_doorbell_id db_id)
{
    if (CCPLEX_PM_DBELL <= db_id && db_id <= APE_DBELL) {
        return true;
    }
    return false;
}

static uint32_t *hsp_get_doorbell_register(tx2_hsp_priv_t *hsp, enum tx2_doorbell_id db_id,
                                           enum dbell_reg_offset offset)
{
    // assert(hsp);
    // assert(DBELL_TRIGGER <= offset && offset <= DBELL_PENDING);
    if (!hsp || !(DBELL_TRIGGER <= offset && offset <= DBELL_PENDING)) {
        sel4cp_dbg_puts("Something wrong with hsp_get_doorbell_register!\n");
        return -1;
    }
    return hsp->doorbell_base + db_id * HSP_DOORBELL_BLOCK_STRIDE + offset;
}

// don't need to clean up in sel4cp
static int hsp_destroy(void *data)
{
    // tx2_hsp_priv_t *hsp_priv = data;

    // /* The doorbell base is just an offset from the hsp base, so we only need
    //  * to deallocate the hsp base */
    // if (hsp_priv->hsp_base) {
    //     ps_io_unmap(&hsp_priv->io_ops->io_mapper, hsp_priv->hsp_base, hsp_priv->tx2_hsp_region.length);
    // }

    // ps_io_ops_t *temp_ops = hsp_priv->io_ops;

    // sel4cp_dbg_puts_IF(ps_free(&temp_ops->malloc_ops, sizeof(*hsp_priv), hsp_priv),
    //            "Failed to de-allocate the private data for HSP");

    return 0;
}

static int hsp_doorbell_ring(void *data, enum tx2_doorbell_id db_id)
{
    if (!check_doorbell_id_is_valid(db_id)) {
        sel4cp_dbg_puts("Invalid doorbell ID!");
        
        return -EINVAL;
    }

    tx2_hsp_priv_t *hsp_priv = data;

    /* Write any value to the trigger register to 'ring' the doorbell */
    uint32_t *trigger_reg = hsp_get_doorbell_register(hsp_priv, db_id, DBELL_TRIGGER);
    // assert(trigger_reg);
    if (!trigger_reg) {
        sel4cp_dbg_puts("Something wrong with trigger_reg!!\n");
        return -EINVAL;
    }
    *trigger_reg = 1;

    return 0;
}

static int hsp_doorbell_check(void *data, enum tx2_doorbell_id db_id)
{
    if (!check_doorbell_id_is_valid(db_id)) {
        sel4cp_dbg_puts("Invalid doorbell ID!");
        return -EINVAL;
    }

    tx2_hsp_priv_t *hsp_priv = data;

    /* Checking if the doorbell has been 'rung' requires checking for proper
     * bit in the bitfield. The bitfield is also split into TrustZone secure
     * and TZ non-secure. Refer to Figure 75 in Section 14.8.5 for further details. */
    uint32_t *pending_reg = hsp_get_doorbell_register(hsp_priv, db_id, DBELL_PENDING);

    enum dbell_bitmap_offset bitmap_offset;
    switch (db_id) {
    case CCPLEX_PM_DBELL:
    case CCPLEX_TZ_UNSECURE_DBELL:
    case CCPLEX_TZ_SECURE_DBELL:
        bitmap_offset = CCPLEX_BIT;
        break;
    case BPMP_DBELL:
        bitmap_offset = BPMP_BIT;
        break;
    case SPE_DBELL:
        bitmap_offset = SPE_BIT;
        break;
    case SCE_DBELL:
        bitmap_offset = SCE_BIT;
        break;
    case APE_DBELL:
        bitmap_offset = APE_BIT;
        break;
    default:
        // sel4cp_dbg_puts("We shouldn't get here, doorbell ID is %d", db_id);
        sel4cp_dbg_puts("We shouldn't get here, doorbell ID is (%d not implemented...)\n");
    }

    /* Usermode isn't in TrustZone secure, so we just default to TZ non-secure */
    int is_pending = *pending_reg & (bitmap_offset << HSP_BITMAP_TZ_NONSECURE_SHIFT);

    if (is_pending) {
        *pending_reg &= ~(bitmap_offset << HSP_BITMAP_TZ_NONSECURE_SHIFT);
    }

    return (is_pending != 0);
}

int tx2_hsp_init(tx2_hsp_t *hsp)
{
    if (!hsp) {
        sel4cp_dbg_puts("Arguments are NULL!");
        return -EINVAL;
    }

    int error = 0;

    tx2_hsp_priv_t *hsp_priv = &hsp_priv_mem;

    hsp_priv->hsp_base = (volatile void *)hsp_regs;
    // print("==>|tx2_hsp_init|hsp_priv->hsp_base =");
    // puthex64(hsp_priv->hsp_base);
    // print("\n");
    hsp_priv->tx2_hsp_region = 0x3c00000;

    /* Get the base addr of the doorbell
     * Section 14.8.5: All doorbell registers are in a single page, doorbell
     * {db} has a register range starting at DB{db}_BASE = HSP_{inst}_BASE +
     * (1+ nSM/2 + nSS + nAS) * 64 KiB + {db} * 0x100. */

    int num_sm = 0, num_ss = 0, num_as = 0;

    volatile uint32_t *int_dim_reg = hsp_priv->hsp_base + HSP_INT_DIMENSION_OFFSET;

    num_sm = (*int_dim_reg >> HSP_INT_DIMENSION_SM_SHIFT) & HSP_INT_DIMENSION_NUM_MASK;
    num_ss = (*int_dim_reg >> HSP_INT_DIMENSION_SS_SHIFT) & HSP_INT_DIMENSION_NUM_MASK;
    num_as = (*int_dim_reg >> HSP_INT_DIMENSION_AS_SHIFT) & HSP_INT_DIMENSION_NUM_MASK;

    // print("==>|tx2_hsp_init|num_sm =");
    // puthex64(num_sm);
    // print("\n");


    // print("==>|tx2_hsp_init|num_ss =");
    // puthex64(num_ss);
    // print("\n");

    // print("==>|tx2_hsp_init|num_as =");
    // puthex64(num_as);
    // print("\n");

    hsp_priv->doorbell_base = hsp_priv->hsp_base + (1 + (num_sm / 2) + num_ss + num_as) * 0x10000;

    hsp->data = hsp_priv;
    hsp->ring = hsp_doorbell_ring;
    hsp->check = hsp_doorbell_check;
    hsp->destroy = hsp_destroy;

    return 0;
}
