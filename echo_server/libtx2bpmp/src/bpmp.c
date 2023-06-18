/*
 * Copyright (c) 2016, NVIDIA CORPORATION.
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

/*
 * This is a port of the Tegra186 BPMP sources from U-boot with some additional
 * modifications. Similar to the Tegra IVC protocol, there's no documentation
 * on the BPMP module ABI.
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <sel4cp.h>

// #include <platsupport/pmem.h>
// #include <platsupport/fdt.h>
// #include <platsupport/driver_module.h>
#include "bpmp.h"
#include "hsp.h"
#include "ivc.h"
#include "util.h"
// #include <utils/util.h>

#define BIT(n) (1ul<<(n))


#define BPMP_IVC_FRAME_COUNT 1
#define BPMP_IVC_FRAME_SIZE 128

#define BPMP_FLAG_DO_ACK	BIT(0)
#define BPMP_FLAG_RING_DOORBELL	BIT(1)

#define TX_SHMEM 0
#define RX_SHMEM 1
#define NUM_SHMEM 2

#define TIMEOUT_THRESHOLD 2000000ul

uintptr_t bpmp_tx_shmem_base;
uintptr_t bpmp_rx_shmem_base;

struct tx2_bpmp_priv {
    // ps_io_ops_t *io_ops;
    tx2_hsp_t hsp;
    bool hsp_initialised;
    struct tegra_ivc ivc;
    volatile void *tx_base; // Virtual address base of the TX shared memory channel
    volatile void *rx_base; // Virtual address base of the RX shared memory channel
    // pmem_region_t bpmp_shmems[NUM_SHMEM];
};


static bool bpmp_initialised = false;
static unsigned int bpmp_refcount = 0;
static struct tx2_bpmp_priv bpmp_data = {0};

static int bpmp_call(void *data, int mrq, void *tx_msg, size_t tx_size, void *rx_msg, size_t rx_size)
{   
    // sel4cp_dbg_puts("|bpmp call| called\n");
	int ret, err;
	void *ivc_frame = NULL;
	struct mrq_request *req;
	struct mrq_response *resp;
	unsigned long timeout = TIMEOUT_THRESHOLD;

    struct tx2_bpmp_priv *bpmp_priv = data;

    // print("bpmp_priv =");
    // puthex64(bpmp_priv);
    // print("\n");

	if ((tx_size > BPMP_IVC_FRAME_SIZE) || (rx_size > BPMP_IVC_FRAME_SIZE))
		return -EINVAL;

    // print("&bpmp_priv->ivc =");
    // puthex64(&bpmp_priv->ivc);
    // print("\n");

	ret = tegra_ivc_write_get_next_frame(&bpmp_priv->ivc, &ivc_frame);
	if (ret) {
		sel4cp_dbg_puts("tegra_ivc_write_get_next_frame() failed: %d(not implemented)\n");
		return ret;
	}

    // print("ivc_frame =");
    // puthex64(ivc_frame);
    // print("\n");

	req = ivc_frame;
	req->mrq = mrq;
	req->flags = BPMP_FLAG_DO_ACK | BPMP_FLAG_RING_DOORBELL;
	memcpy(req + 1, tx_msg, tx_size);

    // sel4cp_dbg_puts("|bpmp_call|memcpy \n");

	ret = tegra_ivc_write_advance(&bpmp_priv->ivc);
	if (ret) {
		// sel4cp_dbg_puts("tegra_ivc_write_advance() failed: %d\n", ret);
		sel4cp_dbg_puts("tegra_ivc_write_advance() failed: %d\n");
		return ret;
	}

    // sel4cp_dbg_puts("|bpmp_call| before timeout \n");


	for (; timeout > 0; timeout--) {
        // sel4cp_dbg_puts("|bpmp_call| in timeout \n");
		ret = tegra_ivc_channel_notified(&bpmp_priv->ivc);
		if (ret) {
			// sel4cp_dbg_puts("tegra_ivc_channel_notified() failed: %d\n", ret);
			sel4cp_dbg_puts("tegra_ivc_channel_notified() failed: %d\n");
			return ret;
		}

		ret = tegra_ivc_read_get_next_frame(&bpmp_priv->ivc, &ivc_frame);
		if (!ret)
			break;
	}

    // sel4cp_dbg_puts("|bpmp_call| after timeout loop \n");


    if (!timeout) {
        // sel4cp_dbg_puts("tegra_ivc_read_get_next_frame() timed out (%d)\n", ret);
        sel4cp_dbg_puts("tegra_ivc_read_get_next_frame() timed out (%d)\n");
        return -ETIMEDOUT;
    }

	resp = ivc_frame;

    // print("ivc_frame =");
    // puthex64(ivc_frame);
    // print("\n");
    
	err = resp->err;
    
    // print("err =");
    // puthex64(err);
    // print("\n");

	if (!err && rx_msg && rx_size) {

        // print("rx_msg =");
        // puthex64(rx_msg);
        // print("\n");

        // print("resp + 1 =");
        // puthex64(resp + 1);
        // print("\n");
        
        // print("rx_size =");
        // puthex64(rx_size);
        // print("\n");

        // sel4cp_dbg_puts("My own memcpy\n");
        
        // for (int i = 0; i < rx_size; ++i) {
        //     char dest_byte = ((char *)rx_msg)[i];
        //     char src_byte = ((char *)(resp + 1))[i];
        //     // puthex64(src_byte);
        //     // print(" ");
            
        //     ((char *)rx_msg)[i] = src_byte;
        // }

        // based on gcc memcpy
        // https://github.com/gcc-mirror/gcc/blob/master/libgcc/memcpy.c
        int len = rx_size;
        char *d = rx_msg;
        const char *s = resp + 1;
        while (len--)
            *d++ = *s++;

        // TODO !!! Temporarily commenting out memcpy (this is faulting for some reason)
		// memcpy(rx_msg, resp + 1, rx_size);
    }


    // sel4cp_dbg_puts("|bpmp_call| before tegra_ivc_read_advance \n");

	ret = tegra_ivc_read_advance(&bpmp_priv->ivc);
	if (ret) {
		// sel4cp_dbg_puts("tegra_ivc_write_advance() failed: %d\n", ret);
		sel4cp_dbg_puts("tegra_ivc_write_advance() failed: %d\n");
		return ret;
	}

	if (err) {
		// sel4cp_dbg_puts("BPMP responded with error %d\n", err);
		sel4cp_dbg_puts("BPMP responded with error %d\n");
		/* err isn't a U-Boot error code, so don't that */
		return -EIO;
	}

	return rx_size;
}

static void bpmp_ivc_notify(struct tegra_ivc *ivc, void *token)
{
	struct tx2_bpmp_priv *bpmp_priv = token;
	int ret;

	ret = tx2_hsp_doorbell_ring(&bpmp_priv->hsp, BPMP_DBELL);
	if (ret)
		// sel4cp_dbg_puts("Failed to ring BPMP's doorbell in the HSP: %d\n", ret);
		sel4cp_dbg_puts("Failed to ring BPMP's doorbell in the HSP: %d\n");
}

static int bpmp_destroy(void *data)
{
    struct tx2_bpmp_priv *bpmp_priv = data;

    bpmp_refcount--;

    if (bpmp_refcount != 0) {
        /* Only cleanup the BPMP structure if there are no more references that are valid. */
        return 0;
    }

    // if (bpmp_priv->hsp_initialised) {
    //     sel4cp_dbg_puts_IF(tx2_hsp_destroy(&bpmp_priv->hsp),
    //                "Failed to clean up after a failed BPMP initialisation process!");
    // }

    // don't need to unmap in sel4cp
    // /* Unmapping the shared memory also destroys the IVC */
    // if (bpmp_priv->tx_base) {
    //     ps_io_unmap(&bpmp_priv->io_ops->io_mapper, bpmp_priv->tx_base, bpmp_data.bpmp_shmems[TX_SHMEM].length);
    // }

    // if (bpmp_priv->rx_base) {
    //     ps_io_unmap(&bpmp_priv->io_ops->io_mapper, bpmp_priv->rx_base, bpmp_data.bpmp_shmems[RX_SHMEM].length);
    // }

    return 0;
}


int tx2_bpmp_init(struct tx2_bpmp *bpmp)
{
    print("|tx2_bpmp_init| called\n");
    
    if (!bpmp) {
        sel4cp_dbg_puts("Arguments are NULL!\n");
        return -EINVAL;
    }

    if (bpmp_initialised) {
        /* If we've initialised the BPMP once, just fill the private data with
         * what we've initialised */
        goto success;
    }

    int ret = 0;
    /* Not sure if this is too long or too short. */
    unsigned long timeout = TIMEOUT_THRESHOLD;

    ret = tx2_hsp_init(&bpmp_data.hsp);
    if (ret) {
        sel4cp_dbg_puts("Failed to initialise the HSP device for BPMP");
        return ret;
    }

    // bpmp_data.io_ops = io_ops;

    bpmp_data.hsp_initialised = true;


    // ==== setup bpmp register mapping
    bpmp_data.tx_base = (volatile void *) bpmp_tx_shmem_base;
    // bpmp_data.bpmp_shmems[TX_SHMEM] = 0x3004e000;
    bpmp_data.rx_base = (volatile void *) bpmp_rx_shmem_base;
    // bpmp_data.bpmp_shmems[RX_SHMEM] = 0x3004f000;

    // ==== setup ivc
    ret = tegra_ivc_init(&bpmp_data.ivc, (unsigned long) bpmp_data.rx_base, (unsigned long) bpmp_data.tx_base,
                         BPMP_IVC_FRAME_COUNT, BPMP_IVC_FRAME_SIZE, bpmp_ivc_notify, (void *) &bpmp_data);
    if (ret) {
        // sel4cp_dbg_puts("tegra_ivc_init() failed: %d", ret);
        sel4cp_dbg_puts("tegra_ivc_init() failed: %d (not implemented)\n");
        goto fail;
    }

    tegra_ivc_channel_reset(&bpmp_data.ivc);
    for (; timeout > 0; timeout--) {
        ret = tegra_ivc_channel_notified(&bpmp_data.ivc);
        if (!ret) {
            break;
        }
    }

    if (!timeout) {
        // sel4cp_dbg_puts("Initial IVC reset timed out (%d)", ret);
        sel4cp_dbg_puts("Initial IVC reset timed out (%d not impl)");

        ret = -ETIMEDOUT;
        goto fail;
    }

    // TODO need to check if this is required

    // ret = ps_interface_register(&io_ops->interface_registration_ops, TX2_BPMP_INTERFACE,
    //                               bpmp, NULL);
    // if (ret) {
    //     sel4cp_dbg_puts("Failed to register the BPMP interface");
    //     goto fail;
    // }

success:
    bpmp_refcount++;

    bpmp->data = &bpmp_data;
    bpmp->call = bpmp_call;
    
    bpmp->destroy = bpmp_destroy;
    bpmp_initialised = true;
    /* Register this BPMP interface so that the reset driver can access it */

    return 0;

fail:
    // sel4cp_dbg_puts_IF(bpmp_destroy(&bpmp_data), "Failed to cleanup the BPMP after a failed initialisation");
    return ret;
}

