/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

#include <sel4cp.h>
#include "reset-bindings.h"

typedef enum reset_id reset_id_t;

typedef struct reset_sys {
    int (*reset_assert)(void *data, reset_id_t id);
    int (*reset_deassert)(void *data, reset_id_t id);
    void *data;
} reset_sys_t;


// TODO temporarily exposed to make things work
typedef struct tx2_reset {
    // ps_io_ops_t *io_ops;
    struct tx2_bpmp *bpmp;
} tx2_reset_t;

int reset_sys_init( void *dependencies, reset_sys_t *reset);

static inline int reset_sys_assert(reset_sys_t *reset, reset_id_t id)
{
    if (!reset) {
        sel4cp_dbg_puts("Reset sub system is invalid!");
        return -EINVAL;
    }

    if (!reset->reset_assert) {
        sel4cp_dbg_puts("not implemented");
        return -ENOSYS;
    }

    return reset->reset_assert(reset->data, id);
}

static inline int reset_sys_deassert(reset_sys_t *reset, reset_id_t id)
{
    if (!reset) {
        sel4cp_dbg_puts("Reset sub system is invalid!");
        return -EINVAL;
    }

    if (!reset->reset_deassert) {
        sel4cp_dbg_puts("not implemented");
        return -ENOSYS;
    }

    return reset->reset_deassert(reset->data, id);
}
