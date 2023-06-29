/*
 * Copyright 2019, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#pragma once

struct gpio_sys;
typedef struct gpio_sys gpio_sys_t;
typedef int gpio_id_t;

#include <stdbool.h>
#include <errno.h>
// #include <utils/util.h>
// #include <platsupport/io.h>

#define BIT(n) (1ul<<(n))

#define GPIOID(port, pin)             ((port) * 32 + (pin))
#define GPIOID_PORT(gpio)             ((gpio) / 32)
#define GPIOID_PIN(gpio)              ((gpio) % 32)

typedef struct gpio gpio_t;
struct gpio {
/// GPIO port identifier
    gpio_id_t id;
/// GPIO subsystem handle
    gpio_sys_t *gpio_sys;
};

typedef enum gpio_dir {
/// Input direction
    GPIO_DIR_IN,
    /* Output direction:
     * DEFAULT_LOW will ensure that the pin stays low even while the driver is
     * initializing the pin.
     * DEFAULT_HIGH will ensure that the pin stays high even while the driver is
     * initializing the pin.
     */
    GPIO_DIR_OUT_DEFAULT_LOW,
    GPIO_DIR_OUT = GPIO_DIR_OUT_DEFAULT_LOW,
    GPIO_DIR_OUT_DEFAULT_HIGH,

/// Input direction with IRQ on low logic level
    GPIO_DIR_IRQ_LOW,
/// Input direction with IRQ on high logic level
    GPIO_DIR_IRQ_HIGH,
/// Input direction with IRQ on falling edge
    GPIO_DIR_IRQ_FALL,
/// Input direction with IRQ on rising edge
    GPIO_DIR_IRQ_RISE,
/// Input direction with IRQ on both rising and falling edges
    GPIO_DIR_IRQ_EDGE
} gpio_dir_t;

typedef enum gpio_level {
    /* GPIO input/output levels */
    GPIO_LEVEL_LOW,
    GPIO_LEVEL_HIGH
} gpio_level_t;

struct gpio_sys {
    /** Initialize a GPIO pin.
     * @param   gpio_sys        Initialized gpio driver instance.
     * @param   id              ID of the pin to initialize a handle to.
     * @param   gpio_dir        Configure the pin for input/output/IRQ.
     *                          Use GPIO_DIR_OUT_DEFAULT_HIGH and
     *                          GPIO_DIR_OUT_DEFAULT_LOW to set the pin's
     *                          default logic level. Can be useful for things
     *                          like GPIO pins used as SPI chipselects where
     *                          you want to ensure that a pin stays in a certain
     *                          logic level even while this initialization
     *                          function is running.
     * @param   gpio[out]       Pointer to a gpio_t structure to be initialised.
     * @return 0 on success. Non-zero on error.
     */
    int (*init)(gpio_sys_t *gpio_sys, gpio_id_t id, enum gpio_dir dir, gpio_t *gpio);

    /**
     * Set a GPIO's output level. The pin must be configured for output
     * for this to work.
     * @param   gpio    Initialised GPIO pin instance.
     * @param   level   The output level to set for the pin.
     * @return 0 on success. Non-zero on error.
     */
    int (*set_level)(gpio_t *gpio, enum gpio_level level);

    /**
     * Read a pin's input level. The pin must be configured for input for
     * this to work.
     * @param   gpio    Initialised GPIO pin instance.
     * @return GPIO_LEVEL_LOW or GPIO_LEVEL_HIGH depending on the input level.
     *         Negative integer on error.
     */
    int (*read_level)(gpio_t *gpio);

    /**
     * Read and manipulate the status of a pending IRQ.
     * @param   gpio    Initialised GPIO pin instance.
     * @param   clear   Flag indicating whether or not the pending IRQ
     *                  should be cleared.
     * @return 0 (none) or 1 (pending) depending on the status of the IRQ.
     *         Negative integer on error.
     */
    int (*pending_status)(gpio_t *gpio, bool clear);

    /**
     * Enable or disable the IRQ signal from the pin.
     * @param   gpio    Initialised GPIO pin instance.
     * @param   enable  Flag indicating whether or not the IRQ signal
     *                  should be enabled.
     * @return 0 on success. Non-zero on error.
     */
    int (*irq_enable_disable)(gpio_t *gpio, bool enable);

/// platform specific private data
    volatile void *priv;
};

static inline bool gpio_sys_valid(const gpio_sys_t *gpio_sys)
{
    return gpio_sys != NULL && gpio_sys->priv != NULL;
}

static inline bool gpio_instance_valid(const gpio_t *gpio)
{
    if (!gpio) {
        sel4cp_dbg_puts("Handle to GPIO not supplied!");
        return false;
    }
    if (!gpio->gpio_sys) {
        sel4cp_dbg_puts("GPIO pin's parent controller handle invalid!");
        return false;
    }
    return true;
}

/**
 * Initialise the GPIO subsystem and provide a handle for access
 * @param[out] gpio_sys A gpio handle structure to initialise
 * @return              0 on success, errno value otherwise
 */
int gpio_sys_init(gpio_sys_t *gpio_sys);

/**
 * Clear a GPIO pin
 * @param[in] a handle to a GPIO
 * @return    0 on success, otherwise errno value
 */
static inline int gpio_clr(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->set_level) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->set_level(gpio, GPIO_LEVEL_LOW);
}

/**
 * Return the state of a GPIO pin
 * @param[in] a handle to a GPIO
 * @return    the value of the pin, otherwise errno value
 */
static inline int gpio_get(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->read_level) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->read_level(gpio);
}

/**
 * Set a GPIO pin
 * @param[in] a handle to a GPIO
 * @return    0 on success, otherwise errno value
 */
static inline int gpio_set(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->set_level) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->set_level(gpio, GPIO_LEVEL_HIGH);
}

/**
 * Check if an IRQ is pending for this GPIO
 * @param[in]  a handle to a GPIO
 * @return     errno value on error
 *             0 if an IRQ is not pending
 *             1 if an IRQ is pending
 */
static inline int gpio_is_pending(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->pending_status) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->pending_status(gpio, false);
}

/**
 * Clear pending IRQs for this GPIO
 * @param[in] a handle to a GPIO
 * @return    0 on success, errno value on error
 */
static inline int gpio_pending_clear(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->pending_status) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    int ret = gpio->gpio_sys->pending_status(gpio, true);
    if (ret < 0) {
        return ret;
    }
    return 0;
}

/**
 * Enable the IRQ signal from the pin.
 * @param[in] gpio Handle to the pin to manipulate
 * @return 0 for success, errno value on error.
 */
static inline int gpio_irq_enable(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->irq_enable_disable) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->irq_enable_disable(gpio, true);
}

/**
 * Disable the IRQ signal from the pin.
 * @param[in] gpio Handle to the pin to manipulate
 * @return 0 for success, errno value on error.
 */
static inline int gpio_irq_disable(gpio_t *gpio)
{
    if (!gpio_instance_valid(gpio)) {
        return -EINVAL;
    }
    if (!gpio->gpio_sys->irq_enable_disable) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }
    return gpio->gpio_sys->irq_enable_disable(gpio, false);
}

/**
 * Acquire a handle to a GPIO pin
 * @param[in]  gpio_sys  a handle to an initialised GPIO subsystem\
 * @param[in]  id        A pin identifier obtained from the macro
 *                       GPIOID(port, pin)
 * @param[in]  dir       The direction of the pin
 * @param[out] gpio      a GPIO handle to initialise
 * @return               0 on success
 */
static inline int gpio_new(gpio_sys_t *gpio_sys, gpio_id_t id, enum gpio_dir dir, gpio_t *gpio)
{
    if (!gpio_sys) {
        sel4cp_dbg_puts("Handle to GPIO controller not supplied!");
        return -EINVAL;
    }

    if (!gpio_sys->init) {
        sel4cp_dbg_puts("Unimplemented");
        return -ENOSYS;
    }

    if (!gpio) {
        sel4cp_dbg_puts("Handle to output pin structure not supplied!");
        return -EINVAL;
    }

    return gpio_sys->init(gpio_sys, id, dir, gpio);
}

#define TX2_GPIO_PADDR 0x2210000
#define TX2_GPIO_SIZE 0x10000

enum gpio_port {
    GPIO_PORT_A,
    GPIO_PORT_B,
    GPIO_PORT_C,
    GPIO_PORT_D,
    GPIO_PORT_E,
    GPIO_PORT_F,
    GPIO_PORT_G,
    GPIO_PORT_H,
    GPIO_PORT_I,
    GPIO_PORT_J,
    GPIO_PORT_K,
    GPIO_PORT_L,
    GPIO_PORT_M,
    GPIO_PORT_N,
    GPIO_PORT_O,
    GPIO_PORT_P,
    GPIO_PORT_Q,
    GPIO_PORT_R,
    GPIO_PORT_T,
    GPIO_PORT_X,
    GPIO_PORT_Y,
    GPIO_PORT_BB,
    GPIO_PORT_CC,
    GPIO_NPORTS
};

enum gpio_pin {
    GPIO_PA0 = 0,
    GPIO_PA1,
    GPIO_PA2,
    GPIO_PA3,
    GPIO_PA4,
    GPIO_PA5,
    GPIO_PA6,
    GPIO_PB0 = 8, /* the pins are port aligned to help with pin-to-port conversion */
    GPIO_PB1,
    GPIO_PB2,
    GPIO_PB3,
    GPIO_PB4,
    GPIO_PB5,
    GPIO_PB6,
    GPIO_PC0 = 16,
    GPIO_PC1,
    GPIO_PC2,
    GPIO_PC3,
    GPIO_PC4,
    GPIO_PC5,
    GPIO_PC6,
    GPIO_PD0 = 24,
    GPIO_PD1,
    GPIO_PD2,
    GPIO_PD3,
    GPIO_PD4,
    GPIO_PD5,
    GPIO_PE0 = 32,
    GPIO_PE1,
    GPIO_PE2,
    GPIO_PE3,
    GPIO_PE4,
    GPIO_PE5,
    GPIO_PE6,
    GPIO_PE7,
    GPIO_PF0 = 40,
    GPIO_PF1,
    GPIO_PF2,
    GPIO_PF3,
    GPIO_PF4,
    GPIO_PF5,
    GPIO_PG0 = 48,
    GPIO_PG1,
    GPIO_PG2,
    GPIO_PG3,
    GPIO_PG4,
    GPIO_PG5,
    GPIO_PH0 = 56,
    GPIO_PH1,
    GPIO_PH2,
    GPIO_PH3,
    GPIO_PH4,
    GPIO_PH5,
    GPIO_PH6,
    GPIO_PI0 = 64,
    GPIO_PI1,
    GPIO_PI2,
    GPIO_PI3,
    GPIO_PI4,
    GPIO_PI5,
    GPIO_PI6,
    GPIO_PI7,
    GPIO_PJ0 = 72,
    GPIO_PJ1,
    GPIO_PJ2,
    GPIO_PJ3,
    GPIO_PJ4,
    GPIO_PJ5,
    GPIO_PJ6,
    GPIO_PJ7,
    GPIO_PK0 = 80,
    GPIO_PL0 = 88,
    GPIO_PL1,
    GPIO_PL2,
    GPIO_PL3,
    GPIO_PL4,
    GPIO_PL5,
    GPIO_PL6,
    GPIO_PL7,
    GPIO_PM0 = 96,
    GPIO_PM1,
    GPIO_PM2,
    GPIO_PM3,
    GPIO_PM4,
    GPIO_PM5,
    GPIO_PN0 = 104,
    GPIO_PN1,
    GPIO_PN2,
    GPIO_PN3,
    GPIO_PN4,
    GPIO_PN5,
    GPIO_PN6,
    GPIO_PO0 = 112,
    GPIO_PO1,
    GPIO_PO2,
    GPIO_PO3,
    GPIO_PP0 = 120,
    GPIO_PP1,
    GPIO_PP2,
    GPIO_PP3,
    GPIO_PP4,
    GPIO_PP5,
    GPIO_PP6,
    GPIO_PQ0 = 128,
    GPIO_PQ1,
    GPIO_PQ2,
    GPIO_PQ3,
    GPIO_PQ4,
    GPIO_PQ5,
    GPIO_PR0 = 136,
    GPIO_PR1,
    GPIO_PR2,
    GPIO_PR3,
    GPIO_PR4,
    GPIO_PR5,
    GPIO_PT0 = 144,
    GPIO_PT1,
    GPIO_PT2,
    GPIO_PT3,
    GPIO_PX0 = 152,
    GPIO_PX1,
    GPIO_PX2,
    GPIO_PX3,
    GPIO_PX4,
    GPIO_PX5,
    GPIO_PX6,
    GPIO_PX7,
    GPIO_PY0 = 160,
    GPIO_PY1,
    GPIO_PY2,
    GPIO_PY3,
    GPIO_PY4,
    GPIO_PY5,
    GPIO_PY6,
    GPIO_PBB0 = 168,
    GPIO_PBB1,
    GPIO_PCC0 = 176,
    GPIO_PCC1,
    GPIO_PCC2,
    GPIO_PCC3,
};

#define MAX_GPIO_ID GPIO_PCC3
