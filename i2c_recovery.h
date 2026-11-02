/**
 ******************************************************************************
 * @file    i2c_recovery.h
 * @brief   Recovery of an STM32U5 I2C peripheral and of a stuck I2C bus.
 *
 *          Both functions block and use HAL_Delay(): call them from the main
 *          loop only, never from an interrupt handler, and only while no
 *          transfer is being started on the same bus.
 ******************************************************************************
 */

#ifndef I2C_RECOVERY_H
#define I2C_RECOVERY_H

#include "stm32u5xx_hal.h"
#include <stdint.h>

/** Result of a recovery call. */
typedef enum
{
    I2C_RECOVERY_OK = 0,        /* Recovery completed                        */
    I2C_RECOVERY_ERR_PARAM,     /* NULL pointer                              */
    I2C_RECOVERY_ERR_HAL,       /* HAL de-init / init / filter setup failed  */
    I2C_RECOVERY_ERR_BUS_STUCK  /* SDA or SCL still low after the bus clear  */
} i2c_recovery_status_t;

/** The GPIO pins of one I2C bus. */
typedef struct
{
    GPIO_TypeDef *scl_port;
    uint16_t scl_pin;
    GPIO_TypeDef *sda_port;
    uint16_t sda_pin;
} i2c_recovery_pins_t;

/**
 * @brief  Reset the I2C peripheral and its HAL handle, keeping hi2c->Init.
 */
i2c_recovery_status_t i2c_recovery_reset_peripheral(I2C_HandleTypeDef *hi2c);

/**
 * @brief  Report whether SDA and SCL both read high (bus idle levels).
 * @retval 1 if both lines are high, otherwise 0 (also for a NULL pointer).
 */
uint8_t i2c_recovery_lines_high(const i2c_recovery_pins_t *pins);

/**
 * @brief  Free a bus held low by a target: clock SCL by hand, send a STOP,
 *         then return the pins to the I2C peripheral.
 */
i2c_recovery_status_t i2c_recovery_clear_bus(I2C_HandleTypeDef *hi2c,
                                             const i2c_recovery_pins_t *pins);

#endif /* I2C_RECOVERY_H */
