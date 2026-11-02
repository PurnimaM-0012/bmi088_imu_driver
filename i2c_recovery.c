/**
 ******************************************************************************
 * @file    i2c_recovery.c
 * @brief   Recovery of an STM32U5 I2C peripheral and of a stuck I2C bus.
 *
 *          Design rules followed in this file (MISRA C:2012):
 *            - every HAL return value is checked             (Dir 4.7)
 *            - every pointer argument is checked for NULL    (Dir 4.1)
 *            - single point of exit per function             (Rule 15.5)
 *            - every if / else if chain ends with else       (Rule 15.7)
 *            - all loops have a fixed upper bound
 ******************************************************************************
 */

#include "i2c_recovery.h"
#include <stddef.h>

#define I2C_RECOVERY_MAX_CLOCKS     (9U)    /* Enough to finish any byte + ACK */
#define I2C_RECOVERY_HALF_PERIOD_MS (1U)    /* SCL half period; HAL_Delay() waits
                                               1-2 ms, far below any I2C limit */

/**
 * @brief  De-initialise and re-initialise the peripheral from hi2c->Init.
 * @note   HAL_I2C_DeInit() clears PE, which releases SCL/SDA and resets the
 *         state machine and flags (RM0456 65.4.6), and also clears the handle
 *         state, error code and lock, so a transfer that never finished cannot
 *         keep the handle busy. The filters are set as in MX_I2C2_Init().
 *
 * @param  hi2c  I2C bus handle.
 * @retval I2C_RECOVERY_OK, I2C_RECOVERY_ERR_PARAM or I2C_RECOVERY_ERR_HAL.
 */
i2c_recovery_status_t i2c_recovery_reset_peripheral(I2C_HandleTypeDef *hi2c)
{
    i2c_recovery_status_t status = I2C_RECOVERY_OK;

    if (hi2c == NULL)
    {
        status = I2C_RECOVERY_ERR_PARAM;
    }
    else if (HAL_I2C_DeInit(hi2c) != HAL_OK)
    {
        status = I2C_RECOVERY_ERR_HAL;
    }
    else if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        status = I2C_RECOVERY_ERR_HAL;
    }
    else if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        status = I2C_RECOVERY_ERR_HAL;
    }
    else if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0U) != HAL_OK)
    {
        status = I2C_RECOVERY_ERR_HAL;
    }
    else
    {
        /* Peripheral re-initialised. */
    }

    return status;
}

/**
 * @brief  Report whether SDA and SCL both read high.
 * @note   The input data register follows the pin in both I2C alternate
 *         function mode and open-drain output mode.
 *
 * @param  pins  GPIO pins of the bus.
 * @retval 1 if both lines are high, otherwise 0 (also for a NULL pointer).
 */
uint8_t i2c_recovery_lines_high(const i2c_recovery_pins_t *pins)
{
    uint8_t high = 0U;

    if (pins != NULL)
    {
        if ((HAL_GPIO_ReadPin(pins->scl_port, pins->scl_pin) == GPIO_PIN_SET) &&
            (HAL_GPIO_ReadPin(pins->sda_port, pins->sda_pin) == GPIO_PIN_SET))
        {
            high = 1U;
        }
    }

    return high;
}

/**
 * @brief  Free a bus held low by a target.
 * @note   A target interrupted in the middle of a byte may keep SDA low while
 *         it waits for clocks. Sequence:
 *           1. take the pins from the peripheral, as open-drain outputs
 *              released high (no internal pull-up: the bus has its own);
 *           2. clock SCL until SDA is released, at most 9 times;
 *           3. send a STOP so the target returns to idle;
 *           4. read both lines, then give the pins back to the peripheral.
 *         The peripheral is re-initialised even if the bus stays stuck, so
 *         the handle is left in a known state.
 *
 * @param  hi2c  I2C bus handle.
 * @param  pins  GPIO pins of the same bus.
 * @retval I2C_RECOVERY_OK, I2C_RECOVERY_ERR_PARAM, I2C_RECOVERY_ERR_HAL or
 *         I2C_RECOVERY_ERR_BUS_STUCK.
 */
i2c_recovery_status_t i2c_recovery_clear_bus(I2C_HandleTypeDef *hi2c,
                                             const i2c_recovery_pins_t *pins)
{
    i2c_recovery_status_t status = I2C_RECOVERY_OK;
    GPIO_InitTypeDef gpio = {0};
    uint8_t clocks = 0U;
    uint8_t lines_high = 0U;

    if ((hi2c == NULL) || (pins == NULL))
    {
        status = I2C_RECOVERY_ERR_PARAM;
    }
    else if (HAL_I2C_DeInit(hi2c) != HAL_OK)
    {
        status = I2C_RECOVERY_ERR_HAL;
    }
    else
    {
        HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(pins->sda_port, pins->sda_pin, GPIO_PIN_SET);

        gpio.Mode = GPIO_MODE_OUTPUT_OD;
        gpio.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_LOW;
        gpio.Pin = pins->scl_pin;
        HAL_GPIO_Init(pins->scl_port, &gpio);
        gpio.Pin = pins->sda_pin;
        HAL_GPIO_Init(pins->sda_port, &gpio);
        HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);

        while ((clocks < I2C_RECOVERY_MAX_CLOCKS) &&
               (HAL_GPIO_ReadPin(pins->sda_port, pins->sda_pin) == GPIO_PIN_RESET))
        {
            HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, GPIO_PIN_RESET);
            HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);
            HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, GPIO_PIN_SET);
            HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);
            clocks = (uint8_t)(clocks + 1U);
        }

        /* STOP: SDA rises while SCL is high. */
        HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, GPIO_PIN_RESET);
        HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);
        HAL_GPIO_WritePin(pins->sda_port, pins->sda_pin, GPIO_PIN_RESET);
        HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);
        HAL_GPIO_WritePin(pins->scl_port, pins->scl_pin, GPIO_PIN_SET);
        HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);
        HAL_GPIO_WritePin(pins->sda_port, pins->sda_pin, GPIO_PIN_SET);
        HAL_Delay(I2C_RECOVERY_HALF_PERIOD_MS);

        lines_high = i2c_recovery_lines_high(pins);

        if (HAL_I2C_Init(hi2c) != HAL_OK)
        {
            status = I2C_RECOVERY_ERR_HAL;
        }
        else if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
        {
            status = I2C_RECOVERY_ERR_HAL;
        }
        else if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0U) != HAL_OK)
        {
            status = I2C_RECOVERY_ERR_HAL;
        }
        else if (lines_high == 0U)
        {
            status = I2C_RECOVERY_ERR_BUS_STUCK;
        }
        else
        {
            /* Bus released and peripheral re-initialised. */
        }
    }

    return status;
}
