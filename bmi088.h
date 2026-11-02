/**
 ******************************************************************************
 * @file    bmi088.h
 * @brief   BMI088 6-axis IMU driver (accelerometer + gyroscope) over I2C.
 *
 *          Target : STM32U575 (I2C master), STM32U5 HAL.
 *          Coding : MISRA C:2012.
 ******************************************************************************
 */

#ifndef BMI088_H
#define BMI088_H

#include "stm32u5xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------------- */
/* I2C                                                                       */
/* ------------------------------------------------------------------------- */
#define BMI088_I2C_TIMEOUT_MS (10U)    /* Max blocking time per transfer */

/* ------------------------------------------------------------------------- */
/* Gyroscope address and registers (BMI088 datasheet, gyro register map)    */
/* ------------------------------------------------------------------------- */
/* 7-bit address 0x69 (SDO2 = VDDIO, as on the Grove module; 0x68 if
 * SDO2 = GND), shifted left by 1 for the HAL. */
#define BMI088_GYRO_I2C_ADDR       (0x69U << 1U)

#define BMI088_GYRO_CHIP_ID_REG    (0x00U)   /* RO                             */
#define BMI088_GYRO_CHIP_ID_VAL    (0x0FU)   /* Expected identifier            */

#define BMI088_GYRO_RATE_DATA_REG (0x02U)   /* First of 6 data registers      */
#define BMI088_GYRO_DATA_LEN       (6U)      /* X, Y, Z: LSB then MSB each     */

#define BMI088_GYRO_RANGE_REG      (0x0FU)   /* RW, all 8 bits read back       */
#define BMI088_GYRO_RANGE_MASK     (0xFFU)
/* Selected range. VAL and DPS must match each other (datasheet GYRO_RANGE
 * table: 0x04 = +/-125 deg/s, 262.144 LSB/deg/s). */
#define BMI088_GYRO_RANGE_VAL      (0x04U)   /* Value written to GYRO_RANGE    */
#define BMI088_GYRO_RANGE_DPS      (125.0F)  /* Full scale for that value, deg/s */

#define BMI088_GYRO_BW_REG         (0x10U)   /* GYRO_BANDWIDTH                 */
#define BMI088_GYRO_BW_VAL         (0x07U)   /* ODR 100 Hz, filter BW 32 Hz    */
#define BMI088_GYRO_BW_MASK        (0x7FU)   /* Bit 7 always reads back as 1   */
#define BMI088_GYRO_BW_RESET_VAL   (0x80U)   /* Read-back after a soft reset   */

#define BMI088_GYRO_LPM1_REG       (0x11U)   /* Power mode                     */
#define BMI088_GYRO_LPM1_NORMAL    (0x00U)   /* Normal mode                    */

#define BMI088_GYRO_SOFTRESET_REG  (0x14U)   /* WO                             */
#define BMI088_GYRO_SOFTRESET_CMD  (0xB6U)   /* Only value that triggers reset */

/* Wait after soft reset, and after a power-mode change. Datasheet: 30 ms;
 * 5 ms margin added (engineering choice) for temperature and part spread. */
#define BMI088_GYRO_RESET_DELAY_MS (35U)
#define BMI088_GYRO_MODE_DELAY_MS  (35U)

/* ------------------------------------------------------------------------- */
/* Fault handling                                                            */
/* ------------------------------------------------------------------------- */
/* Consecutive failed updates before the gyro is marked BMI088_STATE_ERROR.
 * Engineering choice, not from the datasheet; subject to change. */
#define BMI088_MAX_CONSEC_ERRORS (5U)

/* ------------------------------------------------------------------------- */
/* Gyroscope data                                                            */
/* ------------------------------------------------------------------------- */
#define BMI088_AXES (3U)    /* X, Y, Z */

/* Conversion: deg/s = counts * range_dps / 32768 (32768 / 125 = 262.144
 * LSB per deg/s, datasheet GYRO_RANGE table); rad/s = deg/s * pi / 180. */
#define BMI088_GYRO_FULL_SCALE_COUNTS (32768.0F)
#define BMI088_DEG_TO_RAD             (3.14159265358979F / 180.0F)

/* ------------------------------------------------------------------------- */
/* Types                                                                     */
/* ------------------------------------------------------------------------- */

/** Result of a single driver call. */
typedef enum
{
    BMI088_OK = 0,          /* Call succeeded                                  */
    BMI088_ERR_PARAM,       /* NULL pointer or invalid argument                */
    BMI088_ERR_I2C,         /* HAL I2C transfer failed / timed out             */
    BMI088_ERR_VERIFY,      /* Register read-back did not match written value  */
    BMI088_ERR_CHIP_ID,     /* Chip ID register did not hold the expected ID   */
    BMI088_ERR_STUCK,       /* New sample identical to the previous sample     */
    BMI088_ERR_STATE,       /* Gyro not in BMI088_STATE_NORMAL; nothing read   */
    BMI088_ERR_BUSY         /* Previous interrupt-driven read still running    */
} bmi088_status_t;

/** Health of the device, kept across calls. */
typedef enum
{
    BMI088_STATE_UNINIT = 0, /* Init not run yet; data not usable             */
    BMI088_STATE_NORMAL,     /* Init passed; data usable                      */
    BMI088_STATE_ERROR       /* Too many consecutive faults; data not trusted */
} bmi088_dev_state_t;

/** 32-bit floating point (MISRA Dir 4.6: type names show size). */
typedef float bmi088_float32_t;

/** Everything the driver keeps about one BMI088 gyroscope. */
typedef struct
{
    I2C_HandleTypeDef *hi2c;               /* I2C bus the gyro is on            */
    uint16_t dev_addr;                     /* 7-bit address << 1 (HAL format)   */
    bmi088_float32_t range_dps;            /* Full-scale range in use, deg/s    */
    int16_t sample[BMI088_AXES];           /* Latest sample, sensor counts      */
    bmi088_float32_t angular_velocity[BMI088_AXES]; /* Latest rate, rad/s   */
    uint8_t rx_buf[BMI088_GYRO_DATA_LEN];  /* Interrupt-driven read lands here  */
    volatile bool busy;                    /* Read started, not yet finished;
                                              shared with interrupt handlers    */
    uint8_t fault_count;                   /* Consecutive faulty updates        */
    bmi088_dev_state_t state;              /* Device health                     */
} bmi088_gyro_t;

/* ------------------------------------------------------------------------- */
/* Public functions                                                          */
/* ------------------------------------------------------------------------- */

/**
 * @brief  Initialise the gyro: reset it, check it, configure range and
 *         bandwidth. Call once after MX_I2C2_Init(), and again to recover
 *         from BMI088_STATE_ERROR. Blocks for about 35-70 ms.
 */
bmi088_status_t bmi088_gyro_init(bmi088_gyro_t *gyro, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Read a new sample and convert it to rad/s in gyro->angular_velocity.
 *         Call periodically (100 Hz, matching the gyro ODR) after init.
 *         Use the data only while gyro->state == BMI088_STATE_NORMAL.
 */
bmi088_status_t bmi088_gyro_update(bmi088_gyro_t *gyro);

/**
 * @brief  Start a non-blocking read of one sample (interrupt-driven I2C).
 *         Call from the 100 Hz timer interrupt. The result is handled when
 *         the I2C transfer finishes (bmi088_gyro_read_complete).
 */
bmi088_status_t bmi088_gyro_start_read(bmi088_gyro_t *gyro);

/** @brief Process a finished read. Call from HAL_I2C_MemRxCpltCallback(). */
bmi088_status_t bmi088_gyro_read_complete(bmi088_gyro_t *gyro);

/** @brief Handle a failed read. Call from HAL_I2C_ErrorCallback(). */
bmi088_status_t bmi088_gyro_read_error(bmi088_gyro_t *gyro);

#endif /* BMI088_H */
