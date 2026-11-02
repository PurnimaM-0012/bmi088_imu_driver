/**
 ******************************************************************************
 * @file    bmi088.c
 * @brief   BMI088 6-axis IMU driver (accelerometer + gyroscope) over I2C.
 *
 *          Design rules followed in this file (MISRA C:2012):
 *            - every HAL return value is checked             (Dir 4.7)
 *            - every pointer argument is checked for NULL    (Dir 4.1)
 *            - single point of exit per function             (Rule 15.5)
 *            - every if / else if chain ends with else       (Rule 15.7)
 *            - all I2C transfers have a bounded timeout
 ******************************************************************************
 */

#include "bmi088.h"
#include <stddef.h>

/* ------------------------------------------------------------------------- */
/* Private helpers                                                           */
/* ------------------------------------------------------------------------- */

/**
 * @brief  Read @p len consecutive registers starting at @p reg.
 * @note   BMI088 auto-increments the register address on I2C burst reads.
 *         Over I2C there is no dummy byte (that applies to SPI only).
 *
 * @param  hi2c      I2C bus handle.
 * @param  dev_addr  Device address, 7-bit address shifted left by 1 (HAL format).
 * @param  reg       First register to read.
 * @param  buf       Destination buffer, at least @p len bytes.
 * @param  len       Number of bytes to read (> 0).
 * @retval BMI088_OK, BMI088_ERR_PARAM or BMI088_ERR_I2C.
 */
static bmi088_status_t bmi088_read_regs(I2C_HandleTypeDef *hi2c,
                                        uint16_t dev_addr,
                                        uint8_t reg,
                                        uint8_t *buf,
                                        uint16_t len)
{
    bmi088_status_t status = BMI088_OK;

    if ((hi2c == NULL) || (buf == NULL) || (len == 0U))
    {
        status = BMI088_ERR_PARAM;
    }
    else if (HAL_I2C_Mem_Read(hi2c, dev_addr, (uint16_t)reg,
                              I2C_MEMADD_SIZE_8BIT, buf, len,
                              BMI088_I2C_TIMEOUT_MS) != HAL_OK)
    {
        status = BMI088_ERR_I2C;
    }
    else
    {
        /* Read completed. */
    }

    return status;
}

/**
 * @brief  Write one byte to one register, without read-back.
 * @note   Used only where a read-back right after the write is not valid
 *         (power mode change). Configuration registers use the verified
 *         write, and the soft reset has its own function.
 *
 * @param  hi2c      I2C bus handle.
 * @param  dev_addr  Device address, 7-bit address shifted left by 1 (HAL format).
 * @param  reg       Register to write.
 * @param  value     Byte to write.
 * @retval BMI088_OK, BMI088_ERR_PARAM or BMI088_ERR_I2C.
 */
static bmi088_status_t bmi088_write_reg(I2C_HandleTypeDef *hi2c,
                                        uint16_t dev_addr,
                                        uint8_t reg,
                                        uint8_t value)
{
    bmi088_status_t status = BMI088_OK;
    uint8_t tx = value;

    if (hi2c == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else if (HAL_I2C_Mem_Write(hi2c, dev_addr, (uint16_t)reg,
                               I2C_MEMADD_SIZE_8BIT, &tx, 1U,
                               BMI088_I2C_TIMEOUT_MS) != HAL_OK)
    {
        status = BMI088_ERR_I2C;
    }
    else
    {
        /* Write completed. */
    }

    return status;
}

/**
 * @brief  Write one register, read it back and compare the bits selected
 *         by @p verify_mask.
 * @note   Used for every configuration register. Bits outside the mask are
 *         not compared (e.g. GYRO_BANDWIDTH bit 7 always reads back as 1).
 *
 * @param  hi2c         I2C bus handle.
 * @param  dev_addr     Device address, 7-bit address shifted left by 1 (HAL format).
 * @param  reg          Register to write.
 * @param  value        Byte to write.
 * @param  verify_mask  Bits that must read back equal to @p value.
 * @retval BMI088_OK, BMI088_ERR_PARAM, BMI088_ERR_I2C or BMI088_ERR_VERIFY.
 */
static bmi088_status_t bmi088_write_reg_verify(I2C_HandleTypeDef *hi2c,
                                               uint16_t dev_addr,
                                               uint8_t reg,
                                               uint8_t value,
                                               uint8_t verify_mask)
{
    bmi088_status_t status;
    uint8_t readback = 0U;

    status = bmi088_write_reg(hi2c, dev_addr, reg, value);

    if (status == BMI088_OK)
    {
        status = bmi088_read_regs(hi2c, dev_addr, reg, &readback, 1U);
    }

    if (status == BMI088_OK)
    {
        if ((readback & verify_mask) != (value & verify_mask))
        {
            status = BMI088_ERR_VERIFY;
        }
    }

    return status;
}

/**
 * @brief  Combine the LSB and MSB bytes of one axis into a signed 16-bit value.
 * @note   The BMI088 stores each rate as a 16-bit two's-complement number
 *         split over two registers: value = MSB * 256 + LSB (datasheet 5.5.2).
 *         Unsigned arithmetic plus an explicit sign step is used so the result
 *         does not depend on the implementation-defined conversion of an
 *         out-of-range unsigned value to a signed type.
 * @note   Every (lsb, msb) pair is a valid input, so there is nothing to check.
 *
 * @param  lsb  Low byte (even register: 0x02, 0x04, 0x06).
 * @param  msb  High byte (odd register: 0x03, 0x05, 0x07).
 * @retval Signed value, -32768 to +32767.
 */
static int16_t bmi088_to_int16(uint8_t lsb, uint8_t msb)
{
    uint32_t unsigned_value = ((uint32_t)msb << 8U) | (uint32_t)lsb;
    int32_t signed_value = (int32_t)unsigned_value;

    if (unsigned_value >= 0x8000U)
    {
        signed_value = signed_value - 65536;
    }

    return (int16_t)signed_value;
}

/**
 * @brief  Convert one axis from sensor counts to rad/s.
 * @note   deg/s = counts * range_dps / 32768 (datasheet GYRO_RANGE table),
 *         rad/s = deg/s * pi / 180. Worst-case float32 rounding over all
 *         65536 inputs at +/-125 deg/s: 1.4e-7 rad/s (checked on the PC).
 *
 * @param  counts     Signed sample, -32768..32767.
 * @param  range_dps  Full-scale range in use, deg/s.
 * @retval Angular rate in rad/s.
 */
static bmi088_float32_t bmi088_counts_to_rad_s(int16_t counts,
                                               bmi088_float32_t range_dps)
{
    return (((bmi088_float32_t)counts * range_dps) /
            BMI088_GYRO_FULL_SCALE_COUNTS) * BMI088_DEG_TO_RAD;
}

/**
 * @brief  Update the consecutive-fault counter with the result of one call
 *         and mark the device as failed when the limit is reached.
 * @note   Counted:     BMI088_ERR_I2C, BMI088_ERR_STUCK, BMI088_ERR_BUSY
 *         (a 6-byte read takes about 1 ms, so still busy 10 ms later means
 *         the transfer is stuck).
 *         Not counted: BMI088_ERR_PARAM (programming error, not a sensor
 *         fault; retrying cannot fix it) and BMI088_ERR_VERIFY (only occurs
 *         in init, where a single failure already fails init).
 *         BMI088_OK resets the counter but does not change @p state: leaving
 *         BMI088_STATE_UNINIT or BMI088_STATE_ERROR is done only by init.
 *         The counter saturates at BMI088_MAX_CONSEC_ERRORS, so it never
 *         wraps back to 0 on a long-lasting fault.
 *
 * @param  result       Result of the driver call being accounted for.
 * @param  fault_count  Consecutive-fault counter, updated in place.
 * @param  state        Device state, set to BMI088_STATE_ERROR at the limit.
 * @retval BMI088_OK or BMI088_ERR_PARAM.
 */
static bmi088_status_t bmi088_count_fault(bmi088_status_t result,
                                          uint8_t *fault_count,
                                          bmi088_dev_state_t *state)
{
    bmi088_status_t status = BMI088_OK;

    if ((fault_count == NULL) || (state == NULL))
    {
        status = BMI088_ERR_PARAM;
    }
    else if (result == BMI088_OK)
    {
        *fault_count = 0U;
    }
    else if ((result == BMI088_ERR_I2C) || (result == BMI088_ERR_STUCK) ||
             (result == BMI088_ERR_BUSY))
    {
        if (*fault_count < BMI088_MAX_CONSEC_ERRORS)
        {
            *fault_count = (uint8_t)(*fault_count + 1U);
        }

        if (*fault_count >= BMI088_MAX_CONSEC_ERRORS)
        {
            *state = BMI088_STATE_ERROR;
        }
    }
    else
    {
        /* BMI088_ERR_PARAM, BMI088_ERR_VERIFY, BMI088_ERR_CHIP_ID,
         * BMI088_ERR_STATE: not counted. */
    }

    return status;
}

/**
 * @brief  Set all measurement and health fields of @p gyro to a known
 *         starting value: samples and rates to zero, fault counter to zero,
 *         state to BMI088_STATE_UNINIT.
 * @note   @p gyro->hi2c, @p gyro->dev_addr and @p gyro->range_dps are not
 *         changed; they are configuration, set by init.
 *
 * @param  gyro  Gyro to clear.
 * @retval BMI088_OK or BMI088_ERR_PARAM.
 */
static bmi088_status_t bmi088_gyro_clear_state(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;
    uint8_t axis;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else
    {
        for (axis = 0U; axis < BMI088_AXES; axis++)
        {
            gyro->sample[axis] = 0;
            gyro->angular_velocity[axis] = 0.0F;
        }

        gyro->busy = false;
        gyro->fault_count = 0U;
        gyro->state = BMI088_STATE_UNINIT;
    }

    return status;
}

/**
 * @brief  Send the soft-reset command (0xB6 to GYRO_SOFTRESET).
 * @note   Observed on the bench (logic analyser): during the ACK clock of the
 *         0xB6 byte, SDA rises while SCL is high. The I2C peripheral sees that
 *         as a misplaced STOP and reports a bus error (RM0456 65.4.17), although
 *         the reset takes effect. So for this one write only, a bus error alone
 *         with the data byte already handed over is accepted. The caller must
 *         prove the reset worked by reading the sensor back afterwards.
 *
 * @param  hi2c      I2C bus handle.
 * @param  dev_addr  Device address, 7-bit address shifted left by 1 (HAL format).
 * @retval BMI088_OK, BMI088_ERR_PARAM or BMI088_ERR_I2C.
 */
static bmi088_status_t bmi088_gyro_soft_reset(I2C_HandleTypeDef *hi2c,
                                              uint16_t dev_addr)
{
    bmi088_status_t status = BMI088_OK;
    uint8_t tx = BMI088_GYRO_SOFTRESET_CMD;

    if (hi2c == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else if (HAL_I2C_Mem_Write(hi2c, dev_addr,
                               (uint16_t)BMI088_GYRO_SOFTRESET_REG,
                               I2C_MEMADD_SIZE_8BIT, &tx, 1U,
                               BMI088_I2C_TIMEOUT_MS) != HAL_OK)
    {
        if ((HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_BERR) ||
            (hi2c->XferCount != 0U))
        {
            status = BMI088_ERR_I2C;
        }
    }
    else
    {
        /* Write completed without the bus error. */
    }

    return status;
}

/**
 * @brief  Make sure the gyro is in normal power mode.
 * @note   Reads GYRO_LPM1; if not normal, writes normal, waits for the mode
 *         change (a read-back right after the write could show the old mode)
 *         and reads it again.
 *
 * @param  hi2c      I2C bus handle.
 * @param  dev_addr  Device address, 7-bit address shifted left by 1 (HAL format).
 * @retval BMI088_OK, BMI088_ERR_PARAM, BMI088_ERR_I2C or BMI088_ERR_VERIFY.
 */
static bmi088_status_t bmi088_gyro_ensure_normal_mode(I2C_HandleTypeDef *hi2c,
                                                      uint16_t dev_addr)
{
    bmi088_status_t status;
    uint8_t value = 0U;

    status = bmi088_read_regs(hi2c, dev_addr, BMI088_GYRO_LPM1_REG, &value, 1U);

    if ((status == BMI088_OK) && (value != BMI088_GYRO_LPM1_NORMAL))
    {
        status = bmi088_write_reg(hi2c, dev_addr, BMI088_GYRO_LPM1_REG,
                                  BMI088_GYRO_LPM1_NORMAL);

        if (status == BMI088_OK)
        {
            HAL_Delay(BMI088_GYRO_MODE_DELAY_MS);
            status = bmi088_read_regs(hi2c, dev_addr, BMI088_GYRO_LPM1_REG,
                                      &value, 1U);
        }

        if ((status == BMI088_OK) && (value != BMI088_GYRO_LPM1_NORMAL))
        {
            status = BMI088_ERR_VERIFY;
        }
    }

    return status;
}

/**
 * @brief  Soft-reset the gyro, prove the reset worked and make sure the gyro
 *         is in normal power mode.
 * @note   Sequence (BMI088 datasheet):
 *           1. send 0xB6 to GYRO_SOFTRESET (see bmi088_gyro_soft_reset());
 *           2. wait for the reset to complete (all settings return to reset
 *              values);
 *           3. read GYRO_CHIP_ID, expect 0x0F;
 *           4. read GYRO_BANDWIDTH, expect its reset read-back value 0x80;
 *           5. read GYRO_LPM1; if not normal, write normal, wait for the
 *              mode change and read it again.
 *         Uses HAL_Delay(): must not be called from an interrupt handler.
 *
 * @param  gyro  Gyro to reset; only @p gyro->hi2c and @p gyro->dev_addr
 *               are used.
 * @retval BMI088_OK, BMI088_ERR_PARAM, BMI088_ERR_I2C, BMI088_ERR_CHIP_ID
 *         or BMI088_ERR_VERIFY (bandwidth did not return to its reset value,
 *         or power mode is still not normal).
 */
static bmi088_status_t bmi088_gyro_reset_check(const bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;
    uint8_t value = 0U;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM; 
    }
    else
    {
        status = bmi088_gyro_soft_reset(gyro->hi2c, gyro->dev_addr);
    }

    if (status == BMI088_OK)
    {
        HAL_Delay(BMI088_GYRO_RESET_DELAY_MS);
        status = bmi088_read_regs(gyro->hi2c, gyro->dev_addr,
                                  BMI088_GYRO_CHIP_ID_REG, &value, 1U);
    }

    if ((status == BMI088_OK) && (value != BMI088_GYRO_CHIP_ID_VAL))
    {
        status = BMI088_ERR_CHIP_ID;
    }

    if (status == BMI088_OK)
    {
        status = bmi088_read_regs(gyro->hi2c, gyro->dev_addr,
                                  BMI088_GYRO_BW_REG, &value, 1U);
    }

    if ((status == BMI088_OK) && (value != BMI088_GYRO_BW_RESET_VAL))
    {
        status = BMI088_ERR_VERIFY;
    }

    if (status == BMI088_OK)
    {
        status = bmi088_gyro_ensure_normal_mode(gyro->hi2c, gyro->dev_addr);
    }

    return status;
}

/**
 * @brief  Read one sample of all three axes into @p gyro->sample.
 * @note   One 6-byte burst read from RATE_X_LSB (0x02): X_LSB, X_MSB, Y_LSB,
 *         Y_MSB, Z_LSB, Z_MSB. Reading each LSB before its MSB lets the
 *         sensor's data shadowing keep both bytes from the same sample
 *         (datasheet, rate data registers).
 *         @p gyro->sample is changed only if the read succeeds, so after a
 *         failed read it still holds the last good sample.
 *
 * @param  gyro  Gyro to read; uses hi2c and dev_addr, writes sample[].
 * @retval BMI088_OK, BMI088_ERR_PARAM or BMI088_ERR_I2C.
 */
static bmi088_status_t bmi088_gyro_read(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;
    uint8_t buf[BMI088_GYRO_DATA_LEN] = {0U};
    uint8_t axis;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else
    {
        status = bmi088_read_regs(gyro->hi2c, gyro->dev_addr,
                                  BMI088_GYRO_RATE_DATA_REG, buf,
                                  BMI088_GYRO_DATA_LEN);
    }

    if (status == BMI088_OK)
    {
        for (axis = 0U; axis < BMI088_AXES; axis++)
        {
            gyro->sample[axis] = bmi088_to_int16(buf[2U * axis],
                                                 buf[(2U * axis) + 1U]);
        }
    }

    return status;
}

/* ------------------------------------------------------------------------- */
/* Public functions                                                          */
/* ------------------------------------------------------------------------- */

/**
 * @brief  Initialise the gyro.
 * @note   Sequence:
 *           1. clear the driver state (UNINIT, counters and data zero);
 *           2. store bus, address and the selected range;
 *           3. soft reset, chip ID and power-mode check;
 *           4. verified write of GYRO_BANDWIDTH;
 *           5. verified write of GYRO_RANGE;
 *           6. state = NORMAL if every step passed, otherwise ERROR.
 *         Steps 4-5 come after the reset wait, so the reset cannot
 *         overwrite them. Uses HAL_Delay(): not for interrupt context.
 *
 * @param  gyro      Gyro state, filled in by this function.
 * @param  hi2c      Initialised I2C bus handle (e.g. &hi2c2). The device
 *                   address is fixed: BMI088_GYRO_I2C_ADDR.
 * @retval BMI088_OK, or the first error met: BMI088_ERR_PARAM,
 *         BMI088_ERR_I2C, BMI088_ERR_CHIP_ID or BMI088_ERR_VERIFY.
 */
bmi088_status_t bmi088_gyro_init(bmi088_gyro_t *gyro, I2C_HandleTypeDef *hi2c)
{
    bmi088_status_t status;

    if ((gyro == NULL) || (hi2c == NULL))
    {
        status = BMI088_ERR_PARAM;
    }
    else
    {
        status = bmi088_gyro_clear_state(gyro);
        gyro->hi2c = hi2c;
        gyro->dev_addr = BMI088_GYRO_I2C_ADDR;
        gyro->range_dps = BMI088_GYRO_RANGE_DPS;
    }

    if (status == BMI088_OK)
    {
        status = bmi088_gyro_reset_check(gyro);
    }

    if (status == BMI088_OK)
    {
        status = bmi088_write_reg_verify(gyro->hi2c, gyro->dev_addr,
                                         BMI088_GYRO_BW_REG,
                                         BMI088_GYRO_BW_VAL,
                                         BMI088_GYRO_BW_MASK);
    }

    if (status == BMI088_OK)
    {
        status = bmi088_write_reg_verify(gyro->hi2c, gyro->dev_addr,
                                         BMI088_GYRO_RANGE_REG,
                                         BMI088_GYRO_RANGE_VAL,
                                         BMI088_GYRO_RANGE_MASK);
    }

    if (gyro != NULL)
    {
        if (status == BMI088_OK)
        {
            gyro->state = BMI088_STATE_NORMAL;
        }
        else
        {
            gyro->state = BMI088_STATE_ERROR;
        }
    }

    return status;
}

/**
 * @brief  Read a new gyro sample, check it and convert it to rad/s.
 * @note   Sequence:
 *           1. do nothing unless state is BMI088_STATE_NORMAL;
 *           2. keep the previous sample, read a new one;
 *           3. convert counts to rad/s into angular_velocity[];
 *           4. stuck check: all three axes equal to the previous sample is
 *              counted as a fault, but the data is valid and OK is returned;
 *           5. count the result: BMI088_MAX_CONSEC_ERRORS faults in a row
 *              (I2C or stuck) set state to BMI088_STATE_ERROR.
 *         The return value says whether this update produced new data; the
 *         gyro's health is gyro->state, which the caller must also check.
 *         After a failed read, sample[] and angular_velocity[] keep the last
 *         good values.
 *
 * @param  gyro  Initialised gyro.
 * @retval BMI088_OK (new data read), BMI088_ERR_PARAM, BMI088_ERR_STATE
 *         or BMI088_ERR_I2C.
 */
bmi088_status_t bmi088_gyro_update(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;
    bmi088_status_t fault;
    int16_t previous[BMI088_AXES] = {0};
    uint8_t same_axes = 0U;
    uint8_t axis;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else if (gyro->state != BMI088_STATE_NORMAL)
    {
        status = BMI088_ERR_STATE;
    }
    else
    {
        for (axis = 0U; axis < BMI088_AXES; axis++)
        {
            previous[axis] = gyro->sample[axis];
        }

        status = bmi088_gyro_read(gyro);
    }

    if (status == BMI088_OK)
    {
        for (axis = 0U; axis < BMI088_AXES; axis++)
        {
            if (gyro->sample[axis] == previous[axis])
            {
                same_axes = (uint8_t)(same_axes + 1U);
            }

            gyro->angular_velocity[axis] =
                bmi088_counts_to_rad_s(gyro->sample[axis], gyro->range_dps);
        }
    }

    if (gyro != NULL)
    {
        fault = status;
        if ((status == BMI088_OK) && (same_axes == BMI088_AXES))
        {
            fault = BMI088_ERR_STUCK;
        }

        (void)bmi088_count_fault(fault, &gyro->fault_count, &gyro->state);
    }

    return status;
}

/**
 * @brief  Start a non-blocking, interrupt-driven read of one gyro sample.
 * @note   Called from the 100 Hz timer interrupt. HAL_I2C_Mem_Read_IT() only
 *         starts the transfer and returns; the 6 bytes arrive in
 *         gyro->rx_buf and are processed in the I2C completion callback
 *         (Step 13c). gyro->busy is set before the start, because the
 *         completion interrupt may run before HAL_I2C_Mem_Read_IT() returns.
 *         A start that the HAL refuses counts as an I2C fault, and a read
 *         skipped because the previous one is still running counts as
 *         BMI088_ERR_BUSY, so a transfer that never finishes is detected.
 *
 * @param  gyro  Initialised gyro.
 * @retval BMI088_OK (read started), BMI088_ERR_PARAM, BMI088_ERR_STATE,
 *         BMI088_ERR_BUSY or BMI088_ERR_I2C.
 */
bmi088_status_t bmi088_gyro_start_read(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else if (gyro->state != BMI088_STATE_NORMAL)
    {
        status = BMI088_ERR_STATE;
    }
    else if (gyro->busy)
    {
        status = BMI088_ERR_BUSY;
        (void)bmi088_count_fault(status, &gyro->fault_count, &gyro->state);
    }
    else
    {
        gyro->busy = true;

        if (HAL_I2C_Mem_Read_IT(gyro->hi2c, gyro->dev_addr,
                                (uint16_t)BMI088_GYRO_RATE_DATA_REG,
                                I2C_MEMADD_SIZE_8BIT, gyro->rx_buf,
                                (uint16_t)BMI088_GYRO_DATA_LEN) != HAL_OK)
        {
            gyro->busy = false;
            status = BMI088_ERR_I2C;
            (void)bmi088_count_fault(status, &gyro->fault_count, &gyro->state);
        }
    }

    return status;
}

/**
 * @brief  Process a finished interrupt-driven read (rx_buf -> sample, rate).
 * @note   Called from HAL_I2C_MemRxCpltCallback(). Converts the 6 bytes,
 *         runs the stuck check (all three axes equal to the previous
 *         sample is counted as a fault; the data is still valid and
 *         stored) and updates the fault counter. busy is cleared last, so
 *         a new read cannot start while this sample is being processed.
 *
 * @param  gyro  Gyro whose read has completed.
 * @retval BMI088_OK or BMI088_ERR_PARAM.
 */
bmi088_status_t bmi088_gyro_read_complete(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;
    bmi088_status_t fault = BMI088_OK;
    uint8_t same_axes = 0U;
    uint8_t axis;
    int16_t counts;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else
    {
        for (axis = 0U; axis < BMI088_AXES; axis++)
        {
            counts = bmi088_to_int16(gyro->rx_buf[2U * axis],
                                     gyro->rx_buf[(2U * axis) + 1U]);
            if (counts == gyro->sample[axis])
            {
                same_axes = (uint8_t)(same_axes + 1U);
            }

            gyro->sample[axis] = counts;
            gyro->angular_velocity[axis] =
                bmi088_counts_to_rad_s(counts, gyro->range_dps);
        }

        if (same_axes == BMI088_AXES)
        {
            fault = BMI088_ERR_STUCK;
        }

        (void)bmi088_count_fault(fault, &gyro->fault_count, &gyro->state);
        gyro->busy = false;
    }

    return status;
}

/**
 * @brief  Handle a failed interrupt-driven read.
 * @note   Called from HAL_I2C_ErrorCallback() (NACK, bus error, arbitration
 *         loss, overrun). sample[] and angular_velocity[] keep the last good
 *         values; the failure is counted as BMI088_ERR_I2C.
 *
 * @param  gyro  Gyro whose read has failed.
 * @retval BMI088_OK or BMI088_ERR_PARAM.
 */
bmi088_status_t bmi088_gyro_read_error(bmi088_gyro_t *gyro)
{
    bmi088_status_t status = BMI088_OK;

    if (gyro == NULL)
    {
        status = BMI088_ERR_PARAM;
    }
    else
    {
        (void)bmi088_count_fault(BMI088_ERR_I2C, &gyro->fault_count,
                                 &gyro->state);
        gyro->busy = false;
    }

    return status;
}
