#include "app/sensors/mpu6050.h"
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR_0 (0x68U)
#define MPU6050_ADDR_1 (0x69U)
#define MPU6050_REG_SMPLRT_DIV (0x19U)
#define MPU6050_REG_CONFIG (0x1AU)
#define MPU6050_REG_GYRO_CONFIG (0x1BU)
#define MPU6050_REG_ACCEL_CONFIG (0x1CU)
#define MPU6050_REG_ACCEL_XOUT_H (0x3BU)
#define MPU6050_REG_PWR_MGMT_1 (0x6BU)
#define MPU6050_REG_WHO_AM_I (0x75U)
#define MPU6050_WHO_AM_I_VAL_0 (0x68U)
#define MPU6050_WHO_AM_I_VAL_1 (0x72U)
#define MPU6050_I2C_TIMEOUT_MS (10U)
#define MPU6050_BURST_FRAME_BYTES (14U)
#define MPU6050_ACCEL_LSB_PER_G (16384.0f)
#define MPU6050_GYRO_LSB_PER_DPS (131.0f)
#define MPU6050_STANDARD_GRAVITY_MPS2 (9.80665f)

static uint8_t s_mpu6050_addr = 0U;
static uint8_t s_mpu6050_last_error = MPU6050_ERR_OK;
static uint8_t s_mpu6050_last_failed_reg = 0x00U;
static uint8_t s_mpu6050_last_who_am_i = 0x00U;

static void MPU6050_SetError(uint8_t error_code, uint8_t reg)
{
  s_mpu6050_last_error = error_code;
  s_mpu6050_last_failed_reg = reg;
}

static bool MPU6050_I2CWaitIdle(void)
{
  uint32_t start_tick = HAL_GetTick();
  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      MPU6050_SetError(MPU6050_ERR_I2C_BUSY_TIMEOUT, 0x00U);
      return false;
    }
  }
  return true;
}

static bool MPU6050_I2CStartAndAddress(uint8_t addr_7bit, bool read_direction)
{
  uint32_t start_tick = HAL_GetTick();
  I2C1->CR1 |= I2C_CR1_START;

  while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_START_ADDR_TIMEOUT, 0x00U);
      return false;
    }
  }

  I2C1->DR = (uint8_t)((addr_7bit << 1) | (read_direction ? 1U : 0U));
  start_tick = HAL_GetTick();
  // if ADDR is set it is ACKed, if AF is set it is NACKed
  while ((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_START_ADDR_TIMEOUT, 0x00U);
      return false;
    }
  }

  // if AF is set it is NACKed and we return false
  if ((I2C1->SR1 & I2C_SR1_AF) != 0U) {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    MPU6050_SetError(MPU6050_ERR_ADDR_NACK, 0x00U);
    return false;
  }

  return true;
}

static bool MPU6050_ReadReg8(uint8_t addr_7bit, uint8_t reg, uint8_t *value)
{
  uint32_t start_tick = 0U;

  if (value == NULL) {
    return false;
  }

  if (!MPU6050_I2CWaitIdle()) {
    return false;
  }

  if (!MPU6050_I2CStartAndAddress(addr_7bit, false)) {
    return false;
  }
  // clear ADDR
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_TXE_TIMEOUT, reg);
      return false;
    }
  }
  I2C1->DR = reg;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_BTF) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_BTF_TIMEOUT, reg);
      return false;
    }
  }

  if (!MPU6050_I2CStartAndAddress(addr_7bit, true)) {
    return false;
  }

  I2C1->CR1 &= ~I2C_CR1_ACK;
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_RXNE) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_ACK;
      MPU6050_SetError(MPU6050_ERR_RXNE_TIMEOUT, reg);
      return false;
    }
  }

  *value = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_ACK;
  return true;
}

static bool MPU6050_WriteReg8(uint8_t addr_7bit, uint8_t reg, uint8_t value)
{
  uint32_t start_tick = 0U;

  if (!MPU6050_I2CWaitIdle()) {
    return false;
  }

  if (!MPU6050_I2CStartAndAddress(addr_7bit, false)) {
    return false;
  }

  // clear ADDR
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_TXE_TIMEOUT, reg);
      return false;
    }
  }
  I2C1->DR = reg;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_TXE) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_TXE_TIMEOUT, reg);
      return false;
    }
  }
  I2C1->DR = value;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_BTF) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      MPU6050_SetError(MPU6050_ERR_BTF_TIMEOUT, reg);
      return false;
    }
  }

  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

static bool MPU6050_ReadBurst(
  uint8_t addr_7bit,
  uint8_t start_reg,
  uint8_t *buffer,
  uint32_t length
)
{
  uint32_t index = 0U;

  if ((buffer == NULL) || (length == 0U)) {
    return false;
  }

  /*
   * Bring-up-safe path: read one register at a time using the validated
   * single-byte routine. This avoids edge cases in multi-byte state handling
   * while hardware/wiring is still being validated.
   */
  for (index = 0U; index < length; index++) {
    if (!MPU6050_ReadReg8(
          addr_7bit, (uint8_t)(start_reg + index), &buffer[index]
        ))
    {
      MPU6050_SetError(
        MPU6050_ERR_BURST_READ_FAIL, (uint8_t)(start_reg + index)
      );
      return false;
    }
  }

  return true;
}

static bool MPU6050_DetectAddressAndIdentity(void)
{
  uint8_t who_am_i = 0U;

  if (MPU6050_ReadReg8(MPU6050_ADDR_0, MPU6050_REG_WHO_AM_I, &who_am_i)) {
    s_mpu6050_last_who_am_i = who_am_i;
    if (who_am_i == MPU6050_WHO_AM_I_VAL_0 || who_am_i == MPU6050_WHO_AM_I_VAL_1) {
      s_mpu6050_addr = MPU6050_ADDR_0;
      return true;
    }
  }

  if (MPU6050_ReadReg8(MPU6050_ADDR_1, MPU6050_REG_WHO_AM_I, &who_am_i)) {
    s_mpu6050_last_who_am_i = who_am_i;
    if (who_am_i == MPU6050_WHO_AM_I_VAL_0 || who_am_i == MPU6050_WHO_AM_I_VAL_1) {
      s_mpu6050_addr = MPU6050_ADDR_1;
      return true;
    }
  }

  s_mpu6050_addr = 0U;
  MPU6050_SetError(MPU6050_ERR_DETECT_FAIL, MPU6050_REG_WHO_AM_I);
  return false;
}

bool MPU6050_Init(void)
{
  if (!MPU6050_DetectAddressAndIdentity()) {
    return false;
  }

  if (!MPU6050_WriteReg8(s_mpu6050_addr, MPU6050_REG_PWR_MGMT_1, 0x01U)) {
    MPU6050_SetError(MPU6050_ERR_INIT_CONFIG_FAIL, MPU6050_REG_PWR_MGMT_1);
    return false;
  }
  if (!MPU6050_WriteReg8(s_mpu6050_addr, MPU6050_REG_GYRO_CONFIG, 0x00U)) {
    MPU6050_SetError(MPU6050_ERR_INIT_CONFIG_FAIL, MPU6050_REG_GYRO_CONFIG);
    return false;
  }
  if (!MPU6050_WriteReg8(s_mpu6050_addr, MPU6050_REG_ACCEL_CONFIG, 0x00U)) {
    MPU6050_SetError(MPU6050_ERR_INIT_CONFIG_FAIL, MPU6050_REG_ACCEL_CONFIG);
    return false;
  }
  if (!MPU6050_WriteReg8(s_mpu6050_addr, MPU6050_REG_CONFIG, 0x03U)) {
    MPU6050_SetError(MPU6050_ERR_INIT_CONFIG_FAIL, MPU6050_REG_CONFIG);
    return false;
  }
  if (!MPU6050_WriteReg8(s_mpu6050_addr, MPU6050_REG_SMPLRT_DIV, 0x09U)) {
    MPU6050_SetError(MPU6050_ERR_INIT_CONFIG_FAIL, MPU6050_REG_SMPLRT_DIV);
    return false;
  }

  MPU6050_SetError(MPU6050_ERR_OK, 0x00U);
  return true;
}

bool MPU6050_ReadGyroAccel(ImuSample *sample)
{
  uint8_t raw[MPU6050_BURST_FRAME_BYTES] = {0};
  int16_t accel_x_raw = 0;
  int16_t accel_y_raw = 0;
  int16_t accel_z_raw = 0;
  int16_t gyro_x_raw = 0;
  int16_t gyro_y_raw = 0;
  int16_t gyro_z_raw = 0;

  if (sample == NULL) {
    return false;
  }

  if (s_mpu6050_addr == 0U) {
    MPU6050_SetError(MPU6050_ERR_NO_DETECTED_ADDR, 0x00U);
    return false;
  }

  if (!MPU6050_ReadBurst(
        s_mpu6050_addr, MPU6050_REG_ACCEL_XOUT_H, raw, MPU6050_BURST_FRAME_BYTES
      ))
  {
    return false;
  }

  accel_x_raw = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  accel_y_raw = (int16_t)(((uint16_t)raw[2] << 8) | raw[3]);
  accel_z_raw = (int16_t)(((uint16_t)raw[4] << 8) | raw[5]);
  gyro_x_raw = (int16_t)(((uint16_t)raw[8] << 8) | raw[9]);
  gyro_y_raw = (int16_t)(((uint16_t)raw[10] << 8) | raw[11]);
  gyro_z_raw = (int16_t)(((uint16_t)raw[12] << 8) | raw[13]);

  sample->timestamp_ms = HAL_GetTick();
  sample->accel_mps2[0] = (((float)accel_x_raw) / MPU6050_ACCEL_LSB_PER_G) *
                          MPU6050_STANDARD_GRAVITY_MPS2;
  sample->accel_mps2[1] = (((float)accel_y_raw) / MPU6050_ACCEL_LSB_PER_G) *
                          MPU6050_STANDARD_GRAVITY_MPS2;
  sample->accel_mps2[2] = (((float)accel_z_raw) / MPU6050_ACCEL_LSB_PER_G) *
                          MPU6050_STANDARD_GRAVITY_MPS2;
  sample->gyro_dps[0] = ((float)gyro_x_raw) / MPU6050_GYRO_LSB_PER_DPS;
  sample->gyro_dps[1] = ((float)gyro_y_raw) / MPU6050_GYRO_LSB_PER_DPS;
  sample->gyro_dps[2] = ((float)gyro_z_raw) / MPU6050_GYRO_LSB_PER_DPS;
  sample->is_placeholder = false;
  MPU6050_SetError(MPU6050_ERR_OK, 0x00U);
  return true;
}

uint8_t MPU6050_GetLastError(void) { return s_mpu6050_last_error; }

uint8_t MPU6050_GetLastFailedRegister(void)
{
  return s_mpu6050_last_failed_reg;
}

uint8_t MPU6050_GetLastWhoAmI(void) { return s_mpu6050_last_who_am_i; }
