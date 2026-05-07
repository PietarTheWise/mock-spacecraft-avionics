#include "app/sensors/mpu6050.h"
#include "stm32f4xx_hal.h"

#define MPU6050_ADDR_0 (0x68U)
#define MPU6050_ADDR_1 (0x69U)
#define MPU6050_REG_WHO_AM_I (0x75U)
#define MPU6050_WHO_AM_I_MASK (0x7EU)
#define MPU6050_WHO_AM_I_EXPECTED (0x68U)
#define MPU6050_I2C_TIMEOUT_MS (10U)

static uint8_t s_mpu6050_addr = 0U;

static bool MPU6050_I2CWaitIdle(void) {
  uint32_t start_tick = HAL_GetTick();
  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      return false;
    }
  }
  return true;
}

static bool MPU6050_I2CStartAndAddress(uint8_t addr_7bit, bool read_direction) {
  uint32_t start_tick = HAL_GetTick();
  I2C1->CR1 |= I2C_CR1_START;

  while ((I2C1->SR1 & I2C_SR1_SB) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
  }

  I2C1->DR = (uint8_t)((addr_7bit << 1) | (read_direction ? 1U : 0U));
  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
      return false;
    }
  }

  if ((I2C1->SR1 & I2C_SR1_AF) != 0U) {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  return true;
}

static bool MPU6050_ReadReg8(uint8_t addr_7bit, uint8_t reg, uint8_t *value) {
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
      return false;
    }
  }
  I2C1->DR = reg;

  start_tick = HAL_GetTick();
  while ((I2C1->SR1 & I2C_SR1_BTF) == 0U) {
    if ((HAL_GetTick() - start_tick) >= MPU6050_I2C_TIMEOUT_MS) {
      I2C1->CR1 |= I2C_CR1_STOP;
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
      return false;
    }
  }

  *value = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_ACK;
  return true;
}

static bool MPU6050_DetectAddressAndIdentity(void) {
  uint8_t who_am_i = 0U;

  if (MPU6050_ReadReg8(MPU6050_ADDR_0, MPU6050_REG_WHO_AM_I, &who_am_i) &&
      ((who_am_i & MPU6050_WHO_AM_I_MASK) == MPU6050_WHO_AM_I_EXPECTED)) {
    s_mpu6050_addr = MPU6050_ADDR_0;
    return true;
  }

  if (MPU6050_ReadReg8(MPU6050_ADDR_1, MPU6050_REG_WHO_AM_I, &who_am_i) &&
      ((who_am_i & MPU6050_WHO_AM_I_MASK) == MPU6050_WHO_AM_I_EXPECTED)) {
    s_mpu6050_addr = MPU6050_ADDR_1;
    return true;
  }

  s_mpu6050_addr = 0U;
  return false;
}

bool MPU6050_Init(void) { return MPU6050_DetectAddressAndIdentity(); }

bool MPU6050_ReadGyroAccel(ImuSample *sample) {
  static uint32_t mock_counter = 0U;
  float phase = 0.0f;

  if (sample == NULL) {
    return false;
  }

  mock_counter++;
  phase = (float)(mock_counter % 200U) / 200.0f;

  sample->timestamp_ms = HAL_GetTick();

  /* TODO(WIP-HW): Replace with real accel register reads from MPU-6050. */
  sample->accel_mps2[0] = 0.10f * phase;
  sample->accel_mps2[1] = -0.10f * phase;
  sample->accel_mps2[2] = 9.81f;

  /* TODO(WIP-HW): Replace with real gyro register reads from MPU-6050. */
  sample->gyro_dps[0] = 0.50f * phase;
  sample->gyro_dps[1] = -0.30f * phase;
  sample->gyro_dps[2] = 0.15f * phase;

  sample->is_placeholder = true;
  return true;
}
