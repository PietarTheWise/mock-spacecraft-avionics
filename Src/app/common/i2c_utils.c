#include "app/common/i2c_utils.h"
#include "stm32f4xx_hal.h"

#define I2C1_PROBE_TIMEOUT_MS (10U)
#define I2C1_PROBE_RETRIES (5U)

bool I2C1_PollStatusRegisterUntilFlagSet(uint32_t mask, uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  while ((I2C1->SR1 & mask) == 0U) {
    if ((HAL_GetTick() - start_tick) >= timeout_ms) {
      return false;
    }
  }
  return true;
}

bool I2C1_WaitIdle(uint32_t timeout_ms)
{
  uint32_t start_tick = HAL_GetTick();
  while ((I2C1->SR2 & I2C_SR2_BUSY) != 0U) {
    if ((HAL_GetTick() - start_tick) >= timeout_ms) {
      return false;
    }
  }
  return true;
}

bool I2C1_Probe7bitAddress(uint8_t address_7bit)
{
  if (!I2C1_WaitIdle(I2C1_PROBE_TIMEOUT_MS)) {
    return false;
  }

  I2C1->CR1 |= I2C_CR1_START;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_SB, I2C1_PROBE_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  I2C1->DR = (uint8_t)(address_7bit << 1);
  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_ADDR | I2C_SR1_AF, I2C1_PROBE_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  if ((I2C1->SR1 & I2C_SR1_AF) != 0U) {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    return false;
  }

  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

bool I2C1_ProbeKnownAddresses(const uint8_t *addresses, uint32_t count)
{
  uint32_t idx = 0U;
  uint32_t retry = 0U;

  if ((addresses == NULL) || (count == 0U)) {
    return false;
  }

  for (idx = 0U; idx < count; idx++) {
    for (retry = 0U; retry < I2C1_PROBE_RETRIES; retry++) {
      if (I2C1_Probe7bitAddress(addresses[idx])) {
        return true;
      }
      HAL_Delay(2U);
    }
  }

  return false;
}
