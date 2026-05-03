#include "app/sensors/mpu6050.h"
#include "stm32f4xx_hal.h"

bool MPU6050_Init(void) {
  /* TODO(WIP-HW): Replace placeholder with MPU-6050 register init over I2C. */
  return true;
}

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
