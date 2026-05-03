#include "app/sensors/bmp280.h"
#include "stm32f4xx_hal.h"

bool BMP280_Init(void) {
  /* TODO(WIP-HW): Replace placeholder with BMP280 register init over I2C. */
  return true;
}

bool BMP280_ReadTemperaturePressure(Bmp280Sample *sample) {
  static uint32_t mock_counter = 0U;
  float phase = 0.0f;

  if (sample == NULL) {
    return false;
  }

  mock_counter++;
  phase = (float)(mock_counter % 100U) / 100.0f;

  sample->timestamp_ms = HAL_GetTick();

  /* TODO(WIP-HW): Replace with real temperature and pressure register reads. */
  sample->temperature_c = 24.0f + (phase * 2.0f);
  sample->pressure_hpa = 1013.25f + ((phase - 0.5f) * 1.5f);
  sample->is_placeholder = true;

  return true;
}
