#include "app/telemetry/telemetry.h"

#include "stm32f4xx_hal.h" // IWYU pragma: keep
#include <stddef.h>
#include <stdint.h>

extern int snprintf(char *str, unsigned int size, const char *format, ...);

static UART_HandleTypeDef *s_huart = NULL;

void Telemetry_Init(UART_HandleTypeDef *huart) { s_huart = huart; }

void Telemetry_SendString(const char *str)
{
  if (s_huart == NULL || str == NULL) {
    return;
  }
  uint16_t len = 0U;
  while (str[len] != '\0') {
    len++;
  }
  if (len > 0U) {
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)str, len, 50U);
  }
}

void Telemetry_SendImuSample(const ImuSample *sample)
{
  if (s_huart == NULL || sample == NULL) {
    return;
  }

  /* %f is disabled in newlib nano — convert to integer centi-units */
  int ax = (int)(sample->accel_mps2[0] * 100.0f);
  int ay = (int)(sample->accel_mps2[1] * 100.0f);
  int az = (int)(sample->accel_mps2[2] * 100.0f);
  int gx = (int)(sample->gyro_dps[0] * 100.0f);
  int gy = (int)(sample->gyro_dps[1] * 100.0f);
  int gz = (int)(sample->gyro_dps[2] * 100.0f);

  char msg[120];
  int len = snprintf(
    msg,
    sizeof(msg),
    "t=%lu ax=%d.%02d ay=%d.%02d az=%d.%02d gx=%d.%02d gy=%d.%02d gz=%d.%02d\r\n",
    (unsigned long)sample->timestamp_ms,
    ax / 100,
    (ax < 0 ? -ax : ax) % 100,
    ay / 100,
    (ay < 0 ? -ay : ay) % 100,
    az / 100,
    (az < 0 ? -az : az) % 100,
    gx / 100,
    (gx < 0 ? -gx : gx) % 100,
    gy / 100,
    (gy < 0 ? -gy : gy) % 100,
    gz / 100,
    (gz < 0 ? -gz : gz) % 100
  );
  if (len > 0 && len <= (int)sizeof(msg)) {
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)msg, (uint16_t)len, 50U);
  }
}

void Telemetry_SendBmp280Sample(const Bmp280Sample *sample)
{
  if (s_huart == NULL || sample == NULL) {
    return;
  }

  int temp_c = (int)(sample->temperature_c * 100.0f);
  int press_hpa = (int)(sample->pressure_hpa * 100.0f);

  char msg[80];
  int len = snprintf(
    msg,
    sizeof(msg),
    "BMP t=%lu temp=%d.%02d hpa=%d.%02d ph=%u\r\n",
    (unsigned long)sample->timestamp_ms,
    temp_c / 100,
    (temp_c < 0 ? -temp_c : temp_c) % 100,
    press_hpa / 100,
    (press_hpa < 0 ? -press_hpa : press_hpa) % 100,
    sample->is_placeholder ? 1U : 0U
  );
  if (len > 0 && len <= (int)sizeof(msg)) {
    (void)HAL_UART_Transmit(s_huart, (uint8_t *)msg, (uint16_t)len, 50U);
  }
}
