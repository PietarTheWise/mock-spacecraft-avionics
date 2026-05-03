#ifndef APP_COMMON_APP_TYPES_H
#define APP_COMMON_APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  uint32_t timestamp_ms;
  float accel_mps2[3];
  float gyro_dps[3];
  bool is_placeholder;
} ImuSample;

typedef struct {
  uint32_t timestamp_ms;
  float temperature_c;
  float pressure_hpa;
  bool is_placeholder;
} Bmp280Sample;

typedef struct {
  uint32_t timestamp_ms;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float gyro_bias_dps[3];
  float temperature_c;
  bool is_placeholder;
} FusionState;

typedef struct {
  uint32_t sequence;
  uint32_t timestamp_ms;
  ImuSample imu;
  Bmp280Sample bmp280;
  FusionState fusion;
  uint32_t status_flags;
} TelemetryFrame;

#define APP_STATUS_FLAG_PLACEHOLDER_DATA (1UL << 0)
#define APP_STATUS_FLAG_MPU6050_WIP (1UL << 1)
#define APP_STATUS_FLAG_BMP280_WIP (1UL << 2)
#define APP_STATUS_FLAG_FUSION_WIP (1UL << 3)
#define APP_STATUS_FLAG_TELEMETRY_WIP (1UL << 4)

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_APP_TYPES_H */
