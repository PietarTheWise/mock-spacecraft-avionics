#ifndef APP_TELEMETRY_TELEMETRY_H
#define APP_TELEMETRY_TELEMETRY_H

#include "app/common/app_types.h"

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bind the UART handle used for all telemetry output.
 *        Must be called once before any other Telemetry_* function.
 */
void Telemetry_Init(UART_HandleTypeDef *huart);

/** @brief Transmit a null-terminated string over UART. */
void Telemetry_SendString(const char *str);

/** @brief Format and transmit an IMU sample line. */
void Telemetry_SendImuSample(const ImuSample *sample);

/** @brief Format and transmit a BMP280 temperature/pressure sample line. */
void Telemetry_SendBmp280Sample(const Bmp280Sample *sample);

#ifdef __cplusplus
}
#endif

#endif /* APP_TELEMETRY_TELEMETRY_H */
