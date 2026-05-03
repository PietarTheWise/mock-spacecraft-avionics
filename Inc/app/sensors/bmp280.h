#ifndef APP_SENSORS_BMP280_H
#define APP_SENSORS_BMP280_H

#include "app/common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool BMP280_Init(void);
bool BMP280_ReadTemperaturePressure(Bmp280Sample *sample);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_BMP280_H */
