#ifndef APP_SENSORS_MPU6050_H
#define APP_SENSORS_MPU6050_H

#include "app/common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool MPU6050_Init(void);
bool MPU6050_ReadGyroAccel(ImuSample *sample);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_MPU6050_H */
