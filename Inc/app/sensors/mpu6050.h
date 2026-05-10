#ifndef APP_SENSORS_MPU6050_H
#define APP_SENSORS_MPU6050_H

#include "app/common/app_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MPU6050_ERR_OK (0x00U)
#define MPU6050_ERR_I2C_BUSY_TIMEOUT (0x10U)
#define MPU6050_ERR_START_ADDR_TIMEOUT (0x11U)
#define MPU6050_ERR_ADDR_NACK (0x12U)
#define MPU6050_ERR_TXE_TIMEOUT (0x13U)
#define MPU6050_ERR_BTF_TIMEOUT (0x14U)
#define MPU6050_ERR_RXNE_TIMEOUT (0x15U)
#define MPU6050_ERR_DETECT_FAIL (0x30U)
#define MPU6050_ERR_INIT_CONFIG_FAIL (0x31U)
#define MPU6050_ERR_BURST_READ_FAIL (0x40U)
#define MPU6050_ERR_NO_DETECTED_ADDR (0x41U)

    bool MPU6050_Init( void );
    bool MPU6050_ReadGyroAccel( ImuSample* sample );
    uint8_t MPU6050_GetLastError(void);
    uint8_t MPU6050_GetLastFailedRegister(void);
    uint8_t MPU6050_GetLastWhoAmI(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_MPU6050_H */
