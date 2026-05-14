#ifndef APP_SENSORS_BMP280_H
#define APP_SENSORS_BMP280_H

#include "app/common/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Error codes -------------------------------------------------------- */
#define BMP280_ERR_OK (0x00U)
#define BMP280_ERR_I2C_BUSY_TIMEOUT (0x10U)
#define BMP280_ERR_START_ADDR_TIMEOUT (0x11U)
#define BMP280_ERR_ADDR_NACK (0x12U)
#define BMP280_ERR_TXE_TIMEOUT (0x13U)
#define BMP280_ERR_BTF_TIMEOUT (0x14U)
#define BMP280_ERR_RXNE_TIMEOUT (0x15U)
#define BMP280_ERR_DETECT_FAIL (0x30U)
#define BMP280_ERR_INIT_CONFIG_FAIL (0x31U)
#define BMP280_ERR_CALIB_READ_FAIL (0x32U)
#define BMP280_ERR_BURST_READ_FAIL (0x40U)
#define BMP280_ERR_NO_DETECTED_ADDR (0x41U)

/* ---- Public API --------------------------------------------------------- */
bool BMP280_Init(void);
bool BMP280_ReadTemperaturePressure(Bmp280Sample *sample);
uint8_t BMP280_GetLastError(void);
uint8_t BMP280_GetLastFailedRegister(void);
uint8_t BMP280_GetLastChipId(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_SENSORS_BMP280_H */
