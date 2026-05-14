#include "app/sensors/bmp280.h"
#include "app/common/i2c_utils.h"
#include "stm32f4xx_hal.h"

/* ---- I2C addresses (SDO pin selects) ------------------------------------ */
#define BMP280_ADDR_0 (0x76U) /* SDO → GND */
#define BMP280_ADDR_1 (0x77U) /* SDO → VCC */

/* ---- Registers ---------------------------------------------------------- */
#define BMP280_REG_CALIB_START (0x88U) /* first byte of 24-byte trim block  */
#define BMP280_REG_CHIP_ID (0xD0U)
#define BMP280_REG_RESET (0xE0U)
#define BMP280_REG_CTRL_MEAS (0xF4U)
#define BMP280_REG_CONFIG (0xF5U)
#define BMP280_REG_PRESS_MSB (0xF7U) /* first byte of 6-byte data block   */

/* ---- Chip IDs ----------------------------------------------------------- */
#define BMP280_CHIP_ID (0x58U) /* genuine BMP280    */
#define BME280_CHIP_ID (0x60U) /* humidity variant, same pinout/registers */

/* ---- Init values -------------------------------------------------------- */
#define BMP280_RESET_VALUE (0xB6U)
/*
 * ctrl_meas (0xF4):
 *   osrs_t[7:5] = 010  → temperature ×2 oversampling
 *   osrs_p[4:2] = 101  → pressure ×16 oversampling
 *   mode[1:0]   = 11   → normal (continuous) mode
 *   0b 010 101 11 = 0x57
 */
#define BMP280_CTRL_MEAS_VALUE (0x57U)
/*
 * config (0xF5):
 *   t_sb[7:5]  = 000  → 0.5 ms standby between measurements
 *   filter[4:2]= 100  → IIR coefficient 16 (smooths out short pressure spikes)
 *   0b 000 100 00 = 0x10
 */
#define BMP280_CONFIG_VALUE (0x10U)

#define BMP280_I2C_TIMEOUT_MS (10U)
#define BMP280_CALIB_BYTES (24U)
#define BMP280_DATA_BYTES (6U)

/* ---- Calibration trim coefficients -------------------------------------- */
typedef struct {
  uint16_t dig_T1;
  int16_t dig_T2;
  int16_t dig_T3;
  uint16_t dig_P1;
  int16_t dig_P2;
  int16_t dig_P3;
  int16_t dig_P4;
  int16_t dig_P5;
  int16_t dig_P6;
  int16_t dig_P7;
  int16_t dig_P8;
  int16_t dig_P9;
} Bmp280Calib;

/* ---- Module state ------------------------------------------------------- */
static uint8_t s_bmp280_addr = 0U;
static uint8_t s_bmp280_last_error = BMP280_ERR_OK;
static uint8_t s_bmp280_last_failed_reg = 0x00U;
static uint8_t s_bmp280_chip_id = 0x00U;
static Bmp280Calib s_calib = {0};
static int32_t s_t_fine = 0; /* shared between temp/pressure compensation */

/* ---- Error helper ------------------------------------------------------- */
static void BMP280_SetError(uint8_t error_code, uint8_t reg)
{
  s_bmp280_last_error = error_code;
  s_bmp280_last_failed_reg = reg;
}

/* =========================================================================
 * Low-level I2C primitives
 * ========================================================================= */

static bool BMP280_I2CStartAndAddress(uint8_t addr_7bit, bool read_direction)
{
  I2C1->CR1 |= I2C_CR1_START;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_SB, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_START_ADDR_TIMEOUT, 0x00U);
    return false;
  }

  I2C1->DR = (uint8_t)((addr_7bit << 1) | (read_direction ? 1U : 0U));
  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_ADDR | I2C_SR1_AF, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_START_ADDR_TIMEOUT, 0x00U);
    return false;
  }

  if ((I2C1->SR1 & I2C_SR1_AF) != 0U) {
    I2C1->SR1 &= ~I2C_SR1_AF;
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_ADDR_NACK, 0x00U);
    return false;
  }

  return true;
}

static bool BMP280_ReadReg8(uint8_t addr_7bit, uint8_t reg, uint8_t *value)
{
  if (value == NULL) {
    return false;
  }
  if (!I2C1_WaitIdle(BMP280_I2C_TIMEOUT_MS)) {
    BMP280_SetError(BMP280_ERR_I2C_BUSY_TIMEOUT, 0x00U);
    return false;
  }
  if (!BMP280_I2CStartAndAddress(addr_7bit, false)) {
    return false;
  }
  /* clear ADDR */
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_TXE, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_TXE_TIMEOUT, reg);
    return false;
  }

  I2C1->DR = reg;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_BTF, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_BTF_TIMEOUT, reg);
    return false;
  }

  if (!BMP280_I2CStartAndAddress(addr_7bit, true)) {
    return false;
  }

  I2C1->CR1 &= ~I2C_CR1_ACK;
  (void)I2C1->SR1;
  (void)I2C1->SR2;
  I2C1->CR1 |= I2C_CR1_STOP;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_RXNE, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_ACK;
    BMP280_SetError(BMP280_ERR_RXNE_TIMEOUT, reg);
    return false;
  }

  *value = (uint8_t)I2C1->DR;
  I2C1->CR1 |= I2C_CR1_ACK;
  return true;
}

static bool BMP280_WriteReg8(uint8_t addr_7bit, uint8_t reg, uint8_t value)
{
  if (!I2C1_WaitIdle(BMP280_I2C_TIMEOUT_MS)) {
    BMP280_SetError(BMP280_ERR_I2C_BUSY_TIMEOUT, 0x00U);
    return false;
  }
  if (!BMP280_I2CStartAndAddress(addr_7bit, false)) {
    return false;
  }
  /* clear ADDR */
  (void)I2C1->SR1;
  (void)I2C1->SR2;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_TXE, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_TXE_TIMEOUT, reg);
    return false;
  }
  I2C1->DR = reg;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_TXE, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_TXE_TIMEOUT, reg);
    return false;
  }
  I2C1->DR = value;

  if (!I2C1_PollStatusRegisterUntilFlagSet(I2C_SR1_BTF, BMP280_I2C_TIMEOUT_MS)) {
    I2C1->CR1 |= I2C_CR1_STOP;
    BMP280_SetError(BMP280_ERR_BTF_TIMEOUT, reg);
    return false;
  }

  I2C1->CR1 |= I2C_CR1_STOP;
  return true;
}

static bool BMP280_ReadBurst(uint8_t addr_7bit, uint8_t start_reg, uint8_t *buffer, uint32_t length)
{
  uint32_t index = 0U;

  if ((buffer == NULL) || (length == 0U)) {
    return false;
  }

  for (index = 0U; index < length; index++) {
    if (!BMP280_ReadReg8(addr_7bit, (uint8_t)(start_reg + index), &buffer[index])) {
      BMP280_SetError(BMP280_ERR_BURST_READ_FAIL, (uint8_t)(start_reg + index));
      return false;
    }
  }

  return true;
}

/* =========================================================================
 * Identity and calibration
 * ========================================================================= */

static bool BMP280_DetectAddressAndIdentity(void)
{
  uint8_t chip_id = 0U;

  if (BMP280_ReadReg8(BMP280_ADDR_0, BMP280_REG_CHIP_ID, &chip_id)) {
    s_bmp280_chip_id = chip_id;
    if (chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID) {
      s_bmp280_addr = BMP280_ADDR_0;
      return true;
    }
  }

  if (BMP280_ReadReg8(BMP280_ADDR_1, BMP280_REG_CHIP_ID, &chip_id)) {
    s_bmp280_chip_id = chip_id;
    if (chip_id == BMP280_CHIP_ID || chip_id == BME280_CHIP_ID) {
      s_bmp280_addr = BMP280_ADDR_1;
      return true;
    }
  }

  s_bmp280_addr = 0U;
  BMP280_SetError(BMP280_ERR_DETECT_FAIL, BMP280_REG_CHIP_ID);
  return false;
}

static bool BMP280_ReadCalibration(void)
{
  uint8_t raw[BMP280_CALIB_BYTES] = {0};

  if (!BMP280_ReadBurst(s_bmp280_addr, BMP280_REG_CALIB_START, raw, BMP280_CALIB_BYTES)) {
    BMP280_SetError(BMP280_ERR_CALIB_READ_FAIL, BMP280_REG_CALIB_START);
    return false;
  }

  /* Trim registers are little-endian (low byte first) per BMP280 datasheet */
  s_calib.dig_T1 = (uint16_t)(((uint16_t)raw[1] << 8) | raw[0]);
  s_calib.dig_T2 = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
  s_calib.dig_T3 = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
  s_calib.dig_P1 = (uint16_t)(((uint16_t)raw[7] << 8) | raw[6]);
  s_calib.dig_P2 = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
  s_calib.dig_P3 = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);
  s_calib.dig_P4 = (int16_t)(((uint16_t)raw[13] << 8) | raw[12]);
  s_calib.dig_P5 = (int16_t)(((uint16_t)raw[15] << 8) | raw[14]);
  s_calib.dig_P6 = (int16_t)(((uint16_t)raw[17] << 8) | raw[16]);
  s_calib.dig_P7 = (int16_t)(((uint16_t)raw[19] << 8) | raw[18]);
  s_calib.dig_P8 = (int16_t)(((uint16_t)raw[21] << 8) | raw[20]);
  s_calib.dig_P9 = (int16_t)(((uint16_t)raw[23] << 8) | raw[22]);

  return true;
}

/* =========================================================================
 * Compensation formulas — direct from BMP280 datasheet section 4.2.3
 * ========================================================================= */

/*
 * Returns temperature in hundredths of a degree C (e.g. 2345 = 23.45 °C).
 * Also updates s_t_fine, which the pressure formula depends on — so this
 * MUST be called before BMP280_CompensatePressure for each sample.
 */
static int32_t BMP280_CompensateTemp(int32_t adc_T)
{
  int32_t var1 = 0;
  int32_t var2 = 0;

  var1 = ((((adc_T >> 3) - ((int32_t)s_calib.dig_T1 << 1))) * ((int32_t)s_calib.dig_T2)) >> 11;
  var2 =
    ((((adc_T >> 4) - ((int32_t)s_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)s_calib.dig_T1))) >> 12
    ) *
      ((int32_t)s_calib.dig_T3) >>
    14;
  s_t_fine = var1 + var2;
  return (s_t_fine * 5 + 128) >> 8;
}

/*
 * Returns pressure in Pa as a Q24.8 fixed-point value.
 * Divide by 256 to get Pa, then divide by 100 to get hPa.
 * Uses int64_t as specified by the datasheet to avoid overflow.
 */
static uint32_t BMP280_CompensatePressure(int32_t adc_P)
{
  int64_t var1 = 0;
  int64_t var2 = 0;
  int64_t p = 0;

  var1 = ((int64_t)s_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)s_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)s_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)s_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)s_calib.dig_P3) >> 8) + ((var1 * (int64_t)s_calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)s_calib.dig_P1) >> 33;

  if (var1 == 0) {
    return 0U; /* prevent division by zero */
  }

  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)s_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)s_calib.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)s_calib.dig_P7) << 4);

  return (uint32_t)p;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

bool BMP280_Init(void)
{
  if (!BMP280_DetectAddressAndIdentity()) {
    return false;
  }

  /* Soft-reset the chip to put it in a known state */
  if (!BMP280_WriteReg8(s_bmp280_addr, BMP280_REG_RESET, BMP280_RESET_VALUE)) {
    BMP280_SetError(BMP280_ERR_INIT_CONFIG_FAIL, BMP280_REG_RESET);
    return false;
  }
  HAL_Delay(10U); /* datasheet: NVM copy takes ~2 ms after reset */

  if (!BMP280_ReadCalibration()) {
    return false;
  }

  if (!BMP280_WriteReg8(s_bmp280_addr, BMP280_REG_CTRL_MEAS, BMP280_CTRL_MEAS_VALUE)) {
    BMP280_SetError(BMP280_ERR_INIT_CONFIG_FAIL, BMP280_REG_CTRL_MEAS);
    return false;
  }

  if (!BMP280_WriteReg8(s_bmp280_addr, BMP280_REG_CONFIG, BMP280_CONFIG_VALUE)) {
    BMP280_SetError(BMP280_ERR_INIT_CONFIG_FAIL, BMP280_REG_CONFIG);
    return false;
  }

  BMP280_SetError(BMP280_ERR_OK, 0x00U);
  return true;
}

bool BMP280_ReadTemperaturePressure(Bmp280Sample *sample)
{
  uint8_t raw[BMP280_DATA_BYTES] = {0};
  int32_t adc_P = 0;
  int32_t adc_T = 0;
  int32_t temp_x100 = 0;
  uint32_t press_q24_8 = 0U;

  if (sample == NULL) {
    return false;
  }

  if (s_bmp280_addr == 0U) {
    BMP280_SetError(BMP280_ERR_NO_DETECTED_ADDR, 0x00U);
    return false;
  }

  /*
   * Burst-read 6 bytes starting at 0xF7:
   *   raw[0..2] = press_msb, press_lsb, press_xlsb
   *   raw[3..5] = temp_msb,  temp_lsb,  temp_xlsb
   * Each raw value is 20-bit, big-endian, with 4-bit zero-padding in xlsb[3:0].
   */
  if (!BMP280_ReadBurst(s_bmp280_addr, BMP280_REG_PRESS_MSB, raw, BMP280_DATA_BYTES)) {
    return false;
  }

  adc_P = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
  adc_T = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));

  /* Temperature must be compensated first — it populates s_t_fine for pressure
   */
  temp_x100 = BMP280_CompensateTemp(adc_T);
  press_q24_8 = BMP280_CompensatePressure(adc_P);

  sample->timestamp_ms = HAL_GetTick();
  sample->temperature_c = (float)temp_x100 / 100.0f;
  /* press_q24_8 is Pa in Q24.8 → divide by 256 for Pa → divide by 100 for hPa
   */
  sample->pressure_hpa = ((float)press_q24_8 / 256.0f) / 100.0f;
  sample->is_placeholder = false;

  BMP280_SetError(BMP280_ERR_OK, 0x00U);
  return true;
}

uint8_t BMP280_GetLastError(void) { return s_bmp280_last_error; }
uint8_t BMP280_GetLastFailedRegister(void) { return s_bmp280_last_failed_reg; }
uint8_t BMP280_GetLastChipId(void) { return s_bmp280_chip_id; }
