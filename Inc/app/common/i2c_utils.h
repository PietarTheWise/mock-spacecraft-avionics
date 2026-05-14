#ifndef APP_COMMON_I2C_UTILS_H
#define APP_COMMON_I2C_UTILS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Poll I2C1 SR1 until the given mask bits are all set,
 * or until timeout_ms elapses.
 * Returns true if the bits are set, false on timeout.
 */
bool I2C1_PollStatusRegisterUntilFlagSet(uint32_t mask, uint32_t timeout_ms);

/*
 * Wait for the I2C1 bus to become idle (SR2 BUSY cleared).
 * Returns true if idle within timeout_ms, false otherwise.
 */
bool I2C1_WaitIdle(uint32_t timeout_ms);

/*
 * Probe a 7-bit I2C address by sending a START + address byte and checking
 * for ACK. Returns true if a device ACKs, false otherwise.
 */
bool I2C1_Probe7bitAddress(uint8_t address_7bit);

/*
 * Try each address in the array (with retries). Returns true as soon as one
 * ACKs, false if none respond.
 */
bool I2C1_ProbeKnownAddresses(const uint8_t *addresses, uint32_t count);

#ifdef __cplusplus
}
#endif

#endif /* APP_COMMON_I2C_UTILS_H */
