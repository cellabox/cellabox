/**
 * Copyright (c) 2017, Cellabox
 *
 * All rights reserved.
 *
 *
 */

#ifndef CB_ERROR_H__
#define CB_ERROR_H__

#include "nrf_error.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup cellabox_errors
 * @{
 * @ingroup cellabox
 * @brief CELLABOX_ERRORS_BASE Error Codes Base number definitions. NRF errors are in nrf_error.h or sdk_errors.h!!!
 */

#define CB_ERROR_BASE_NUM (0xC000)	///< Cellabox error base

#define CB_NO_ERROR                			(NRF_SUCCESS)  ///< Successful command
#define CB_I2C_TIMEOUT_ERROR   				(CB_ERROR_BASE_NUM + 1)   ///< I2C bus timeout.
#define CB_I2C_NACK_ERROR					(CB_ERROR_BASE_NUM + 2)   ///< I2C not acknowledge received.
#define CB_I2C_TX_BUFFER_OVERFLOW			(CB_ERROR_BASE_NUM + 3)   ///< I2C trasmit (tx) buffer overflow
#define CB_I2C_SHTC1_NACK_ERROR				(CB_ERROR_BASE_NUM + 10)  ///< I2C not acknowledge received from SHTC1.
#define CB_I2C_SHTC1_TIMEOUT_ERROR  		(CB_ERROR_BASE_NUM + 11)  ///< I2C timeout during communication with SHTC1.
#define CB_SHTC1_CHECKSUM_ERROR				(CB_ERROR_BASE_NUM + 15)  ///< SHTC1 CRC checksum error. Something went wrong with I2C SHTC1 communication.
#define CB_LPS22HB_ERROR					(CB_ERROR_BASE_NUM + 20)  ///< LPS22 absolute pressure sensor error.
#define CB_ILLEGAL_MODULE_CONFIG			(CB_ERROR_BASE_NUM + 30)  ///< Board configuration could not be assigned to a module type (e.g. T&H, IAQ, ...).
#define CB_I2C_SGPC3_NACK_ERROR				(CB_ERROR_BASE_NUM + 40)  ///< I2C not acknowledge received from SGPC3.
#define CB_I2C_SGPC3_TIMEOUT_ERROR			(CB_ERROR_BASE_NUM + 41)  ///< I2C timeout during communication with SGPC3.
#define CB_SGPC3_CHECKSUM_ERROR				(CB_ERROR_BASE_NUM + 45)  ///< SGPC3 CRC checksum error. Something went wrong with I2C SGPC3 communication.
#define CB_SGPC3_UNKNOWN_COMMAND			(CB_ERROR_BASE_NUM + 46)  ///< SGPC3 command is unknown.
#define CB_SGPC3_UNKNOWN_PROFILE			(CB_ERROR_BASE_NUM + 47)  ///< SGPC3 unknown profile to a given profile number.
#define CB_SGPC3_CANNOT_EXTRACT_FEATURESET	(CB_ERROR_BASE_NUM + 48)  ///< SGPC3: cannot extract feature set.
#define CB_SGPC3_INVALID_BASELINE			(CB_ERROR_BASE_NUM + 49)  ///< SGPC3: invalid baseline correction value detected.
#define CB_SGPC3_ABSOLUTE_HUM_TOO_HIGH		(CB_ERROR_BASE_NUM + 50)  ///< SGPC3: absolute humidity for humidity compensation is too high.
#define CB_SGPC3_INVALID_HIGH_POWER_TIME    (CB_ERROR_BASE_NUM + 51)  ///< SGPC3: the choosen duration of high power for InitAirQuality is not supported.
#define CB_SGPC3_FLASH_WRITE_ERROR			(CB_ERROR_BASE_NUM + 60)  ///< Error happended when tried to write to flash.
#define CB_SHT30_CHECKSUM_ERROR				(CB_ERROR_BASE_NUM + 70)  ///< SHT30 CRC checksum error. Something went wrong with I2C SHTC1 communication.
#define CB_LMP91000_I2C_DATA_ERROR			(CB_ERROR_BASE_NUM + 80)  ///< LMP91000: comparison of read and written data to a LMP91000 register failed.
#define CB_ADC_ILLEGAL_PARAMETER			(CB_ERROR_BASE_NUM + 90)  ///< Illegal ADC parameter for initialization.

/** @} */

#ifdef __cplusplus
}
#endif

#endif // CB_ERROR_H__
