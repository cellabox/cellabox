/**@file cb_sgpc3.h
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 23-Sept-2017
 * @brief : nRF52840 interface to Sensirion SGPC3 sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#ifndef CB_SGPC3_H__
#define CB_SGPC3_H__

#include "nrf_error.h"

#ifdef __cplusplus
extern "C" {
#endif

//-- Defines -------------------------------------------------------------------
// CRC
#define SGPC3_POLYNOMIAL 0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
// Feature set ID mask (bits 0...7) of SGPC3
#define SGPC3_FEATURE_SET_ID_MASK	0x00FF // SGPC3 feature set ID = xxxx'xxxx'0000'0000 (where x=unspecific information)
// Feature set ID of SGPC3 --> shift to LSB
#define SGPC3_FEATURE_SET_ID_SHIFT 0 // Number of shift operations to shift feature set ID to LSB0.
// Product type ID mask (bits 12...15) of SGPC3
#define SGPC3_PRODUCT_TYPE_ID_MASK	0xF000 // SGPC3 product type ID = 0001'xxxx'xxxx'xxxx (where x=unspecific information)
// Product type ID of SGPC3 --> shift to LSB
#define SGPC3_PRODUCT_TYPE_ID_SHIFT 12 // Number of shift operations to shift product type ID to LSB0.
// Product type ID SGPC3
#define SGPC3_PRODUCT_TYPE_ID		0x1 // SGPC3 product type ID = 1
// Init air quality high power duration
#define SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_0SEC		0 	// 0sec
#define SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_16SEC	16 	// 16sec
#define SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_64SEC	64 	// 64sec
#define SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_184SEC	184 // 184sec
#define SGPC3_INIT_DURATION_INVALID_BASELINE_SEC			SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_64SEC // Set the init duration if baseline in persistent storage is not valid
#define SGPC3_INIT_DURATION_VALID_BASELINE_SEC				SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_0SEC // Set the init duration if baseline in persistent storage is valid
// Measurement timings
#define SGPC3_MEASUREMENT_TIME_MAX_USEC						50000 // Measurement time in [usec]
#define SGPC3_INIT_MEASUREMENT_TIME_MAX_USEC				10000 // Initialization measurement time in [usec]
// Gas concentration calculation constants
#define SGPC3_CREF_PPB										300 // Reference concentration of TVOC of clean air
#define SGPC3_A												512 // Necessary factor for TVOC calculation out of ethanol raw signal
//-- Enumerations --------------------------------------------------------------
// Sensor Commands
typedef enum{
	CB_eSgpc3GetVersionId = 0x202F, // command: response: 3 bytes including CRC, last 5 bits of the product version (bits 12-16 of the LSB) are subject to change. This is used to track new features added to the SGP multi-pixel platform.
	CB_eSgpc3InitAirQuality64sec = 0x2003, // command: initialize air quality sensor with 64sec heating plate on
	CB_eSgpc3InitAirQuality0sec = 0x2089, // command: initialize air quality sensor with no heating plate on
	CB_eSgpc3InitAirQuality16sec = 0x2024, // command: initialize air quality sensor with 16sec heating plate on
	CB_eSgpc3InitAirQuality184sec = 0x206A, // command: initialize air quality sensor with 184sec heating plate on
	CB_eSgpc3MeasureAirQuality = 0x2008, // command: measure air quality, hot plate on for 40...50msec (48mA), call every 2 sec. Response: 3 bytes including CRC
	CB_eSgpc3MeasureRawSignal = 0x204D, // command: measure raw signal (ethanol), intended for part verification and test purposed, hot plate on for 40...50msec (48mA), call every 2 sec. Response: 3 bytes including CRC
	CB_eSgpc3MeasureRawSignalAndAirQuality = 0x2046, // command: measure raw-signal (ethanol) and air quality, hot plate on for 40...50msec (48mA), call every 2 sec. Response: 6 bytes including CRC
	CB_eSgpc3GetBaseline = 0x2015, // command: get baseline. Response: 3 byte including CRC
	CB_eSgpc3SetBaseline = 0x201E, // command: set baseline. Parameters: 3 byte including CRC
	CB_eSgpc3MeasureTest = 0x2032 // command: intended for production line on-chip self test. Hot plate on for 200...220msec. Response: 3 bytes including CRC.
}CB_eSgpc3Commands_t;
// I2C address
typedef enum{
	CB_eSgpc3I2CAddress = 0x58, // sensor I2C address (7-bit)
	CB_eSgpc3I2cAddressAndWriteBit = 0xB0, // sensor I2C address + write bit
	CB_eSgpc3I2cAddressAndReadBit = 0xB1 // sensor I2C address + read bit
}CB_eSgpc3I2cAddress_t;

//==============================================================================
uint32_t CB_SGPC3_GetProductTypeId(uint16_t *id);
//==============================================================================
// Gets the product type ID from the sensor.
//------------------------------------------------------------------------------
// output: *id pointer to a integer, where the id will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_GetFeatureSetId(uint16_t *id);
//==============================================================================
// Gets the feature set ID from the sensor.
//------------------------------------------------------------------------------
// output: *id pointer to a integer, where the id will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_InitAirQualityMeasurement(const uint16_t duration_sec);
//==============================================================================
// Initializes the air quality measurement. For the first 20s after starting
// to send the “Measure_air_quality” command the sensor is in an initialization
// phase during which a “Measure_air_quality” command returns a fixed value of
// 0 ppb.
//------------------------------------------------------------------------------
// input: duration in [sec] of initialization procedure (0, 16, 64, 184),
//        during this time, hotplate is on (draws about 48mA)
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_MeasureAirQuality(uint16_t *tvoc_ppb);
//==============================================================================
// Gets the Total Volatile Organic Compounds (TVOC) measured by sensor.
// Call this every 2sec for optimum results.
//------------------------------------------------------------------------------
// output: *tvoc_ppb pointer to a integer, where the TVOC will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_MeasureRawSignal(uint16_t *ethanol_ppb);
//==============================================================================
// Gets the measured raw signal (ethanol signal, ppb) by sensor.
//------------------------------------------------------------------------------
// output: *ethanol_ppm pointer to a integer, where ethanol signal will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_GetBaseline(uint16_t *baseline_raw);
//==============================================================================
// Gets the baseline correction value from the sensor [ethanol_raw].
//------------------------------------------------------------------------------
// output: *baseline_raw pointer to a integer, where the baseline will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SGPC3_SetBaseline(const uint16_t baseline_raw);
//==============================================================================
// Sets the baseline correction value of the sensor [ethanol_raw].
//------------------------------------------------------------------------------
// input: baseline_raw
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint16_t CB_SGPC3_GetCalculatedTvoc(const uint16_t sref_ppb, const uint16_t sout_ppb, const float correctionFactor);
//==============================================================================
// Calculates the tvoc value out of an ethanol raw signal and reference signal
//------------------------------------------------------------------------------
// input: sref_ppb integer, where raw ethanol reference signal is stored [ppb] for clean air
// input: sout_ppb integer, where raw ethanol output signal is stored [ppb]
// input: correction factor float, factor to correct e.g. measurement with lower sampling rate (10Hz instead of 2Hz)
//
// return: tvoc_ppb pointer to integer, where calculated tvoc signal will be stored
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

#ifdef __cplusplus
}
#endif

#endif // CB_SGPC3_H__
