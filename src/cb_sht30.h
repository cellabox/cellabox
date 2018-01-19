/**@file cb_sht30.h
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 14-Oct-2017
 * @brief : nRF52840 interface to Sensirion SHT30 sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#ifndef CB_SHT30_H_
#define CB_SHT30_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//-- Defines -------------------------------------------------------------------
// CRC
#define SHT30_POLYNOMIAL 0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
// ID register mask (bits 0...5 are SHT30-specific product code
#define SHT30_PRODUCT_ID_MASK	0x003F // SHT30-product ID = xxxx'xxxx'xx00'0111 (where x=unspecific information)
//-- Enumerations --------------------------------------------------------------
// Sensor Commands
typedef enum{
	CB_eSht30SoftReset = 0x30A2, // soft reset
	CB_eSht30MeasureTHSingleShotHighRepClockstr = 0x2C06, // meas. read T first, clock stretching disabled, high repeatibility
	CB_eSht30MeasureTHSingleShotMedRepClockstr = 0x2C0D, // meas. read T first, clock stretching disabled, high repeatibility
	CB_eSht30MeasureTHSingleShotLowRepClockstr = 0x2C10, // meas. read T first, clock stretching disabled, high repeatibility
	CB_eSht30MeasureTHSingleShotHighRep = 0x2400, // meas. read T first, clock stretching disabled, high repeatibility
	CB_eSht30MeasureTHSingleShotMedRep = 0x240B, // meas. read T first, clock stretching disabled, high repeatibility
	CB_eSht30MeasureTHSingleShotLowRep = 0x2416, // meas. read T first, clock stretching disabled, high repeatibility
}CB_eSht30Commands_t;
// I2C address
typedef enum{
	CB_eSht30I2CAddress = 0x44, // sensor I2C address (7-bit)
	CB_eSht30I2cAddressAndWriteBit = 0x88, // sensor I2C address + write bit
	CB_eSht30I2cAddressAndReadBit = 0x89 // sensor I2C address + read bit
}CB_eSht30I2cAddress_t;
//==============================================================================
uint32_t CB_SHT30_Init(void);
//==============================================================================
// Initializes the I2C bus for communication with the sensor.
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_SHT30_GetTempAndHumi(float *temp, float *humi);
//==============================================================================
// Gets the temperature [°C] and the humidity [%RH].
//------------------------------------------------------------------------------
// output: *temp pointer to a floating point value, where the calculated
// temperature will be stored
// output: *humi pointer to a floating point value, where the calculated
// humidity will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error
//
// remark: If you use this function, then the sensor blocks the I2C-bus with
// clock stretching during the measurement.

#ifdef __cplusplus
}
#endif

#endif // CB_SHT30_H_
