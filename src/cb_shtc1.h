/**@file cb_shtc1.h
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 16-Jun-2017
 * @brief : nRF52840 interface to Sensirion SHTC1 sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#ifndef CB_SHTC1_H_
#define CB_SHTC1_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//-- Defines -------------------------------------------------------------------
// CRC
#define SHTC1_POLYNOMIAL 0x131 // P(x) = x^8 + x^5 + x^4 + 1 = 100110001
// ID register mask (bits 0...5 are SHTC1-specific product code
#define SHTC1_PRODUCT_ID_MASK	0x003F // SHTC1-product ID = xxxx'xxxx'xx00'0111 (where x=unspecific information)
//-- Enumerations --------------------------------------------------------------
// Sensor Commands
typedef enum{
	CB_eShtc1ReadId = 0xEFC8, // command: read ID register
	CB_eShtc1SoftReset = 0x805D, // soft reset
	CB_eShtc1MeasureTempHumPolling = 0x7866, // meas. read T first, clock stretching disabled
	CB_eShtc1MeasureTempHumClockstr = 0x7CA2, // meas. read T first, clock stretching enabled
	CB_eShtc1MeasureHumTempPolling = 0x58E0, // meas. read RH first, clock stretching disabled
	CB_eShtc1MeasureHumTempClockstr = 0x5C24 // meas. read RH first, clock stretching enabled
}CB_eShtc1Commands_t;
// I2C address
typedef enum{
	CB_eShtc1I2CAddress = 0x70, // sensor I2C address (7-bit)
	CB_eShtc1I2cAddressAndWriteBit = 0xE0, // sensor I2C address + write bit
	CB_eShtc1I2cAddressAndReadBit = 0xE1 // sensor I2C address + read bit
}CB_eShtc1I2cAddress_t;
//==============================================================================
uint32_t CB_SHTC1_Init(void);
//==============================================================================
// Initializes the I2C bus for communication with the sensor.
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_SHTC1_GetId(uint16_t *id);
//==============================================================================
// Gets the ID from the sensor.
//------------------------------------------------------------------------------
// output: *id pointer to a integer, where the id will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error
//==============================================================================
uint32_t CB_SHTC1_GetTempAndHumi(float *temp, float *humi);
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
//==============================================================================
uint32_t CB_SHTC1_GetTempAndHumiPolling(float *temp, float *humi);
//==============================================================================
// Gets the temperature [°C] and the humidity [%RH]. This function polls every
// 1ms until measurement is ready.
//------------------------------------------------------------------------------
// output: *temp pointer to a floating point value, where the calculated
// temperature will be stored
// output: *humi pointer to a floating point value, where the calculated
// humidity will be stored
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_SHTC1_StartWriteAccess(void);
//==============================================================================
// Writes a start condition and the sensor I2C address with the write flag.
//------------------------------------------------------------------------------
// return: error: ACK_ERROR = no acknowledgment from sensor
// NO_ERROR = no error
//==============================================================================
uint32_t CB_SHTC1_StartReadAccess(void);
//==============================================================================
// Writes a start condition and the sensor I2C address with the read flag.
//------------------------------------------------------------------------------
// return: error: ACK_ERROR = no acknowledgment from sensor
// NO_ERROR = no error
//==============================================================================
void CB_SHTC1_StopAccess(void);
//==============================================================================
// Writes a stop condition.
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_SHTC1_Read2BytesAndCrc(uint16_t *data);
//==============================================================================
// Reads two bytes plus the checksum and verifies this. If the checksum
// verification is successful, then the two bytes stored in a 16-bit integer.
//------------------------------------------------------------------------------
// input: *data pointer to a 16-bit int, where the data will be stored
//
// return: error: CHECKSUM_ERROR = checksum does not match
// NO_ERROR = checksum matches
//==============================================================================
uint32_t CB_SHTC1_WriteCommand(CB_eShtc1Commands_t cmd);
//==============================================================================
// Writes command to the sensor.
//------------------------------------------------------------------------------
// input: cmd command which is to be written to the sensor
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// NO_ERROR = no error


#ifdef __cplusplus
}
#endif

#endif // CB_SHTC1_H_
