/** Copyright (c) 2018, Cellabox
*
* All rights reserved.

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**@file cb_shtc1.c
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 16-Jun-2017
 * @brief : nRF52840 interface to Sensirion SHTC1 sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#include "nrf_delay.h"
#include "cb_error.h"
#include "cb_i2c.h"
#include "cb_shtc1.h"

//==============================================================================
// Calculates the temperature [°C] as a floating point value from the raw data
// that are read from the sensor.
//------------------------------------------------------------------------------
// input: rawValue temperature raw value (16bit scaled)
//
// return: temperature [°C] as a floating point value
//==============================================================================
float CB_SHTC1_CalcTemperature(uint16_t rawValue){
	// calculate temperature [°C]
	// T = -45 + 175 * rawValue / 2^16
	return 175 * (float)rawValue / 65536 - 45;
}
//==============================================================================
// Calculates the relative humidity [%RH] as a floating point value from the raw
// data that are read from the sensor.
//------------------------------------------------------------------------------
// input: rawValue humidity raw value (16bit scaled)
//
// return: relative humidity [%RH] as a floating point value
//==============================================================================
float CB_SHTC1_CalcHumidity(uint16_t rawValue){
	// calculate relative humidity [%RH]
	// RH = rawValue / 2^16 * 100
	return 100 * (float)rawValue / 65536;
}
//==============================================================================
// Calculates checksum for n bytes of data and compares it with expected
// checksum.
//------------------------------------------------------------------------------
// input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
// checksum expected checksum
//
// return: error: CHECKSUM_ERROR = checksum does not match
// NO_ERROR = checksum matches
//==============================================================================
uint32_t CB_SHTC1_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum){
	uint8_t bit; // bit mask
	uint8_t crc = 0xFF; // calculated checksum
	uint8_t byteCtr; // byte counter

	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
	{
		crc ^= (data[byteCtr]);
		for(bit = 8; bit > 0; --bit)
		{
			if(crc & 0x80) crc = (crc << 1) ^ SHTC1_POLYNOMIAL;
			else crc = (crc << 1);
		}
	}

	// verify checksum
	if(crc != checksum) return CB_SHTC1_CHECKSUM_ERROR;
	else return CB_NO_ERROR;
}
//==============================================================================
// Calls the soft reset mechanism that forces the sensor into a well-defined
// state without removing the power supply.
//------------------------------------------------------------------------------
// return: error: ACK_ERROR = no acknowledgment from sensor
// NO_ERROR = no error
//==============================================================================
uint32_t CB_SHTC1_SoftReset(void){
	uint32_t error = CB_NO_ERROR; // error code
	uint8_t i2cData[2];
	i2cData[0] = (CB_eShtc1SoftReset >> 8);
	i2cData[1] = (0xFF & CB_eShtc1SoftReset);
	error = CB_I2C_Write(CB_eShtc1I2CAddress, 2, i2cData);
	return error;
}
//==============================================================================
uint32_t CB_SHTC1_Init(void){
	uint32_t error = CB_NO_ERROR; // error code
	error = CB_SHTC1_SoftReset();
	nrf_delay_us(230); // give some time for getting ready after soft reset (<230usec)
	return error;
}
//==============================================================================
uint32_t CB_SHTC1_GetId(uint16_t *id){
	uint32_t error; // error code
	uint8_t i2cData[2];
	i2cData[0] = (CB_eShtc1ReadId >> 8);
	i2cData[1] = (0xFF & CB_eShtc1ReadId);
	error = CB_I2C_Write(CB_eShtc1I2CAddress, 2, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eShtc1I2CAddress, 2, i2cData);
	}
	*id = SHTC1_PRODUCT_ID_MASK & (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]));
	return error;
}
//==============================================================================
uint32_t CB_SHTC1_GetTempAndHumi(float *temp, float *humi){
	uint32_t error = CB_NO_ERROR; // error code
	uint16_t rawValueTemp; // temperature raw value from sensor
	uint16_t rawValueHumi; // humidity raw value from sensor
	uint8_t i2cData[6];
	i2cData[0] = (CB_eShtc1MeasureTempHumClockstr >> 8);
	i2cData[1] = (0xFF & CB_eShtc1MeasureTempHumClockstr);
	error = CB_I2C_Write(CB_eShtc1I2CAddress, 2, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eShtc1I2CAddress, 6, i2cData);
	}
	// verify checksum temperature
	if (CB_NO_ERROR == error)
	{
		error = CB_SHTC1_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	// verify checksum humidity
	if (CB_NO_ERROR == error)
	{
		error = CB_SHTC1_CheckCrc(&(i2cData[3]), 2, i2cData[5]);
	}
	if (CB_NO_ERROR == error)
	{
		rawValueTemp = ((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]);
		rawValueHumi = ((uint16_t)(i2cData[3]) << 8) + (uint16_t)(i2cData[4]);
		*temp = CB_SHTC1_CalcTemperature(rawValueTemp);
		*humi = CB_SHTC1_CalcHumidity(rawValueHumi);
	}
	return error;
}
