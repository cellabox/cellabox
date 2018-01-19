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

/**@file cb_sgpc3.c
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 23-Sept-2017
 * @brief : nRF52840 interface to Sensirion SGPC3 sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#include <math.h> // exp() calculation
#include "nrf_delay.h"
#include "cb_error.h"
#include "cb_i2c.h"
#include "cb_sgpc3.h"

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
uint32_t CB_SGPC3_CheckCrc(uint8_t data[], uint8_t nbrOfBytes, uint8_t checksum){
	uint8_t bit; // bit mask
	uint8_t crc = 0xFF; // calculated checksum
	uint8_t byteCtr; // byte counter

	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
	{
		crc ^= (data[byteCtr]);
		for(bit = 8; bit > 0; --bit)
		{
			if(crc & 0x80) crc = (crc << 1) ^ SGPC3_POLYNOMIAL;
			else crc = (crc << 1);
		}
	}

	// verify checksum
	if(crc != checksum) return CB_SGPC3_CHECKSUM_ERROR;
	else return CB_NO_ERROR;
}

//==============================================================================
// Returns the CRC checksum for n bytes of data.
//------------------------------------------------------------------------------
// input: data[] checksum is built based on this data
// nbrOfBytes checksum is built for n bytes of data
//
// return: CRC checksum
//==============================================================================
uint8_t CB_SGPC3_CalculateCrcChecksum(uint8_t data[], uint8_t nbrOfBytes){
	uint8_t bit; // bit mask
	uint8_t crc = 0xFF; // calculated checksum
	uint8_t byteCtr; // byte counter

	// calculates 8-Bit checksum with given polynomial
	for(byteCtr = 0; byteCtr < nbrOfBytes; byteCtr++)
	{
		crc ^= (data[byteCtr]);
		for(bit = 8; bit > 0; --bit)
		{
			if(crc & 0x80) crc = (crc << 1) ^ SGPC3_POLYNOMIAL;
			else crc = (crc << 1);
		}
	}
	return crc;
}

//==============================================================================
uint32_t CB_SGPC3_GetProductTypeId(uint16_t *id){
	uint32_t error; // error code
	uint8_t i2cData[3];
	i2cData[0] = (CB_eSgpc3GetVersionId >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3GetVersionId);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eSgpc3I2CAddress, 3, i2cData);
	}
	// verify checksum
	if (CB_NO_ERROR == error)
	{
		error = CB_SGPC3_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	*id = ((SGPC3_PRODUCT_TYPE_ID_MASK & (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]))) >> SGPC3_PRODUCT_TYPE_ID_SHIFT);
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_GetFeatureSetId(uint16_t *id){
	uint32_t error; // error code
	uint8_t i2cData[3];
	i2cData[0] = (CB_eSgpc3GetVersionId >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3GetVersionId);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eSgpc3I2CAddress, 3, i2cData);
	}
	// verify checksum
	if (CB_NO_ERROR == error)
	{
		error = CB_SGPC3_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	*id = ((SGPC3_FEATURE_SET_ID_MASK & (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]))) >> SGPC3_FEATURE_SET_ID_SHIFT);
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_InitAirQualityMeasurement(const uint16_t initDuration_sec){
	uint32_t error; // error code
	CB_eSgpc3Commands_t initCommand; // init command for SGPC3
	uint8_t i2cData[3];
	switch (initDuration_sec)
	{
		case SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_0SEC:
		initCommand = CB_eSgpc3InitAirQuality0sec;
		break;
		case SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_16SEC:
		initCommand = CB_eSgpc3InitAirQuality16sec;
		break;
		case SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_64SEC:
		initCommand = CB_eSgpc3InitAirQuality64sec;
		break;
		case SGPC3_INIT_AIR_QUALITY_HIGH_POWER_DURATION_184SEC:
		initCommand = CB_eSgpc3InitAirQuality184sec;
		break;
		default: return CB_SGPC3_INVALID_HIGH_POWER_TIME;
	}
	i2cData[0] = (initCommand >> 8);
	i2cData[1] = (0xFF & initCommand);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	nrf_delay_us(SGPC3_INIT_MEASUREMENT_TIME_MAX_USEC);
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_MeasureAirQuality(uint16_t *tvoc_ppb){
	uint32_t error; // error code
	uint8_t i2cData[3];
	i2cData[0] = (CB_eSgpc3MeasureAirQuality >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3MeasureAirQuality);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	nrf_delay_us(SGPC3_MEASUREMENT_TIME_MAX_USEC);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eSgpc3I2CAddress, 3, i2cData);
	}
	// verify checksum
	if (CB_NO_ERROR == error)
	{
		error = CB_SGPC3_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	*tvoc_ppb = (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]));
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_MeasureRawSignal(uint16_t *ethanol_ppb){
	uint32_t error; // error code
	uint8_t i2cData[3];
	i2cData[0] = (CB_eSgpc3MeasureRawSignal >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3MeasureRawSignal);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	nrf_delay_us(SGPC3_MEASUREMENT_TIME_MAX_USEC);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eSgpc3I2CAddress, 3, i2cData);
	}
	// verify checksum
	if (CB_NO_ERROR == error)
	{
		error = CB_SGPC3_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	*ethanol_ppb = (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]));
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_GetBaseline(uint16_t *baseline_raw){
	uint32_t error; // error code
	uint8_t i2cData[3];
	i2cData[0] = (CB_eSgpc3GetBaseline >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3GetBaseline);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 2, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eSgpc3I2CAddress, 3, i2cData);
	}
	// verify checksum
	if (CB_NO_ERROR == error)
	{
		error = CB_SGPC3_CheckCrc(&(i2cData[0]), 2, i2cData[2]);
	}
	*baseline_raw = (((uint16_t)(i2cData[0]) << 8) + (uint16_t)(i2cData[1]));
	return error;
}

//==============================================================================
uint32_t CB_SGPC3_SetBaseline(const uint16_t baseline_raw){
	uint32_t error; // error code
	uint8_t i2cData[5];
	i2cData[0] = (CB_eSgpc3SetBaseline >> 8);
	i2cData[1] = (0xFF & CB_eSgpc3SetBaseline);
	i2cData[2] = (baseline_raw >> 8);
	i2cData[3] = (0xFF & baseline_raw);	
	i2cData[4] = CB_SGPC3_CalculateCrcChecksum(&(i2cData[2]),2);
	error = CB_I2C_Write(CB_eSgpc3I2CAddress, 5, i2cData);
	return error;
}
//==============================================================================
uint16_t CB_SGPC3_GetCalculatedTvoc(const uint16_t sref_ppb, const uint16_t sout_ppb, const float correctionFactor)
{
	float tvoc_ppb = ((float)SGPC3_CREF_PPB*exp(((float)sref_ppb-(float)sout_ppb)/(float)SGPC3_A)-(float)SGPC3_CREF_PPB)*correctionFactor;
	if (tvoc_ppb < 0.0f)
	{
		tvoc_ppb = 0; // do not allow negative values
	}
	return (uint16_t)(tvoc_ppb);
}
