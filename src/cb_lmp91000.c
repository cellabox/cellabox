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

/**@file cb_lmp91000.c
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 01-Nov-2017
 * @brief : nRF52840 interface to TI LMP91000 with 3-lead electrochemical gas sensor.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "cb_error.h"
#include "cb_i2c.h"
#include "cb_lmp91000.h"


CB_eLmp91000BiasVoltageSelection_t CB_LMP91000_GetBiasVoltageSelectionValue(const int16_t biasVoltage_mV, const float vRefValueV)
{
	uint8_t biasVToRefV_Perc = (uint8_t)((float)(abs(biasVoltage_mV))/(vRefValueV*10.0f)); // 100/vRefValueV*(biasVoltage_mV/1000)
	if (biasVToRefV_Perc == 0)
	{
		return eBiasVoltage0PercVref; // default
	}
	else if (biasVToRefV_Perc == 1)
	{
		return eBiasVoltage1PercVref;
	}
	else if (biasVToRefV_Perc == 2)
	{
		return eBiasVoltage2PercVref;
	}
	else if (biasVToRefV_Perc <= 4)
	{
		return eBiasVoltage4PercVref;
	}
	else if (biasVToRefV_Perc <= 6)
	{
		return eBiasVoltage6PercVref;
	}
	else if (biasVToRefV_Perc <= 8)
	{
		return eBiasVoltage8PercVref;
	}
	else if (biasVToRefV_Perc <= 10)
	{
		return eBiasVoltage10PercVref;
	}
	else if (biasVToRefV_Perc <= 12)
	{
		return eBiasVoltage12PercVref;
	}
	else if (biasVToRefV_Perc <= 14)
	{
		return eBiasVoltage14PercVref;
	}
	else if (biasVToRefV_Perc <= 16)
	{
		return eBiasVoltage16PercVref;
	}
	else if (biasVToRefV_Perc <= 18)
	{
		return eBiasVoltage18PercVref;
	}
	else if (biasVToRefV_Perc <= 20)
	{
		return eBiasVoltage20PercVref;
	}
	else if (biasVToRefV_Perc <= 22)
	{
		return eBiasVoltage22PercVref;
	}
	else // if (biasVToRefV_Perc > 22%) --> assign maximum possible value
	{
		return eBiasVoltage24PercVref;
	}
}



uint32_t CB_LMP91000_Unlock(const uint8_t menbPinNbr)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(menbPinNbr); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	i2cData[0] = CB_LMP91000_LOCK_REG_ADDR;
	i2cData[1] = eLmp91000Unlocked;
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	return error;
}

uint32_t CB_LMP91000_Lock(const uint8_t menbPinNbr)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(menbPinNbr); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	i2cData[0] = CB_LMP91000_LOCK_REG_ADDR;
	i2cData[1] = eLmp91000Locked;
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	nrf_gpio_pin_set(menbPinNbr); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_GetStatus(const uint8_t moduleEnablePinNumber, CB_eLmp91000Status_t* status)
{
	uint32_t error; // error code
	uint8_t i2cData[1];
	nrf_gpio_pin_clear(moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	i2cData[0] = CB_LMP91000_STATUS_REG_ADDR;
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 1, i2cData);
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
	}
	if (0 == i2cData[0])
	{
		*status = eLmp91000NotReady;
	}
	else
	{
		*status = eLmp91000Ready;
	}
	nrf_gpio_pin_set(moduleEnablePinNumber); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_SetTiaCtrlRegsiter(CB_stLmp91000_t const * sensorConfig)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(sensorConfig->moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	// Write TIACN register
	i2cData[0] = CB_LMP91000_TIACN_REG_ADDR;
	i2cData[1] = ((sensorConfig->rFeedback) << 2) + (sensorConfig->rLoad);
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	// Reach TIACN register
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
		// Check TIACN register
		if (CB_NO_ERROR == error)
		{
			if (i2cData[0] != i2cData[1])
			{
				error = CB_LMP91000_I2C_DATA_ERROR;
			}
		}
	}
	nrf_gpio_pin_set(sensorConfig->moduleEnablePinNumber); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_SetRefCtrlRegsiter(const CB_stLmp91000_t* lmpConfig)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(lmpConfig->moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	// Write REFCN register
	i2cData[0] = CB_LMP91000_REFCN_REG_ADDR;
	i2cData[1] = ((lmpConfig->vRefSource) << 7) + ((lmpConfig->zeroSelection) << 5) + ((lmpConfig->biasVoltageSign) << 4) + (lmpConfig->biasVoltageSelection);
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	// Reach REFCN register
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
		// Check REFCN register
		if (CB_NO_ERROR == error)
		{
			if (i2cData[0] != i2cData[1])
			{
				error = CB_LMP91000_I2C_DATA_ERROR;
			}
		}
	}
	nrf_gpio_pin_set(lmpConfig->moduleEnablePinNumber); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_SetModeCtrlRegister(const CB_stLmp91000_t* lmpConfig)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(lmpConfig->moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	// Write MODCN register
	i2cData[0] = CB_LMP91000_MODECN_REG_ADDR;
	i2cData[1] = ((lmpConfig->fetShortFeature) << 7) + (lmpConfig->mode);
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	// Reach MODCN register
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
		// Check MODCN register
		if (CB_NO_ERROR == error)
		{
			if (i2cData[0] != i2cData[1])
			{
				error = CB_LMP91000_I2C_DATA_ERROR;
			}
		}
	}
	nrf_gpio_pin_set(lmpConfig->moduleEnablePinNumber); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_Init(CB_stLmp91000_t const * lmpConfig)
{
	uint16_t error;

	error = CB_LMP91000_Unlock(lmpConfig->moduleEnablePinNumber);
	if (CB_NO_ERROR == error)
	{
		error = CB_LMP91000_SetTiaCtrlRegsiter(lmpConfig);
		if (CB_NO_ERROR == error)
		{
			error = CB_LMP91000_SetRefCtrlRegsiter(lmpConfig);
			if (CB_NO_ERROR == error)
			{
				error = CB_LMP91000_SetModeCtrlRegister(lmpConfig);
				if (CB_NO_ERROR == error)
				{
					error = CB_LMP91000_Lock(lmpConfig->moduleEnablePinNumber);
				}
			}
		}
	}
	return error;
}

uint32_t CB_LMP91000_Set3LeadAmperometricCellMode(CB_stLmp91000_t* lmpConfig)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(lmpConfig->moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	lmpConfig->mode = e3LeadAmperometricCell;
	// Write MODCN register
	i2cData[0] = CB_LMP91000_MODECN_REG_ADDR;
	i2cData[1] = ((lmpConfig->fetShortFeature) << 7) + (lmpConfig->mode);
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	// Reach MODCN register
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
		// Check MODCN register
		if (CB_NO_ERROR == error)
		{
			if (i2cData[0] != i2cData[1])
			{
				error = CB_LMP91000_I2C_DATA_ERROR;
			}
		}
	}
	nrf_gpio_pin_set(lmpConfig->moduleEnablePinNumber); // disable LMP91000
	return error;
}

uint32_t CB_LMP91000_SetStandbyMode(CB_stLmp91000_t* lmpConfig)
{
	uint32_t error; // error code
	uint8_t i2cData[2];
	nrf_gpio_pin_clear(lmpConfig->moduleEnablePinNumber); // enable LMP91000
	nrf_delay_us(CB_LMP91000_ENABLE_WAIT_DELAY_USEC);
	lmpConfig->mode = eStandby;
	// Write MODCN register
	i2cData[0] = CB_LMP91000_MODECN_REG_ADDR;
	i2cData[1] = ((lmpConfig->fetShortFeature) << 7) + (lmpConfig->mode);
	error = CB_I2C_Write(CB_eLmp91000I2CAddress, 2, i2cData);
	// Reach MODCN register
	if (CB_NO_ERROR == error)
	{
		error = CB_I2C_Read(CB_eLmp91000I2CAddress, 1, i2cData);
		// Check MODCN register
		if (CB_NO_ERROR == error)
		{
			if (i2cData[0] != i2cData[1])
			{
				error = CB_LMP91000_I2C_DATA_ERROR;
			}
		}
	}
	nrf_gpio_pin_set(lmpConfig->moduleEnablePinNumber); // disable LMP91000
	return error;
}
