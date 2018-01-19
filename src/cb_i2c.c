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

/**@file cb_i2c.c
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 16-Jun-2017
 * @brief : I2C hardware abstraction layer.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#include "nrf52840.h"
#include "nrf52840_bitfields.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_drv_twi.h"
#include "app_util_platform.h"
#include "cb_error.h"
#include "cb_i2c.h"

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); // TWI instance.
static volatile bool m_xfer_done = false; // Indicates if operation on TWI has ended.

//==============================================================================
void i2c_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        default:
            break;
    }
}
//==============================================================================
uint32_t CB_I2C_Init(void)
{
    uint32_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = SCL_GPIO_PIN_NUMBER,
       .sda                = SDA_GPIO_PIN_NUMBER,
       .frequency          = NRF_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL); // TODO: use i2c_handler() for non-blocking mode
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
	
	return err_code;
}
//==============================================================================
uint32_t CB_I2C_Read(const uint8_t i2cAddr, const uint8_t numByteToRead, uint8_t* pData)
{
	m_xfer_done = false;
	uint32_t error = nrf_drv_twi_rx(&m_twi, i2cAddr, pData, numByteToRead); // TODO: use i2c_handler() for non-blocking mode
	return error;
}
//==============================================================================
uint32_t CB_I2C_ReadReg(const uint8_t i2cAddr, const uint8_t regAddr, const uint8_t numByteToRead, uint8_t* pData)
{
	m_xfer_done = false;
	uint32_t error = nrf_drv_twi_tx(&m_twi, i2cAddr, &regAddr, 1, true); // TODO: use i2c_handler() for non-blocking mode
	if (NRF_SUCCESS == error)
	{
		m_xfer_done = false;
		error = nrf_drv_twi_rx(&m_twi, i2cAddr, pData, numByteToRead); // TODO: use i2c_handler() for non-blocking mode
	}
	return error;
}
//==============================================================================
uint32_t CB_I2C_Write(const uint8_t i2cAddr, const uint8_t numByteToWrite, const uint8_t* pData)
{
	m_xfer_done = false;
	uint32_t error = nrf_drv_twi_tx(&m_twi, i2cAddr, pData, numByteToWrite, false); // TODO: use i2c_handler() for non-blocking mode
	return error;
}
//==============================================================================
uint32_t CB_I2C_WriteReg(const uint8_t i2cAddr, const uint8_t regAddr, const uint8_t numByteToWrite, const uint8_t* pData)
{
	m_xfer_done = false;
	uint32_t error;
	if (numByteToWrite < CB_I2C_TX_BUFFER_SIZE_BYTE)
	{
		uint8_t txBuffer[CB_I2C_TX_BUFFER_SIZE_BYTE];
		txBuffer[0] = regAddr;
		for (uint8_t i=0; i<numByteToWrite; i++)
		{
			txBuffer[i+1] = pData[i];
		}
		error = nrf_drv_twi_tx(&m_twi, i2cAddr, txBuffer, (numByteToWrite+1), false); // TODO: use i2c_handler() for non-blocking mode
	}
	else
	{
		error = CB_I2C_TX_BUFFER_OVERFLOW;
	}
	return error;
}
//==============================================================================
/*uint32_t CB_I2C_Init(void) {
	SDA_OPEN(); // I2C-bus idle mode SDA released (high), do this BEFORE setting open-drain
	SCL_OPEN(); // I2C-bus idle mode SCL released (high), do this BEFORE setting open-drain
	//SDA_LOW();
	//SCL_LOW();
    nrf_gpio_cfg(
    	SDA_GPIO_PIN_NUMBER,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_H0D1,
        NRF_GPIO_PIN_NOSENSE);
    nrf_gpio_cfg(
    	SCL_GPIO_PIN_NUMBER,
        NRF_GPIO_PIN_DIR_OUTPUT,
        NRF_GPIO_PIN_INPUT_DISCONNECT,
        NRF_GPIO_PIN_NOPULL,
		NRF_GPIO_PIN_H0D1,
        NRF_GPIO_PIN_NOSENSE);
	return CB_NO_ERROR;
}*/
//==============================================================================
void CB_I2C_StartCondition(void) {
	SDA_OPEN();
	nrf_delay_us(1);
	SCL_OPEN();
	nrf_delay_us(1);
	SDA_LOW();
	nrf_delay_us(10); // hold time start condition (t_HD;STA)
	SCL_LOW();
	nrf_delay_us(10);
}
//==============================================================================
void CB_I2C_StopCondition(void) {
	SCL_LOW();
	nrf_delay_us(1);
	SDA_LOW();
	nrf_delay_us(1);
	SCL_OPEN();
	nrf_delay_us(10); // set-up time stop condition (t_SU;STO)
	SDA_OPEN();
	nrf_delay_us(10);
}
//==============================================================================
uint32_t CB_I2C_WriteByte(uint8_t txByte) {
	uint8_t mask;
	uint32_t error = CB_NO_ERROR;
	for (mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
			{
		if ((mask & txByte) == 0)
			SDA_LOW(); // masking txByte, write bit to SDA-Line
		else
			SDA_OPEN();
		nrf_delay_us(1); // data set-up time (t_SU;DAT)
		SCL_OPEN(); // generate clock pulse on SCL
		nrf_delay_us(5); // SCL high time (t_HIGH)
		SCL_LOW();
		nrf_delay_us(1); // data hold time(t_HD;DAT)
	}
	SDA_OPEN(); // release SDA-line
	SCL_OPEN(); // clk #9 for ack
	nrf_delay_us(1); // data set-up time (t_SU;DAT)
	if (SDA_READ)
		error = CB_I2C_NACK_ERROR; // check ack from i2c slave
	SCL_LOW();
	nrf_delay_us(20); // wait to see byte package on scope
	return error; // return error code
}
//==============================================================================
uint32_t CB_I2C_ReadByte(CB_eI2cAcknowledge_t ack, uint8_t* rxByte) {
	uint8_t mask;
	uint16_t timeoutCounter;
	uint8_t rxByteCopy = 0;
	uint32_t error = CB_NO_ERROR;
	SDA_OPEN(); // release SDA-line
	for (mask = 0x80; mask > 0; mask >>= 1) // shift bit for masking (8 times)
	{
		SCL_OPEN(); // start clock on SCL-line
		nrf_delay_us(1); // data set-up time (t_SU;DAT)
		timeoutCounter = 0;
		while ((SCL_READ == 0) && (CB_I2C_TIMEOUT_USEC > timeoutCounter))
		{
			timeoutCounter++;
			nrf_delay_us(1);
		}
		if (!(CB_I2C_TIMEOUT_USEC > timeoutCounter))
		{
			error = CB_I2C_SHTC1_TIMEOUT_ERROR;
			break;
		}
		nrf_delay_us(3); // SCL high time (t_HIGH)
		if (SDA_READ)
			rxByteCopy = rxByteCopy | mask; // read bit
		SCL_LOW();
		nrf_delay_us(1); // data hold time(t_HD;DAT)
	}
	if (ack == CB_eI2cAcknowledge)
		SDA_LOW(); // send acknowledge if necessary
	else
		SDA_OPEN();
	nrf_delay_us(1); // data set-up time (t_SU;DAT)
	SCL_OPEN(); // clk #9 for ack
	nrf_delay_us(5); // SCL high time (t_HIGH)
	SCL_LOW();
	SDA_OPEN(); // release SDA-line
	nrf_delay_us(20); // wait to see byte package on scope
	*rxByte = rxByteCopy;
	return error; // return error code
}
