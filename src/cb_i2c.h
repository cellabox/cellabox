/**@file cb_i2c.h
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 16-Jun-2017
 * @brief : I2C hardware abstraction layer.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */
#ifndef CB_I2C_H_
#define CB_I2C_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Define nRF52840 specific I2C GPIOs.
 */
// SDA pin nrf52840 P0.26
#define SDA_GPIO_PORT_NUMBER 0
#define SDA_GPIO_PIN_NUMBER  26
#define SDA_LOW() (NRF_P0->OUTCLR = (1 << SDA_GPIO_PIN_NUMBER)) // set SDA to low
#define SDA_OPEN() (NRF_P0->OUTSET = (1 << SDA_GPIO_PIN_NUMBER)) // set SDA to open-drain
#define SDA_READ (NRF_P0->IN & (1 << SDA_GPIO_PIN_NUMBER)) // read SDA
// SCL pin nrf52840 P0.27
#define SCL_GPIO_PORT_NUMBER 0
#define SCL_GPIO_PIN_NUMBER  27
#define SCL_LOW() (NRF_P0->OUTCLR = (1 << SCL_GPIO_PIN_NUMBER)) // set SCL to low
#define SCL_OPEN() (NRF_P0->OUTSET = (1 << SCL_GPIO_PIN_NUMBER)) // set SCL to open-drain
#define SCL_READ (NRF_P0->IN & (1 << SCL_GPIO_PIN_NUMBER)) // read SCL
// I2C defines
#define CB_I2C_TIMEOUT_USEC 		32768 		// I2C timeout e.g. when waiting for (SCL_READ == 0), max. measurement duration SHTC1 = 14.4msec
#define CB_I2C_TX_BUFFER_SIZE_BYTE 	8 			// I2C transmit buffer size in byte
//-- Enumerations --------------------------------------------------------------
// I2C acknowledge
typedef enum{
 CB_eI2cAcknowledge = 0,
 CB_eI2cNoAcknowledge = 1,
 }CB_eI2cAcknowledge_t;
//==============================================================================
uint32_t CB_I2C_Init(void);
//==============================================================================
// Initializes the ports for I2C interface.
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_I2C_Read(const uint8_t i2cAddr, const uint8_t numByteToRead, uint8_t* pData);
//==============================================================================
// Read sequence of byte(s)
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_I2C_ReadReg(const uint8_t i2cAddr, const uint8_t regAddr, const uint8_t numByteToRead, uint8_t* pData);
//==============================================================================
// Read register
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_I2C_Write(const uint8_t i2cAddr, const uint8_t numByteToWrite, const uint8_t* pData);
//==============================================================================
// Write sequence of byte(s)
//------------------------------------------------------------------------------
//==============================================================================
uint32_t CB_I2C_WriteReg(const uint8_t i2cAddr, const uint8_t regAddr, const uint8_t numByteToWrite, const uint8_t* pData);
//==============================================================================
// Read register
//------------------------------------------------------------------------------
//==============================================================================
void CB_I2C_StartCondition(void);
//==============================================================================
// Writes a start condition on I2C-Bus.
//------------------------------------------------------------------------------
// remark: Timing (delay) may have to be changed for different microcontroller.
// _____
// SDA: |_____
// _______
// SCL: |___
//==============================================================================
void CB_I2C_StopCondition(void);
//==============================================================================
// Writes a stop condition on I2C-Bus.
//------------------------------------------------------------------------------
// remark: Timing (delay) may have to be changed for different microcontroller.
// _____
// SDA: _____|
// _______
// SCL: ___|
//==============================================================================
uint32_t CB_I2C_WriteByte(uint8_t txByte);
//==============================================================================
// Writes a byte to I2C-Bus and checks acknowledge.
//------------------------------------------------------------------------------
// input: txByte transmit byte
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// NO_ERROR = no error
//
// remark: Timing (delay) may have to be changed for different microcontroller.
//==============================================================================
uint32_t CB_I2C_ReadByte(CB_eI2cAcknowledge_t ack, uint8_t* rxByte);
//==============================================================================
// Reads a byte on I2C-Bus.
//------------------------------------------------------------------------------
// input: ack Acknowledge: ACK or NO_ACK
//
// output: acknowledge
// output: received byte
//
// return: error code
//
// remark: Timing (delay) may have to be changed for different microcontroller.

#ifdef __cplusplus
}
#endif

#endif // CB_I2C_H_
