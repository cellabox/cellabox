/** Copyright (c) 2016-2018 Cellabox.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement: “This product includes software
 *    developed by Cellabox, Switzerland and its contributors.”
 *
 * 4. Neither the name of Cellabox nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**@file cb_lmp91000.h
 * @copyright: Cellabox, Waeni 10, 8840 Trachslau, Switzerland
 * @author : Reto Keler
 * @version: 1.0
 * @date : 01-Nov-2017
 * @brief : nRF52840 interface to electrochemical sensors on LMP91000.
 * -# Controller: nRF52840
 * -# IDE : Eclipse Mars 4.5.2
 * -# Compiler : ARM GCC
 */

#ifndef CB_LMP91000_H_
#define CB_LMP91000_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

//-- Defines -------------------------------------------------------------------
#define CB_LMP91000_STATUS_REG_ADDR			(0x00)	// Read only status register
#define CB_LMP91000_LOCK_REG_ADDR      		(0x01)	// Protection Register (lock register)
#define CB_LMP91000_TIACN_REG_ADDR 			(0x10)	// TIA Control Register
#define CB_LMP91000_REFCN_REG_ADDR     		(0x11)	// Reference Control Register
#define CB_LMP91000_MODECN_REG_ADDR   		(0x12)	// Mode Control Register

#define CB_LMP91000_TIACN_REG_VALUE  		(0x03) 	// default
#define CB_LMP91000_REFCN_REG_VALUE    		(0x20)	// default
#define CB_LMP91000_MODECN_REG_VALUE  		(0x00)	// default

#define CB_LMP91000_ENABLE_WAIT_DELAY_USEC	(1)		// t_en;startmin = 600nsec.

//-- Enumerations --------------------------------------------------------------
// I2C address
typedef enum CB_eLmp91000I2cAddress_tag{
	CB_eLmp91000I2CAddress = 0x48, // sensor I2C address (7-bit)
	CB_eLmp91000I2cAddressAndWriteBit = 0x90, // sensor I2C address + write bit
	CB_eLmp91000I2cAddressAndReadBit = 0x91 // sensor I2C address + read bit
}CB_eLmp91000I2cAddress_t;
// LMP91000 status
typedef enum CB_eLmp91000Status_tag
{
	eLmp91000NotReady = 0, // default
	eLmp91000Ready = 1
}CB_eLmp91000Status_t;
// LMP91000 lock status
typedef enum CB_eLmp91000Lock_tag
{
	eLmp91000Unlocked = 0, // write mode
	eLmp91000Locked = 1 // read-only mode, default
}CB_eLmp91000Lock_t;
// LMP91000 modes
typedef enum CB_eLmp91000Mode_tag
{
	eDeepSleep = 0, // A1 = off, TIA = off, temperature sensor =  off --> suggested when gas sensor is not used and 0mV bias is required (between WE and RE)
	e2LeadGroundGalvanicCell = 1,
	eStandby = 2, // A1 = on, TIA = off, temperature sensor = off --> suggested when gas sensor is not used and short warm-up time is required
	e3LeadAmperometricCell = 3,
	eTemperatureMeasurementTiaOff = 4,
	eTemperatureMeasurementTiaOn = 5
}CB_eLmp91000Mode_t;
// LMP91000 voltage reference
typedef enum CB_eLmp91000VRefSource_tag
{
	eInternalVoltageReference = 0,
	eExternalVoltageReference = 1
}CB_eLmp91000VRefSource_t;
// LMP91000 load resistor
typedef enum CB_eLmp91000LoadResistorOhm_tag
{
	eLoadResistor10Ohm = 0,
	eLoadResistor33Ohm = 1,
	eLoadResistor50Ohm = 2,
	eLoadResistor100Ohm = 3 // default
}CB_eLmp91000LoadResistorOhm_t;
// LMP91000 Trans-Impedance Amplifier (TIA) feedback resistor
typedef enum CB_eLmp91000FeedbackResistorkOhm_tag
{
	eFeedbackResistorExternal = 0, // default
	eFeedbackResistor2_75kOhm = 1,
	eFeedbackResistor3_5kOhm = 2,
	eFeedbackResistor7kOhm = 3,
	eFeedbackResistor14kOhm = 4,
	eFeedbackResistor35kOhm = 5,
	eFeedbackResistor120kOhm = 6,
	eFeedbackResistor350kOhm = 7
}CB_eLmp91000FeedbackResistorkOhm_t;
// LMP91000 internal zero selection
typedef enum CB_eLmp91000InternalZeroSelection_tag
{
	eInternalZeroSelection20Perc = 0,
	eInternalZeroSelection50Perc = 1, // default
	eInternalZeroSelection67Perc = 2,
	eInternalZeroSelectionBypass = 3
}CB_eLmp91000InternalZeroSelection_t;
// LMP91000 internal zero selection
typedef enum CB_eLmp91000BiasVoltageSelection_tag
{
	eBiasVoltage0PercVref = 0, // default
	eBiasVoltage1PercVref = 1,
	eBiasVoltage2PercVref = 2,
	eBiasVoltage4PercVref = 3,
	eBiasVoltage6PercVref = 4,
	eBiasVoltage8PercVref = 5,
	eBiasVoltage10PercVref = 6,
	eBiasVoltage12PercVref = 7,
	eBiasVoltage14PercVref = 8,
	eBiasVoltage16PercVref = 9,
	eBiasVoltage18PercVref = 10,
	eBiasVoltage20PercVref = 11,
	eBiasVoltage22PercVref = 12,
	eBiasVoltage24PercVref = 13
}CB_eLmp91000BiasVoltageSelection_t;
// LMP91000 Bias sign
typedef enum CB_eLmp91000BiasSign_tag
{
	eNegativeBias = 0, // default
	ePositiveBias = 1
}CB_eLmp91000BiasSign_t;
// LMP91000 FET feature
typedef enum CB_eLmp91000FetFeature_tag
{
	eShortingFetFeatureDisabled = 0, // default
	eShortingFetFeatureEnabled = 1
}CB_eLmp91000FetFeature_t;
typedef struct CB_stLmp91000_tag
{
	CB_eLmp91000Status_t status;
	CB_eLmp91000Mode_t mode;
	CB_eLmp91000VRefSource_t vRefSource;
	float vRefValueV; // Refrence voltage [V]
	CB_eLmp91000LoadResistorOhm_t rLoad;
	CB_eLmp91000FeedbackResistorkOhm_t rFeedback;
	CB_eLmp91000InternalZeroSelection_t zeroSelection;
	CB_eLmp91000BiasSign_t biasVoltageSign;
	CB_eLmp91000BiasVoltageSelection_t biasVoltageSelection;
	CB_eLmp91000FetFeature_t fetShortFeature;
	uint8_t moduleEnablePinNumber; // Pin number of port of nRF52: 0...62
	float temperature_DegC; // LMP91000 temperature sensor value
}CB_stLmp91000_t;

//==============================================================================
CB_eLmp91000BiasVoltageSelection_t CB_LMP91000_GetBiasVoltageSelectionValue(const int16_t biasVoltage_mV, const float vRefValueV);
//==============================================================================
// Get LMP91000 Bias Voltage Selector for register REFCN.
//------------------------------------------------------------------------------
// input: biasVoltage_mV: Bias voltage of electrochemical sensor [mV]
// input: vRefValueV: LMP91000 reference voltage [V]
// return: error: Bias selector
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_LMP91000_GetStatus(const uint8_t moduleEnablePinNumber, CB_eLmp91000Status_t* status);
//==============================================================================
// Get LMP91000 status.
//------------------------------------------------------------------------------
// input: moduleEnablePinNumber: MENB pin number of the nRF52
// output: status: LMP91000 status [ready or not-ready]
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_LMP91000_Init(CB_stLmp91000_t const * lmpConfig);
//==============================================================================
// Initializes the I2C bus and LMP91000 for measurement of gas.
// The gas sensor is in standby state after initialization.
//------------------------------------------------------------------------------
// input: LMP91000 sensor configuration
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_LMP91000_Set3LeadAmperometricCellMode(CB_stLmp91000_t* lmpConfig);
//==============================================================================
// Set the LMP91000 into 3-lead amperometric cell mode
//------------------------------------------------------------------------------
// input: LMP91000 sensor configuration
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error

//==============================================================================
uint32_t CB_LMP91000_SetStandbyMode(CB_stLmp91000_t* lmpConfig);
//==============================================================================
// Set the LMP91000 into standby mode (TIA is OFF, while the A1 control amplifier is ON)
//------------------------------------------------------------------------------
// input: LMP91000 sensor configuration
//
// return: error: ACK_ERROR = no acknowledgment from sensor
// CHECKSUM_ERROR = checksum mismatch
// NO_ERROR = no error


#ifdef __cplusplus
}
#endif

#endif // CB_LMP91000_H_
