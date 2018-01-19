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

/* Includes ------------------------------------------------------------------*/
#include "cb_error.h"
#include "cb_i2c.h"
#include "cb_lps22hb.h"

#ifdef  USE_FULL_ASSERT_LPS22HB
#include <stdio.h>
#endif

/** @addtogroup Environmental_Sensor
* @{
*/

/** @defgroup CB_LPS22HB_DRIVER
* @brief LPS22HB DRIVER
* @{
*/


/**
* @brief  Read CB_LPS22HB Registers
* @param  uint8_t regAddr:       address of the first register to read
* @param  uint8_t numByteToRead:   number of registers to read
* @param  uint8_t *pData:          pointer to the destination data buffer
* @retval CB_LPS22HB_ERROR or CB_NO_ERROR
*/
// the user must redefine the proper CB_LPS22HB_ReadReg
uint32_t CB_LPS22HB_ReadReg(const uint8_t regAddr, const uint8_t numByteToRead, uint8_t* pData)
{
	uint32_t error = CB_I2C_ReadReg(CB_LPS22HB_ADDRESS, regAddr, numByteToRead, pData);
	return error;
}

/**
* @brief  Write CB_LPS22HB Registers
* @param  uint8_t regAddr:      address of the register to write
* @param  uint8_t *pData:         pointer to the source data buffer
* @param  uint8_t numByteToWrite:           Number of bytes to write
* @retval CB_LPS22HB_ERROR or CB_NO_ERROR
*/
// the user must redefine the proper CB_LPS22HB_WriteReg
uint32_t CB_LPS22HB_WriteReg(const uint8_t regAddr, const uint8_t numByteToWrite, const uint8_t* pData)
{
	if (numByteToWrite > CB_I2C_TX_BUFFER_SIZE_BYTE)
	{
		return CB_I2C_TX_BUFFER_OVERFLOW;
	}
	uint32_t error = CB_I2C_WriteReg(CB_LPS22HB_ADDRESS, regAddr, numByteToWrite, pData);
	return error;
}



/** @defgroup CB_LPS22HB_Public_Functions
* @{
*/


/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DeviceID(uint8_t* deviceid)
{  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_WHO_AM_I_REG, 1, deviceid))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief  Get the LPS22HB driver version.
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DriverVersion(CB_LPS22HB_DriverVersion_st *Version)
{  
  Version->Major = CB_LPS22HB_DriverVersion_Major;
  Version->Minor = CB_LPS22HB_DriverVersion_Minor;
  Version->Point = CB_LPS22HB_DriverVersion_Point;
  
  return CB_NO_ERROR;
}


/**
* @brief  Set LPS22HB Low Power or Low Noise Mode Configuration 
* @param  CB_LPS22HB_LowNoise or CB_LPS22HB_LowPower mode
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PowerMode(CB_LPS22HB_PowerMode_et mode)
{  
  uint8_t tmp;

  CB_LPS22HB_assert_param(IS_CB_LPS22HB_PowerMode(mode));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_RES_CONF_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_LCEN_MASK;
  tmp |= (uint8_t)mode;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_RES_CONF_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Get LPS22HB Power Mode
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PowerMode(CB_LPS22HB_PowerMode_et* mode)
{
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_RES_CONF_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  *mode = (CB_LPS22HB_PowerMode_et)(tmp & CB_LPS22HB_LCEN_MASK);
  
  return CB_NO_ERROR;
}


/**
* @brief  Set LPS22HB Output Data Rate
* @param  Output Data Rate
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_Odr(CB_LPS22HB_Odr_et odr)
{  
  uint8_t tmp;
 
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_ODR(odr));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_ODR_MASK;
  tmp |= (uint8_t)odr;

  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
 
  return CB_NO_ERROR;
}

/**
* @brief  Get LPS22HB Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Odr(CB_LPS22HB_Odr_et* odr)
{
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  *odr = (CB_LPS22HB_Odr_et)(tmp & CB_LPS22HB_ODR_MASK);
  
  return CB_NO_ERROR;
}

/**
* @brief  Enable/Disale low-pass filter on LPS22HB pressure data
* @param  state: enable or disable
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_LowPassFilter(CB_LPS22HB_State_et state)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(state));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_LPFP_MASK;
  tmp |= ((uint8_t)state)<<CB_LPS22HB_LPFP_BIT;
  

  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;

  
  return CB_NO_ERROR;
}


/**
* @brief  Set low-pass filter cutoff configuration on LPS22HB pressure data
* @param  Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_LowPassFilterCutoff(CB_LPS22HB_LPF_Cutoff_et cutoff){
  
  uint8_t tmp;
 
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_LPF_Cutoff(cutoff));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_LPFP_CUTOFF_MASK;
  tmp |= (uint8_t)cutoff;
  

  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
 
  return CB_NO_ERROR;
  
}

/**
* @brief  Set Block Data Mode
* @detail It is recommended to set BDU bit to ‘1’.
* @detail This feature avoids reading LSB and MSB related to different samples.
* @param  CB_LPS22HB_BDU_CONTINUOUS_UPDATE, CB_LPS22HB_BDU_NO_UPDATE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/

uint32_t CB_LPS22HB_Set_Bdu(CB_LPS22HB_Bdu_et bdu)
{  
  uint8_t tmp;

  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_BDUMode(bdu));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_BDU_MASK;
  tmp |= ((uint8_t)bdu);
  

  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
  return CB_NO_ERROR;
  
   return CB_NO_ERROR;
}

/**
* @brief  Get Block Data Mode
* @param Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Bdu(CB_LPS22HB_Bdu_et* bdu)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  *bdu = (CB_LPS22HB_Bdu_et)(tmp & CB_LPS22HB_BDU_MASK);
  
  return CB_NO_ERROR;
}

/**
* @brief  Set SPI mode: 3 Wire Interface or 4 Wire Interface
* @param CB_LPS22HB_SPI_3_WIRE, CB_LPS22HB_SPI_4_WIRE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_SpiInterface(CB_LPS22HB_SPIMode_et spimode)
{  
  uint8_t tmp;
 
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_SPIMode(spimode));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_SIM_MASK;
  tmp |= (uint8_t)spimode;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;

  return CB_NO_ERROR;
}

/**
* @brief  Clock Tree Configuration
* @param  CB_LPS22HB_CTE_NotBalanced, CB_LPS22HB_CTE_ABalanced
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_ClockTreeConfifuration(CB_LPS22HB_CTE_et mode)
{  
  uint8_t tmp;
 
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_CTE(mode));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_CTE_MASK;
  tmp |= (uint8_t)mode;
  

  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CLOCK_TREE_CONFIGURATION, 1, &tmp))
    return CB_LPS22HB_ERROR;

  return CB_NO_ERROR;
}


/**
* @brief  Get SPI mode: 3 Wire Interface or 4 Wire Interface
* @param Buffet to empty with spi mode read from Sensor
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_SpiInterface(CB_LPS22HB_SPIMode_et* spimode)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  *spimode = (CB_LPS22HB_SPIMode_et)(tmp & CB_LPS22HB_SIM_MASK);
  
  return CB_NO_ERROR;
}
    
  /**
* @brief   Software Reset. Self-clearing upon completion
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_SwReset(void)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp |= (0x01<<CB_LPS22HB_SW_RESET_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Reboot Memory Content
* @param None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/

uint32_t CB_LPS22HB_MemoryBoot(void)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp |= (0x01<<CB_LPS22HB_BOOT_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief   Software Reset ann Reboot Memory Content. 
* @detail  The device is reset to the power on configuration if the SWRESET bit is set to ‘1’ 
 + and BOOT is set to ‘1’; Self-clearing upon completion.
* @param 	None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_SwResetAndMemoryBoot(void)
{
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp |= ((0x01<<CB_LPS22HB_SW_RESET_BIT) | (0x01<<CB_LPS22HB_BOOT_BIT));
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

  
/**
* @brief   Enable/Disable FIFO Mode 
* @param CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoModeUse(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_FIFO_EN_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_FIFO_EN_BIT;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}
/**
* @brief   Enable/Disable FIFO Watermark Level Use 
* @param   CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoWatermarkLevelUse(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_WTM_EN_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_WTM_EN_BIT;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}
  
 /**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE. Default is CB_LPS22HB_ENABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutomaticIncrementRegAddress(CB_LPS22HB_State_et status){
  
  uint8_t tmp;
  
 CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_ADD_INC_MASK;
  tmp |= (((uint8_t)status)<<CB_LPS22HB_ADD_INC_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;

}

/**
* @brief  Enable/Disable I2C Interface 
* @param State: CB_LPS22HB_ENABLE (reset bit)/ CB_LPS22HB_DISABLE (set bit)
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_I2C(CB_LPS22HB_State_et statei2c)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(statei2c));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  /*Reset Bit->I2C Enabled*/
  tmp &= ~CB_LPS22HB_I2C_MASK;
  tmp|=((uint8_t)~statei2c)<<CB_LPS22HB_I2C_BIT;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief   Set the one-shot bit in order to start acquisition when the ONE SHOT mode
*          has been selected by the ODR configuration.
* @detail  Once the measurement is done, ONE_SHOT bit will self-clear.
* @param   None
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_StartOneShotMeasurement(void)
{
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  /* Set the one shot bit */
  /* Once the measurement is done, one shot bit will self-clear*/
  tmp |= CB_LPS22HB_ONE_SHOT_MASK;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
  
}

/**
* @brief  Set Interrupt Active on High or Low Level
* @param  CB_LPS22HB_ActiveHigh/CB_LPS22HB_ActiveLow
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptActiveLevel(CB_LPS22HB_InterruptActiveLevel_et mode)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_InterruptActiveLevel(mode));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_INT_H_L_MASK;
  tmp |= ((uint8_t)mode);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief   Push-pull/open drain selection on interrupt pads. Default tmp: 0
* @param   CB_LPS22HB_PushPull/CB_LPS22HB_OpenDrain
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptOutputType(CB_LPS22HB_OutputType_et output)
{
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_OutputType(output));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_PP_OD_MASK;
  tmp |= (uint8_t)output;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Set Data signal on INT pad control bits.
* @param  CB_LPS22HB_DATA,CB_LPS22HB_P_HIGH_CB_LPS22HB_P_LOW,CB_LPS22HB_P_LOW_HIGH
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptControlConfig(CB_LPS22HB_OutputSignalConfig_et config)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_OutputSignal(config));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
    tmp &= ~(CB_LPS22HB_INT_S12_MASK);
    tmp |= (uint8_t)config;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_DRDYInterrupt(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_DRDY_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_DRDY_BIT;
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}
    
 /**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_OVR_Interrupt(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_FIFO_OVR_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_FIFO_OVR_BIT;
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

 /**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_FTH_Interrupt(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_FIFO_FTH_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_FIFO_FTH_BIT;
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_FULL_Interrupt(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_FIFO_FULL_MASK;
  tmp |= ((uint8_t)status)<<CB_LPS22HB_FIFO_FULL_BIT;
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}



/**
* @brief   Enable AutoRifP function
* @param   none
* @detail When this function is enabled, an internal register is set with the current pressure values 
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*               the AutoRifP is slf creared.
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutoRifP(void)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp |= ((uint8_t)CB_LPS22HB_AUTORIFP_MASK);
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief   Disable AutoRifP function
* @param   none
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_ResetAutoRifP(void)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  
  tmp |= ((uint8_t)CB_LPS22HB_RESET_ARP_MASK);
   
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/*
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17) 
* @param  None
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutoZeroFunction(void)
{  
  uint8_t tmp;
    
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp |= CB_LPS22HB_AUTOZERO_MASK;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/*
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_ResetAutoZeroFunction(void)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  /* Set the RESET_AZ bit*/
  /* RESET_AZ is self cleared*/
  tmp |= CB_LPS22HB_RESET_AZ_MASK;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
   
  return CB_NO_ERROR;
}


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  CB_LPS22HB_ENABLE,CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_State_et diff_en)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(diff_en));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_DIFF_EN_MASK;
  tmp |= ((uint8_t)diff_en)<<CB_LPS22HB_DIFF_EN_BIT;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief  Get the DIFF_EN bit value
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptDifferentialGeneration(CB_LPS22HB_State_et* diff_en)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  *diff_en= (CB_LPS22HB_State_et)((tmp & CB_LPS22HB_DIFF_EN_MASK)>>CB_LPS22HB_DIFF_EN_BIT);
  
  return CB_NO_ERROR;
}

/**
* @brief  Latch Interrupt request to the INT_SOURCE register. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_LatchInterruptRequest(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
 CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_LIR_MASK;
  tmp |= (((uint8_t)status)<<CB_LPS22HB_LIR_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

  
    
  /**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PLE(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_PLE_MASK;
  tmp |= (((uint8_t)status)<<CB_LPS22HB_PLE_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PHE(CB_LPS22HB_State_et status)
{  
  uint8_t tmp;
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_State(status));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_PHE_MASK;
  tmp |= (((uint8_t)status)<<CB_LPS22HB_PHE_BIT);
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptDifferentialEventStatus(CB_LPS22HB_InterruptDiffStatus_st* interruptsource)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_SOURCE_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  interruptsource->PH = (uint8_t)(tmp & CB_LPS22HB_PH_MASK);
  interruptsource->PL = (uint8_t)((tmp & CB_LPS22HB_PL_MASK)>>CB_LPS22HB_PL_BIT);
  interruptsource->IA = (uint8_t)((tmp & CB_LPS22HB_IA_MASK)>>CB_LPS22HB_IA_BIT);
  interruptsource->BOOT= (uint8_t)((tmp & CB_LPS22HB_BOOT_STATUS_MASK)>>CB_LPS22HB_BOOT_STATUS_BIT);
  
  return CB_NO_ERROR;
}
  
/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DataStatus(CB_LPS22HB_DataStatus_st* datastatus)
{
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_STATUS_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  datastatus->PressDataAvailable = (uint8_t)(tmp & CB_LPS22HB_PDA_MASK);
  datastatus->TempDataAvailable = (uint8_t)((tmp & CB_LPS22HB_TDA_MASK)>>CB_LPS22HB_PDA_BIT);
  datastatus->TempDataOverrun = (uint8_t)((tmp & CB_LPS22HB_TOR_MASK)>>CB_LPS22HB_TOR_BIT);
  datastatus->PressDataOverrun = (uint8_t)((tmp & CB_LPS22HB_POR_MASK)>>CB_LPS22HB_POR_BIT);
  
  return CB_NO_ERROR;
}



/**
* @brief  Get the LPS22HB raw pressure value
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/

uint32_t CB_LPS22HB_Get_RawPressure(int32_t *raw_press)
{
  uint8_t buffer[3];
  uint32_t tmp = 0;  
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_PRESS_OUT_XL_REG, 3, buffer))
    return CB_LPS22HB_ERROR;
  
  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
    tmp |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tmp & 0x00800000)
    tmp |= 0xFF000000;
  
  *raw_press = ((int32_t)tmp);
  
  return CB_NO_ERROR;
}

/**
* @brief    Get the LPS22HB Pressure value in hPA.
* @detail   The data are expressed as PRESS_OUT_H/_L/_XL in 2’s complement.
            Pout(hPA)=PRESS_OUT / 4096
* @param      The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Pressure(int32_t* Pout)
{  
  int32_t raw_press;
  
  if(CB_LPS22HB_Get_RawPressure(&raw_press))
    return CB_LPS22HB_ERROR;
  
  *Pout = (raw_press*100)/4096;
  
  return CB_NO_ERROR;
}

/**
* @brief    Get the Raw Temperature value.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*            Tout(degC)=TEMP_OUT/100
* @param     Buffer to empty with the temperature raw tmp.
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_RawTemperature(int16_t* raw_data)
{
  uint8_t buffer[2];
  uint16_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_TEMP_OUT_L_REG, 2, buffer))
    return CB_LPS22HB_ERROR;
  
  /* Build the raw tmp */
  tmp = (((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0];
  
  *raw_data = ((int16_t)tmp);
  
  return CB_NO_ERROR;
}


/**
* @brief    Get the Temperature value in °C.
* @detail   Temperature data are expressed as TEMP_OUT_H&TEMP_OUT_L as 2’s complement number.
*           Tout(degC)=TEMP_OUT/100
* @param Buffer to empty with the temperature value that must be divided by 100 to get the value in °C
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Temperature(int16_t* Tout)
{  
  int16_t raw_data;
  
  if(CB_LPS22HB_Get_RawTemperature(&raw_data))
    return CB_LPS22HB_ERROR;
  
  *Tout = raw_data;
  
  return CB_NO_ERROR;
}

/**
* @brief    Get the threshold value used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.							
* @param    Buffer to empty with the pressure threshold in hPA
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PressureThreshold(int16_t* P_ths)
{
  uint8_t tempReg[2];
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_THS_P_LOW_REG, 2, tempReg))
    return CB_LPS22HB_ERROR;
  
  *P_ths= (((((uint16_t)tempReg[1])<<8) + tempReg[0])/16);
  
  return CB_NO_ERROR;
}

/**
* @brief    Set the threshold value  used for pressure interrupt generation (hPA).
* @detail   THS_P=THS_P_H&THS_P_L and is expressed as unsigned number. P_ths(hPA)=(THS_P)/16.							
* @param 	  Pressure threshold in hPA
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PressureThreshold(int16_t P_ths)
{  
  uint8_t buffer[2];
  
  buffer[0] = (uint8_t)(16 * P_ths);
  buffer[1] = (uint8_t)(((uint16_t)(16 * P_ths))>>8);   
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_THS_P_LOW_REG, 2, buffer))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Set Fifo Mode.
* @param  Fifo Mode struct
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoMode(CB_LPS22HB_FifoMode_et fifomode)
{  
  uint8_t tmp;  
  
 CB_LPS22HB_assert_param(IS_CB_LPS22HB_FifoMode(fifomode));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_FIFO_MODE_MASK;
  tmp |= (uint8_t)fifomode;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief    Get Fifo Mode					
* @param   buffer to empty with fifo mode tmp
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoMode(CB_LPS22HB_FifoMode_et* fifomode)
{  
  uint8_t tmp;  
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= CB_LPS22HB_FIFO_MODE_MASK;
  *fifomode = (CB_LPS22HB_FifoMode_et)tmp;
  
  return CB_NO_ERROR;
}
  
/**
* @brief    Set Fifo Watermark Level.
* @param    Watermark level value [0 31]
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoWatermarkLevel(uint8_t wtmlevel)
{  
  uint8_t tmp; 
  
  CB_LPS22HB_assert_param(IS_CB_LPS22HB_WtmLevel(wtmlevel));
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  tmp &= ~CB_LPS22HB_WTM_POINT_MASK;
  tmp |= wtmlevel;
  
  if(CB_LPS22HB_WriteReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief   Get FIFO Watermark Level
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoWatermarkLevel(uint8_t *wtmlevel)
{  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_FIFO_REG, 1, wtmlevel))
    return CB_LPS22HB_ERROR;
  
  *wtmlevel &= CB_LPS22HB_WTM_POINT_MASK;
  
  return CB_NO_ERROR;
}

/**
* @brief    Get the Fifo Status		
* @param    Status Flag: FIFO_FTH,FIFO_EMPTY,FIFO_FULL,FIFO_OVR and level of the FIFO->FIFO_LEVEL
* @retval   Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoStatus(CB_LPS22HB_FifoStatus_st* status)
{  
  uint8_t tmp;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_STATUS_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  status->FIFO_FTH = (uint8_t)((tmp & CB_LPS22HB_FTH_FIFO_MASK)>>CB_LPS22HB_FTH_FIFO_BIT);
  status->FIFO_OVR=(uint8_t)((tmp & CB_LPS22HB_OVR_FIFO_MASK)>>CB_LPS22HB_OVR_FIFO_BIT);
  status->FIFO_LEVEL = (uint8_t)(tmp & CB_LPS22HB_LEVEL_FIFO_MASK);
  
  if(status->FIFO_LEVEL ==CB_LPS22HB_FIFO_EMPTY)
    status->FIFO_EMPTY=0x01;
  else
    status->FIFO_EMPTY=0x00;
    
  if (status->FIFO_LEVEL ==CB_LPS22HB_FIFO_FULL)
     status->FIFO_FULL=0x01;
  else
    status->FIFO_FULL=0x00;
    
  
  return CB_NO_ERROR;
}

/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param buffer to empty with the he pressure value (hPA)
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PressureOffsetValue(int16_t *pressoffset)
{  
  uint8_t buffer[2];
  int16_t raw_press;  
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_RPDS_L_REG, 2, buffer))
    return CB_LPS22HB_ERROR;
  
  raw_press = (int16_t)((((uint16_t)buffer[1]) << 8) + (uint16_t)buffer[0]);
  
  *pressoffset = (raw_press*100)/4096;	
  
  return CB_NO_ERROR;
}


/**
* @brief  Get the Reference Pressure value 
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  Buffer to empty with reference pressure value
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_ReferencePressure(int32_t* RefP)
{  
  uint8_t buffer[3];
  uint32_t tempVal=0;
  int32_t raw_press;  
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_REF_P_XL_REG, 3, buffer))
    return CB_LPS22HB_ERROR;
  
  /* Build the raw data */
  for(uint8_t i=0; i<3; i++)
    tempVal |= (((uint32_t)buffer[i]) << (8*i));
  
  /* convert the 2's complement 24 bit to 2's complement 32 bit */
  if(tempVal & 0x00800000)
    tempVal |= 0xFF000000;
  
  raw_press =((int32_t)tempVal);
  *RefP = (raw_press*100)/4096;

  
  return CB_NO_ERROR;
}


/**
* @brief  Check if the single measurement has completed.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/  
uint32_t CB_LPS22HB_IsMeasurementCompleted(uint8_t* Is_Measurement_Completed)
{  
  uint8_t tmp;
  CB_LPS22HB_DataStatus_st datastatus;
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_STATUS_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  datastatus.TempDataAvailable=(uint8_t)((tmp&CB_LPS22HB_TDA_MASK)>>CB_LPS22HB_TDA_BIT);
  datastatus.PressDataAvailable= (uint8_t)(tmp&CB_LPS22HB_PDA_MASK);
  
  *Is_Measurement_Completed=(uint8_t)((datastatus.PressDataAvailable) & (datastatus.TempDataAvailable));
  
  return CB_NO_ERROR;
}

/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature tmp
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Measurement(CB_LPS22HB_MeasureTypeDef_st *Measurement_Value)
{  
  int16_t Tout;
  int32_t Pout;
  
  if(CB_LPS22HB_Get_Temperature(&Tout))
    return CB_LPS22HB_ERROR;
  
  Measurement_Value->Tout=Tout;
  
  if(CB_LPS22HB_Get_Pressure(&Pout))
    return CB_LPS22HB_ERROR;
  
  Measurement_Value->Pout=Pout;
  
  return CB_NO_ERROR;
  
}

/**
* @brief  Initialization function for LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic configuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*        NO FIFO; NO Interrupt Enabled.
* @param  None.
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Init(void)
{    
  CB_LPS22HB_ConfigTypeDef_st pLPS22HBInit;
  
  /* Make LPS22HB Reset and Reboot */ 
  if(CB_LPS22HB_SwResetAndMemoryBoot())
    return CB_LPS22HB_ERROR;
  
 pLPS22HBInit.PowerMode=CB_LPS22HB_LowPower;
 pLPS22HBInit.OutputDataRate=CB_LPS22HB_ODR_25HZ;
 pLPS22HBInit.LowPassFilter=CB_LPS22HB_DISABLE;
 pLPS22HBInit.LPF_Cutoff=CB_LPS22HB_ODR_9;
 pLPS22HBInit.BDU=CB_LPS22HB_BDU_NO_UPDATE;
 pLPS22HBInit.IfAddInc=CB_LPS22HB_ENABLE; //default
 pLPS22HBInit.Sim=CB_LPS22HB_SPI_4_WIRE;
 
 /* Set Generic Configuration*/
 if(CB_LPS22HB_Set_GenericConfig(&pLPS22HBInit))
   return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  De initialization function for LPS22HB.
*         This function make a memory boot and clear the data output flags.
* @param  None.
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_DeInit(void)
{    
  CB_LPS22HB_MeasureTypeDef_st Measurement_Value;
  
  /* Make LPS22HB Reset and Reboot */ 
  if(CB_LPS22HB_SwResetAndMemoryBoot())
    return CB_LPS22HB_ERROR;
  
  /* Dump of data output */
  if(CB_LPS22HB_Get_Measurement(& Measurement_Value))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}


/**
* @brief   Set Generic Configuration
* @param   Struct to empty with the chosen values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_GenericConfig(CB_LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{  
   
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
   if(CB_LPS22HB_Set_PowerMode(pxLPS22HBInit->PowerMode))
    return CB_LPS22HB_ERROR;
  
  /* Init the Output Data Rate*/
  if(CB_LPS22HB_Set_Odr(pxLPS22HBInit->OutputDataRate))
    return CB_LPS22HB_ERROR;
  
  /* BDU bit is used to inhibit the output registers update between the reading of upper and
  lower register parts. In default mode (BDU = ‘0’), the lower and upper register parts are
  updated continuously. If it is not sure to read faster than output data rate, it is recommended
  to set BDU bit to ‘1’. In this way, after the reading of the lower (upper) register part, the
  content of that output registers is not updated until the upper (lower) part is read too.
  This feature avoids reading LSB and MSB related to different samples.*/
  
  if(CB_LPS22HB_Set_Bdu(pxLPS22HBInit->BDU))
    return CB_LPS22HB_ERROR;
    
  /*Enable/Disale low-pass filter on LPS22HB pressure data*/
  if(CB_LPS22HB_Set_LowPassFilter(pxLPS22HBInit->LowPassFilter))
    return CB_LPS22HB_ERROR;
  
   /* Set low-pass filter cutoff configuration*/
  if(CB_LPS22HB_Set_LowPassFilterCutoff(pxLPS22HBInit->LPF_Cutoff))
    return CB_LPS22HB_ERROR;
  
  /* SIM bit selects the SPI serial interface mode.*/
  /* This feature has effect only if SPI interface is used*/
 
    if(CB_LPS22HB_Set_SpiInterface(pxLPS22HBInit->Sim))
    return CB_LPS22HB_ERROR;
  
  /*Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)*/  
  if(CB_LPS22HB_Set_AutomaticIncrementRegAddress(pxLPS22HBInit->IfAddInc))
    return CB_LPS22HB_ERROR;
   
    
  return CB_NO_ERROR;
}

/**
* @brief  Get Generic configuration 
* @param  Struct to empty with configuration values
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_GenericConfig(CB_LPS22HB_ConfigTypeDef_st* pxLPS22HBInit)
{
  
  uint8_t tmp;
 
  /*Read CB_LPS22HB_RES_CONF_REG*/
 if(CB_LPS22HB_Get_PowerMode(&pxLPS22HBInit->PowerMode))
   return CB_LPS22HB_ERROR;
  
  /*Read CB_LPS22HB_CTRL_REG1*/
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG1, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  pxLPS22HBInit->OutputDataRate= (CB_LPS22HB_Odr_et)(tmp & CB_LPS22HB_ODR_MASK);
  pxLPS22HBInit->BDU=(CB_LPS22HB_Bdu_et)(tmp & CB_LPS22HB_BDU_MASK);
  pxLPS22HBInit->Sim=(CB_LPS22HB_SPIMode_et)(tmp& CB_LPS22HB_SIM_MASK);
  pxLPS22HBInit->LowPassFilter=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_LPFP_MASK)>>CB_LPS22HB_LPFP_BIT);
  pxLPS22HBInit->LPF_Cutoff=(CB_LPS22HB_LPF_Cutoff_et)(tmp& CB_LPS22HB_LPFP_CUTOFF_MASK);
  
  /*Read CB_LPS22HB_CTRL_REG2*/
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  pxLPS22HBInit->IfAddInc=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_ADD_INC_MASK)>>CB_LPS22HB_ADD_INC_BIT);
 
  return CB_NO_ERROR;
}


/**
* @brief  Set Interrupt configuration 
* @param  Struct holding the configuration values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptConfig(CB_LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
  /* Enable Low Current Mode (low Power) or Low Noise Mode*/
   if(CB_LPS22HB_Set_InterruptActiveLevel(pLPS22HBInt->INT_H_L))
    return CB_LPS22HB_ERROR;
   
   /* Push-pull/open drain selection on interrupt pads.*/
   if(CB_LPS22HB_Set_InterruptOutputType(pLPS22HBInt->PP_OD))
    return CB_LPS22HB_ERROR;
   
   /* Set Data signal on INT pad control bits.*/
   if(CB_LPS22HB_Set_InterruptControlConfig(pLPS22HBInt->OutputSignal_INT))
    return CB_LPS22HB_ERROR;
   
   /* Enable/Disable Data-ready signal on INT_DRDY pin. */
   if(CB_LPS22HB_Set_DRDYInterrupt(pLPS22HBInt->DRDY))
    return CB_LPS22HB_ERROR;
   
    /* Enable/Disable FIFO overrun interrupt on INT_DRDY pin. */
   if(CB_LPS22HB_Set_FIFO_OVR_Interrupt(pLPS22HBInt->FIFO_OVR))
    return CB_LPS22HB_ERROR;
   
   /* Enable/Disable FIFO Treshold interrupt on INT_DRDY pin. */
   if(CB_LPS22HB_Set_FIFO_FTH_Interrupt(pLPS22HBInt->FIFO_FTH))
    return CB_LPS22HB_ERROR;
   
   /* Enable/Disable FIFO FULL interrupt on INT_DRDY pin. */
   if(CB_LPS22HB_Set_FIFO_FULL_Interrupt(pLPS22HBInt->FIFO_FULL))
    return CB_LPS22HB_ERROR;
  
  /* Latch Interrupt request to the INT_SOURCE register. */
    if(CB_LPS22HB_LatchInterruptRequest(pLPS22HBInt->LatchIRQ))
      return CB_LPS22HB_ERROR;
   
    /* Set the threshold value  used for pressure interrupt generation (hPA). */
   if(CB_LPS22HB_Set_PressureThreshold(pLPS22HBInt->THS_threshold))
      return CB_LPS22HB_ERROR;
   
   /*Enable/Disable  AutoRifP function */
  if(pLPS22HBInt->AutoRifP==CB_LPS22HB_ENABLE){
    if(CB_LPS22HB_Set_AutoRifP())
      return CB_LPS22HB_ERROR;
  }
  else{
    if(CB_LPS22HB_ResetAutoRifP())
      return CB_LPS22HB_ERROR;
  }
  
  /*Enable/Disable AutoZero function*/
  if(pLPS22HBInt->AutoZero==CB_LPS22HB_ENABLE){
    if(CB_LPS22HB_Set_AutoZeroFunction())
      return CB_LPS22HB_ERROR;
  }
  else{
    if(CB_LPS22HB_ResetAutoZeroFunction())
      return CB_LPS22HB_ERROR;
  }
  
   
   if(pLPS22HBInt->OutputSignal_INT==CB_LPS22HB_P_HIGH)
   {
    /* Enable\Disable Interrupt Generation on differential pressure high event*/
      if(CB_LPS22HB_Set_PHE(CB_LPS22HB_ENABLE))
        return CB_LPS22HB_ERROR;
       if(CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_ENABLE))
          return CB_LPS22HB_ERROR;
   }
   else  if(pLPS22HBInt->OutputSignal_INT==CB_LPS22HB_P_LOW)
      {
    /* Enable Interrupt Generation on differential pressure Loe event*/
      if(CB_LPS22HB_Set_PLE(CB_LPS22HB_ENABLE))
        return CB_LPS22HB_ERROR;
       if(CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_ENABLE))
          return CB_LPS22HB_ERROR;
   }
    else  if(pLPS22HBInt->OutputSignal_INT==CB_LPS22HB_P_LOW_HIGH)
    {
      /* Enable Interrupt Generation on differential pressure high event*/
      if(CB_LPS22HB_Set_PHE(CB_LPS22HB_ENABLE))
        return CB_LPS22HB_ERROR;
    /* Enable\Disable Interrupt Generation on differential pressure Loe event*/
      if(CB_LPS22HB_Set_PLE(CB_LPS22HB_ENABLE))
        return CB_LPS22HB_ERROR;
       if(CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_ENABLE))
          return CB_LPS22HB_ERROR;
   }
   else
   {
      if(CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_DISABLE))
          return CB_LPS22HB_ERROR;
      /* Disable Interrupt Generation on differential pressure High event*/
      if(CB_LPS22HB_Set_PHE(CB_LPS22HB_DISABLE))
        return CB_LPS22HB_ERROR;
     /* Disable Interrupt Generation on differential pressure Low event*/
      if(CB_LPS22HB_Set_PLE(CB_LPS22HB_DISABLE))
        return CB_LPS22HB_ERROR;
   }
  
  return CB_NO_ERROR;
}

/**
* @brief  LPS22HBGet_InterruptConfig 
* @param  Struct to empty with configuration values
* @retval S Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptConfig(CB_LPS22HB_InterruptTypeDef_st* pLPS22HBInt)
{
   uint8_t tmp;
  
  /*Read CB_LPS22HB_CTRL_REG3*/
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG3, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  pLPS22HBInt->INT_H_L= (CB_LPS22HB_InterruptActiveLevel_et)(tmp & CB_LPS22HB_INT_H_L_MASK);
  pLPS22HBInt->PP_OD=(CB_LPS22HB_OutputType_et)(tmp & CB_LPS22HB_PP_OD_MASK);
  pLPS22HBInt->OutputSignal_INT=(CB_LPS22HB_OutputSignalConfig_et)(tmp& CB_LPS22HB_INT_S12_MASK);
  pLPS22HBInt->DRDY=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_DRDY_MASK)>>CB_LPS22HB_DRDY_BIT);
  pLPS22HBInt->FIFO_OVR=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_FIFO_OVR_MASK)>>CB_LPS22HB_FIFO_OVR_BIT);
  pLPS22HBInt->FIFO_FTH=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_FIFO_FTH_MASK)>>CB_LPS22HB_FIFO_FTH_BIT);
  pLPS22HBInt->FIFO_FULL=(CB_LPS22HB_State_et)((tmp& CB_LPS22HB_FIFO_FULL_MASK)>>CB_LPS22HB_FIFO_FULL_BIT);
  
  /*Read CB_LPS22HB_INTERRUPT_CFG_REG*/
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_INTERRUPT_CFG_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;

  pLPS22HBInt->LatchIRQ= (CB_LPS22HB_State_et)((tmp& CB_LPS22HB_LIR_MASK)>>CB_LPS22HB_LIR_BIT);
  
  if(CB_LPS22HB_Get_PressureThreshold(&pLPS22HBInt->THS_threshold))
    return CB_LPS22HB_ERROR;
     
  //AutoRifP and Autozero are self clear //
  pLPS22HBInt->AutoRifP=CB_LPS22HB_DISABLE;
  pLPS22HBInt->AutoZero=CB_LPS22HB_DISABLE;
  
  return CB_NO_ERROR;
}

/**
* @brief  Set Fifo configuration 
* @param  Struct holding the configuration values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoConfig(CB_LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{
  
   if(pLPS22HBFIFO->FIFO_MODE == CB_LPS22HB_FIFO_BYPASS_MODE) {
    /* FIFO Disable-> FIFO_EN bit=0 in CTRL_REG2*/
    if(CB_LPS22HB_Set_FifoModeUse(CB_LPS22HB_DISABLE))
      return CB_LPS22HB_ERROR;
    /* Force->Disable FIFO Watermark Level Use*/ 
     if(CB_LPS22HB_Set_FifoWatermarkLevelUse(CB_LPS22HB_DISABLE))
          return CB_LPS22HB_ERROR;
          
    /* Force->Disable FIFO Treshold interrupt on INT_DRDY pin. */
     if(CB_LPS22HB_Set_FIFO_FTH_Interrupt(CB_LPS22HB_DISABLE))
            return CB_LPS22HB_ERROR;
  } 
  else {
    /* FIFO Enable-> FIFO_EN bit=1 in CTRL_REG2*/
    if(CB_LPS22HB_Set_FifoModeUse(CB_LPS22HB_ENABLE))
      return CB_LPS22HB_ERROR;
   
      if (pLPS22HBFIFO->WTM_INT){
        /* Enable FIFO Watermark Level Use*/ 
        if(CB_LPS22HB_Set_FifoWatermarkLevelUse(CB_LPS22HB_ENABLE))
          return CB_LPS22HB_ERROR;
        /*Set Fifo Watermark Level*/
        if(CB_LPS22HB_Set_FifoWatermarkLevel(pLPS22HBFIFO->WTM_LEVEL))
          return CB_LPS22HB_ERROR;
        /* Force->Enable FIFO Treshold interrupt on INT_DRDY pin. */
        if(CB_LPS22HB_Set_FIFO_FTH_Interrupt(CB_LPS22HB_ENABLE))
            return CB_LPS22HB_ERROR;
      } 
  }
  
  if(CB_LPS22HB_Set_FifoMode(pLPS22HBFIFO->FIFO_MODE))
    return CB_LPS22HB_ERROR;
  
  return CB_NO_ERROR;
}

/**
* @brief  Get Fifo configuration 
* @param  Struct to empty with the configuration values
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoConfig(CB_LPS22HB_FIFOTypeDef_st* pLPS22HBFIFO)
{
   uint8_t tmp;
    
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_FIFO_REG, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
  /*!< Fifo Mode Selection */
  pLPS22HBFIFO->FIFO_MODE= (CB_LPS22HB_FifoMode_et)(tmp& CB_LPS22HB_FIFO_MODE_MASK);
  
  /*!< FIFO threshold/Watermark level selection*/
  pLPS22HBFIFO->WTM_LEVEL= (uint8_t)(tmp& CB_LPS22HB_WTM_POINT_MASK);
  
  if(CB_LPS22HB_ReadReg(CB_LPS22HB_CTRL_REG2, 1, &tmp))
    return CB_LPS22HB_ERROR;
  
   /*!< Enable/Disable the watermark interrupt*/
  pLPS22HBFIFO->WTM_INT= (CB_LPS22HB_State_et)((tmp& CB_LPS22HB_WTM_EN_MASK)>>CB_LPS22HB_WTM_EN_BIT);
  
  
  return CB_NO_ERROR;
}

#ifdef  USE_FULL_ASSERT_LPS22HB
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval : None
*/
void CB_LPS22HB_assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
  printf("Wrong parameters tmp: file %s on line %d\r\n", file, line);
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
