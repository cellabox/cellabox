/**
******************************************************************************
* @file    cb_CB_LPS22HB.h
* @author  HESA Application Team
* @version 1.0.0
* @date    15/01/2015
* @brief   CB_LPS22HB driver header file

* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
* TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
* DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
* FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
* CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
* THIS SOURCE CODE IS PROTECTED BY A LICENSE.
* FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
* IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
*
* <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>

******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CB_LPS22HB_H__
#define CB_LPS22HB_H__

#include <stdint.h>
// the user must include the proper file where HAL_ReadReg and HAL_WriteReg are implemented
//#include "HAL_EnvSensors.h"

/* Uncomment the line below to expanse the "assert_param" macro in the  drivers code */
//#define USE_FULL_ASSERT_CB_LPS22HB

/* Exported macro ------------------------------------------------------------*/
#ifdef  USE_FULL_ASSERT_CB_LPS22HB

/**
* @brief  The assert_param macro is used for function's parameters check.
* @param  expr: If expr is false, it calls assert_failed function which reports 
*         the name of the source file and the source line number of the call 
*         that failed. If expr is true, it returns no value.
* @retval None
*/
#define CB_LPS22HB_assert_param(expr) ((expr) ? (void)0 : CB_LPS22HB_assert_failed((uint8_t *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
void CB_LPS22HB_assert_failed(uint8_t* file, uint32_t line);
#else
#define CB_LPS22HB_assert_param(expr) ((void)0)
#endif /* USE_FULL_ASSERT_CB_LPS22HB */


#ifdef __cplusplus
extern "C" {
#endif
  
  /** @addtogroup Environmental_Sensor
  * @{
  */
  
  /** @addtogroup CB_LPS22HB_DRIVER
  * @{
  */
  
  /* Exported Types -------------------------------------------------------------*/
  /** @defgroup CB_LPS22HB_Exported_Types
  * @{
  */
  
  /**
  * @brief  Enable/Disable type. 
  */
  typedef enum {CB_LPS22HB_DISABLE = (uint8_t)0, CB_LPS22HB_ENABLE = !CB_LPS22HB_DISABLE} CB_LPS22HB_State_et;
#define IS_CB_LPS22HB_State(MODE) ((MODE == CB_LPS22HB_ENABLE) || (MODE == CB_LPS22HB_DISABLE) )
  
  /**
  * @brief  Bit status type. 
  */
  typedef enum {CB_LPS22HB_RESET = (uint8_t)0, CB_LPS22HB_SET = !CB_LPS22HB_RESET} CB_LPS22HB_BitStatus_et;
#define IS_CB_LPS22HB_BitStatus(MODE) ((MODE == CB_LPS22HB_RESET) || (MODE == CB_LPS22HB_SET))
  
  /*RES_CONF see LC_EN bit*/
 /**
* @brief  CB_LPS22HB Power/Noise Mode configuration.
*/
typedef enum {
  CB_LPS22HB_LowNoise   =  (uint8_t)0x00,       /*!< Low Noise mode */
  CB_LPS22HB_LowPower   =  (uint8_t)0x01        /*!< Low Current mode */
} CB_LPS22HB_PowerMode_et;

#define IS_CB_LPS22HB_PowerMode(MODE) ((MODE == CB_LPS22HB_LowNoise) || (MODE == CB_LPS22HB_LowPower))

/**
* @brief  Output data rate configuration.
*/
typedef enum {  
  
  CB_LPS22HB_ODR_ONE_SHOT  = (uint8_t)0x00,         /*!< Output Data Rate: one shot */
  CB_LPS22HB_ODR_1HZ       = (uint8_t)0x10,         /*!< Output Data Rate: 1Hz */
  CB_LPS22HB_ODR_10HZ       = (uint8_t)0x20,         /*!< Output Data Rate: 10Hz */
  CB_LPS22HB_ODR_25HZ    = (uint8_t)0x30,         /*!< Output Data Rate: 25Hz */
  CB_LPS22HB_ODR_50HZ      = (uint8_t)0x40,          /*!< Output Data Rate: 50Hz */
  CB_LPS22HB_ODR_75HZ      = (uint8_t)0x50          /*!< Output Data Rate: 75Hz */
} CB_LPS22HB_Odr_et;

#define IS_CB_LPS22HB_ODR(ODR) ((ODR == CB_LPS22HB_ODR_ONE_SHOT) || (ODR == CB_LPS22HB_ODR_1HZ) || \
(ODR == CB_LPS22HB_ODR_10HZ) || (ODR == CB_LPS22HB_ODR_25HZ)|| (ODR == CB_LPS22HB_ODR_50HZ) || (ODR == CB_LPS22HB_ODR_75HZ))

/**
* @brief  Low Pass Filter Cutoff Configuration.
*/
typedef enum {  
  
  CB_LPS22HB_ODR_9  = (uint8_t)0x00,         /*!< Filter Cutoff ODR/9 */
  CB_LPS22HB_ODR_20 = (uint8_t)0x04          /*!< Filter Cutoff ODR/20 */
} CB_LPS22HB_LPF_Cutoff_et;

#define IS_CB_LPS22HB_LPF_Cutoff(CUTOFF) ((CUTOFF == CB_LPS22HB_ODR_9) || (CUTOFF == CB_LPS22HB_ODR_20) )

/**
* @brief  Block data update.
*/

typedef enum {
  CB_LPS22HB_BDU_CONTINUOUS_UPDATE     =  (uint8_t)0x00,  /*!< Data updated continuously */
  CB_LPS22HB_BDU_NO_UPDATE             =  (uint8_t)0x02   /*!< Data updated after a read operation */
} CB_LPS22HB_Bdu_et;
#define IS_CB_LPS22HB_BDUMode(MODE) ((MODE == CB_LPS22HB_BDU_CONTINUOUS_UPDATE) || (MODE == CB_LPS22HB_BDU_NO_UPDATE))
  
/**
* @brief  CB_LPS22HB Spi Mode configuration.
*/
typedef enum {
  CB_LPS22HB_SPI_4_WIRE   =  (uint8_t)0x00,
  CB_LPS22HB_SPI_3_WIRE   =  (uint8_t)0x01
} CB_LPS22HB_SPIMode_et;

#define IS_CB_LPS22HB_SPIMode(MODE) ((MODE == CB_LPS22HB_SPI_4_WIRE) || (MODE == CB_LPS22HB_SPI_3_WIRE))


/**
* @brief  CB_LPS22HB Interrupt Active Level Configuration (on High or Low)
*/
typedef enum
{
  CB_LPS22HB_ActiveHigh = (uint8_t)0x00,
  CB_LPS22HB_ActiveLow  = (uint8_t)0x80
}CB_LPS22HB_InterruptActiveLevel_et;
#define IS_CB_LPS22HB_InterruptActiveLevel(MODE) ((MODE == CB_LPS22HB_ActiveHigh) || (MODE == CB_LPS22HB_ActiveLow))

/**
* @brief  CB_LPS22HB Push-pull/Open Drain selection on Interrupt pads.
*/
typedef enum
{
  CB_LPS22HB_PushPull = (uint8_t)0x00,
  CB_LPS22HB_OpenDrain  = (uint8_t)0x40
}CB_LPS22HB_OutputType_et;
#define IS_CB_LPS22HB_OutputType(MODE) ((MODE == CB_LPS22HB_PushPull) || (MODE == CB_LPS22HB_OpenDrain))


/**
* @brief  Data Signal on INT pad control bits.
*/
typedef enum
{
  CB_LPS22HB_DATA = (uint8_t)0x00,
  CB_LPS22HB_P_HIGH = (uint8_t)0x01,
  CB_LPS22HB_P_LOW = (uint8_t)0x02,
  CB_LPS22HB_P_LOW_HIGH = (uint8_t)0x03
}CB_LPS22HB_OutputSignalConfig_et;
#define IS_CB_LPS22HB_OutputSignal(MODE) ((MODE == CB_LPS22HB_DATA) || (MODE == CB_LPS22HB_P_HIGH)||\
(MODE == CB_LPS22HB_P_LOW) || (MODE == CB_LPS22HB_P_LOW_HIGH))



/**
* @brief  CB_LPS22HB Interrupt Differential Status.
*/

typedef struct
{
  uint8_t PH;          /*!< High Differential Pressure event occured */ 
  uint8_t PL;          /*!< Low Differential Pressure event occured */  
  uint8_t IA;          /*!< One or more interrupt events have been  generated.Interrupt Active */ 
  uint8_t BOOT;        /*!< i '1' indicates that the Boot (Reboot) phase is running */ 
}CB_LPS22HB_InterruptDiffStatus_st;


/**
* @brief  CB_LPS22HB Pressure and Temperature data status.
*/
typedef struct
{
  uint8_t TempDataAvailable;           /*!< Temperature data available bit */
  uint8_t PressDataAvailable;          /*!< Pressure data available bit */  
  uint8_t TempDataOverrun;             /*!< Temperature data over-run bit */ 
  uint8_t PressDataOverrun;            /*!< Pressure data over-run bit */  
}CB_LPS22HB_DataStatus_st;


/**
* @brief  CB_LPS22HB Clock Tree  configuration.
*/
typedef enum {
  CB_LPS22HB_CTE_NotBalanced   =  (uint8_t)0x00,
  CB_LPS22HB_CTE_Balanced   =  (uint8_t)0x20
} CB_LPS22HB_CTE_et;

#define IS_CB_LPS22HB_CTE(MODE) ((MODE == CB_LPS22HB_CTE_NotBalanced) || (MODE == CB_LPS22HB_CTE_Balanced))

/**
* @brief  CB_LPS22HB Fifo Mode.
*/

typedef enum {
  CB_LPS22HB_FIFO_BYPASS_MODE             	      = (uint8_t)0x00,	  /*!< The FIFO is disabled and empty. The pressure is read directly*/
  CB_LPS22HB_FIFO_MODE                           = (uint8_t)0x20,    /*!< Stops collecting data when full */
  CB_LPS22HB_FIFO_STREAM_MODE                    = (uint8_t)0x40,    /*!< Keep the newest measurements in the FIFO*/
  CB_LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE      = (uint8_t)0x60,    /*!< STREAM MODE until trigger deasserted, then change to FIFO MODE*/
  CB_LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE    = (uint8_t)0x80,    /*!< BYPASS MODE until trigger deasserted, then STREAM MODE*/
  CB_LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE      = (uint8_t)0xE0     /*!< BYPASS mode until trigger deasserted, then FIFO MODE*/
} CB_LPS22HB_FifoMode_et;

#define IS_CB_LPS22HB_FifoMode(MODE) ((MODE == CB_LPS22HB_FIFO_BYPASS_MODE) || (MODE ==CB_LPS22HB_FIFO_MODE)||\
(MODE == CB_LPS22HB_FIFO_STREAM_MODE) || (MODE == CB_LPS22HB_FIFO_TRIGGER_STREAMTOFIFO_MODE)||\
  (MODE == CB_LPS22HB_FIFO_TRIGGER_BYPASSTOSTREAM_MODE) ||  (MODE == CB_LPS22HB_FIFO_TRIGGER_BYPASSTOFIFO_MODE))


/**
* @brief  CB_LPS22HB Fifo Satus.
*/
typedef struct {
  uint8_t FIFO_LEVEL;          /*!< FIFO Stored data level: 00000: FIFO empty; 10000: FIFO is FULL and ha 32 unread samples  */ 
  uint8_t FIFO_EMPTY;          /*!< Empty FIFO Flag .1 FIFO is empty (see FIFO_level)	*/  
  uint8_t FIFO_FULL;          /*!< Full FIFO flag.1 FIFO is Full (see FIFO_level)	*/  
  uint8_t FIFO_OVR;           /*!< Overrun bit status. 1 FIFO is full and at least one sample in the FIFO has been overwritten */ 
  uint8_t FIFO_FTH;            /*!< FIFO Threshold (Watermark) Status. 1 FIFO filling is equal or higher then FTH (wtm) level.*/ 
}CB_LPS22HB_FifoStatus_st;



/**
* @brief  CB_LPS22HB Configuration structure definition.
*/
typedef struct
{
  CB_LPS22HB_PowerMode_et   PowerMode;                    /*!< Enable Low Current Mode (low Power) or Low Noise Mode*/
  CB_LPS22HB_Odr_et         OutputDataRate;                /*!< Output Data Rate */
  CB_LPS22HB_Bdu_et         BDU;                	        /*!< Enable to inhibit the output registers update between the reading of upper and lower register parts.*/
  CB_LPS22HB_State_et   	 LowPassFilter;		        /*!< Enable/ Disable Low Pass Filter */
  CB_LPS22HB_LPF_Cutoff_et  LPF_Cutoff;                    /*!< Low Pass Filter Configuration */
  CB_LPS22HB_SPIMode_et 	 Sim;  			        /*!< SPI Serial Interface Mode selection */
  CB_LPS22HB_State_et       IfAddInc;                       /*!< Enable/Disable Register address automatically inceremented during a multiple byte access */
}CB_LPS22HB_ConfigTypeDef_st;


  /**
* @brief  CB_LPS22HB Interrupt structure definition .
*/
typedef struct {
  CB_LPS22HB_InterruptActiveLevel_et	INT_H_L;         	/*!< Interrupt active high, low. Default value: 0 */
  CB_LPS22HB_OutputType_et 			PP_OD; 		        /*!< Push-pull/open drain selection on interrupt pads. Default value: 0 */
  CB_LPS22HB_OutputSignalConfig_et		OutputSignal_INT;	/*!< Data signal on INT Pad: Data,Pressure High, Preessure Low,P High or Low*/
  CB_LPS22HB_State_et            		DRDY;             	/*!< Enable/Disable Data Ready Interrupt on INT_DRDY Pin*/
  CB_LPS22HB_State_et           		FIFO_OVR;         	/*!< Enable/Disable FIFO Overrun Interrupt on INT_DRDY Pin*/
  CB_LPS22HB_State_et            		FIFO_FTH;        	/*!< Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin.*/
  CB_LPS22HB_State_et             		FIFO_FULL;       	/*!< Enable/Disable FIFO FULL interrupt on INT_DRDY pin.*/
  CB_LPS22HB_State_et					LatchIRQ;			/*!< Latch Interrupt request in to INT_SOURCE reg*/
  int16_t 							THS_threshold;		/*!< Threshold value for pressure interrupt generation*/
  CB_LPS22HB_State_et       			AutoRifP;         	/*!< Enable/Disable  AutoRifP function */
  CB_LPS22HB_State_et       			AutoZero;    		/*!< Enable/Disable  AutoZero function */
}CB_LPS22HB_InterruptTypeDef_st;

/**
* @brief  CB_LPS22HB FIFO structure definition.
*/
typedef struct {
  CB_LPS22HB_FifoMode_et 	FIFO_MODE;   	/*!< Fifo Mode Selection */
  CB_LPS22HB_State_et		WTM_INT; 		/*!< Enable/Disable the watermark interrupt*/
  uint8_t 				WTM_LEVEL;		/*!< FIFO threshold/Watermark level selection*/
}CB_LPS22HB_FIFOTypeDef_st;

#define IS_CB_LPS22HB_WtmLevel(LEVEL) ((LEVEL > 0) && (LEVEL <=31))
/**
* @brief  CB_LPS22HB Measure Type definition.
*/  
typedef struct {
  int16_t Tout;
  int32_t Pout;
}CB_LPS22HB_MeasureTypeDef_st;


/**
* @brief  CB_LPS22HB Driver Version Info structure definition.
*/   
typedef struct {
  uint8_t   Major;
  uint8_t   Minor;
  uint8_t Point; 
}CB_LPS22HB_DriverVersion_st;


/**
* @brief  Bitfield positioning.
*/
#define CB_LPS22HB_BIT(x) ((uint8_t)x)

/**
* @brief  I2C address.
*/
/* SD0/SA0(pin 5) is connected to the voltage supply*/
//#define CB_LPS22HB_ADDRESS  (uint8_t)0x5CD
/*SDO/SA0 (pin5) is connected to the GND*/
#define CB_LPS22HB_ADDRESS  (uint8_t)0x5C

/**
* @brief  Set the CB_LPS22HB driver version.
*/

#define CB_LPS22HB_DriverVersion_Major (uint8_t)1
#define CB_LPS22HB_DriverVersion_Minor (uint8_t)0
#define CB_LPS22HB_DriverVersion_Point (uint8_t)0

/**
* @}
*/


/* Exported Constants ---------------------------------------------------------*/	 
/** @defgroup CB_LPS22HB_Exported_Constants
* @{
*/


/**
* @addtogroup CB_LPS22HB_Register
* @{
*/



/**
* @brief Device Identification register.
* \code
* Read
* Default value: 0xB1
* 7:0 This read-only register contains the device identifier that, for CB_LPS22HB, is set to B1h.
* \endcode
*/

#define CB_LPS22HB_WHO_AM_I_REG         (uint8_t)0x0F

/**
* @brief Device Identification value.
*/
#define CB_LPS22HB_WHO_AM_I_VAL         (uint8_t)0xB1


/**
* @brief Reference Pressure  Register(LSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL7-0: Lower part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define CB_LPS22HB_REF_P_XL_REG         (uint8_t)0x15


/**
* @brief Reference Pressure Register (Middle data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL15-8: Middle part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define CB_LPS22HB_REF_P_L_REG          (uint8_t)0x16

/**
* @brief Reference Pressure Register (MSB data)
* \code
* Read/write
* Default value: 0x00
* 7:0 REFL23-16 Higest part of the reference pressure value that
*      is sum to the sensor output pressure.
* \endcode
*/
#define CB_LPS22HB_REF_P_H_REG          (uint8_t)0x17


/**
* @brief Pressure and temperature resolution mode Register
* \code
* Read/write
* Default value: 0x05
* 7:2 These bits must be set to 0 for proper operation of the device
* 1: Reserved
* 0 LC_EN: Low Current Mode Enable. Default 0
* \endcode
*/
#define CB_LPS22HB_RES_CONF_REG     (uint8_t)0x1A

#define CB_LPS22HB_LCEN_MASK        (uint8_t)0x01

/**
* @brief Control Register 1
* \code
* Read/write
* Default value: 0x00
* 7: This bit must be set to 0 for proper operation of the device
* 6:4 ODR2, ODR1, ODR0: output data rate selection.Default 000
*     ODR2  | ODR1  | ODR0  | Pressure output data-rate(Hz)  | Pressure output data-rate(Hz)
*   ----------------------------------------------------------------------------------
*      0    |  0    |  0    |         one shot               |         one shot 
*      0    |  0    |  1    |            1                   |            1 
*      0    |  1    |  0    |            10                  |           10     
*      0    |  1    |  1    |            25                  |           25  
*      1    |  0    |  0    |            50                  |           50    
*      1    |  0    |  1    |            75                  |         75  
*      1    |  1    |  0    |         Reserved               |         Reserved 
*      1    |  1    |  1    |         Reserved               |         Reserved
*
* 3 EN_LPFP: Enable Low Pass filter on Pressure data. Default value:0
* 2:LPF_CFG Low-pass configuration register. (0: Filter cutoff is ODR/9; 1: filter cutoff is ODR/20)
* 1 BDU: block data update. 0 - continuous update; 1 - output registers not updated until MSB and LSB reading.
* 0 SIM: SPI Serial Interface Mode selection. 0 - SPI 4-wire; 1 - SPI 3-wire 
* \endcode
*/
#define CB_LPS22HB_CTRL_REG1      (uint8_t)0x10

#define CB_LPS22HB_ODR_MASK                (uint8_t)0x70
#define CB_LPS22HB_LPFP_MASK               (uint8_t)0x08
#define CB_LPS22HB_LPFP_CUTOFF_MASK        (uint8_t)0x04
#define CB_LPS22HB_BDU_MASK                (uint8_t)0x02
#define CB_LPS22HB_SIM_MASK                (uint8_t)0x01

#define CB_LPS22HB_LPFP_BIT    CB_LPS22HB_BIT(3)


/**
* @brief Control  Register 2
* \code
* Read/write
* Default value: 0x10
* 7 BOOT:  Reboot memory content. 0: normal mode; 1: reboot memory content. Self-clearing upon completation
* 6 FIFO_EN: FIFO Enable. 0: disable; 1:  enable
* 5 STOP_ON_FTH: Stop on FIFO Threshold  FIFO Watermark level use. 0: disable; 1: enable
* 4 IF_ADD_INC: Register address automatically incrementeed during a multiple byte access with a serial interface (I2C or SPI). Default value 1.( 0: disable; 1: enable)
* 3 I2C DIS:  Disable I2C interface 0: I2C Enabled; 1: I2C disabled
* 2 SWRESET: Software reset. 0: normal mode; 1: SW reset. Self-clearing upon completation
* 1 AUTO_ZERO: Autozero enable. 0: normal mode; 1: autozero enable.
* 0 ONE_SHOT: One shot enable. 0: waiting for start of conversion; 1: start for a new dataset
* \endcode
*/
#define CB_LPS22HB_CTRL_REG2      (uint8_t)0x11

#define CB_LPS22HB_BOOT_BIT       CB_LPS22HB_BIT(7)
#define CB_LPS22HB_FIFO_EN_BIT    CB_LPS22HB_BIT(6)
#define CB_LPS22HB_WTM_EN_BIT     CB_LPS22HB_BIT(5)
#define CB_LPS22HB_ADD_INC_BIT    CB_LPS22HB_BIT(4)
#define CB_LPS22HB_I2C_BIT        CB_LPS22HB_BIT(3)
#define CB_LPS22HB_SW_RESET_BIT   CB_LPS22HB_BIT(2)

#define CB_LPS22HB_FIFO_EN_MASK   (uint8_t)0x40
#define CB_LPS22HB_WTM_EN_MASK    (uint8_t)0x20
#define CB_LPS22HB_ADD_INC_MASK   (uint8_t)0x10
#define CB_LPS22HB_I2C_MASK       (uint8_t)0x08
#define CB_LPS22HB_ONE_SHOT_MASK  (uint8_t)0x01


/**
* @brief CTRL Reg3 Interrupt Control Register
* \code
* Read/write
* Default value: 0x00
* 7 INT_H_L: Interrupt active high, low. 0:active high; 1: active low.
* 6 PP_OD: Push-Pull/OpenDrain selection on interrupt pads. 0: Push-pull; 1: open drain.
* 5 F_FSS5: FIFO full flag on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 4 F_FTH: FIFO threshold (watermark) status on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 3 F_OVR: FIFO overrun interrupt on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 2 DRDY: Data-ready signal on INT_DRDY pin. Defaul value: 0. (0: Diasable; 1 : Enable).
* 1:0 INT_S2, INT_S1: data signal on INT pad control bits.
*    INT_S2  | INT_S1  | INT pin
*   ------------------------------------------------------
*        0       |      0      |     Data signal( in order of priority:PTH_DRDY or F_FTH or F_OVR_or F_FSS5
*        0       |      1      |     Pressure high (P_high) 
*        1       |      0      |     Pressure low (P_low)     
*        1       |      1      |     P_low OR P_high 
* \endcode
*/
#define CB_LPS22HB_CTRL_REG3      (uint8_t)0x12

#define CB_LPS22HB_PP_OD_BIT       CB_LPS22HB_BIT(6)
#define CB_LPS22HB_FIFO_FULL_BIT   CB_LPS22HB_BIT(5)
#define CB_LPS22HB_FIFO_FTH_BIT    CB_LPS22HB_BIT(4)
#define CB_LPS22HB_FIFO_OVR_BIT    CB_LPS22HB_BIT(3)
#define CB_LPS22HB_DRDY_BIT        CB_LPS22HB_BIT(2)


#define CB_LPS22HB_INT_H_L_MASK            (uint8_t)0x80
#define CB_LPS22HB_PP_OD_MASK              (uint8_t)0x40
#define CB_LPS22HB_FIFO_FULL_MASK          (uint8_t)0x20
#define CB_LPS22HB_FIFO_FTH_MASK           (uint8_t)0x10
#define CB_LPS22HB_FIFO_OVR_MASK           (uint8_t)0x08
#define CB_LPS22HB_DRDY_MASK               (uint8_t)0x04
#define CB_LPS22HB_INT_S12_MASK            (uint8_t)0x03


/**
* @brief Interrupt Differential configuration Register
* \code
* Read/write
* Default value: 0x00.
* 7 AUTORIFP: AutoRifP Enable ??
* 6 RESET_ARP: Reset AutoRifP function
* 4 AUTOZERO: Autozero enabled
* 5 RESET_AZ: Reset Autozero Function
* 3 DIFF_EN: Interrupt generation enable
* 2 LIR: Latch Interrupt request into INT_SOURCE register. 0 - interrupt request not latched; 1 - interrupt request latched
* 1 PL_E: Enable interrupt generation on differential pressure low event. 0 - disable; 1 - enable
* 0 PH_E: Enable interrupt generation on differential pressure high event. 0 - disable; 1 - enable
* \endcode
*/
#define CB_LPS22HB_INTERRUPT_CFG_REG  (uint8_t)0x0B

#define CB_LPS22HB_DIFF_EN_BIT       CB_LPS22HB_BIT(3)
#define CB_LPS22HB_LIR_BIT           CB_LPS22HB_BIT(2)
#define CB_LPS22HB_PLE_BIT           CB_LPS22HB_BIT(1)
#define CB_LPS22HB_PHE_BIT           CB_LPS22HB_BIT(0)

#define CB_LPS22HB_AUTORIFP_MASK     (uint8_t)0x80
#define CB_LPS22HB_RESET_ARP_MASK    (uint8_t)0x40
#define CB_LPS22HB_AUTOZERO_MASK     (uint8_t)0x20
#define CB_LPS22HB_RESET_AZ_MASK     (uint8_t)0x10
#define CB_LPS22HB_DIFF_EN_MASK      (uint8_t)0x08
#define CB_LPS22HB_LIR_MASK          (uint8_t)0x04
#define CB_LPS22HB_PLE_MASK          (uint8_t)0x02
#define CB_LPS22HB_PHE_MASK          (uint8_t)0x01



/**
* @brief Interrupt source Register (It is cleared by reading it)
* \code
* Read
* Default value: ----.
* 7 BOOT_STATUS:  If 1 indicates that the Boot (Reboot) phase is running.
* 6:3 Reserved: Keep these bits at 0
* 2 IA: Interrupt Active.0: no interrupt has been generated; 1: one or more interrupt events have been generated.
* 1 PL: Differential pressure Low. 0: no interrupt has been generated; 1: Low differential pressure event has occurred.
* 0 PH: Differential pressure High. 0: no interrupt has been generated; 1: High differential pressure event has occurred.
* \endcode
*/
#define CB_LPS22HB_INTERRUPT_SOURCE_REG   (uint8_t)0x25

#define CB_LPS22HB_BOOT_STATUS_BIT        CB_LPS22HB_BIT(7)
#define CB_LPS22HB_IA_BIT                 CB_LPS22HB_BIT(2)
#define CB_LPS22HB_PL_BIT                 CB_LPS22HB_BIT(1)
#define CB_LPS22HB_PH_BIT                 CB_LPS22HB_BIT(0)

#define CB_LPS22HB_BOOT_STATUS_MASK      (uint8_t)0x80
#define CB_LPS22HB_IA_MASK               (uint8_t)0x04
#define CB_LPS22HB_PL_MASK               (uint8_t)0x02
#define CB_LPS22HB_PH_MASK               (uint8_t)0x01


/**
* @brief  Status Register
* \code
* Read
* Default value: ---
* 7:6 Reserved: 0
* 5 T_OR: Temperature data overrun. 0: no overrun has occurred; 1: a new data for temperature has overwritten the previous one.
* 4 P_OR: Pressure data overrun. 0: no overrun has occurred; 1: new data for pressure has overwritten the previous one.
* 3:2 Reserved: 0
* 1 T_DA: Temperature data available. 0: new data for temperature is not yet available; 1: new data for temperature is available.
* 0 P_DA: Pressure data available. 0: new data for pressure is not yet available; 1: new data for pressure is available.
* \endcode
*/
#define CB_LPS22HB_STATUS_REG         (uint8_t)0x27

#define CB_LPS22HB_TOR_BIT            CB_LPS22HB_BIT(5)
#define CB_LPS22HB_POR_BIT            CB_LPS22HB_BIT(4)
#define CB_LPS22HB_TDA_BIT            CB_LPS22HB_BIT(1)
#define CB_LPS22HB_PDA_BIT            CB_LPS22HB_BIT(0)

#define CB_LPS22HB_TOR_MASK           (uint8_t)0x20
#define CB_LPS22HB_POR_MASK           (uint8_t)0x10
#define CB_LPS22HB_TDA_MASK           (uint8_t)0x02
#define CB_LPS22HB_PDA_MASK           (uint8_t)0x01



/**
* @brief  Pressure data (LSB) register.
* \code
* Read
* Default value: 0x00.(To be verified)
* POUT7 - POUT0: Pressure data LSB (2's complement).     
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/

#define CB_LPS22HB_PRESS_OUT_XL_REG        (uint8_t)0x28
/**
* @brief  Pressure data (Middle part) register.
* \code
* Read
* Default value: 0x80.
* POUT15 - POUT8: Pressure data middle part (2's complement).    
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/ 
#define CB_LPS22HB_PRESS_OUT_L_REG        (uint8_t)0x29

/**
* @brief  Pressure data (MSB) register.
* \code
* Read
* Default value: 0x2F.
* POUT23 - POUT16: Pressure data MSB (2's complement).
* Pressure output data: Pout(hPA)=(PRESS_OUT_H & PRESS_OUT_L &
* PRESS_OUT_XL)[dec]/4096.
* \endcode
*/
#define CB_LPS22HB_PRESS_OUT_H_REG        (uint8_t)0x2A

/**
* @brief  Temperature data (LSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT7 - TOUT0: temperature data LSB. 
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define CB_LPS22HB_TEMP_OUT_L_REG         (uint8_t)0x2B

/**
* @brief  Temperature data (MSB) register.
* \code
* Read
* Default value: 0x00.
* TOUT15 - TOUT8: temperature data MSB. 
* Tout(degC)=TEMP_OUT/100
* \endcode
*/
#define CB_LPS22HBH_TEMP_OUT_H_REG         (uint8_t)0x2C

/**
* @brief Threshold pressure (LSB) register.
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS7-THS0: LSB Threshold pressure Low part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(hPA)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define CB_LPS22HB_THS_P_LOW_REG           (uint8_t)0x0C

/**
* @brief Threshold pressure (MSB)
* \code
* Read/write
* Default value: 0x00.
* 7:0 THS15-THS8: MSB Threshold pressure. High part of threshold value for pressure interrupt
* generation. The complete threshold value is given by THS_P_H & THS_P_L and is
* expressed as unsigned number. P_ths(mbar)=(THS_P_H & THS_P_L)[dec]/16.
* \endcode
*/
#define CB_LPS22HB_THS_P_HIGH_REG         (uint8_t)0x0D

/**
* @brief FIFO control register 
* \code
* Read/write
* Default value: 0x00
* 7:5 F_MODE2, F_MODE1, F_MODE0: FIFO mode selection.
*     FM2   | FM1   | FM0   |    FIFO MODE
*   ---------------------------------------------------
*      0    |  0    |  0    | BYPASS MODE               
*      0    |  0    |  1    | FIFO MODE. Stops collecting data when full               
*      0    |  1    |  0    | STREAM MODE: Keep the newest measurements in the FIFO                  
*      0    |  1    |  1    | STREAM MODE until trigger deasserted, then change to FIFO MODE              
*      1    |  0    |  0    | BYPASS MODE until trigger deasserted, then STREAM MODE                 
*      1    |  0    |  1    | Reserved for future use          
*      1    |  1    |  0    | Reserved 
*      1    |  1    |  1    | BYPASS mode until trigger deasserted, then FIFO MODE              
*
* 4:0 WTM_POINT4-0 : FIFO Watermark level selection (0-31)
*/
#define CB_LPS22HB_CTRL_FIFO_REG          (uint8_t)0x14

#define CB_LPS22HB_FIFO_MODE_MASK        (uint8_t)0xE0
#define CB_LPS22HB_WTM_POINT_MASK        (uint8_t)0x1F


/**
* @brief FIFO Status register 
* \code
* Read
* Default value: ----
* 7 FTH_FIFO: FIFO threshold status. 0:FIFO filling is lower than FTH level; 1: FIFO is equal or higher than FTH level.           
* 6 OVR: Overrun bit status. 0 - FIFO not full; 1 -FIFO is full and at least one sample in the FIFO has been overwritten.
* 5:0 FSS: FIFO Stored data level. 000000: FIFO empty, 100000: FIFO is full and has 32 unread samples.
* \endcode
*/
#define CB_LPS22HB_STATUS_FIFO_REG        (uint8_t)0x26

#define CB_LPS22HB_FTH_FIFO_BIT          CB_LPS22HB_BIT(7)
#define CB_LPS22HB_OVR_FIFO_BIT          CB_LPS22HB_BIT(6)

#define CB_LPS22HB_FTH_FIFO_MASK         (uint8_t)0x80
#define CB_LPS22HB_OVR_FIFO_MASK         (uint8_t)0x40
#define CB_LPS22HB_LEVEL_FIFO_MASK       (uint8_t)0x3F
#define CB_LPS22HB_FIFO_EMPTY            (uint8_t)0x00
#define CB_LPS22HB_FIFO_FULL             (uint8_t)0x20



/**
* @brief Pressure offset register  (LSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS7-0:Pressure Offset for 1 point calibration (OPC) after soldering. 
* This register contains the low part of the pressure offset value after soldering,for
* differential pressure computing. The complete value is given by RPDS_L & RPDS_H
* and is expressed as signed 2 complement value. 
* \endcode
*/
#define CB_LPS22HB_RPDS_L_REG        (uint8_t)0x18

/**
* @brief Pressure offset register (MSB)
* \code
* Read/write
* Default value: 0x00
* 7:0 RPDS15-8:Pressure Offset for 1 point calibration  (OPC) after soldering.      
* This register contains the high part of the pressure offset value after soldering (see description RPDS_L) 
* \endcode
*/
#define CB_LPS22HB_RPDS_H_REG        (uint8_t)0x19


/**
* @brief Clock Tree Configuration register
* \code
* Read/write
* Default value: 0x00
* 7:6 Reserved.   
* 5: CTE: Clock Tree Enhancement
* \endcode
*/

#define CB_LPS22HB_CLOCK_TREE_CONFIGURATION        (uint8_t)0x43

#define CB_LPS22HB_CTE_MASK           (uint8_t)0x20

/**
* @}
*/


/**
* @}
*/


/* Exported Functions -------------------------------------------------------------*/
/** @defgroup CB_LPS22HB_Exported_Functions
* @{
*/

/**
* @brief  Init the HAL layer.
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
//#define CB_LPS22HB_HalInit  (uint32_t)HAL_Init_I2C

/**
* @brief  DeInit the HAL layer.
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
//#define CB_LPS22HB_HalDeInit  (uint32_t)HAL_DeInit_I2C

/**
* @brief  Read CB_LPS22HB Registers
* @param  uint8_t RegAddr:       address of the first register to read
* @param  uint8_t NumByteToRead:   number of registers to read
* @param  uint8_t *Data:          pointer to the destination data buffer
* @retval CB_LPS22HB_ERROR or CB_NO_ERROR
*/
// the user must redefine the proper CB_LPS22HB_ReadReg
uint32_t CB_LPS22HB_ReadReg(const uint8_t regAddr, const uint8_t numByteToRead, uint8_t* data);

/**
* @brief  Write CB_LPS22HB Registers
* @param  uint8_t RegAddr:      address of the register to write
* @param  uint8_t *Data:         pointer to the source data buffer
* @param  uint8_t NumByteToWrite:           Number of bytes to write
* @retval CB_LPS22HB_ERROR or CB_NO_ERROR
*/
// the user must redefine the proper CB_LPS22HB_WriteReg
uint32_t CB_LPS22HB_WriteReg(const uint8_t regAddr, const uint8_t numByteToWrite, const uint8_t* data);


/**
* @brief  Get the CB_LPS22HB driver version.
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DriverVersion(CB_LPS22HB_DriverVersion_st *Version);

/**
* @brief  Initialization function for CB_LPS22HB.
*         This function make a memory boot.
*         Init the sensor with a standard basic confifuration.
*         Low Power, ODR 25 Hz, Low Pass Filter disabled; BDU enabled; I2C enabled;
*         NO FIFO; NO Interrupt Enabled.
* @param  None.
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Init(void);

/**
* @brief  DeInit the LPS22HB driver.
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/

uint32_t CB_LPS22HB_DeInit(void);


/**
* @brief  Read identification code by WHO_AM_I register
* @param  Buffer to empty by Device identification Value.
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DeviceID(uint8_t* deviceid);


/**
* @brief  Set CB_LPS22HB Low Power or Low Noise Mode Configuration
* @param  CB_LPS22HB_LowNoise or CB_LPS22HB_LowPower mode
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PowerMode(CB_LPS22HB_PowerMode_et mode);

/**
* @brief  Get CB_LPS22HB Power Mode
* @param   Buffer to empty with Mode: Low Noise or Low Current
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PowerMode(CB_LPS22HB_PowerMode_et* mode);


/**
* @brief  Set CB_LPS22HB Output Data Rate
* @param  Output Data Rate
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_Odr(CB_LPS22HB_Odr_et odr);


/**
* @brief  Get CB_LPS22HB Output Data Rate
* @param  Buffer to empty with Output Data Rate
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Odr(CB_LPS22HB_Odr_et* odr);

/**
* @brief  Enable/Disale low-pass filter on CB_LPS22HB pressure data
* @param  state: enable or disable
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_LowPassFilter(CB_LPS22HB_State_et state);


/**
* @brief  Set low-pass filter cutoff configuration on CB_LPS22HB pressure data
* @param Filter Cutoff ODR/9 or ODR/20
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_LowPassFilterCutoff(CB_LPS22HB_LPF_Cutoff_et cutoff);

/**
* @brief  Set Block Data Update mode
* @param  CB_LPS22HB_BDU_CONTINUOS_UPDATE/ CB_LPS22HB_BDU_NO_UPDATE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_BDU(CB_LPS22HB_Bdu_et bdu);


/**
* @brief  Get Block Data Update mode
* @param  Buffer to empty whit the bdu mode read from sensor
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_BDU(CB_LPS22HB_Bdu_et* bdu);

/**
* @brief  Set SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  CB_LPS22HB_SPI_4_WIRE/CB_LPS22HB_SPI_3_WIRE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_SpiInterface(CB_LPS22HB_SPIMode_et spimode);

/**
* @brief  Get SPI mode: 3 Wire Interface OR 4 Wire Interface
* @param  buffer to empty with SPI mode
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_SpiInterface(CB_LPS22HB_SPIMode_et* spimode);

/**
* @brief Software Reset
* @param  void
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_SwReset(void);

/**
* @brief Reboot Memory Content.
* @param  void
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_MemoryBoot(void);

/**
* @brief Software Reset ann BOOT
* @param  void
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_SwResetAndMemoryBoot(void);


/**
* @brief  Enable or Disable FIFO 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoModeUse(CB_LPS22HB_State_et status);

/**
* @brief  Enable or Disable FIFO Watermark level use. Stop on FIFO Threshold
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoWatermarkLevelUse(CB_LPS22HB_State_et status);

/**
* @brief  Enable or Disable the Automatic increment register address during a multiple byte access with a serial interface (I2C or SPI)
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE. Default is CB_LPS22HB_ENABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutomaticIncrementRegAddress(CB_LPS22HB_State_et status);


/**
* @brief  Set One Shot bit to start a new conversion (ODR mode has to be 000)
* @param  void
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_StartOneShotMeasurement(void);

/**
* @brief  Enable/Disable I2C 
* @param  State. Enable (reset bit)/ Disable (set bit)
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_I2C(CB_LPS22HB_State_et i2cstate);


/*CTRL_REG3 Interrupt Control*/
/**
* @brief  Set Interrupt Active on High or Low Level
* @param  CB_LPS22HB_ActiveHigh/CB_LPS22HB_ActiveLow
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptActiveLevel(CB_LPS22HB_InterruptActiveLevel_et mode);

/**
* @brief  Set Push-pull/open drain selection on interrupt pads.
* @param  CB_LPS22HB_PushPull/CB_LPS22HB_OpenDrain
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptOutputType(CB_LPS22HB_OutputType_et output);

/**
* @brief  Set Data signal on INT1 pad control bits.
* @param  CB_LPS22HB_DATA,CB_LPS22HB_P_HIGH_CB_LPS22HB_P_LOW,CB_LPS22HB_P_LOW_HIGH
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptControlConfig(CB_LPS22HB_OutputSignalConfig_et config);


/**
* @brief   Enable/Disable Data-ready signal on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_DRDYInterrupt(CB_LPS22HB_State_et status);

 /**
* @brief   Enable/Disable FIFO overrun interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_OVR_Interrupt(CB_LPS22HB_State_et status);

 /**
* @brief   Enable/Disable FIFO threshold (Watermark) interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_FTH_Interrupt(CB_LPS22HB_State_et status);

/**
* @brief   Enable/Disable FIFO FULL interrupt on INT_DRDY pin. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FIFO_FULL_Interrupt(CB_LPS22HB_State_et status);

/**
* @brief   Enable AutoRifP function
* @param   none
* @detail When this function is enabled, an internal register is set with the current pressure values 
*         and the content is subtracted from the pressure output value and result is used for the interrupt generation.
*        the AutoRifP is slf creared.
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutoRifP(void);

/**
* @brief   Disable AutoRifP 
* @param   none
* @detail  the RESET_ARP bit is used to disable the AUTORIFP function. This bis i is selfdleared
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_ResetAutoRifP(void);

/**?????
* @brief  Set AutoZero Function bit
* @detail When set to ‘1’, the actual pressure output is copied in the REF_P reg (@0x15..0x17) 
* @param  None
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_AutoZeroFunction(void);

/**???
* @brief  Set ResetAutoZero Function bit
* @details REF_P reg (@0x015..17) set pressure reference to default value RPDS reg (0x18/19).
* @param  None
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_ResetAutoZeroFunction(void);


/**
* @brief  Enable/ Disable the computing of differential pressure output (Interrupt Generation)
* @param  CB_LPS22HB_ENABLE,CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptDifferentialGeneration(CB_LPS22HB_State_et diff_en) ;



/**
* @brief  Get the DIFF_EN bit value
* @param  buffer to empty with the read value of DIFF_EN bit
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptDifferentialGeneration(CB_LPS22HB_State_et* diff_en);


/**
* @brief  Latch Interrupt request to the INT_SOURCE register. 
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_LatchInterruptRequest(CB_LPS22HB_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure Low event
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PLE(CB_LPS22HB_State_et status);

/**
* @brief  Enable\Disable Interrupt Generation on differential pressure High event
* @param  CB_LPS22HB_ENABLE/CB_LPS22HB_DISABLE
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PHE(CB_LPS22HB_State_et status);

/**
* @brief   Get the Interrupt Generation on differential pressure status event and the Boot Status.
* @detail  The INT_SOURCE register is cleared by reading it.
* @param   Status Event Flag: BOOT, PH,PL,IA
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptDifferentialEventStatus(CB_LPS22HB_InterruptDiffStatus_st* interruptsource);


/**
* @brief  Get the status of Pressure and Temperature data
* @param  Data Status Flag:  TempDataAvailable, TempDataOverrun, PressDataAvailable, PressDataOverrun
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_DataStatus(CB_LPS22HB_DataStatus_st* datastatus);


/**
* @brief  Get the CB_LPS22HB raw presure value
* @param  The buffer to empty with the pressure raw value
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_RawPressure(int32_t *raw_press);

/**
* @brief  Get the CB_LPS22HB Pressure value in hPA.
* @param  The buffer to empty with the pressure value that must be divided by 100 to get the value in hPA
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Pressure(int32_t* Pout);

/**
* @brief  Read CB_LPS22HB output register, and calculate the raw temperature.
* @param  The buffer to empty with the temperature raw value
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_RawTemperature(int16_t *raw_data);

/**
* @brief  Read the Temperature value in °C.
* @param  The buffer to empty with the temperature value that must be divided by 10 to get the value in ['C]
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_Temperature(int16_t* Tout);

/**
* @brief  Get the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval  Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PressureThreshold(int16_t *P_ths);

/**
* @brief  Set the threshold value used for pressure interrupt generation.
* @param  The buffer to empty with the temperature value
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_PressureThreshold(int16_t P_ths);

/**
* @brief  Set Fifo Mode.
* @param  Fifo Mode struct
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoMode(CB_LPS22HB_FifoMode_et fifomode);
/**
* @brief  Get Fifo Mode.
* @param  Buffer to empty with fifo mode value
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoMode(CB_LPS22HB_FifoMode_et* fifomode);

/**
* @brief  Set Fifo Watermark Level.
* @param  Watermark level value [0 31]
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoWatermarkLevel(uint8_t wtmlevel);

/**
* @brief   Get FIFO Watermark Level
* @param   buffer to empty with watermak level[0,31] value read from sensor
* @retval  Status [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoWatermarkLevel(uint8_t *wtmlevel);


/**
* @brief  Get Fifo Status.
* @param  Buffer to empty with fifo status 
* @retval Status [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoStatus(CB_LPS22HB_FifoStatus_st* status);


/**
* @brief  Get the reference pressure after soldering for computing differential pressure (hPA)
* @param buffer to empty with the he pressure value (hPA)
* @retval  Status [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_PressureOffsetValue(int16_t *pressoffset);

/**
* @brief  Get the Reference Pressure value 
* @detail  It is a 24-bit data added to the sensor output measurement to detect a measured pressure beyond programmed limits.
* @param  Buffer to empty with reference pressure value
* @retval  Status [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_ReferencePressure(int32_t* RefP);


/**
* @brief  Check if the single measurement has completed.
* @param  the returned value is set to 1, when the measurement is completed
* @retval Status [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/  
uint32_t CB_LPS22HB_IsMeasurementCompleted(uint8_t* Is_Measurement_Completed);


/**
* @brief  Get the values of the last single measurement.
* @param  Pressure and temperature value
* @retvalStatus [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/ 
uint32_t CB_LPS22HB_Get_Measurement(CB_LPS22HB_MeasureTypeDef_st *Measurement_Value);


/**
* @brief   Set Generic Configuration
* @param   Struct to empty with the chosen values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_GenericConfig(CB_LPS22HB_ConfigTypeDef_st* pxCB_LPS22HBInit);

/**
* @brief  Get Generic configuration 
* @param  Struct to empty with configuration values
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_GenericConfig(CB_LPS22HB_ConfigTypeDef_st* pxCB_LPS22HBInit);

/**
* @brief  Set Interrupt configuration 
* @param  Struct holding the configuration values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_InterruptConfig(CB_LPS22HB_InterruptTypeDef_st* pCB_LPS22HBInt);

/**
* @brief  CB_LPS22HBGet_InterruptConfig
* @param  Struct to empty with configuration values
* @retval S Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_InterruptConfig(CB_LPS22HB_InterruptTypeDef_st* pCB_LPS22HBInt);

/**
* @brief  Set Fifo configuration 
* @param  Struct holding the configuration values
* @retval  Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_FifoConfig(CB_LPS22HB_FIFOTypeDef_st* pCB_LPS22HBFIFO);

/**
* @brief  Get Fifo configuration 
* @param  Struct to empty with the configuration values
* @retval Error code[CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Get_FifoConfig(CB_LPS22HB_FIFOTypeDef_st* pCB_LPS22HBFIFO);

/**
* @brief  Clock Tree Confoguration
* @param  CB_LPS22HB_CTE_NotBalanced, CB_LPS22HB_CTE_ABalanced
* @retval Error Code [CB_LPS22HB_ERROR, CB_NO_ERROR]
*/
uint32_t CB_LPS22HB_Set_ClockTreeConfifuration(CB_LPS22HB_CTE_et mode);


#ifdef __cplusplus
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

#endif /* CB_LPS22HB_H__ */

/******************* (C) COPYRIGHT 2013 STMicroelectronics *****END OF FILE****/
