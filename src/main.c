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

/** @file
 *
 * @defgroup Cellabox main.c
 * @ingroup Cellabox
 * @brief Cellabox main file: send sensor data to thethings.iO.
 *
 * @details This example demonstrates a CoAP client application that sends emulated
 *          temperature value to the thethings.io cloud. Example uses NAT64 on the
 *          Nordic's Thread Border Router solution for IPv4 connectivity.
 *
 *			THREAD needs the following resources:
 *			- RTC2
 *			- UART0
 *			- RADIO
 *			- RNG
 *			- SWI3 (only used for dynamic multiprotocol)
 *
 *			Cellaboxes need the following resources:
 *			- TWI0/TWIM0/I2C (sensors, EEPROM)
 *			- RTC0 (sensor data interrupt after sleep, RTC uses less power than timer)
 *			- UART1 (PM2.5, PM10 sensor)
 *			- SAADC (battery capacity measurement, sensors)
 *			- WTD (watchdog register 0)
 *
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_config.h" // Cellabox module specific HW stuff enabled
#include "sdk_config.h" // SDK specific configuration of cloud_coap_client example
#include "app_timer.h" // timer
//#include "bsp_thread.h" // board-support-package for Thread
#include "nrf_log_ctrl.h" // log
#include "nrf_log.h" // log
#include "nrf_log_default_backends.h" // log

#include "thread_coap_utils.h" // thread, cloud_coap_client example
#include "thread_dns_utils.h" // thread, cloud_coap_client example
#include "thread_utils.h" // thread, cloud_coap_client example
#include <openthread/openthread.h> // thread, cloud_coap_client example

#include "nrf_gpio.h" // gpio
#include "nrf_delay.h" // delay
#include "nrf_drv_rtc.h" // rtc
#include "nrf_drv_saadc.h" // rtc

#include "cb_error.h" // Cellabox errors
#include "cb_i2c.h" // I2C for SHTC1
#include "cb_shtc1.h" // SHTC1 temperature and humidity sensor (1.8V)
#include "cb_sgpc3.h" // SGPC3 VOC sensor
#include "cb_lps22hb.h" // LPS22HB absolute pressure sensor
#include "cb_sht30.h" // SHT330 temperature and humidity sensor (2.4-5.5V)
#include "cb_lmp91000.h" // LMP91000 analog front-end for 3-lead electrochemical gas sensors

/***************************************************************************************************
 * @section Cellabox compiler directives
 **************************************************************************************************/
// Module tokens:
// IAQ_LT1: 7S0BOkqyOxesjqOkycLgT5XvpZgxqevbMvc8iyPUFF0 (indoor-air-quality, quality long-term 1)
// IAQ_LT2: 7nkSIdu1ls0o_9M-YDQX9jnm6DZsRKQBm0pRgfOX4jo (indoor-air-quality, quality long-term 2)
// OAQ_LT1: xVd185tcEz4dB4K53Kiztd7m5jUqJp7_YBcU6mon5sA (output-air-quality, quality long-term 1)
// OAQ_LT2: psU3c1PSTgi4N_l5AFjlClRSbO0v3A3GAzYDAVs9Nu0 (output-air-quality, quality long-term 2)
#define CLOUD_URI_PATH "v2/things/7S0BOkqyOxesjqOkycLgT5XvpZgxqevbMvc8iyPUFF0" 	// TODO: assign URI dynamically


/***************************************************************************************************
 * @section CoAP and Thread defines
 **************************************************************************************************/

#define CLOUD_HOSTNAME            "coap.thethings.io" /**< Hostname of the thethings.io cloud. */
#define CLOUD_COAP_CONTENT_FORMAT 50                  /**< Use application/json content format type. */
static thread_coap_cloud_information_t m_cloud_information =
{
    .p_cloud_hostname            = CLOUD_HOSTNAME,
    .p_cloud_uri_path            = CLOUD_URI_PATH,
    .cloud_coap_content_format   = CLOUD_COAP_CONTENT_FORMAT,
};

/***************************************************************************************************
 * @section Cellabox defines
 **************************************************************************************************/

#define CB_FIRMWARE_VERSION							"0.0.0.1806" // TODO: set version: major-release.minor-release.bug-fix.build-number(yynn)

#define CB_LED_GREEN_PIN_NUMBER						(13) // P0.13
#define CB_LED_RED_PIN_NUMBER						(14) // P0.14
#define CB_MODULE_CONFIG_PIN0_NUMBER 				(P0_PIN_NUM+2) // P1.02
#define CB_MODULE_CONFIG_PIN1_NUMBER 				(P0_PIN_NUM+3) // P1.03
#define CB_MODULE_CONFIG_PIN2_NUMBER 				(P0_PIN_NUM+4) // P1.04
#define CB_MODULE_CONFIG_PIN3_NUMBER 				(P0_PIN_NUM+5) // P1.05
#define CB_MODULE_CONFIG_PIN4_NUMBER 				(P0_PIN_NUM+6) // P1.06
#define CB_MODULE_CONFIG_PIN5_NUMBER 				(P0_PIN_NUM+7) // P1.07
#define CB_LMP91000_nENABLE_O3						(P0_PIN_NUM+12) // P1.12, active-low
#define CB_LMP91000_nENABLE_NO2						(P0_PIN_NUM+13) // P1.13, active-low
#define CB_LMP91000_nENABLE_SO2						(P0_PIN_NUM+14) // P1.14, active-low
#define CB_LMP91000_nENABLE_CO						(P0_PIN_NUM+15) // P1.15, active-low
#define CB_LMP91000_nENABLE_H2S						(P0_PIN_NUM+11) // P1.11, active-low
#define CB_LMP91000_REFERENCE_VOLTAGE_mV			(2500) // Gas Sensor Frontent LMP91000 reference voltage [mV]
#define CB_LMP91000_EXT_FEEDBACK_R_OHM				(500000) // External feedback TIA resistor [Ohm]
#define CB_ADC_INPUT_NUMBER_VBAT					(0) // AIN0
#define CB_ADC_INPUT_NUMBER_O3						(1) // AIN1
#define CB_ADC_INPUT_NUMBER_NO2						(4) // AIN2
#define CB_ADC_INPUT_NUMBER_SO2						(5) // AIN3
#define CB_ADC_INPUT_NUMBER_CO						(6) // AIN4
#define CB_ADC_INPUT_NUMBER_H2S						(7) // AIN5
#define CB_ADC_RESOLUTION_BIT						(12) // !!! ALSO CHANGE FULL-SCALE IF YOU CHANGE RESOLUTION !!! ADC resolution [bit]
#define CB_ADC_FULL_SCALE							(4095) // !!! ALSO CHANGE RESOLUTION IF YOU CHANGE FULL-SCALE !!! ADC full scale [LSB]
#define CB_ADC_VREF_MV								(600) // ADC reference voltage [mV]
#define CB_ADC_VBAT_MEASUREMENT_ADC_ATTENUATION		(3) // ADC 1/Gain. E.g. if Gain=1/3, Attenuation = 3. We do not define gain here, because gain would be 0.33333, which we cannot easily compare...
#define CB_ADC_VBAT_MEASUREMENT_RESISTORS_RATIO		(5) // E.g. 5=5MOhm/1MOhm
#define CB_ADC_VBAT_MEASUREMENT_LSB_TO_MV_FACTOR	(((float)CB_ADC_VREF_MV*(float)CB_ADC_VBAT_MEASUREMENT_RESISTORS_RATIO+1)/(float)CB_ADC_FULL_SCALE*(float)CB_ADC_VBAT_MEASUREMENT_ADC_ATTENUATION) // ADC-LSB to [mV] factor for Vbat measurement. E.g. Vbat with 10MOhm and 2MOhm in series to GND, ADC measures voltage over 1MOhm --> (4MOhm+1MOhm)/1MOhm=5
#define CB_ADC_O3_MEASUREMENT_ADC_ATTENUATION		(4) // ADC 1/Gain. E.g. if Gain=1/4, Attenuation = 4. We do not define gain here, because gain would be 0.25, which we cannot easily compare...
#define CB_ADC_NO2_MEASUREMENT_ADC_ATTENUATION		(4) // ADC 1/Gain. E.g. if Gain=1/4, Attenuation = 4. We do not define gain here, because gain would be 0.25, which we cannot easily compare...
#define CB_ADC_SO2_MEASUREMENT_ADC_ATTENUATION		(4) // ADC 1/Gain. E.g. if Gain=1/4, Attenuation = 4. We do not define gain here, because gain would be 0.25, which we cannot easily compare...
#define CB_ADC_CO_MEASUREMENT_ADC_ATTENUATION		(4) // ADC 1/Gain. E.g. if Gain=1/4, Attenuation = 4. We do not define gain here, because gain would be 0.25, which we cannot easily compare...

#define CB_TH_SAMPLING_RATE_SEC						1 // Temperature and Humidity module: sampling time [sec]
#define CB_TH_AVERAGING_LENGTH						16 // TODO: set to 32 // Temperature and Humidity module: nbr of samples averaged before sending to cloud
#define CB_TH_NBR_OF_ADC_CHANNELS					1 // Battery voltage surveillance
#define CB_IAQ_SGPC3_SAMPLING_RATE_SEC				2 // Indoor Air Quality module with SGPC3 low-power (measuring ethanol): sampling time [sec], normal sampling rate for measuring TVOC is 0.5Hz
#define CB_IAQ_SGPC3_AVERAGING_LENGTH				16 // TODO: set to 32 // Temperature and Humidity module: nbr of samples averaged before sending to cloud
#define CB_IAQ_NBR_OF_ADC_CHANNELS					1 // Battery voltage surveillance
#define CB_OAQ_SAMPLING_RATE_SEC					1 // Outdoor Air Quality module: sampling time [sec]
#define CB_OAQ_AVERAGING_LENGTH						16 // TODO: set to 32 // Outdoor Air Quality module: nbr of samples averaged before sending to cloud
#define CB_OAQ_NBR_OF_ADC_CHANNELS					5 // 5 = Battery voltage surveillance + O3 + NO2 + SO2 + CO

#define CB_CLOUD_UPDATE_RATE_SEC					36 // TODO: set to 360 // How often are data sent to cloud?
#define CB_BASELINE_UPDATE_RATE_SEC					(7*24*3600) // 1 week = (7*24*3600). E.g. at least every week the sensor baselines should be adjusted
#define CB_ADC_CALIBRATION_RATE_SEC					(3600)

#define CB_UNDEFINED_TEMPERATURE_DEGC				0
#define CB_UNDEFINED_HUMIDITY_PERC					0
#define CB_UNDEFINED_PRESSURE_Pa					0
#define CB_UNDEFINED_PRESSURE_hPa					0
#define CB_UNDEFINED_TVOC_PPB						0
#define CB_UNDEFINED_TVOC_BASELINE_PPB				0
#define CB_UNDEFINED_ETHANOL_VALUE_PPB				0
#define CB_UNDEFINED_ETHANOL_REFERENCE_VALUE_PPB	0
#define CB_DEFAULT_TVOC_BASELINE_PPB				32767
#define CB_DEFAULT_ETHANOL_REFERENCE_VALUE_PPB		0
#define CB_SGPC3_2HZ_TO_10HZ_TVOC_CORRECTION_FACTOR	(1.0f)// Correction factor for TVOC when sampled with e.g. 10Hz instead of 2Hz

#define CB_UNDEFINED_O3_PPB							0
#define CB_UNDEFINED_O3_LSB							0
#define CB_UNDEFINED_SENSITIVITY_O3_110401_nAPerPpb	0
#define CB_UNDEFINED_BIAS_VOLTATE_O3_110401_mV		0
#define CB_UNDEFINED_NO2_PPB						0
#define CB_UNDEFINED_NO2_LSB						0
#define CB_UNDEFINED_SENSITIVITY_NO2_110501_nAPerPpb 0
#define CB_UNDEFINED_BIAS_VOLTATE_NO2_110501_mV		0
#define CB_UNDEFINED_SO2_PPB						0
#define CB_UNDEFINED_SO2_LSB						0
#define CB_UNDEFINED_SENSITIVITY_SO2_110601_nAPerPpb 0
#define CB_UNDEFINED_BIAS_VOLTATE_SO2_110601_mV		0
#define CB_UNDEFINED_CO_PPB							0
#define CB_UNDEFINED_CO_LSB							0
#define CB_UNDEFINED_SENSITIVITY_CO_110102_nAPerPpb 0
#define CB_UNDEFINED_BIAS_VOLTATE_CO_110102_mV		0

#define CB_DEFAULT_SENSITIVITY_O3_110401_nAPerPpb	((float)(-0.032))
#define CB_DEFAULT_BIAS_VOLTATE_O3_110401_mV		((int16_t)(-20))
#define CB_DEFAULT_SENSITIVITY_NO2_110501_nAPerPpb 	((float)(-0.040))
#define CB_DEFAULT_BIAS_VOLTATE_NO2_110501_mV		((int16_t)(-200))
#define CB_DEFAULT_SENSITIVITY_SO2_110601_nAPerPpb 	((float)(0.025))
#define CB_DEFAULT_BIAS_VOLTATE_SO2_110601_mV		((int16_t)(200))
#define CB_DEFAULT_SENSITIVITY_CO_110102_nAPerPpb 	((float)(0.00475))
#define CB_DEFAULT_BIAS_VOLTATE_CO_110102_mV		((int16_t)(20))

#define CB_THREAD_POLL_PERIOD_SEC					3600 // Thread Sleepy End Device polling period. [s]
#define CB_THREAD_DEFAULT_CHILD_TIMEOUT_SEC			(4*CB_THREAD_POLL_PERIOD_SEC) // Thread child timeout. [s]
#define CB_THREAD_DEFAULT_POLL_PERIOD_MSEC			(1000*CB_THREAD_POLL_PERIOD_SEC) // Thread Sleepy End Device polling period. [ms]

#define CB_LFCLK_FREQUENCY_HZ						32768 // Low-frequency clock frequency in [Hz]
#define CB_RTC_FREQUENCY_HZ 						32 // RTC frequency, should be a power-of-two (12bit pre-scaler, f_rtc=32.768kHz, pre-scaler = 1024 for e.g f_rtc0=32Hz)
#define CB_RTC_INTERRUPT_PRIORITY					7  // Must not be 0-4 according to Nordic. -3=Reset, -2=NMI, -1=HardFault, 0=SoftDeviceTimeCritical, 1=SoftDeviceMemoryProtection, 2=ApplicationInterrupt, 3=ApplicationInterrupts, 4=SoftDeviceAPI, 5=ApplicationInterrupts, 6=ApplicationInterrupts, 7=ApplicationInterrupts
#define CB_ADC_INTERRUPT_PRIORITY					CB_RTC_INTERRUPT_PRIORITY // Must not be 0-4 according to Nordic. Set the same interrupt priority for RTC and ADC.
#define CB_RTC_CC_INSTANCE_SENSOR_SAMPLING			0  // RTC capture compare register number which is used for sensor sampling

#define CB_ADC_NBR_OF_CHANNELS_ENABLED_MAX			8 // Maximum number of ADC channels enabled

#define CB_EEPROM_VALIDATION_TAG					"Cbox" // This tag sequence is used to verify if flash is new or already written on
#define CB_EEPROM_SGPC3_TVOC_BASELINE_ADDR			16 // Address of lower byte of SGPC3 TVOC baseline
#define CB_EEPROM_SGPC3_ETH_REFERENCE_ADDR			20 // Address of lower byte of SGPC3 ethanol reference signal (sref)

#define CB_LMP91000_VOLTAGE_REFERENCE_V				(2.5f) // Reference voltage of LMP91000 in [V]

/***************************************************************************************************
 * @section Cellabox typedefs
 **************************************************************************************************/
typedef enum CB_eModuleType_tag
{
	CB_eTH_SHTC1 = 0, // Temperature & Humidity. Sensors = SHTC1
	CB_eIAQ_SHTC1_SGPC3 = 1, // Indoor Air Quality. Sensors = SHTC1, SGPC3
	CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO = 2 // Outdoor Air Quality. Sensors = SHTC30, LPS22H, CO = 3SP_CO_1000 Package 110-102, NO2 = 3SP_NO2_20 P Package 110-501, O3 = 3SP_O3_20 P Package 110-401, SO2 = 3SP_SO2_20 P Package 110-601
} CB_eModuleType_t;
typedef struct CB_stVersion_tag {
  uint8_t major;
  uint8_t minor;
  uint8_t revision;
}CB_stVersion_t;
typedef enum CB_eTempHumSensorType_tag
{
	CB_eTempHumSensorUndefined = -1,
	CB_eSHTC1 = 0,
	CB_eSHT30 = 1
} CB_eTempHumSensorType_t;
typedef enum CB_eVocType_tag
{
	CB_eVocSensorUndefined = -1,
	CB_eSGPC3 = 0
} CB_eVocSensorType_t;
typedef enum CB_eElectroChemicalSensorType_tag
{
	CB_EcSensorUndefined = -1,
	CB_eO3_Spec110401  = 0,
	CB_eO3_Spec110407  = 1,
	CB_eNO2_Spec110501 = 20,
	CB_eSO2_Spec110601 = 30,
	CB_eCO_Spec110102  = 40
} CB_eElectroChemicalSensorType_t;
typedef struct CB_stTempHumSensor_tag
{
	CB_eTempHumSensorType_t eTempHumSensorType;
	float temperature_DegC; // Last measured value
	float temperatureAvg_DegC; // Averaged value
	float humidity_Perc; // Last measured value
	float humidityAvg_Perc; // Averaged value
	uint32_t deviceId;
} CB_stTempHumSensor_t;
typedef struct CB_stVocSensor_tag
{
	CB_eVocSensorType_t eVocSensorType;
	uint16_t tvoc_ppb; // Last measured value [PPB]
	uint16_t tvocAvg_ppb; // Averaged value [PPB]
	uint16_t tvocCalcAvg_ppb; // Averaged value calculated out of ethanol raw signal [PPB]
	uint16_t tvocBaseline_ppb; // Last measured baseline value, this value is not averaged, because it has low-noise [PPB]
	uint16_t ethanol_ppb; // Last measured value [PPB]
	uint16_t ethanolAvg_ppb; // Averaged value [PPB]
	uint16_t ethanolAvgPeak_ppb; // Averaged peak value, used for baseline algorithm [PPB]
	uint16_t ethanolRef_ppb; // Ethanol reference signal sref (used for tvoc calculation) for clean air [PPB]
	uint8_t initHighPowerDuration_sec; // e.g. 0, 16, 64 or 184sec [sec]
	uint16_t featureSetVersion; // e.g. SGPC3 has feature set version of 5 bits (product version).
	uint64_t deviceId; // e.g. SGPC3 has a unique serial ID with length of 48 bits.
} CB_stVocSensor_t;
typedef struct CB_stElectroChemicalSensor_tag
{
	CB_eElectroChemicalSensorType_t eElectroChemicalSensorType;
	uint16_t concentrationAvg_ppb; // Averaged value [PPB]
	uint16_t concentrationAvg_lsb; // Average value [LSB]
	uint16_t concentrationAvgPeak_lsb; // Averaged peak value, used for baseline algorithm [LSB]
	uint16_t zeroValue_lsb; // Value when no gas is present [LSB]
	float sensitivity_nAPerPpb; // Sensitivity [nA/PPB]
	int16_t biasVoltage_mV; // Bias voltage [mV]
	uint8_t adcPortNumber; // Port number of nRF52: 0, 1
	uint8_t adcPinNumber; // Pin number of port of nRF52: 0...31
} CB_stElectroChemicalSensor_t;
typedef enum CB_ePressSensorType_tag
{
	CB_ePressSensorUndefined = -1,
	CB_eLPS22HB = 0
} CB_ePressSensorType_t;
typedef struct CB_stPressSensor_tag
{
	CB_ePressSensorType_t ePressSensorType;
	int32_t pressure_Pa; // Last measured value
	int32_t pressureAvg_Pa; // Averaged value
	float pressureAvg_hPa; // Averaged value
	int16_t temperature_0_01DegC;
	float temperature_DegC;
	uint32_t deviceId;
	CB_stVersion_t driverVersion;
} CB_stPressSensor_t;
typedef struct CB_stModlue_tag
{
	CB_eModuleType_t eModuleType; // Eg. is it a T&H or IAQ or OAQ device?
	uint16_t batteryVoltage_mV; // Battery voltage [mV]
} CB_stModule_t;


/***************************************************************************************************
 * @section Cellabox module global variables
 **************************************************************************************************/

static CB_stModule_t m_stModule; // Struct of the module
static CB_stTempHumSensor_t m_stTempHumSensor;
static CB_stVocSensor_t m_stVocSensor;
static CB_stElectroChemicalSensor_t m_stO3Sensor;
static CB_stElectroChemicalSensor_t m_stNO2Sensor;
static CB_stElectroChemicalSensor_t m_stSO2Sensor;
static CB_stElectroChemicalSensor_t m_stCOSensor;
static CB_stLmp91000_t m_stO3Afe; // O3 analog frontend
static CB_stLmp91000_t m_stNO2Afe; // NO2 analog frontend
static CB_stLmp91000_t m_stSO2Afe; // SO2 analog frontend
static CB_stLmp91000_t m_stCOAfe; // CO analog frontend
static CB_stPressSensor_t m_stPressureSensor;

static uint16_t m_SamplingRate_Sec; // Sampling rate of the sensor CB_SENSOR_SAMPLE_RATE_SEC
static uint16_t m_AveragingLength_Samples; // Number of samples taken for calculating the average value
static uint16_t m_CloudUpdateRateToSampleRate_Ratio; // (CB_CLOUD_UPDATE_RATE_SEC/CB_SENSOR_SAMPLE_RATE_SEC)
static uint16_t m_BaselineUpdateToCloudUpdate_Ratio; // (CB_BASELINE_UPDATE_RATE_SEC/CB_CLOUD_UPDATE_RATE_SEC)
static uint16_t m_AdcCalibToCloudUpdate_Ratio; // (CB_ADC_CALIBRATION_RATE_SEC/CB_CLOUD_UPDATE_RATE_SEC)
static uint16_t m_WatchdogTimeout_Sec; // #define CB_WATCHDOG_TIMEOUT_SEC (2*CB_CLOUD_UPDATE_RATE_SEC) // Watchdog should reset CPU when there was no cloud update
static uint16_t m_RtcCcValueForSensorSampling; // #define CB_RTC_CC_VALUE_SENSOR_SAMPLING (CB_SENSOR_SAMPLE_RATE_SEC*CB_RTC_FREQUENCY_HZ) // Value of the capture compare register of the RTC
static const nrf_drv_rtc_t m_SensorDataIsrRtc = NRF_DRV_RTC_INSTANCE(RTC_ISR_INSTANCE_ID); // Declaring an instance of nrf_drv_rtc for RTC0.

static float m_TemperatureSum_DegC = 0; // Sum needed for average value calculation
static float m_HumiditySum_Perc = 0; // Sum needed for average value calculation
static uint32_t m_BatteryVotageSum_LSB = 0; // Sum needed for average value calculation
static uint32_t m_EthSum_ppb = 0; // Sum needed for average value calculation
//static uint32_t m_TvocSum_ppb = 0; // Sum needed for average value calculation
static int32_t m_PressureSum_Pa = 0; // Sum needed for average value calculation
static uint32_t m_O3Sum_LSB = 0; // Sum needed for average value calculation
static uint32_t m_NO2Sum_LSB = 0; // Sum needed for average value calculation
static uint32_t m_SO2Sum_LSB = 0; // Sum needed for average value calculation
static uint32_t m_COSum_LSB = 0; // Sum needed for average value calculation

static uint8_t m_DoSampleContinuously; // 1 = do always sample, 0 = do only sample shortly before sending data to cloud
static int16_t m_AdcBuffer[CB_ADC_NBR_OF_CHANNELS_ENABLED_MAX]; // Buffer for all ADC data
static uint16_t m_AdcBufferLength_Samples; // Length of ADC buffer in number of [samples]
static bool m_AdcCalibrationIsRunning;

static uint16_t m_SampleDataCounter = 0; // Counter of number of data samples were taken since last averaging
static uint16_t m_AdcSamplesCounter = 0; // Counts the number of Vbat, O3, etc. samples, this counter may NOT have the same value as m_SampleDataCounter!!!
static uint32_t m_CloudUpdateCounter = 0; // Counts the number of cloud updates

/***************************************************************************************************
 * @section CoAP and Thread
 **************************************************************************************************/

static void dns_response_handler(void         * p_context,
                                 const char   * p_hostname,
                                 otIp6Address * p_resolved_address,
                                 uint32_t       ttl,
                                 otError        error)
{
    if (error != OT_ERROR_NONE)
    {
        NRF_LOG_INFO("DNS response error %d.\r\n", error);
        return;
    }
    thread_coap_utils_peer_addr_set(p_resolved_address);
}

static void thread_state_changed_callback(uint32_t flags, void * p_context)
{
    if (flags & OT_CHANGED_THREAD_ROLE)
    {
        switch(otThreadGetDeviceRole(p_context))
        {
            case OT_DEVICE_ROLE_CHILD:
            case OT_DEVICE_ROLE_ROUTER:
            case OT_DEVICE_ROLE_LEADER:
                UNUSED_VARIABLE(thread_dns_utils_hostname_resolve(p_context,
                                                                  m_cloud_information.p_cloud_hostname,
                                                                  dns_response_handler,
                                                                  NULL));
                break;

            case OT_DEVICE_ROLE_DISABLED:
            case OT_DEVICE_ROLE_DETACHED:
            default:
                thread_coap_utils_peer_addr_clear();
                break;
        }
    }

    NRF_LOG_INFO("State changed! Flags: 0x%08x Current role: %d\r\n",
                flags,
                otThreadGetDeviceRole(p_context));
}

/***************************************************************************************************
 * @section Log and Thread Initialization
 **************************************************************************************************/

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void CB_ThreadInit(void)
{
	// Thread: initialize Thread Stack
	thread_configuration_t thread_configuration =
    {
        .role              = RX_ON_WHEN_IDLE,
        .autocommissioning = true,
    };
    thread_init(&thread_configuration);
    thread_cli_init();
    thread_state_changed_callback_set(thread_state_changed_callback);

    // CoAP: initializing the Constrained Application Protocol Module
    thread_coap_configuration_t thread_coap_configuration =
    {
        .coap_server_enabled               = false,
        .coap_client_enabled               = true,
        .coap_cloud_enabled                = true,
        .configurable_led_blinking_enabled = false,
    };
    thread_coap_utils_init(&thread_coap_configuration);
}

/***************************************************************************************************
 * @section Cellabox initialization
 **************************************************************************************************/

void CB_PowerInit(void)
{
	NRF_POWER->DCDCEN = 1; // Enable DC/DC converter for REG1 stage.
	//for(uint8_t i=0; i<CB_NUMBER_OF_RAM_SECTIONS; i++)
	//{
	//	(NRF_POWER->RAM[i]).POWERSET = 0xFFFFFFFF; // Enable RAM retention when CPU = OFF
	//}
}

void CB_LedInit(void)
{
	// TODO: start a timer here and stop timer when Thread connection is established
	for(int i=0;i<5;i++)
	{
		nrf_gpio_pin_toggle(CB_LED_GREEN_PIN_NUMBER);
		nrf_delay_ms(200);
	}
}

void CB_GpioInit(void)
{
	// LED GPIOs
	nrf_gpio_cfg_output(CB_LED_GREEN_PIN_NUMBER); nrf_gpio_pin_clear(CB_LED_GREEN_PIN_NUMBER);
	nrf_gpio_cfg_output(CB_LED_RED_PIN_NUMBER); nrf_gpio_pin_clear(CB_LED_RED_PIN_NUMBER);
	// Init HW and module config GPIOs
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN0_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN1_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN2_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN3_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN4_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	nrf_gpio_cfg_input(CB_MODULE_CONFIG_PIN5_NUMBER, NRF_GPIO_PIN_PULLDOWN);
	// LMP91000 enable \MENB pin
	nrf_gpio_cfg_output(CB_LMP91000_nENABLE_H2S); nrf_gpio_pin_set(CB_LMP91000_nENABLE_H2S);
	nrf_gpio_cfg_output(CB_LMP91000_nENABLE_O3); nrf_gpio_pin_set(CB_LMP91000_nENABLE_O3);
	nrf_gpio_cfg_output(CB_LMP91000_nENABLE_NO2); nrf_gpio_pin_set(CB_LMP91000_nENABLE_NO2);
	nrf_gpio_cfg_output(CB_LMP91000_nENABLE_SO2); nrf_gpio_pin_set(CB_LMP91000_nENABLE_SO2);
	nrf_gpio_cfg_output(CB_LMP91000_nENABLE_CO); nrf_gpio_pin_set(CB_LMP91000_nENABLE_CO);
	// ADC channel initialization is done in function CB_AdcInit();
}


void CB_WatchdogInit(void)
{
	NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos);   // Configure Watchdog. a) Pause watchdog while the CPU is halted by the debugger.  b) Keep the watchdog running while the CPU is sleeping.
	NRF_WDT->CRV = m_WatchdogTimeout_Sec*CB_LFCLK_FREQUENCY_HZ; // Set watchdog timeout
	NRF_WDT->RREN |= WDT_RREN_RR0_Msk; // Enable reload register 0
	NRF_WDT->TASKS_START = 0; // Stop the watchdog timer
}

void CB_WatchdogStart(void)
{
	NRF_WDT->TASKS_START = 1; // Start the watchdog timer
}

void CB_WatchdogReload(void)
{
	NRF_WDT->RR[0] = WDT_RR_RR_Reload; // Reload watchdog register 0
}

void CB_GetModuleType(void)
{
	uint32_t err_code = CB_NO_ERROR;
	uint8_t id;
	id = (nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN5_NUMBER)<<5) +
		 (nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN4_NUMBER)<<4) +
		 (nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN3_NUMBER)<<3) +
		 (nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN2_NUMBER)<<2) +
		 (nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN1_NUMBER)<<1) +
		  nrf_gpio_pin_read(CB_MODULE_CONFIG_PIN0_NUMBER);
	if (id == (uint8_t)CB_eTH_SHTC1)
	{
		m_stModule.eModuleType = CB_eTH_SHTC1;
	}
	else if (id == (uint8_t)CB_eIAQ_SHTC1_SGPC3)
	{
		m_stModule.eModuleType = CB_eIAQ_SHTC1_SGPC3;
	}
	else if (id == (uint8_t)CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO)
	{
		m_stModule.eModuleType = CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO;
	}
	else
	{
		err_code = CB_ILLEGAL_MODULE_CONFIG;
	}
	APP_ERROR_CHECK(err_code);
}

void CB_SetModuleParameters(void)
{
	// ---------------------------
	// Module depending parameters
	// ---------------------------
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		m_DoSampleContinuously = 0;
		m_SamplingRate_Sec = CB_TH_SAMPLING_RATE_SEC;
		m_AveragingLength_Samples = CB_TH_AVERAGING_LENGTH;
		m_AdcBufferLength_Samples = CB_TH_NBR_OF_ADC_CHANNELS;
		break;
	case CB_eIAQ_SHTC1_SGPC3:
		m_DoSampleContinuously = 1;
		m_SamplingRate_Sec = CB_IAQ_SGPC3_SAMPLING_RATE_SEC;
		m_AveragingLength_Samples = CB_IAQ_SGPC3_AVERAGING_LENGTH;
		m_AdcBufferLength_Samples = CB_IAQ_NBR_OF_ADC_CHANNELS;
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		m_DoSampleContinuously = 0;
		m_SamplingRate_Sec = CB_OAQ_SAMPLING_RATE_SEC;
		m_AveragingLength_Samples = CB_OAQ_AVERAGING_LENGTH;
		m_AdcBufferLength_Samples = CB_OAQ_NBR_OF_ADC_CHANNELS;
		break;
	default:
		APP_ERROR_CHECK(CB_ILLEGAL_MODULE_CONFIG);
		break;
	}
	// ---------------------------
	// True for all modules
	// ---------------------------
	m_SampleDataCounter = 0;
	m_AdcSamplesCounter = 0;
	m_CloudUpdateCounter = 0;
	m_CloudUpdateRateToSampleRate_Ratio = (CB_CLOUD_UPDATE_RATE_SEC/m_SamplingRate_Sec);
	m_BaselineUpdateToCloudUpdate_Ratio = (CB_BASELINE_UPDATE_RATE_SEC/m_CloudUpdateRateToSampleRate_Ratio);
	m_AdcCalibToCloudUpdate_Ratio = (CB_ADC_CALIBRATION_RATE_SEC/CB_CLOUD_UPDATE_RATE_SEC);
	m_WatchdogTimeout_Sec = (2*CB_CLOUD_UPDATE_RATE_SEC);
	m_RtcCcValueForSensorSampling = (m_SamplingRate_Sec*CB_RTC_FREQUENCY_HZ);
}

uint32_t CB_WriteCalibValuesToStorage(void)
{
	uint32_t  error = CB_NO_ERROR;

	// TODO Set validation tag ("Cbox") to invalid BEFORE write to EEPROM

	// Write valid calibration data to storage first
	switch (m_stModule.eModuleType)
	{
		case CB_eTH_SHTC1:
			break;
		case CB_eIAQ_SHTC1_SGPC3:
			break;
    	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
    		break;
		default:
			break;
	}

	// TODO: Write tag to flash AFTER valid data was written to flash

	return error;
}

void CB_ReadCalibDataFromPersistentStorage(void)
{
    uint16_t i;

    // TODO: Check if data in storage is valid ("Cbox")
    i = sizeof(CB_EEPROM_VALIDATION_TAG);

	if (i==sizeof(CB_EEPROM_VALIDATION_TAG))
	{
		// Validation TAG is valid
		switch (m_stModule.eModuleType)
		{
			case CB_eTH_SHTC1:
				break;
			case CB_eIAQ_SHTC1_SGPC3:
				break;
	    	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
	    		break;
			default:
				break;
		}
	}
}

void CB_I2cInit(void)
{
    // I2C is used to communicate with sensors
	uint32_t err_code = CB_I2C_Init();
    APP_ERROR_CHECK(err_code);
}

uint16_t CB_ElectroChemicalSensors_LsbToPpb(const CB_stElectroChemicalSensor_t* stSensor)
{
	uint16_t concentration_ppb;
	float sensorCurrent_nA;

	sensorCurrent_nA = (float)(stSensor->concentrationAvg_lsb - stSensor->zeroValue_lsb) / (float)CB_ADC_FULL_SCALE * CB_LMP91000_REFERENCE_VOLTAGE_mV / CB_LMP91000_EXT_FEEDBACK_R_OHM * 1E6;
	concentration_ppb = sensorCurrent_nA / stSensor->sensitivity_nAPerPpb;
	//nA = ((double)ADC - (double)ADC_Zero) / 65535 * 2.5 / (double)Rgain * 1E9; //formula for 2.5V reference
    //Zc = nA - (Zcf * (T_float - Tkz));
    //Sc = 1 - (Scf / 100 * (T_float - Tks));
    //Cx = Zc * Sc / (double)nA_per_PPM_x100 * 1E5; //Concentration with temperature compensation
	return concentration_ppb;
}

void CB_AdcCallback(nrf_drv_saadc_evt_t const * p_event)
{
    uint32_t error;
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
    	m_AdcSamplesCounter++;
    	m_BatteryVotageSum_LSB += m_AdcBuffer[CB_ADC_INPUT_NUMBER_VBAT]; // Battery voltage is used by every module do this always
    	switch (m_stModule.eModuleType)
    	{
    	case CB_eTH_SHTC1:
    	case CB_eIAQ_SHTC1_SGPC3:
    		break;
    	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
    		error = CB_LMP91000_SetStandbyMode(&m_stO3Afe); // Set all LMP91000 back to standby after measurement
    		APP_ERROR_CHECK(error);
    		error = CB_LMP91000_SetStandbyMode(&m_stNO2Afe); // Set all LMP91000 back to standby after measurement
    		APP_ERROR_CHECK(error);
    		error = CB_LMP91000_SetStandbyMode(&m_stSO2Afe); // Set all LMP91000 back to standby after measurement
    		APP_ERROR_CHECK(error);
    		error = CB_LMP91000_SetStandbyMode(&m_stCOAfe); // Set all LMP91000 back to standby after measurement
			APP_ERROR_CHECK(error);
    		m_O3Sum_LSB += m_AdcBuffer[CB_ADC_INPUT_NUMBER_O3];
    		m_NO2Sum_LSB += m_AdcBuffer[CB_ADC_INPUT_NUMBER_NO2];
    		m_SO2Sum_LSB += m_AdcBuffer[CB_ADC_INPUT_NUMBER_SO2];
    		m_COSum_LSB += m_AdcBuffer[CB_ADC_INPUT_NUMBER_CO];
    		if (m_AveragingLength_Samples == m_AdcSamplesCounter)
        	{
    			m_stO3Sensor.concentrationAvg_lsb = (uint16_t)((float)m_O3Sum_LSB/(float)m_AveragingLength_Samples);
    			m_stNO2Sensor.concentrationAvg_lsb = (uint16_t)((float)m_NO2Sum_LSB/(float)m_AveragingLength_Samples);
    			m_stSO2Sensor.concentrationAvg_lsb = (uint16_t)((float)m_SO2Sum_LSB/(float)m_AveragingLength_Samples);
    			m_stCOSensor.concentrationAvg_lsb = (uint16_t)((float)m_COSum_LSB/(float)m_AveragingLength_Samples);
        		m_O3Sum_LSB = 0; // Set it to zero in AdcCallback()
        		m_NO2Sum_LSB = 0; // Set it to zero in AdcCallback()
        		m_SO2Sum_LSB = 0; // Set it to zero in AdcCallback()
        		m_COSum_LSB = 0; // Set it to zero in AdcCallback()
        		CB_ElectroChemicalSensors_LsbToPpb(&m_stO3Sensor);
        		CB_ElectroChemicalSensors_LsbToPpb(&m_stNO2Sensor);
        		CB_ElectroChemicalSensors_LsbToPpb(&m_stSO2Sensor);
        		CB_ElectroChemicalSensors_LsbToPpb(&m_stCOSensor);
        	}
			break;
    	default:
			break;
    	}
    	if (m_AveragingLength_Samples == m_AdcSamplesCounter)
    	{
    		m_stModule.batteryVoltage_mV = (uint16_t)(((float)m_BatteryVotageSum_LSB/(float)m_AveragingLength_Samples)*CB_ADC_VBAT_MEASUREMENT_LSB_TO_MV_FACTOR);
    		m_BatteryVotageSum_LSB = 0; // Set it to zero in AdcCallback()
    		m_AdcSamplesCounter = 0; // Reset ADC samples counter ONLY ONCE!!!
    	}
    	// Start to calibrate ADC offset from time to time
    	if((m_CloudUpdateCounter % m_AdcCalibToCloudUpdate_Ratio) == 0)	// Evaluate if offset calibration should be performed
        {
    		nrf_drv_saadc_abort(); // Abort all ongoing conversions. Calibration cannot be run if SAADC is busy
    		m_AdcCalibrationIsRunning = true;
    		error = nrf_drv_saadc_calibrate_offset(); // Start calibration. Function is non-blocking and trigger DONE and RESULTDONE events
    		APP_ERROR_CHECK(error);
        }
    }
    else if (p_event->type == NRF_DRV_SAADC_EVT_CALIBRATEDONE) // Capture offset calibration complete event
    {
    	m_AdcCalibrationIsRunning = false;
    }
    if (false == m_AdcCalibrationIsRunning)
    {
    	error = nrf_drv_saadc_buffer_convert(m_AdcBuffer, m_AdcBufferLength_Samples);	//Set buffer so the SAADC can write to it again.
    	APP_ERROR_CHECK(error);
    }
}

uint32_t CB_AdcInit(void)
{
    uint32_t error = CB_NO_ERROR;
	nrf_drv_saadc_config_t saadc_config;
    nrf_saadc_channel_config_t channel_config;

    // ---------------------------
	// Setup SAADC common settings
    // ---------------------------
    saadc_config.low_power_mode = true; // Enable low power mode (init SAADC when sampling and uninit when sampling is complete)
	if (14 == CB_ADC_RESOLUTION_BIT)
	{
		saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT; // Set SAADC resolution to 14-bit, 14-bit only with oversampling
	}
	else if (12 == CB_ADC_RESOLUTION_BIT)
	{
		saadc_config.resolution = NRF_SAADC_RESOLUTION_12BIT; // Set SAADC resolution to 12-bit
	}
	else if (10 == CB_ADC_RESOLUTION_BIT)
	{
		saadc_config.resolution = NRF_SAADC_RESOLUTION_10BIT; // Set SAADC resolution to 10-bit
	}
	else if (8 == CB_ADC_RESOLUTION_BIT)
	{
		saadc_config.resolution = NRF_SAADC_RESOLUTION_8BIT; // Set SAADC resolution to 8-bit
	}
	else
	{
		APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
	}
	saadc_config.oversample = NRF_SAADC_OVERSAMPLE_DISABLED; // If oversampling is enabled, only one channel can be enabled and resolution lower than 14 bit(!)
	saadc_config.interrupt_priority = CB_ADC_INTERRUPT_PRIORITY; // Set SAADC interrupt level.
	error = nrf_drv_saadc_init(&saadc_config, CB_AdcCallback); // Configure SAADC, defining a callback function is a must
	APP_ERROR_CHECK(error);

	// ---------------------------
	// AIN0: battery surveillance
	// ---------------------------
	if (600 == CB_ADC_VREF_MV)
	{
		channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Set internal reference of fixed 0.6 volts
	}
	else
	{
		APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
	}
	if (3 == CB_ADC_VBAT_MEASUREMENT_ADC_ATTENUATION)
	{
		channel_config.gain = NRF_SAADC_GAIN1_3; //Set input gain to 1/3. The maximum SAADC input voltage is then 0.6V/(1/3)=1.8V. The single ended input range is then 0V-3.6V
	}
	else
	{
		APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
	}
	channel_config.acq_time = NRF_SAADC_ACQTIME_40US; // Set acquisition time. Set high acquisition time to allow maximum source resistance up to 800 kohm, see the SAADC electrical specification in the PS.
	channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
	channel_config.pin_p = NRF_SAADC_INPUT_AIN0; // Select the input pin for the channel. AIN0 pin maps to physical pin P0.02.
	channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
	channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED; // Disable pullup resistor on the input pin
	channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; // Disable pulldown resistor on the input pin
	error = nrf_drv_saadc_channel_init(CB_ADC_INPUT_NUMBER_VBAT , &channel_config); // Initialize SAADC AIN0 with the channel configuration
    APP_ERROR_CHECK(error);

    // ---------------------------
    // Module depending init
    // ---------------------------
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
	case CB_eIAQ_SHTC1_SGPC3:
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		// ---------------------------
		// AIN1: O3
		// ---------------------------
		if (600 == CB_ADC_VREF_MV)
		{
			channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Set internal reference of fixed 0.6 volts
		}
		else
		{
			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
		}
		if (4 == CB_ADC_O3_MEASUREMENT_ADC_ATTENUATION)
		{
			channel_config.gain = NRF_SAADC_GAIN1_4; //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-3.6V
		}
		else
		{
			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
		}
		channel_config.acq_time = NRF_SAADC_ACQTIME_3US; // Set acquisition time. Set high short time to save power.
		channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
		channel_config.pin_p = NRF_SAADC_INPUT_AIN1; // Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
		channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
		channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED; // Disable pullup resistor on the input pin
		channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; // Disable pulldown resistor on the input pin
		error = nrf_drv_saadc_channel_init(CB_ADC_INPUT_NUMBER_O3 , &channel_config); // Initialize SAADC AIN0 with the channel configuration
	    APP_ERROR_CHECK(error);
//		// ---------------------------
//		// AIN4: NO2
//		// ---------------------------
//		if (600 == CB_ADC_VREF_MV)
//		{
//			channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Set internal reference of fixed 0.6 volts
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		if (4 == CB_ADC_NO2_MEASUREMENT_ADC_ATTENUATION)
//		{
//			channel_config.gain = NRF_SAADC_GAIN1_4; //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-3.6V
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		channel_config.acq_time = NRF_SAADC_ACQTIME_3US; // Set acquisition time. Set high short time to save power.
//		channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
//		channel_config.pin_p = NRF_SAADC_INPUT_AIN1; // Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
//		channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
//		channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED; // Disable pullup resistor on the input pin
//		channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; // Disable pulldown resistor on the input pin
//		error = nrf_drv_saadc_channel_init(CB_ADC_INPUT_NUMBER_NO2 , &channel_config); // Initialize SAADC AIN0 with the channel configuration
//	    APP_ERROR_CHECK(error);
//		// ---------------------------
//		// AIN5: SO2
//		// ---------------------------
//		if (600 == CB_ADC_VREF_MV)
//		{
//			channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Set internal reference of fixed 0.6 volts
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		if (4 == CB_ADC_SO2_MEASUREMENT_ADC_ATTENUATION)
//		{
//			channel_config.gain = NRF_SAADC_GAIN1_4; //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-3.6V
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		channel_config.acq_time = NRF_SAADC_ACQTIME_3US; // Set acquisition time. Set high short time to save power.
//		channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
//		channel_config.pin_p = NRF_SAADC_INPUT_AIN1; // Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
//		channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
//		channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED; // Disable pullup resistor on the input pin
//		channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; // Disable pulldown resistor on the input pin
//		error = nrf_drv_saadc_channel_init(CB_ADC_INPUT_NUMBER_SO2 , &channel_config); // Initialize SAADC AIN0 with the channel configuration
//	    APP_ERROR_CHECK(error);
//		// ---------------------------
//		// AIN6: CO
//		// ---------------------------
//		if (600 == CB_ADC_VREF_MV)
//		{
//			channel_config.reference = NRF_SAADC_REFERENCE_INTERNAL; // Set internal reference of fixed 0.6 volts
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		if (4 == CB_ADC_CO_MEASUREMENT_ADC_ATTENUATION)
//		{
//			channel_config.gain = NRF_SAADC_GAIN1_4; //Set input gain to 1/4. The maximum SAADC input voltage is then 0.6V/(1/4)=2.4V. The single ended input range is then 0V-3.6V
//		}
//		else
//		{
//			APP_ERROR_CHECK(CB_ADC_ILLEGAL_PARAMETER);
//		}
//		channel_config.acq_time = NRF_SAADC_ACQTIME_3US; // Set acquisition time. Set high short time to save power.
//		channel_config.mode = NRF_SAADC_MODE_SINGLE_ENDED; // Set SAADC as single ended. This means it will only have the positive pin as input, and the negative pin is shorted to ground (0V) internally.
//		channel_config.pin_p = NRF_SAADC_INPUT_AIN1; // Select the input pin for the channel. AIN1 pin maps to physical pin P0.03.
//		channel_config.pin_n = NRF_SAADC_INPUT_DISABLED; // Since the SAADC is single ended, the negative pin is disabled. The negative pin is shorted to ground internally.
//		channel_config.resistor_p = NRF_SAADC_RESISTOR_DISABLED; // Disable pullup resistor on the input pin
//		channel_config.resistor_n = NRF_SAADC_RESISTOR_DISABLED; // Disable pulldown resistor on the input pin
//		error = nrf_drv_saadc_channel_init(CB_ADC_INPUT_NUMBER_CO , &channel_config); // Initialize SAADC AIN0 with the channel configuration
//	    APP_ERROR_CHECK(error);
		break;
	default:
		break;  // Do not initialize ADC if not used.
	}
	m_AdcCalibrationIsRunning = true;
	error = nrf_drv_saadc_calibrate_offset(); // Start calibration. Function is non-blocking and trigger DONE and RESULTDONE events.
	APP_ERROR_CHECK(error);
	// This will be called in CB_AdcCallback() after calibration is done: nrf_drv_saadc_buffer_convert(m_AdcBuffer,m_AdcBufferLength_Samples); //Set SAADC buffer 1. The SAADC will start to write to this buffer
	return error;
}

void CB_SetSensorValuesToUndefined(void)
{
	// Sensor module types
	m_stTempHumSensor.eTempHumSensorType = CB_eTempHumSensorUndefined;
	m_stPressureSensor.ePressSensorType = CB_ePressSensorUndefined;
	m_stVocSensor.eVocSensorType = CB_eVocSensorUndefined;
	m_stO3Sensor.eElectroChemicalSensorType = CB_EcSensorUndefined;
	m_stNO2Sensor.eElectroChemicalSensorType = CB_EcSensorUndefined;
	m_stSO2Sensor.eElectroChemicalSensorType = CB_EcSensorUndefined;
	m_stCOSensor.eElectroChemicalSensorType = CB_EcSensorUndefined;

	// Sensor values to undefined
	m_stTempHumSensor.temperature_DegC = CB_UNDEFINED_TEMPERATURE_DEGC;
	m_stTempHumSensor.temperatureAvg_DegC = CB_UNDEFINED_TEMPERATURE_DEGC;
	m_stTempHumSensor.humidity_Perc = CB_UNDEFINED_HUMIDITY_PERC;
	m_stTempHumSensor.humidityAvg_Perc = CB_UNDEFINED_HUMIDITY_PERC;

	m_stPressureSensor.pressure_Pa = CB_UNDEFINED_PRESSURE_Pa;
	m_stPressureSensor.pressureAvg_Pa = CB_UNDEFINED_PRESSURE_Pa;
	m_stPressureSensor.pressureAvg_hPa = CB_UNDEFINED_PRESSURE_hPa;
	m_stPressureSensor.temperature_DegC = CB_UNDEFINED_TEMPERATURE_DEGC;

	m_stVocSensor.tvoc_ppb = CB_UNDEFINED_TVOC_PPB;
	m_stVocSensor.tvocAvg_ppb = CB_UNDEFINED_TVOC_PPB;
	m_stVocSensor.tvocCalcAvg_ppb = CB_UNDEFINED_TVOC_PPB;
	m_stVocSensor.tvocBaseline_ppb = CB_UNDEFINED_TVOC_BASELINE_PPB;
	m_stVocSensor.ethanol_ppb = CB_UNDEFINED_ETHANOL_VALUE_PPB;
	m_stVocSensor.ethanolAvg_ppb = CB_UNDEFINED_ETHANOL_VALUE_PPB;
	m_stVocSensor.ethanolAvgPeak_ppb = CB_UNDEFINED_ETHANOL_VALUE_PPB;
	m_stVocSensor.ethanolRef_ppb = CB_UNDEFINED_ETHANOL_REFERENCE_VALUE_PPB;

	m_stO3Sensor.concentrationAvg_ppb = CB_UNDEFINED_O3_PPB;
	m_stO3Sensor.concentrationAvg_lsb = CB_UNDEFINED_O3_LSB;
	m_stO3Sensor.concentrationAvgPeak_lsb = CB_UNDEFINED_O3_LSB;
	m_stO3Sensor.sensitivity_nAPerPpb = CB_UNDEFINED_SENSITIVITY_O3_110401_nAPerPpb;
	m_stO3Sensor.biasVoltage_mV = CB_UNDEFINED_BIAS_VOLTATE_O3_110401_mV;

	m_stNO2Sensor.concentrationAvg_ppb = CB_UNDEFINED_NO2_PPB;
	m_stNO2Sensor.concentrationAvg_lsb = CB_UNDEFINED_NO2_LSB;
	m_stNO2Sensor.concentrationAvgPeak_lsb = CB_UNDEFINED_NO2_LSB;
	m_stNO2Sensor.sensitivity_nAPerPpb = CB_UNDEFINED_SENSITIVITY_NO2_110501_nAPerPpb;
	m_stNO2Sensor.biasVoltage_mV = CB_UNDEFINED_BIAS_VOLTATE_NO2_110501_mV;

	m_stSO2Sensor.concentrationAvg_ppb = CB_UNDEFINED_SO2_PPB;
	m_stSO2Sensor.concentrationAvg_lsb = CB_UNDEFINED_SO2_LSB;
	m_stSO2Sensor.concentrationAvgPeak_lsb = CB_UNDEFINED_SO2_LSB;
	m_stSO2Sensor.sensitivity_nAPerPpb = CB_UNDEFINED_SENSITIVITY_SO2_110601_nAPerPpb;
	m_stSO2Sensor.biasVoltage_mV = CB_UNDEFINED_BIAS_VOLTATE_SO2_110601_mV;

	m_stCOSensor.concentrationAvg_ppb = CB_UNDEFINED_CO_PPB;
	m_stCOSensor.concentrationAvg_lsb = CB_UNDEFINED_CO_LSB;
	m_stCOSensor.concentrationAvgPeak_lsb = CB_UNDEFINED_CO_LSB;
	m_stCOSensor.sensitivity_nAPerPpb = CB_UNDEFINED_SENSITIVITY_CO_110102_nAPerPpb;
	m_stCOSensor.biasVoltage_mV = CB_UNDEFINED_BIAS_VOLTATE_CO_110102_mV;
}

void CB_SetSumsToZero(void)
{
	m_TemperatureSum_DegC = 0;
	m_HumiditySum_Perc = 0;
	//m_BatteryVotageSum_LSB = 0; // Set it to zero in AdcCallback()
	m_EthSum_ppb = 0;
	//m_TvocSum_ppb = 0; // Use SGPC3 with MeasureAirQuality() (instead of using ethanol raw signal and calculate TVOC itself)
	m_PressureSum_Pa = 0;
	//m_O3Sum_LSB = 0; // Set it to zero in AdcCallback()
	//m_NO2Sum_LSB = 0; // Set it to zero in AdcCallback()
	//m_SO2Sum_LSB = 0; // Set it to zero in AdcCallback()
	//m_COSum_LSB = 0; // Set it to zero in AdcCallback()
}

void CB_SensorsInit(void)
{
	uint32_t err_code = CB_NO_ERROR;
	uint16_t u16Id;

	CB_SetSensorValuesToUndefined(); // Set values to undefined BEFORE read calibration values, otherwise the read values would be overwritten
	CB_ReadCalibDataFromPersistentStorage(); // Read calibration values from flash BEFORE do initialization
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		m_stTempHumSensor.eTempHumSensorType = CB_eSHTC1;
		err_code = CB_SHTC1_Init();
		APP_ERROR_CHECK(err_code);
		err_code = CB_SHTC1_GetId(&u16Id);
		APP_ERROR_CHECK(err_code);
		m_stTempHumSensor.deviceId = (uint32_t)(u16Id);
		break;
	case CB_eIAQ_SHTC1_SGPC3:
		m_stTempHumSensor.eTempHumSensorType = CB_eSHTC1;
		m_stVocSensor.eVocSensorType = CB_eSGPC3;
		err_code = CB_SHTC1_Init();
		APP_ERROR_CHECK(err_code);
		err_code = CB_SHTC1_GetId(&u16Id);
		APP_ERROR_CHECK(err_code);
		m_stTempHumSensor.deviceId = (uint32_t)(u16Id);
		// Use SGPC3 ethanol raw signal and calculate TVOC in this firmware
		// If no baseline is available or the most recent baseline is more than one week old, it must discarded
		if (CB_UNDEFINED_ETHANOL_REFERENCE_VALUE_PPB == m_stVocSensor.ethanolRef_ppb)
		{
			m_stVocSensor.ethanolRef_ppb = CB_DEFAULT_ETHANOL_REFERENCE_VALUE_PPB;
			m_stVocSensor.initHighPowerDuration_sec = SGPC3_INIT_DURATION_INVALID_BASELINE_SEC;
			m_stVocSensor.tvocBaseline_ppb = CB_DEFAULT_TVOC_BASELINE_PPB; // Set a default baseline to start with
		}
		/* // Use SGPC3 with MeasureAirQuality() (instead of using ethanol raw signal and calculate TVOC itself)
		// If no baseline is available or the most recent baseline is more than one week old, it must discarded
		if (CB_UNDEFINED_TVOC_BASELINE_PPB == m_stVocSensor.tvocBaseline_ppb)
		{
			m_stVocSensor.initHighPowerDuration_sec = SGPC3_INIT_DURATION_INVALID_BASELINE_SEC;
			m_stVocSensor.tvocBaseline_ppb = CB_DEFAULT_TVOC_BASELINE_PPB; // Set a default baseline to start with
		}
		else
		{
			m_stVocSensor.initHighPowerDuration_sec = SGPC3_INIT_DURATION_VALID_BASELINE_SEC;
		}*/
		err_code = CB_SGPC3_InitAirQualityMeasurement(m_stVocSensor.initHighPowerDuration_sec);
		APP_ERROR_CHECK(err_code);
		err_code = CB_SGPC3_SetBaseline(m_stVocSensor.tvocBaseline_ppb); // Write it to SGPC3 AFTER InitAirQuality()
		APP_ERROR_CHECK(err_code);
		err_code = CB_SGPC3_GetFeatureSetId(&u16Id);
		APP_ERROR_CHECK(err_code);
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		// T&H sensor
		m_stTempHumSensor.eTempHumSensorType = CB_eSHT30;
		err_code = CB_SHT30_Init();
		APP_ERROR_CHECK(err_code);
		// O3 sensor
		m_stO3Sensor.eElectroChemicalSensorType = CB_eO3_Spec110401; // TODO: read sensor type from EEPROM
		m_stO3Sensor.sensitivity_nAPerPpb = CB_DEFAULT_SENSITIVITY_O3_110401_nAPerPpb; // TODO: read sensitivity from EEPROM
		m_stO3Sensor.zeroValue_lsb = CB_ADC_FULL_SCALE/2; // TODO: zero value from EEPROM
		m_stO3Sensor.biasVoltage_mV = CB_DEFAULT_BIAS_VOLTATE_O3_110401_mV; // TODO: read bias voltage from EEPROM
		m_stO3Afe.moduleEnablePinNumber = CB_LMP91000_nENABLE_O3;
		m_stO3Afe.vRefValueV = CB_LMP91000_VOLTAGE_REFERENCE_V; // TODO: read sensor type from EEPROM
		m_stO3Afe.rFeedback = eFeedbackResistorExternal; // TODO: read sensor type from EEPROM
		m_stO3Afe.rLoad = eLoadResistor100Ohm; // TODO: read sensor type from EEPROM
		m_stO3Afe.vRefSource = eExternalVoltageReference; // TODO: read sensor type from EEPROM
		m_stO3Afe.zeroSelection = eInternalZeroSelection67Perc; // TODO: read sensor type from EEPROM
		m_stO3Afe.mode = eStandby;
		m_stO3Afe.fetShortFeature = eShortingFetFeatureDisabled;
		if (m_stO3Sensor.biasVoltage_mV > 0)
		{
			m_stO3Afe.biasVoltageSign = ePositiveBias;
		}
		else
		{
			m_stO3Afe.biasVoltageSign = eNegativeBias;
		}
		m_stO3Afe.biasVoltageSelection = CB_LMP91000_GetBiasVoltageSelectionValue(m_stO3Sensor.biasVoltage_mV, m_stO3Afe.vRefValueV);
		err_code = CB_LMP91000_GetStatus(m_stO3Afe.moduleEnablePinNumber, &m_stO3Afe.status);
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Init(&m_stO3Afe);
		APP_ERROR_CHECK(err_code);
		// NO2 sensor
		m_stNO2Sensor.eElectroChemicalSensorType = CB_eNO2_Spec110501 ; // TODO: read sensor type from EEPROM
		m_stNO2Sensor.sensitivity_nAPerPpb = CB_DEFAULT_SENSITIVITY_NO2_110501_nAPerPpb; // TODO: read sensitivity from EEPROM
		m_stNO2Sensor.zeroValue_lsb = CB_ADC_FULL_SCALE/2; // TODO: zero value from EEPROM
		m_stNO2Sensor.biasVoltage_mV = CB_DEFAULT_BIAS_VOLTATE_NO2_110501_mV; // TODO: read bias voltage from EEPROM
		m_stNO2Afe.moduleEnablePinNumber = CB_LMP91000_nENABLE_NO2;
		m_stNO2Afe.vRefValueV = CB_LMP91000_VOLTAGE_REFERENCE_V; // TODO: read sensor type from EEPROM
		m_stNO2Afe.rFeedback = eFeedbackResistorExternal; // TODO: read sensor type from EEPROM
		m_stNO2Afe.rLoad = eLoadResistor100Ohm; // TODO: read sensor type from EEPROM
		m_stNO2Afe.vRefSource = eExternalVoltageReference; // TODO: read sensor type from EEPROM
		m_stNO2Afe.zeroSelection = eInternalZeroSelection67Perc; // TODO: read sensor type from EEPROM
		m_stNO2Afe.mode = eStandby;
		m_stNO2Afe.fetShortFeature = eShortingFetFeatureDisabled;
		if (m_stNO2Sensor.biasVoltage_mV > 0)
		{
			m_stNO2Afe.biasVoltageSign = ePositiveBias;
		}
		else
		{
			m_stNO2Afe.biasVoltageSign = eNegativeBias;
		}
		m_stNO2Afe.biasVoltageSelection = CB_LMP91000_GetBiasVoltageSelectionValue(m_stNO2Sensor.biasVoltage_mV, m_stNO2Afe.vRefValueV);
		err_code = CB_LMP91000_GetStatus(m_stNO2Afe.moduleEnablePinNumber, &m_stNO2Afe.status);
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Init(&m_stNO2Afe);
		APP_ERROR_CHECK(err_code);
		// SO2 sensor
		m_stSO2Sensor.eElectroChemicalSensorType = CB_eSO2_Spec110601; // TODO: read sensor type from EEPROM
		m_stSO2Sensor.sensitivity_nAPerPpb = CB_DEFAULT_SENSITIVITY_SO2_110601_nAPerPpb; // TODO: read sensitivity from EEPROM
		m_stSO2Sensor.zeroValue_lsb = CB_ADC_FULL_SCALE/2; // TODO: zero value from EEPROM
		m_stSO2Sensor.biasVoltage_mV = CB_DEFAULT_BIAS_VOLTATE_SO2_110601_mV; // TODO: read bias voltage from EEPROM
		m_stSO2Afe.moduleEnablePinNumber = CB_LMP91000_nENABLE_SO2;
		m_stSO2Afe.vRefValueV = CB_LMP91000_VOLTAGE_REFERENCE_V; // TODO: read sensor type from EEPROM
		m_stSO2Afe.rFeedback = eFeedbackResistorExternal; // TODO: read sensor type from EEPROM
		m_stSO2Afe.rLoad = eLoadResistor100Ohm; // TODO: read sensor type from EEPROM
		m_stSO2Afe.vRefSource = eExternalVoltageReference; // TODO: read sensor type from EEPROM
		m_stSO2Afe.zeroSelection = eInternalZeroSelection20Perc; // TODO: read sensor type from EEPROM
		m_stSO2Afe.mode = eStandby;
		m_stSO2Afe.fetShortFeature = eShortingFetFeatureDisabled;
		if (m_stSO2Sensor.biasVoltage_mV > 0)
		{
			m_stSO2Afe.biasVoltageSign = ePositiveBias;
		}
		else
		{
			m_stSO2Afe.biasVoltageSign = eNegativeBias;
		}
		m_stSO2Afe.biasVoltageSelection = CB_LMP91000_GetBiasVoltageSelectionValue(m_stSO2Sensor.biasVoltage_mV, m_stSO2Afe.vRefValueV);
		err_code = CB_LMP91000_GetStatus(m_stSO2Afe.moduleEnablePinNumber, &m_stSO2Afe.status);
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Init(&m_stSO2Afe);
		APP_ERROR_CHECK(err_code);
		// CO sensor
		m_stCOSensor.eElectroChemicalSensorType = CB_eCO_Spec110102; // TODO: read sensor type from EEPROM
		m_stCOSensor.sensitivity_nAPerPpb = CB_DEFAULT_SENSITIVITY_CO_110102_nAPerPpb; // TODO: read sensitivity from EEPROM
		m_stCOSensor.zeroValue_lsb = (uint16_t)(((float)CB_ADC_FULL_SCALE)/5.0f); // TODO: zero value from EEPROM
		m_stCOSensor.biasVoltage_mV = CB_DEFAULT_BIAS_VOLTATE_CO_110102_mV; // TODO: read bias voltage from EEPROM
		m_stCOAfe.moduleEnablePinNumber = CB_LMP91000_nENABLE_CO;
		m_stCOAfe.vRefValueV = CB_LMP91000_VOLTAGE_REFERENCE_V; // TODO: read sensor type from EEPROM
		m_stCOAfe.rFeedback = eFeedbackResistorExternal; // TODO: read sensor type from EEPROM
		m_stCOAfe.rLoad = eLoadResistor100Ohm; // TODO: read sensor type from EEPROM
		m_stCOAfe.vRefSource = eExternalVoltageReference; // TODO: read sensor type from EEPROM
		m_stCOAfe.zeroSelection = eInternalZeroSelection20Perc; // TODO: read sensor type from EEPROM
		m_stCOAfe.mode = eStandby;
		m_stCOAfe.fetShortFeature = eShortingFetFeatureDisabled;
		if (m_stCOSensor.biasVoltage_mV > 0)
		{
			m_stCOAfe.biasVoltageSign = ePositiveBias;
		}
		else
		{
			m_stCOAfe.biasVoltageSign = eNegativeBias;
		}
		m_stCOAfe.biasVoltageSelection = CB_LMP91000_GetBiasVoltageSelectionValue(m_stCOSensor.biasVoltage_mV, m_stCOAfe.vRefValueV);
		err_code = CB_LMP91000_GetStatus(m_stCOAfe.moduleEnablePinNumber, &m_stCOAfe.status);
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Init(&m_stCOAfe);
		APP_ERROR_CHECK(err_code);
		// Pressure sensor
		m_stPressureSensor.ePressSensorType = CB_eLPS22HB;
		err_code = CB_LPS22HB_Init();
		APP_ERROR_CHECK(err_code);
		uint8_t deviceId;
		err_code = CB_LPS22HB_Get_DeviceID(&deviceId);
		APP_ERROR_CHECK(err_code);
		m_stPressureSensor.deviceId = deviceId;
		CB_LPS22HB_DriverVersion_st lps22hbDriverVersion;
		err_code = CB_LPS22HB_Get_DriverVersion(&lps22hbDriverVersion);
		APP_ERROR_CHECK(err_code);
		m_stPressureSensor.driverVersion.major = lps22hbDriverVersion.Major;
		m_stPressureSensor.driverVersion.minor = lps22hbDriverVersion.Minor;
		m_stPressureSensor.driverVersion.revision = lps22hbDriverVersion.Point;
		break;
	default:
		err_code = CB_ILLEGAL_MODULE_CONFIG;
	}
	// Write calibration values to storage AFTER do initialization (so the current values are saved)
	// TODO write calibration values to EEPROM: err_code = CB_WriteCalibValuesToStorage();
	APP_ERROR_CHECK(err_code);
}

void CB_CalculateSensorAverageValues(void)
{
	// ---------------------------
	// Every module measures temperature, humidity and battery voltage
	// ---------------------------
	m_stTempHumSensor.temperatureAvg_DegC = m_TemperatureSum_DegC/(float)m_AveragingLength_Samples;
	m_stTempHumSensor.humidityAvg_Perc = m_HumiditySum_Perc/(float)m_AveragingLength_Samples;

	// ---------------------------
	// Here the module depending sensors
	// ---------------------------
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		break;
	case CB_eIAQ_SHTC1_SGPC3:
		m_stVocSensor.ethanolAvg_ppb = (uint16_t)((float)m_EthSum_ppb/(float)m_AveragingLength_Samples);
		m_stVocSensor.tvocCalcAvg_ppb = CB_SGPC3_GetCalculatedTvoc(m_stVocSensor.ethanolRef_ppb, m_stVocSensor.ethanolAvg_ppb, CB_SGPC3_2HZ_TO_10HZ_TVOC_CORRECTION_FACTOR); // Do this calculation AFTER calculating ethanol average
		//m_stVocSensor.tvocAvg_ppb = m_TvocSum_ppb/m_AveragingLength_Samples; // Use SGPC3 with MeasureAirQuality() (instead of using ethanol raw signal and calculate TVOC itself)
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		m_stPressureSensor.pressureAvg_Pa = (int32_t)((float)m_PressureSum_Pa/(float)m_AveragingLength_Samples);
		m_stPressureSensor.pressureAvg_hPa = (float)((float)m_stPressureSensor.pressureAvg_Pa/100.0f);
		break;
	}
}

void CB_HandleEthanolBaselineAlgorithm(void)
{
	if (m_stVocSensor.ethanolAvgPeak_ppb  < m_stVocSensor.ethanolAvg_ppb)
	{
		m_stVocSensor.ethanolAvgPeak_ppb = m_stVocSensor.ethanolAvg_ppb; // Assign highest value (this may happen at power-on when "m_stVocSensor.ethanolAvgPeak_ppb = 0"
	}
	if (m_stVocSensor.ethanolRef_ppb < m_stVocSensor.ethanolAvg_ppb)
	{
		// If "average ethanol value" is bigger than "reference ethanol value" --> increase "reference ethanol value"
		if ((CB_DEFAULT_ETHANOL_REFERENCE_VALUE_PPB == m_stVocSensor.ethanolRef_ppb) ||
			(CB_UNDEFINED_ETHANOL_REFERENCE_VALUE_PPB == m_stVocSensor.ethanolRef_ppb))
		{
			m_stVocSensor.ethanolRef_ppb = m_stVocSensor.ethanolAvg_ppb; // Set sref = sout if sref was undefined before
		}
		else
		{
			m_stVocSensor.ethanolRef_ppb = (uint16_t)((int32_t)(m_stVocSensor.ethanolRef_ppb) + ((int32_t)m_stVocSensor.ethanolAvg_ppb - (int32_t)m_stVocSensor.ethanolRef_ppb)/2);
		}
	}
	else
	{
		// Adapt the "reference ethanol value" within e.g. one week
		if ((0==(m_CloudUpdateCounter%m_BaselineUpdateToCloudUpdate_Ratio)) && (0!=m_CloudUpdateCounter))
		{
			if (m_stVocSensor.ethanolRef_ppb > m_stVocSensor.ethanolAvgPeak_ppb)
			{
				// Correct ethanol reference signal for clean air (sref) if it is bigger than ethanol peak signal during e.g. 1 week
				m_stVocSensor.ethanolRef_ppb = (uint16_t)((int32_t)(m_stVocSensor.ethanolRef_ppb) - ((int32_t)m_stVocSensor.ethanolRef_ppb - (int32_t)m_stVocSensor.ethanolAvgPeak_ppb)/2);
				m_stVocSensor.ethanolAvgPeak_ppb = 0; // Set peak value to zero and restart the next cycle of adaptive baseline correction
			}
		}
	}
}

void CB_HandleOzoneBaselineAlgorithm(void)
{
	// The higher the value in [LSB], the lower the O3 concentration
	if (m_stO3Sensor.concentrationAvgPeak_lsb < m_stO3Sensor.concentrationAvg_lsb)
	{
		m_stO3Sensor.concentrationAvgPeak_lsb = m_stO3Sensor.concentrationAvg_lsb; // Assign highest value (this may happen at power-on when "m_stVocSensor.ethanolAvgPeak_ppb = 0"
	}
	if (m_stO3Sensor.zeroValue_lsb < m_stO3Sensor.concentrationAvg_lsb)
	{
		// If "average O3 value" is bigger than "zero baseline value" --> increase "zero baseline value"
		if (CB_UNDEFINED_O3_LSB == m_stO3Sensor.zeroValue_lsb)
		{
			m_stO3Sensor.zeroValue_lsb = m_stO3Sensor.concentrationAvg_lsb; // Set zero baseline equal average concentration, this is true when starting up
		}
		else
		{
			m_stO3Sensor.zeroValue_lsb = (uint16_t)((int32_t)(m_stO3Sensor.zeroValue_lsb) + ((int32_t)m_stO3Sensor.concentrationAvg_lsb - (int32_t)m_stO3Sensor.zeroValue_lsb)/2);
		}
	}
	else
	{
		// Adapt the "zero baseline value" within e.g. one week
		if ((0==(m_CloudUpdateCounter%m_BaselineUpdateToCloudUpdate_Ratio)) && (0!=m_CloudUpdateCounter))
		{
			if (m_stO3Sensor.zeroValue_lsb > m_stO3Sensor.concentrationAvgPeak_lsb)
			{
				// Correct zero baseline value for clean air if it is bigger than O3 peak signal during e.g. 1 week
				m_stO3Sensor.zeroValue_lsb = (uint16_t)((int32_t)(m_stO3Sensor.zeroValue_lsb) - ((int32_t)m_stO3Sensor.zeroValue_lsb - (int32_t)m_stO3Sensor.concentrationAvgPeak_lsb)/2);
				m_stO3Sensor.zeroValue_lsb = 0; // Set peak value to zero and restart the next cycle of adaptive baseline correction
			}
		}
	}
}

void CB_HandleBaselineAlgorithm()
{
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		break;
	case CB_eIAQ_SHTC1_SGPC3:
		CB_HandleEthanolBaselineAlgorithm();
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		CB_HandleOzoneBaselineAlgorithm();
		break;
	default:
		break;
	}
}

void CB_SendDataToCloud(void)
{
	uint32_t err_code = CB_NO_ERROR;
	m_CloudUpdateCounter++;

	// ---------------------------
	// Write calibration values to storage every time data are sent to cloud
	// ---------------------------
	err_code = CB_WriteCalibValuesToStorage();
	APP_ERROR_CHECK(err_code);
    
	// ---------------------------
	// Baseline algorithms
	// ---------------------------
	CB_HandleBaselineAlgorithm(); // Do baseline algorithm at same interval like cloud update

	// ---------------------------
    // If IPv6 address of the cloud is unspecified try to resolve hostname.
	// ---------------------------
    if (!thread_coap_utils_peer_addr_is_set())
    {
        UNUSED_VARIABLE(thread_dns_utils_hostname_resolve(thread_ot_instance_get(), m_cloud_information.p_cloud_hostname, dns_response_handler, NULL));
        return;
    }

    // ---------------------------
    // Send data to cloud
    // ---------------------------
    char payload_buffer[256];
    uint16_t temperatureFix_DegC;
	uint16_t humidityFix_Perc;
    switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		temperatureFix_DegC = (uint16_t)(m_stTempHumSensor.temperatureAvg_DegC);
		humidityFix_Perc = (uint16_t)(m_stTempHumSensor.humidityAvg_Perc);
		sprintf(payload_buffer,
				"{\"values\":[{\"key\":\"%s\",\"value\":\"%d.%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"}]}",
				"t", temperatureFix_DegC, (uint16_t)((m_stTempHumSensor.temperatureAvg_DegC-temperatureFix_DegC)*10),
				"h", humidityFix_Perc, "b", m_stModule.batteryVoltage_mV);
		thread_coap_utils_cloud_data_update(&m_cloud_information, payload_buffer);
		break;
	case CB_eIAQ_SHTC1_SGPC3:
		temperatureFix_DegC = (uint16_t)(m_stTempHumSensor.temperatureAvg_DegC);
		humidityFix_Perc = (uint16_t)(m_stTempHumSensor.humidityAvg_Perc);
		sprintf(payload_buffer,
				"{\"values\":[{\"key\":\"%s\",\"value\":\"%d.%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"}]}",
				"t", temperatureFix_DegC, (uint16_t)((m_stTempHumSensor.temperatureAvg_DegC-temperatureFix_DegC)*10),
				"h", humidityFix_Perc, "vc", m_stVocSensor.tvocCalcAvg_ppb, "e", m_stVocSensor.ethanolAvg_ppb, "b", m_stModule.batteryVoltage_mV);
//		sprintf(payload_buffer,
//						"{\"values\":[{\"key\":\"%s\",\"value\":\"%d.%d\"}]}",
//						"t", temperatureFix_DegC, (uint16_t)((m_stTempHumSensor.temperatureAvg_DegC-temperatureFix_DegC)*10));
		thread_coap_utils_cloud_data_update(&m_cloud_information, payload_buffer);
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		temperatureFix_DegC = (uint16_t)(m_stTempHumSensor.temperatureAvg_DegC);
		humidityFix_Perc = (uint16_t)(m_stTempHumSensor.humidityAvg_Perc);
		sprintf(payload_buffer,
				"{\"values\":[{\"key\":\"%s\",\"value\":\"%d.%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"},{\"key\":\"%s\",\"value\":\"%d\"}]}",
				"t", temperatureFix_DegC, (uint16_t)((m_stTempHumSensor.temperatureAvg_DegC-temperatureFix_DegC)*10),
				"h", humidityFix_Perc, "o", m_stO3Sensor.concentrationAvg_ppb, "n", m_stNO2Sensor.concentrationAvg_ppb, "c", m_stCOSensor.concentrationAvg_ppb, "s", m_stSO2Sensor.concentrationAvg_ppb, "b", m_stModule.batteryVoltage_mV);
		thread_coap_utils_cloud_data_update(&m_cloud_information, payload_buffer);
		break;
	default:
		err_code = CB_ILLEGAL_MODULE_CONFIG;
	}
	APP_ERROR_CHECK(err_code);

	// ---------------------------
    // Reload watchdog register after a successful cloud update
	// ---------------------------
    CB_WatchdogReload();
}

void CB_GetSensorData(void)
{
	uint32_t err_code;

	// ---------------------------
	// Every module measures temperature, humidity and battery voltage
	// ---------------------------
	nrf_drv_saadc_sample(); // Do start ADC measurements, do no error handling

	// ---------------------------
	// Here the module depending sensors
	// ---------------------------
	switch (m_stModule.eModuleType)
	{
	case CB_eTH_SHTC1:
		err_code = CB_SHTC1_GetTempAndHumi(&m_stTempHumSensor.temperature_DegC, &m_stTempHumSensor.humidity_Perc);
		APP_ERROR_CHECK(err_code);

		break;
	case CB_eIAQ_SHTC1_SGPC3:
		err_code = CB_SHTC1_GetTempAndHumi(&m_stTempHumSensor.temperature_DegC, &m_stTempHumSensor.humidity_Perc);
		APP_ERROR_CHECK(err_code);
		/* // Use SGPC3 with MeasureAirQuality() (instead of using ethanol raw signal and calculate TVOC itself)
		err_code = CB_SGPC3_MeasureAirQuality(&m_stVocSensor.tvoc_ppb);
		APP_ERROR_CHECK(err_code);
		err_code = CB_SGPC3_GetBaseline(&m_stVocSensor.tvocBaseline_ppb);
		APP_ERROR_CHECK(err_code);*/
		// Use SGPC3 ethanol raw signal and calculate TVOC in this firmware
		err_code = CB_SGPC3_MeasureRawSignal(&m_stVocSensor.ethanol_ppb);
		APP_ERROR_CHECK(err_code);
		// Add measured data to sum for average value calculation
		m_EthSum_ppb += m_stVocSensor.ethanol_ppb;
		// m_TvocSum_ppb += m_stVocSensor.tvoc_ppb;
		break;
	case CB_eOAQ_SHTC30_LPS22HB_SPEC_O3_NO2_SO2_CO:
		err_code = CB_SHT30_GetTempAndHumi(&m_stTempHumSensor.temperature_DegC, &m_stTempHumSensor.humidity_Perc);
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Set3LeadAmperometricCellMode(&m_stO3Afe); // Activate amperometric cell here and set back to standby-mode in ADC callback function
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Set3LeadAmperometricCellMode(&m_stNO2Afe); // Activate amperometric cell here and set back to standby-mode in ADC callback function
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Set3LeadAmperometricCellMode(&m_stSO2Afe); // Activate amperometric cell here and set back to standby-mode in ADC callback function
		APP_ERROR_CHECK(err_code);
		err_code = CB_LMP91000_Set3LeadAmperometricCellMode(&m_stCOAfe); // Activate amperometric cell here and set back to standby-mode in ADC callback function
		APP_ERROR_CHECK(err_code);
		err_code = CB_LPS22HB_Get_Pressure(&m_stPressureSensor.pressure_Pa);
		APP_ERROR_CHECK(err_code);
		// Add measured data to sum for average value calculation
		m_PressureSum_Pa += m_stPressureSensor.pressure_Pa;
		break;
	default:
		err_code = CB_ILLEGAL_MODULE_CONFIG;
	}
	// Temperature and humidity are used by every module, sum up at the end
	m_TemperatureSum_DegC += m_stTempHumSensor.temperature_DegC;
	m_HumiditySum_Perc += m_stTempHumSensor.humidity_Perc;
	APP_ERROR_CHECK(err_code);
}

void CB_RtcHandlerSensorData(nrf_drv_rtc_int_type_t int_type)
{
    uint32_t err_code;

    // Disable RTC0
  	nrf_drv_rtc_disable(&m_SensorDataIsrRtc);

    switch (int_type)
    {
    case NRF_DRV_RTC_INT_COMPARE0:
    	if (0 == m_DoSampleContinuously)
    	{
    		CB_GetSensorData();
    		m_SampleDataCounter++;
      		if (m_AveragingLength_Samples > m_SampleDataCounter)
        	{
        		// Sleep until next data sampling is pending
            	err_code = nrf_drv_rtc_cc_set(&m_SensorDataIsrRtc,CB_RTC_CC_INSTANCE_SENSOR_SAMPLING,m_RtcCcValueForSensorSampling,true); // Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match, interrupt is enabled
                APP_ERROR_CHECK(err_code);
        	}
        	else // (m_AveragingLength_Samples == m_sampleDataCounter) // Calculate average value and sent to cloud
        	{
        		// START SAMPLING AS CLOSE AS POSSIBLE TO CLOUD-UPDATE --- SEND LAST VALID AVERAGE VALUE TO CLOUD
            	m_SampleDataCounter = 0;
            	CB_CalculateSensorAverageValues(); // Calculate average values
            	CB_SetSumsToZero(); // Set all sums to zero for next cycle
            	CB_SendDataToCloud(); // Send data to cloud
            	// Sleep until short before next cloud update is pending
            	err_code = nrf_drv_rtc_cc_set(&m_SensorDataIsrRtc,CB_RTC_CC_INSTANCE_SENSOR_SAMPLING,(CB_CLOUD_UPDATE_RATE_SEC-(m_AveragingLength_Samples*m_SamplingRate_Sec)),true); // Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match, interrupt is enabled
            	APP_ERROR_CHECK(err_code);
        	}
    	}
    	else // (1 == m_DoSampleContinuously)
    	{
    		CB_GetSensorData();
      		m_SampleDataCounter++;
    		if(m_CloudUpdateRateToSampleRate_Ratio > m_SampleDataCounter)
    		{
    			if (m_SampleDataCounter%m_AveragingLength_Samples == 0)
    			{
    				CB_CalculateSensorAverageValues(); // Calculate average values
    				CB_SetSumsToZero(); // Set all sums to zero for next cycle
    			}
    		}
    		else // (CB_CLOUD_UPDATE_TO_SAMPLE_RATE_RATIO == m_sampleDataCounter)
    		{
    			// ALWAYS SAMPLE --- BUT ONLY SEND DATA TO CLOUD EVERY NOW AND THEN
    			m_SampleDataCounter = 0;
    			CB_SetSumsToZero(); // Set all sums to zero for next cycle
    			CB_SendDataToCloud(); // Send data to cloud
    		}
    		// Sleep until next data sampling is pending
        	err_code = nrf_drv_rtc_cc_set(&m_SensorDataIsrRtc,CB_RTC_CC_INSTANCE_SENSOR_SAMPLING,m_RtcCcValueForSensorSampling,true); // Set RTC compare value. This needs to be done every time as the nrf_drv_rtc clears the compare register on every compare match, interrupt is enabled
        	APP_ERROR_CHECK(err_code);
    	}

    	// Clear the RTC counter to start count from zero
        nrf_drv_rtc_counter_clear(&m_SensorDataIsrRtc);
        // Enable RTC0
      	nrf_drv_rtc_enable(&m_SensorDataIsrRtc);
    	break;
    default:
    	APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA); // This should not happen: capture compare not configured
    }
}

void CB_RtcInit(void)
{
	uint32_t err_code;

	// LFCLK --> the low-frequency clock 32768Hz is already requested by the OpenThread framework
	// LFCLK --> therefore: no call of nrf_drv_clock_init() and nrf_drv_clock_lfclk_request() here

	// Initialize RTC instance
	nrf_drv_rtc_config_t rtc_config;
	rtc_config.prescaler = RTC_FREQ_TO_PRESCALER(CB_RTC_FREQUENCY_HZ);
	rtc_config.interrupt_priority = CB_RTC_INTERRUPT_PRIORITY;
	err_code = nrf_drv_rtc_init(&m_SensorDataIsrRtc, &rtc_config, CB_RtcHandlerSensorData); // Initialize the RTC with callback function rtc_handler. The rtc_handler must be implemented in this application. Passing NULL here for RTC configuration means that configuration will be taken from the sdk_config.h file.
	APP_ERROR_CHECK(err_code);

	// Set here RTC sleep time = sampling-time --> because we want to have the 1st measurement result as soon as possible
	err_code = nrf_drv_rtc_cc_set(&m_SensorDataIsrRtc,CB_RTC_CC_INSTANCE_SENSOR_SAMPLING,m_RtcCcValueForSensorSampling,true); // Set RTC compare value to trigger interrupt. Configure the interrupt frequency by adjust RTC_CC_VALUE and CB_RTC_FREQUENCY_HZ constant in top of main.c
	APP_ERROR_CHECK(err_code);

	// Power on RTC instance
	nrf_drv_rtc_enable(&m_SensorDataIsrRtc); 	// Enable RTC0

	// Start watchdog after RTC ISR is activated --> reload watchdog register in RTC ISR
	CB_WatchdogStart();
}

/***************************************************************************************************
 * @section Main
 **************************************************************************************************/

int main(int argc, char *argv[])
{
    // nRF log init
	log_init();

	// Cellabox initialization
	CB_PowerInit();

	// CoAP example initialization
    CB_ThreadInit();

    // Cellabox initialization
	CB_GpioInit();
	CB_LedInit(); // LED blinking start --> stop when 1st Thread connection established
	CB_GetModuleType(); // AFTER GPIO initialization, otherwise module configuration cannot be read
    CB_SetModuleParameters(); // Set module depending parameters
    CB_WatchdogInit(); // AFTER set module parameters
    CB_I2cInit();
    CB_SensorsInit();
    CB_AdcInit();
    CB_RtcInit(); // Do RTC init at last --> measurement ISR is started here

    while (true)
    {
    	thread_process();
        if (NRF_LOG_PROCESS() == false)
        {
            thread_sleep();
        }
    }
}

/**
 *@}
 **/
