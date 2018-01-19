/**
 * Copyright (c) 2017, Cellabox
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
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
 * 
 */
/** @file
 * @defgroup app_config.h
 * @{
 * @brief Application specific configuration for overwriting sdk_config.h file.
 *
 * IMPORTANT:
 * #define USE_APP_CONFIG in Eclipse Properties --> C/C++ General --> Preprocessor Include Paths Macros (USE_APP_CONFIG)
 * 
 */
 

#ifndef APP_CONFIG_H__
#define APP_CONFIG_H__

/**
 * @defgroup cellabox_application
 * @{
 * @ingroup cellabox
 * @brief 
 */

#define NRF_LOG_DEFAULT_LEVEL 		4 // TODO RKL: set level to 0 for release. 0=OFF, 1=ERROR, 2=WARNING, 3=INFO, 4=DEBUG

#define TWI_INSTANCE_ID				0	// Using TWI0.
#define TWI_ENABLED					1	// Set to 1 to activate. This is an NRF_CONFIG macro.
#define TWI0_ENABLED				1 	// Enable TWI0 instance. Set to 1 to activate. This is an NRF_CONFIG macro.
#define TWI1_ENABLED				0	// Enable TWI1 instance. Set to 1 to activate. This is an NRF_CONFIG macro.
#define TWI0_USE_EASY_DMA			1 	// Use EasyDMA (if present). Set to 1 to activate. This is an NRF_CONFIG macro.
#define TWI1_USE_EASY_DMA			0 	// Use EasyDMA (if present). Set to 1 to activate. This is an NRF_CONFIG macro.
#define TWI1_USE_EASY_DMA			0 	// Use EasyDMA (if present). Set to 1 to activate. This is an NRF_CONFIG macro.
#define RTC_ISR_INSTANCE_ID 		0   // RTC which is used as interrupt source for sensor data read, e.g. RTC0
#define RTC_ENABLED					1   // RTC is enabled and used as interrupt source.
#define RTC0_ENABLED				1   // RTC0 is enabled.
#define RTC1_ENABLED				0   // RTC1 is disabled.
#define RTC2_ENABLED				0   // RTC2 is disabled here, however later in code RTC2 will be enabled by OpenThread library (according to Nordic: Thread uses RTC2)
#define SAADC_ENABLED				1	// SAADC enabled for measuring battery voltage and some sensors.
#define SAADC_CONFIG_IRQ_PRIORITY	7	// This setting is used as default value: 7 = lowest
#define SAADC_CONFIG_LP_MODE		1	// This setting is used as default value: 1 = low-power mode active
#define SAADC_CONFIG_OVERSAMPLE		0	// This setting is used as default value: 0 = disabled
#define SAADC_CONFIG_RESOLUTION		3	// This setting is used as default value: 0 = 8bit, 1 = 10bit, 2 = 12bit, 3 = 14bit


#endif // APP_CONFIG_H__
