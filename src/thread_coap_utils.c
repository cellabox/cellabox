/**
 * Copyright (c) 2017 - 2017, Nordic Semiconductor ASA
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

#include "thread_coap_utils.h"

#include "app_timer.h"
#include "bsp_thread.h"
#include "nrf_assert.h"
#include "nrf_log.h"
#include "sdk_config.h"
#include "thread_utils.h"

#include <openthread/types.h>
#include <openthread/ip6.h>
#include <openthread/platform/alarm-milli.h>

#define RESPONSE_POLL_PERIOD 100

APP_TIMER_DEF(m_provisioning_timer);
APP_TIMER_DEF(m_led_timer);

static uint32_t m_poll_period;

static const otIp6Address m_unspecified_ipv6 =
{
    .mFields =
    {
        .m8 = {0}
    }
};

static thread_coap_configuration_t m_thread_coap_configuration =
{
    .coap_server_enabled                = false,
    .coap_client_enabled                = false,
    .coap_cloud_enabled                 = false,
    .configurable_led_blinking_enabled  = false,
};

static void thread_coap_utils_handler_default(void                * p_context,
                                              otCoapHeader        * p_header,
                                              otMessage           * p_message,
                                              const otMessageInfo * p_message_info)
{
    (void)p_context;
    (void)p_header;
    (void)p_message;
    (void)p_message_info;

    NRF_LOG_INFO("Received CoAP message that does not match any request or resource\r\n");
}

static thread_coap_utils_status_t m_coap_status =
{
    .provisioning_enabled = false,
    .provisioning_expiry  = 0,
    .led_blinking_is_on   = false,
    .peer_address         =
    {
        .mFields =
        {
            .m8 = {0}
        }
    },
    .multicast_light_on   = false,
};

static uint32_t poll_period_response_set(otInstance * p_instance)
{
    uint32_t error;

    do
    {
        if (otThreadGetLinkMode(p_instance).mRxOnWhenIdle)
        {
            error = NRF_ERROR_INVALID_STATE;
            break;
        }

        if (!m_poll_period)
        {
            m_poll_period = otLinkGetPollPeriod(p_instance);
            NRF_LOG_INFO("Poll Period: %dms set\r\n", RESPONSE_POLL_PERIOD);
            otLinkSetPollPeriod(p_instance, RESPONSE_POLL_PERIOD);

            error =  NRF_SUCCESS;
        }
        else
        {
            error = NRF_ERROR_BUSY;
        }
    } while (false);

    return error;
}


static uint32_t poll_period_restore(otInstance * p_instance)
{
    uint32_t error;

    do
    {
        if (otThreadGetLinkMode(p_instance).mRxOnWhenIdle)
        {
            error = NRF_ERROR_INVALID_STATE;
            break;
        }

        if (m_poll_period)
        {
            otLinkSetPollPeriod(p_instance, m_poll_period);
            NRF_LOG_INFO("Poll Period: %dms restored\r\n", m_poll_period);
            m_poll_period = 0;

            error =  NRF_SUCCESS;
        }
        else
        {
            error = NRF_ERROR_BUSY;
        }
    } while (false);

    return error;
}

static void light_changed_default(thread_coap_utils_light_command_t light_command)
{
    switch (light_command)
    {
        case LIGHT_ON:
            LEDS_ON(BSP_LED_3_MASK);
            break;

        case LIGHT_OFF:
            LEDS_OFF(BSP_LED_3_MASK);
            break;

        case LIGHT_TOGGLE:
            LEDS_INVERT(BSP_LED_3_MASK);
            break;

        default:
            break;
    }
}

static light_changed_handler_t m_light_changed = light_changed_default;

static void thread_coap_utils_provisioning_request_handler(void                * p_context,
                                                           otCoapHeader        * p_header,
                                                           otMessage           * p_message,
                                                           const otMessageInfo * p_message_info)
{
    (void)p_message;
    otMessageInfo message_info;

    if (!thread_coap_utils_provisioning_is_enabled())
    {
        return;
    }

    if ((otCoapHeaderGetType(p_header) == OT_COAP_TYPE_NON_CONFIRMABLE) &&
        (otCoapHeaderGetCode(p_header) == OT_COAP_CODE_GET))
    {
        message_info = *p_message_info;
        memset(&message_info.mSockAddr, 0, sizeof(message_info.mSockAddr));
        if (thread_coap_utils_provisioning_response_send(
                p_context, p_header, &message_info) == OT_ERROR_NONE)
        {
                thread_coap_utils_provisioning_enable(false);
        }
    }
}

static void thread_coap_utils_light_request_handler(void                * p_context,
                                                    otCoapHeader        * p_header,
                                                    otMessage           * p_message,
                                                    const otMessageInfo * p_message_info)
{
    uint8_t command;
    do
    {
        if (otCoapHeaderGetType(p_header) != OT_COAP_TYPE_CONFIRMABLE &&
            otCoapHeaderGetType(p_header) != OT_COAP_TYPE_NON_CONFIRMABLE)
        {
            break;
        }

        if (otCoapHeaderGetCode(p_header) != OT_COAP_CODE_PUT)
        {
            break;
        }

        if (otMessageRead(p_message, otMessageGetOffset(p_message), &command, 1) != 1)
        {
            NRF_LOG_INFO("light handler - missing command\r\n");
        }

        m_light_changed((thread_coap_utils_light_command_t)command);

        switch (command)
        {
            case LIGHT_ON:
                if (m_thread_coap_configuration.configurable_led_blinking_enabled)
                {
                    thread_coap_utils_light_blinking_is_on_set(true);
                }
                break;

            case LIGHT_OFF:
                if (m_thread_coap_configuration.configurable_led_blinking_enabled)
                {
                    thread_coap_utils_light_blinking_is_on_set(false);
                }
                break;

            case LIGHT_TOGGLE:
                if (m_thread_coap_configuration.configurable_led_blinking_enabled)
                {
                    thread_coap_utils_light_blinking_is_on_set(
                                            !thread_coap_utils_light_blinking_is_on_get());
                }
                break;

            default:
                break;
        }

        if (otCoapHeaderGetType(p_header) == OT_COAP_TYPE_CONFIRMABLE)
        {
            thread_coap_utils_light_response_send(p_context, p_header, p_message_info);
        }

    } while (false);
}

static thread_coap_utils_resources_t m_coap_resources = {
    .provisioning_resource = {"provisioning", thread_coap_utils_provisioning_request_handler, NULL,
                              NULL},
    .light_resource        = {"light", thread_coap_utils_light_request_handler, NULL, NULL},
};

static void thread_coap_configuration_set(const thread_coap_configuration_t * p_thread_coap_configuration)
{
    m_thread_coap_configuration = *p_thread_coap_configuration;
}

void thread_coap_utils_init(const thread_coap_configuration_t * p_thread_coap_configuration)
{
    otError error = otCoapStart(thread_ot_instance_get(), OT_DEFAULT_COAP_PORT);
    ASSERT(error == OT_ERROR_NONE);

    otCoapSetDefaultHandler(thread_ot_instance_get(), thread_coap_utils_handler_default, NULL);

    thread_coap_configuration_set(p_thread_coap_configuration);

    if (m_thread_coap_configuration.coap_server_enabled)
    {
        m_coap_resources.light_resource.mContext        = thread_ot_instance_get();
        m_coap_resources.provisioning_resource.mContext = thread_ot_instance_get();

        error = otCoapAddResource(thread_ot_instance_get(), &m_coap_resources.light_resource);
        ASSERT(error == OT_ERROR_NONE);

        error = otCoapAddResource(thread_ot_instance_get(), &m_coap_resources.provisioning_resource);
        ASSERT(error == OT_ERROR_NONE);
    }
}

thread_coap_configuration_t * thread_coap_configuration_get()
{
    return &m_thread_coap_configuration;
}

void thread_coap_utils_peer_addr_set(const otIp6Address * p_peer_addr)
{
    m_coap_status.peer_address = *p_peer_addr;
}

otIp6Address * thread_coap_utils_peer_addr_get(void)
{
    return &m_coap_status.peer_address;
}

void thread_coap_utils_peer_addr_clear(void)
{
    m_coap_status.peer_address = m_unspecified_ipv6;
}

bool thread_coap_utils_peer_addr_is_set(void)
{
    return (!otIp6IsAddressEqual(&m_coap_status.peer_address, &m_unspecified_ipv6));
}

void thread_coap_utils_mcast_light_on_set(bool value)
{
    m_coap_status.multicast_light_on = value;
}

bool thread_coap_utils_mcast_light_on_get(void)
{
    return m_coap_status.multicast_light_on;
}

void thread_coap_utils_mcast_light_on_toggle(void)
{
    m_coap_status.multicast_light_on = !m_coap_status.multicast_light_on;
}

void thread_coap_utils_unicast_light_request_send(otInstance * p_instance, uint8_t command)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo message_info;
    otCoapHeader  header;

    do
    {
        if (otIp6IsAddressEqual(&m_coap_status.peer_address, &m_unspecified_ipv6))
        {
            NRF_LOG_INFO("Failed to send the CoAP Request to the Unspecified IPv6 Address\r\n");
            break;
        }

        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
        otCoapHeaderGenerateToken(&header, 2);
        UNUSED_VARIABLE(otCoapHeaderAppendUriPathOptions(&header, "light"));
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&message_info, 0, sizeof(message_info));
        message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        message_info.mPeerPort    = OT_DEFAULT_COAP_PORT;
        memcpy(&message_info.mPeerAddr, &m_coap_status.peer_address, sizeof(message_info.mPeerAddr));

        error = otCoapSendRequest(p_instance,
                                  p_message,
                                  &message_info,
                                  NULL,
                                  p_instance);
    } while (false);

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}

void thread_coap_utils_multicast_light_request_send(otInstance                        * p_instance,
                                                    uint8_t                             command,
                                                    thread_coap_utils_multicast_scope_t scope)
{
    otError       error = OT_ERROR_NONE;
    otMessage   * p_message;
    otMessageInfo message_info;
    otCoapHeader  header;
    const char  * p_scope = NULL;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_PUT);
        UNUSED_VARIABLE(otCoapHeaderAppendUriPathOptions(&header, "light"));
        otCoapHeaderSetPayloadMarker(&header);

        p_message = otCoapNewMessage(p_instance, &header);
        if (p_message == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_message, &command, sizeof(command));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        switch (scope)
        {
        case THREAD_COAP_UTILS_MULTICAST_LINK_LOCAL:
            p_scope = "FF02::1";
            break;

        case THREAD_COAP_UTILS_MULTICAST_REALM_LOCAL:
            p_scope = "FF03::1";
            break;

        default:
            ASSERT(false);
        }

        memset(&message_info, 0, sizeof(message_info));
        message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        message_info.mPeerPort    = OT_DEFAULT_COAP_PORT;
        UNUSED_VARIABLE(otIp6AddressFromString(p_scope, &message_info.mPeerAddr));

        error = otCoapSendRequest(p_instance, p_message, &message_info, NULL, NULL);
    } while (false);

    if (error != OT_ERROR_NONE && p_message != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_message);
    }
}

static void provisioning_response_handler(void                * p_context,
                                          otCoapHeader        * p_header,
                                          otMessage           * p_message,
                                          const otMessageInfo * p_message_info,
                                          otError               result)
{
    (void)p_context;
    (void)p_header;

    // restore the polling period back to initial slow value
    UNUSED_VARIABLE(poll_period_restore(thread_ot_instance_get()));

    if (result == OT_ERROR_NONE)
    {
        UNUSED_VARIABLE(otMessageRead(p_message,
                                      otMessageGetOffset(p_message),
                                      &m_coap_status.peer_address,
                                      sizeof(m_coap_status.peer_address))
        );
    }
    else
    {
        NRF_LOG_INFO("Provisioning failed: %d\r\n", result);
    }
}

void thread_coap_utils_provisioning_request_send(otInstance * p_instance)
{
    otError       error = OT_ERROR_NO_BUFS;
    otCoapHeader  header;
    otMessage   * p_request;
    otMessageInfo message_info;

    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_GET);
        otCoapHeaderGenerateToken(&header, 2);
        UNUSED_VARIABLE(otCoapHeaderAppendUriPathOptions(&header, "provisioning"));

        p_request = otCoapNewMessage(p_instance, &header);
        if (p_request == NULL)
        {
            break;
        }

        // decrease the polling period for higher responsiveness
        uint32_t err_code = poll_period_response_set(p_instance);
        if (err_code == NRF_ERROR_BUSY)
        {
            break;
        }

        memset(&message_info, 0, sizeof(message_info));
        message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        message_info.mPeerPort    = OT_DEFAULT_COAP_PORT;
        UNUSED_VARIABLE(otIp6AddressFromString("FF03::1", &message_info.mPeerAddr));

        error = otCoapSendRequest(
            p_instance, p_request, &message_info, provisioning_response_handler, p_instance);

    } while (false);

    if (error != OT_ERROR_NONE && p_request != NULL)
    {
        otMessageFree(p_request);
    }

}

static void cloud_data_send(otInstance                            * p_instance,
                            const thread_coap_cloud_information_t * p_thread_coap_cloud_information,
                            const char                            * p_payload)
{
    otError       error;
    otCoapHeader  header;
    otCoapOption  content_format_option;
    otMessage   * p_request;
    otMessageInfo message_info;
    uint8_t content_format = p_thread_coap_cloud_information->cloud_coap_content_format;
    do
    {
        content_format_option.mNumber = OT_COAP_OPTION_CONTENT_FORMAT;
        content_format_option.mLength = 1;
        content_format_option.mValue  = &content_format;

        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_POST);
        UNUSED_VARIABLE(otCoapHeaderAppendUriPathOptions(&header, p_thread_coap_cloud_information->p_cloud_uri_path));
        UNUSED_VARIABLE(otCoapHeaderAppendOption(&header, &content_format_option));
        otCoapHeaderSetPayloadMarker(&header);

        p_request = otCoapNewMessage(p_instance, &header);
        if (p_request == NULL)
        {
            NRF_LOG_INFO("Failed to allocate message for CoAP Request\r\n");
            break;
        }

        error = otMessageAppend(p_request, p_payload, strlen(p_payload));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        memset(&message_info, 0, sizeof(message_info));
        message_info.mInterfaceId = OT_NETIF_INTERFACE_ID_THREAD;
        message_info.mPeerPort = OT_DEFAULT_COAP_PORT;
        message_info.mPeerAddr = m_coap_status.peer_address;

        error = otCoapSendRequest(p_instance, p_request, &message_info, NULL, NULL);

    } while (false);

    if (error != OT_ERROR_NONE && p_request != NULL)
    {
        NRF_LOG_INFO("Failed to send CoAP Request: %d\r\n", error);
        otMessageFree(p_request);
    }
}

void thread_coap_utils_cloud_data_update(
                        const thread_coap_cloud_information_t * p_thread_coap_cloud_information,
						const char                            * p_payload)
{
    // Uncommented by Reto Keller, 24.Nov.2017
	// char payload_buffer[64];
	// sprintf(payload_buffer,
	//        "{\"values\":[{\"key\":\"%s\",\"value\":\"%d\"}]}",
	//        p_thread_coap_cloud_information->p_cloud_thing_resource, data);
	// cloud_data_send(thread_ot_instance_get(), p_thread_coap_cloud_information, payload_buffer);

    cloud_data_send(thread_ot_instance_get(), p_thread_coap_cloud_information, p_payload);
}

bool thread_coap_utils_provisioning_is_enabled(void)
{
    return m_coap_status.provisioning_enabled;
}

void thread_coap_utils_provisioning_enable(bool value)
{
    uint32_t error;

    m_coap_status.provisioning_enabled = value;

    if (value)
    {
        m_coap_status.provisioning_expiry = otPlatAlarmMilliGetNow() + PROVISIONING_EXPIRY_TIME;
        error = app_timer_start(m_provisioning_timer,
                                APP_TIMER_TICKS(PROVISIONING_EXPIRY_TIME),
                                NULL);
        ASSERT(error == NRF_SUCCESS);

        if (m_thread_coap_configuration.configurable_led_blinking_enabled)
        {
            m_light_changed(LIGHT_OFF);
            error = app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL), NULL);
            ASSERT(error == NRF_SUCCESS);
        }
    }
    else
    {
        m_coap_status.provisioning_expiry = 0;
        error = app_timer_stop(m_provisioning_timer);
        ASSERT(error == NRF_SUCCESS);

        if (m_thread_coap_configuration.configurable_led_blinking_enabled)
        {
            error = app_timer_stop(m_led_timer);
            ASSERT(error == NRF_SUCCESS);

            if (m_coap_status.led_blinking_is_on)
            {
                m_light_changed(LIGHT_ON);
            }
            else
            {
                m_light_changed(LIGHT_OFF);
            }
        }
    }
}

static void thread_coap_utils_provisioning_timer_handler(void * p_context)
{
    thread_coap_utils_provisioning_enable(false);
}

void thread_coap_utils_provisioning_timer_init(void)
{
    uint32_t error = app_timer_create(&m_provisioning_timer,
                                      APP_TIMER_MODE_SINGLE_SHOT,
                                      thread_coap_utils_provisioning_timer_handler);
    ASSERT(error == NRF_SUCCESS);
}

otError thread_coap_utils_provisioning_response_send(void                * p_context,
                                                     otCoapHeader        * p_request_header,
                                                     const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NO_BUFS;
    otCoapHeader header;
    otMessage  * p_response;
    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_NON_CONFIRMABLE, OT_COAP_CODE_CONTENT);
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));
        otCoapHeaderSetPayloadMarker(&header);

        p_response = otCoapNewMessage(thread_ot_instance_get(), &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otMessageAppend(
            p_response, otThreadGetMeshLocalEid(thread_ot_instance_get()), sizeof(otIp6Address));
        if (error != OT_ERROR_NONE)
        {
            break;
        }

        error = otCoapSendResponse(thread_ot_instance_get(), p_response, p_message_info);

    } while (false);

    if (error != OT_ERROR_NONE && p_response != NULL)
    {
        otMessageFree(p_response);
    }

    return error;
}

static void led_timer_handler(void * p_context)
{
    if (thread_coap_utils_provisioning_is_enabled())
    {
        LEDS_INVERT(BSP_LED_2_MASK);
    }
    else
    {
        LEDS_OFF(BSP_LED_2_MASK);
    }
}

void thread_coap_utils_led_timer_init(void)
{
    uint32_t error = app_timer_create(&m_led_timer, APP_TIMER_MODE_REPEATED, led_timer_handler);
    ASSERT(error == NRF_SUCCESS);
}

void thread_coap_utils_led_timer_start(void)
{
    LEDS_CONFIGURE(LEDS_MASK);
    LEDS_OFF(LEDS_MASK);

    uint32_t error = app_timer_start(m_led_timer, APP_TIMER_TICKS(LED_INTERVAL), NULL);
    ASSERT(error == NRF_SUCCESS);
}

void thread_coap_utils_light_response_send(void                * p_context,
                                           otCoapHeader        * p_request_header,
                                           const otMessageInfo * p_message_info)
{
    otError      error = OT_ERROR_NO_BUFS;
    otCoapHeader header;
    otMessage  * p_response;
    do
    {
        otCoapHeaderInit(&header, OT_COAP_TYPE_ACKNOWLEDGMENT, OT_COAP_CODE_CHANGED);
        otCoapHeaderSetMessageId(&header, otCoapHeaderGetMessageId(p_request_header));
        otCoapHeaderSetToken(&header,
                             otCoapHeaderGetToken(p_request_header),
                             otCoapHeaderGetTokenLength(p_request_header));

        p_response = otCoapNewMessage(p_context, &header);
        if (p_response == NULL)
        {
            break;
        }

        error = otCoapSendResponse(p_context, p_response, p_message_info);

    } while (false);

    if ((error != OT_ERROR_NONE) && (p_response != NULL))
    {
        otMessageFree(p_response);
    }
}

void thread_coap_utils_light_changed_callback_set(light_changed_handler_t handler)
{
    m_light_changed = handler;
}

void thread_coap_utils_light_blinking_is_on_set(bool value)
{
    m_coap_status.led_blinking_is_on = value;
}

bool thread_coap_utils_light_blinking_is_on_get(void)
{
    return m_coap_status.led_blinking_is_on;
}
