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

#ifndef thread_coap_UTILS_H
#define thread_coap_UTILS_H

#include <stdbool.h>

#include <openthread/coap.h>
#include "thread_utils.h"

#define PROVISIONING_EXPIRY_TIME 5000
#define LED_INTERVAL             100

typedef struct
{
    bool coap_server_enabled;
    bool coap_client_enabled;
    bool coap_cloud_enabled;
    bool configurable_led_blinking_enabled;
} thread_coap_configuration_t;

/**@brief Enumeration describing light commands.
 */
typedef enum
{
    LIGHT_OFF = '0',
    LIGHT_ON,
    LIGHT_TOGGLE
} thread_coap_utils_light_command_t;


/**@brief Type definition of the function used to handle light resource change.
 */
typedef void (*light_changed_handler_t)(thread_coap_utils_light_command_t light_state);


/**@brief Structure holding CoAP cloud server information.
 */
typedef struct
{
    const char * p_cloud_hostname;
    const char * p_cloud_uri_path;
    const char * p_cloud_thing_resource;
    uint8_t      cloud_coap_content_format;
} thread_coap_cloud_information_t;

/**@brief Structure holding CoAP status information.
 */
typedef struct
{
    bool         provisioning_enabled; /**< Information if provisioning is enabled. */
    uint32_t     provisioning_expiry;  /**< Provisioning timeout time. */
    bool         led_blinking_is_on;   /**< Information if leds are blinking */

    otIp6Address peer_address;       /**< An address of a related server node. */
    bool         multicast_light_on; /**< Information which multicast command should be sent next. */
} thread_coap_utils_status_t;

/**@brief Structure holding CoAP resources.
 */
typedef struct
{
    otCoapResource provisioning_resource; /**< CoAP provisioning resource. */
    otCoapResource light_resource;        /**< CoAP light resource. */
} thread_coap_utils_resources_t;

/**@brief Scope of multicast request.
 */
typedef enum
{
    THREAD_COAP_UTILS_MULTICAST_LINK_LOCAL,
    THREAD_COAP_UTILS_MULTICAST_REALM_LOCAL,
} thread_coap_utils_multicast_scope_t;

/**@brief Function that starts the CoAP with specified resources.
 *
 * @details The thread_init() function needs to be executed before calling this function.
 *
 * @param[in] p_thread_coap_configuration A pointer to the Thread CoAP configuration structure.
 */
void thread_coap_utils_init(const thread_coap_configuration_t * p_thread_coap_configuration);

 /**@brief Function that initializes the led timer.
 */
void thread_coap_utils_led_timer_init(void);

/**@brief Function that starts the led timer.
 */
void thread_coap_utils_led_timer_start(void);

/**@brief Function that returns Thread CoAP configuration structure instance pointer.
 *
 * @return A pointer to the Thread CoAP configuration structure.
 */
thread_coap_configuration_t * thread_coap_configuration_get(void);

/**@section CoAP Server function proptypes
 */

/**@brief Function that sends the CoAP response.
 *
 * @param[in] p_context        A pointer to the application-specific context.
 * @param[in] p_request_header A pointer to the OpenThread Coap Header structure.
 * @param[in] p_message_info   A pointer to the constant OpenThread Message Info structure.
 */
void thread_coap_utils_light_response_send(void                * p_context,
                                           otCoapHeader        * p_request_header,
                                           const otMessageInfo * p_message_info);

/**@brief Function that initializes the provisioning timer.
 */
void thread_coap_utils_provisioning_timer_init(void);

/**@brief Function that indicates if device enabled provisioning mode.
 *
 * @return True if provisioning mode is enabled, false otherwise.
 */
bool thread_coap_utils_provisioning_is_enabled(void);

/**@brief Function that sets the information if provisioning has been enabled.
 *
 * @param value A provisioning enabled value to be set.
 */
void thread_coap_utils_provisioning_enable(bool value);

/**@brief Function that sends the provisioning response.
 *
 * @param[in] p_context        A pointer to the application-specific context.
 * @param[in] p_request_header A pointer to the OpenThread Coap Header structure.
 * @param[in] p_message_info   A pointer to the constant OpenThread Message Info structure.
 *
 * @return                     An OpenThread error value. OT_ERROR_NONE in case of success.
 */
otError thread_coap_utils_provisioning_response_send(void                * p_context,
                                                     otCoapHeader        * p_request_header,
                                                     const otMessageInfo * p_message_info);

/**@brief Function that sets the information if multicast light been enabled.
 *
 * @param[in] value A bool to which holds information if the multicast light is enabled.
 */
void thread_coap_utils_mcast_light_on_set(bool value);

/**@brief Function that returns multicast light state.
 *
 * @return True if multicast light is enabled, false otherwise.
 */
bool thread_coap_utils_mcast_light_on_get(void);

/**@brief Function that inverts the information if the multicast light is enabled.
 */
void thread_coap_utils_mcast_light_on_toggle(void);

/**@brief Function used to set the light changed callback function.
 *
 * @param[in] handler Pointer to the function of taking void and returning void used used as light_on implementation.
 */
void thread_coap_utils_light_changed_callback_set(light_changed_handler_t handler);

/**@brief Function that sets the multicast_light_on to passed bool value on.
 *
 * @param[in] value A bool to be written to multicast_light_on.
 */
void thread_coap_utils_light_blinking_is_on_set(bool value);

/**@brief Function that returns the multicast_light_on value.
*
* @return A bool multicast_light_on value.
*/
bool thread_coap_utils_light_blinking_is_on_get(void);

/**@section CoAP Client function proptypes
 */

/**@brief Function that sets the CoAP peer IPv6 address.
 *
 * @param[in] p_peer_addr A const pointer to the otIp6Address.
 */
void thread_coap_utils_peer_addr_set(const otIp6Address * p_peer_addr);

/**@brief Function that returns the current CoAP peer IPv6 address.
 *
 * @return A pointer to the otIp6Address structure.
 */
otIp6Address * thread_coap_utils_peer_addr_get(void);

/**@brief Function that removes the peer address.
 */
void thread_coap_utils_peer_addr_clear(void);

/**@brief Function that checks if peer address is checked.
 *
 * @return false in case the peer address is unspecified (::), true otherwise.
 */
bool thread_coap_utils_peer_addr_is_set(void);

/**@brief Function that sends the provisioning request to the CoAP server.
 *
 * @param[in] otInstance OpenThread instance structure pointer.
 */
void thread_coap_utils_provisioning_request_send(otInstance * p_instance);

/**@brief Function that sends the light request comamnd to the CoAP peer.
 *
 * @param[in] p_instance OpenThread instance structure pointer.
 * @param[in] command    Light command.
 */
void thread_coap_utils_unicast_light_request_send(otInstance * p_instance, uint8_t command);

/**@brief Function that sends the light request comamnd to the multicast IPv6 address FF03::1 (All Nodes Multicast address).
 *
 * @param[in] p_instance OpenThread instance structure pointer.
 * @param[in] command    Light command.
 * @param[in] scope      Multicast address scope.
 */
void thread_coap_utils_multicast_light_request_send(otInstance                        * p_instance,
                                                    uint8_t                             command,
                                                    thread_coap_utils_multicast_scope_t scope);

/**@section CoAP Cloud function proptypes
 */

/**@brief Function that sends the new data to the cloud CoAP server.
 *
 * @param[in] p_thread_coap_cloud_information A thread_coap_cloud_information_t instance structure pointer.
 * @param[in] data                            A point to a char array to the data to be sent to the server.
 */
void thread_coap_utils_cloud_data_update(
                                const thread_coap_cloud_information_t * p_thread_coap_cloud_information,
                                const char                            * data);

#endif /* thread_coap_UTILS_H */
