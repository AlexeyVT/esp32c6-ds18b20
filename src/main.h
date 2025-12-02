/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier:  LicenseRef-Included
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "esp_zigbee_core.h"

/* Basic manufacturer information */
#define ESP_MANUFACTURER_NAME \
    "\x09"                    \
    "ESPRESSIF"                                       /* Customized manufacturer name */
#define ESP_MODEL_IDENTIFIER "\x07" CONFIG_IDF_TARGET /* Customized model identifier */

#define ESP_ZB_ZED_CONFIG()                                                                     \
    {                                                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED, .install_code_policy = INSTALLCODE_POLICY_ENABLE, \
        .nwk_cfg.zed_cfg = {                                                                    \
            .ed_timeout = ED_AGING_TIMEOUT,                                                     \
            .keep_alive = ED_KEEP_ALIVE,                                                        \
        },                                                                                      \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()       \
    {                                       \
        .radio_mode = ZB_RADIO_MODE_NATIVE, \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                          \
    {                                                         \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, \
    }

static inline const char* signal_type_to_string(esp_zb_app_signal_type_t signal_type)
{
    switch (signal_type)
    {
        case ESP_ZB_ZDO_SIGNAL_DEFAULT_START:
            return "0x00 - The device has started in non-BDB commissioning mode.";
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            return "0x01 - Stack framework (scheduler, buffer pool, NVRAM, etc.) startup complete, ready for initializing bdb commissioning.";
        case ESP_ZB_ZDO_SIGNAL_DEVICE_ANNCE:
            return "0x02 - Indicates that a Zigbee device has joined or rejoined the network.";
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            return "0x03 - Indicates that the device itself has left the network.";
        case ESP_ZB_ZDO_SIGNAL_ERROR:
            return "0x04 - Indicates corrupted or incorrect signal information.";
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
            return "0x05 - Indicate the basic network information of factory new device has been initialized, ready for Zigbee commissioning.";
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            return "0x06 - Indicate device joins or rejoins network from the configured network information.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK_STARTED:
            return "0x07 - Indicates that the Touchlink initiator has successfully started a network with the target and is ready for rejoining.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK_JOINED_ROUTER:
            return "0x08 - Indicate Touchlink target has join the initiator network.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK:
            return "0x09 - Indicates the result of the Touchlink initiator commissioning process.";
        case ESP_ZB_BDB_SIGNAL_STEERING:
            return "0x0A - Indicates the completion of BDB network steering.";
        case ESP_ZB_BDB_SIGNAL_FORMATION:
            return "0x0B - Indicates the completion of BDB network formation.";
        case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_TARGET_FINISHED:
            return "0x0C - Indicates the completion of BDB finding and binding (F&B) for a target endpoint.";
        case ESP_ZB_BDB_SIGNAL_FINDING_AND_BINDING_INITIATOR_FINISHED:
            return "0x0D - Indicates the BDB F&B with a Target succeeded or F&B initiator timeout expired or cancelled.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET:
            return "0x0E - Indicates that the Touchlink target is preparing to commission with the initiator.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK_NWK:
            return "0x0F - Indicates that the Touchlink target network has started.";
        case ESP_ZB_BDB_SIGNAL_TOUCHLINK_TARGET_FINISHED:
            return "0x10 - Indicates that the Touchlink target commissioning procedure has finished.";
        case ESP_ZB_NWK_SIGNAL_DEVICE_ASSOCIATED:
            return "0x12 - Indicates that a new device has initiated an association procedure.";
        case ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION:
            return "0x13 - Indicates that a child device has left the network.";
        case ESP_ZB_ZGP_SIGNAL_COMMISSIONING:
            return "0x15 - Indicates the GPCB (Green Power Combo Basic) commissioning signal.";
        case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
            return "0x16 - Indicates the device can enter sleep mode.";
        case ESP_ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            return "0x17 - Indicates whether a specific part of the production configuration was found.";
        case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
            return "0x18 - Indicates that the Neighbor Table has expired, and no active route links remain.";
        case ESP_ZB_ZDO_SIGNAL_DEVICE_AUTHORIZED:
            return "0x2F - Indicates that a new device has been authorized by the Trust Center in the network.";
        case ESP_ZB_ZDO_SIGNAL_DEVICE_UPDATE:
            return "0x30 - Indicates that a device has joined, rejoined, or left the network from the Trust Center or its parents.";
        case ESP_ZB_NWK_SIGNAL_PANID_CONFLICT_DETECTED:
            return "0x31 - Detects a PAN ID conflict and inquires for a resolution.";
        case ESP_ZB_NLME_STATUS_INDICATION:
            return "0x32 - Indicates that a network failure has been detected.";
        case ESP_ZB_BDB_SIGNAL_TC_REJOIN_DONE:
            return "0x35 - Indicates that the Trust Center rejoin procedure has been completed.";
        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            return "0x36 - Indicates the status of the network (open or closed).";
        case ESP_ZB_BDB_SIGNAL_STEERING_CANCELLED:
            return "0x37 - Indicates the result of cancelling BDB steering.";
        case ESP_ZB_BDB_SIGNAL_FORMATION_CANCELLED:
            return "0x38 - Notifies the result of cancelling BDB formation.";
        case ESP_ZB_ZGP_SIGNAL_MODE_CHANGE:
            return "0x3B - Indicates a ZGP mode change.";
        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            return "0x3C - Notify that the destination device is unavailable.";
        case ESP_ZB_ZGP_SIGNAL_APPROVE_COMMISSIONING:
            return "0x3D - ZGP Approve Commissioning.";
        default:
            return "Unknown signal";
    }
}