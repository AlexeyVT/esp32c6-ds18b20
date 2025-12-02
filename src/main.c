#include "main.h"

#include "config.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "led_driver.h"
#include "nvs_flash.h"
#include "thermometer.h"


static const char* TAG = "main.c";

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, , TAG, "Failed to start Zigbee commissioning");
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t* signal_struct)
{
    uint32_t* p_sg_p                  = signal_struct->p_app_signal;
    esp_err_t err_status              = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    ESP_LOGI(TAG, "signal type received: %s", signal_type_to_string(sig_type));
    switch (sig_type)
    {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK)
            {
                if (esp_zb_bdb_is_factory_new())
                {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                }

                led_driver_set(0, 0, 0);
            }
            else
            {
                /* commissioning failed */
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));

                led_driver_set(0xFF, 0, 0);
                vTaskDelay(pdMS_TO_TICKS(5000));
                esp_restart();
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK)
            {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(
                    TAG,
                    "Joined network successfully (Extended PAN ID: "
                    "%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, "
                    "Channel:%d, Short Address: 0x%04hx)",
                    extended_pan_id[7],
                    extended_pan_id[6],
                    extended_pan_id[5],
                    extended_pan_id[4],
                    extended_pan_id[3],
                    extended_pan_id[2],
                    extended_pan_id[1],
                    extended_pan_id[0],
                    esp_zb_get_pan_id(),
                    esp_zb_get_current_channel(),
                    esp_zb_get_short_address());
            }
            else
            {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            break;
    }
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t* message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(
        message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)", message->info.status);
    ESP_LOGI(
        TAG,
        "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), "
        "data size(%d)",
        message->info.dst_endpoint,
        message->info.cluster,
        message->attribute.id,
        message->attribute.data.size);
    return ret;
}

static esp_err_t zb_default_response_handler(const esp_zb_zcl_cmd_default_resp_message_t* message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty default response message");

    ESP_LOGI(
        TAG,
        "Received default response: endpoint(%d), cluster(0x%x), command(0x%x), status(0x%x)",
        message->info.dst_endpoint,
        message->info.cluster,
        message->resp_to_cmd,
        message->status_code);

    // Handle specific status codes if needed
    if (message->status_code != ESP_ZB_ZCL_STATUS_SUCCESS)
    {
        ESP_LOGW(TAG, "Default response indicates error status: 0x%x", message->status_code);
    }

    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void* message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t*)message);
            break;
        case ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID:
            ret = zb_default_response_handler((esp_zb_zcl_cmd_default_resp_message_t*)message);
            break;
        default:
            ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ret;
}

static void esp_zb_task(void* pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    thermometer_add_endpoints();

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    led_driver_init();
    thermometer_init();

    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config  = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
