#include "thermometer.h"

#include "config.h"
#include "ds18b20.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "led_driver.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"

static const char *TAG = "thermometer.c";

typedef uint8_t ds18b20_phy_addr_t[8];

typedef struct
{
    ds18b20_phy_addr_t addr;
    int16_t value;
    uint8_t read_attempts;
    uint8_t skip_unchanged_updates;
} ds18b20_t;

typedef struct
{
    ds18b20_t ds18b20[32];
    uint8_t count;
} thermometer_list_t;

static ds18b20_dev_t ds18b20_dev = {
    .pin           = DS18B20_GPIO,
    .parasite      = false,
    .bitResolution = 12,
    .search =
        {
            .LastDiscrepancy       = 0,
            .LastDeviceFlag        = false,
            .LastFamilyDiscrepancy = 0,
            .ROM_NO                = {0},
        },
};

static thermometer_list_t thermometer_list               = {0};
static esp_zb_user_cb_handle_t temperature_update_handle = 0;

static int ds18b20_compare(const void *a, const void *b) { return memcmp(*(ds18b20_phy_addr_t *)a, *(ds18b20_phy_addr_t *)b, sizeof(ds18b20_phy_addr_t)); }

void set_temperature_unknown(uint8_t ep)
{
    led_driver_set(0, 0xFF, 0xFF);
    ESP_LOGW(TAG, "Setting temperature to unknown for endpoint %d", ep);
    esp_zb_zcl_set_attribute_val(
        ep,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &(int16_t){0x8000},  // 0x8000 is the Zigbee "invalid/measured value unknown" code, not IEEE NaN (z2m skips NaN values - bug)
        false);
}

void thermometer_update_values(void)
{
    led_driver_set(0, 0xFF, 0);
    ds18b20_requestTemperatures(&ds18b20_dev);
    led_driver_set(0, 0, 0);

    for (uint8_t i = 0; i < thermometer_list.count; i++)
    {
        uint8_t ep         = i + DS18B20_FIRST_ENDPOINT;
        ds18b20_t *ds18b20 = &thermometer_list.ds18b20[i];

        int32_t raw = ds18b20_getTemp(&ds18b20_dev, ds18b20->addr);
        if (raw == DEVICE_DISCONNECTED_RAW || raw == 0)
        {
            ESP_LOGW(TAG, "Failed to read temperature for endpoint %d", ep);

            ds18b20->read_attempts++;
            if (ds18b20->read_attempts > DS18B20_READ_FAILURE_ATTEMPTS)
            {
                set_temperature_unknown(ep);
            }
            continue;
        }

        int16_t new_value = (int16_t)((float)raw * 0.78125f);

        if (abs(ds18b20->value - new_value) < 10)
        {
            ds18b20->skip_unchanged_updates++;
            if (ds18b20->skip_unchanged_updates > DS18B20_SKIP_UNCHANGED_UPDATE)
            {
                ds18b20->skip_unchanged_updates = 0;
            }
            else
            {
                continue;
            }
        }
        else
        {
            ds18b20->skip_unchanged_updates = 0;
        }

        ds18b20->value = new_value;

        esp_zb_zcl_status_t status = esp_zb_zcl_set_attribute_val(
            ep, ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &ds18b20->value, false);

        if (status != ESP_ZB_ZCL_STATUS_SUCCESS)
        {
            ESP_LOGW(TAG, "Failed to update temperature for endpoint %d, status: %d", ep, status);
        }
        else
        {
            ESP_LOGI(TAG, "Updated temperature for endpoint %d: %i.%02i°C", ep, ds18b20->value / 100, ds18b20->value % 100);
        }
    }
}

static void temperature_update_callback(void *param)
{
    thermometer_update_values();
    temperature_update_handle = esp_zb_scheduler_user_alarm(temperature_update_callback, NULL, DS18B20_UPDATE_INTERVAL);
}

void thermometer_add_endpoints(void)
{
    if (thermometer_list.count == 0)
    {
        return;
    }

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();

    for (uint8_t i = 0; i < thermometer_list.count; i++)
    {
        uint8_t ep         = i + DS18B20_FIRST_ENDPOINT;
        ds18b20_t *ds18b20 = &thermometer_list.ds18b20[i];

        ds18b20->value = ds18b20_getTempC(&ds18b20_dev, ds18b20->addr);

        esp_zb_temperature_sensor_cfg_t temperature_sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
        esp_zb_cluster_list_t *esp_zb_cluster_list             = esp_zb_temperature_sensor_clusters_create(&temperature_sensor_cfg);

        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint           = ep,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list, endpoint_config);

        ESP_LOGI(
            TAG,
            "Adding endpoint %d for DS18B20 device %02x-%02x%02x%02x%02x%02x%02x",
            ep,
            thermometer_list.ds18b20[i].addr[0],
            thermometer_list.ds18b20[i].addr[6],
            thermometer_list.ds18b20[i].addr[5],
            thermometer_list.ds18b20[i].addr[4],
            thermometer_list.ds18b20[i].addr[3],
            thermometer_list.ds18b20[i].addr[2],
            thermometer_list.ds18b20[i].addr[1]);
    }

    esp_zb_device_register(ep_list);

    if (thermometer_list.count > 0)
    {
        temperature_update_handle = esp_zb_scheduler_user_alarm(temperature_update_callback, NULL, DS18B20_UPDATE_INTERVAL);
    }
}

void thermometer_init(void)
{
    ds18b20_init(&ds18b20_dev, DS18B20_GPIO);

    uint8_t *paddr = thermometer_list.ds18b20[0].addr;

    for (long r = ds18b20_search(&ds18b20_dev, paddr); r > 0 && thermometer_list.count < 32; r = ds18b20_search(&ds18b20_dev, paddr))
    {
        if (r == 1)
        {
            ESP_LOGI(TAG, "Found DS18B20 device %02x-%02x%02x%02x%02x%02x%02x", paddr[0], paddr[6], paddr[5], paddr[4], paddr[3], paddr[2], paddr[1]);
            thermometer_list.count++;
            paddr = thermometer_list.ds18b20[thermometer_list.count].addr;
        }
        else if (r < 0)
        {
            ESP_LOGI(TAG, "Error while search DS18B20 devices: ERRNO %i", r);
            break;
        }
        else
        {
            ESP_LOGI(TAG, "All DS18B20 devices are found");
            break;
        }
    }

    if (thermometer_list.count == 0)
    {
        ESP_LOGI(TAG, "No DS18B20 devices found");
        led_driver_set(0xFF, 0, 0xFF);
        return;
    }

    ESP_LOGI(TAG, "Found %i DS18B20 devices", thermometer_list.count);

    qsort(thermometer_list.ds18b20, thermometer_list.count, sizeof(ds18b20_t), ds18b20_compare);
}
