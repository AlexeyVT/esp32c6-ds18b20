#include "thermometer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "ds18b20.h"
#include "esp_log.h"
#include "esp_zigbee_core.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "led_driver.h"
#include "nvs.h"
#include "zcl/esp_zigbee_zcl_temperature_meas.h"

static const char *TAG = "thermometer.c";

typedef uint8_t ds18b20_phy_addr_t[8];

typedef struct
{
    ds18b20_phy_addr_t addr;
    uint8_t endpoint;
    int16_t value;
    uint8_t read_attempts;
    uint8_t skip_unchanged_updates;
} ds18b20_t;

typedef struct
{
    ds18b20_t ds18b20[DS18B20_MAX_DEVICES];
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

static void thermometer_addr_to_string(const uint8_t *addr, char *buffer, size_t size)
{
    snprintf(buffer, size, "%02x-%02x%02x%02x%02x%02x%02x", addr[0], addr[6], addr[5], addr[4], addr[3], addr[2], addr[1]);
}

static int ds18b20_compare_addr(const void *a, const void *b) { return memcmp(a, b, sizeof(ds18b20_phy_addr_t)); }

static int ds18b20_compare_endpoint(const void *a, const void *b)
{
    const ds18b20_t *left  = a;
    const ds18b20_t *right = b;

    return (int)left->endpoint - (int)right->endpoint;
}

static int thermometer_addr_index(ds18b20_phy_addr_t *scan, uint8_t scan_count, const uint8_t *addr)
{
    for (uint8_t i = 0; i < scan_count; i++)
    {
        if (memcmp(scan[i], addr, sizeof(ds18b20_phy_addr_t)) == 0)
        {
            return i;
        }
    }

    return -1;
}

static void thermometer_nvs_key(uint8_t endpoint, char *key, size_t size) { snprintf(key, size, "rom%d", endpoint); }

static void thermometer_list_load(thermometer_list_t *list)
{
    nvs_handle_t handle;

    list->count = 0;

    if (nvs_open(DS18B20_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGI(TAG, "Endpoint registry is empty");
        return;
    }

    for (uint8_t endpoint = DS18B20_FIRST_ENDPOINT; endpoint < DS18B20_FIRST_ENDPOINT + DS18B20_MAX_DEVICES; endpoint++)
    {
        ds18b20_t *sensor = &list->ds18b20[list->count];
        char key[8];
        size_t size = sizeof(ds18b20_phy_addr_t);

        thermometer_nvs_key(endpoint, key, sizeof(key));
        if (nvs_get_blob(handle, key, sensor->addr, &size) != ESP_OK || size != sizeof(ds18b20_phy_addr_t))
        {
            continue;
        }

        sensor->endpoint = endpoint;
        sensor->value    = (int16_t)DS18B20_TEMPERATURE_UNKNOWN;
        list->count++;
    }

    nvs_close(handle);
    ESP_LOGI(TAG, "Endpoint registry holds %d DS18B20 device(s)", list->count);
}

static void thermometer_list_save(const thermometer_list_t *list)
{
    nvs_handle_t handle;

    if (nvs_open(DS18B20_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to open endpoint registry for writing");
        return;
    }

    for (uint8_t i = 0; i < list->count; i++)
    {
        char key[8];
        thermometer_nvs_key(list->ds18b20[i].endpoint, key, sizeof(key));

        esp_err_t status = nvs_set_blob(handle, key, list->ds18b20[i].addr, sizeof(ds18b20_phy_addr_t));
        if (status != ESP_OK)
        {
            ESP_LOGW(TAG, "Failed to store endpoint %d, status: %s", list->ds18b20[i].endpoint, esp_err_to_name(status));
        }
    }

    esp_err_t status = nvs_commit(handle);
    if (status != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to commit endpoint registry, status: %s", esp_err_to_name(status));
    }

    nvs_close(handle);
}

static void thermometer_list_seed(thermometer_list_t *list, ds18b20_phy_addr_t *scan, uint8_t scan_count)
{
    qsort(scan, scan_count, sizeof(ds18b20_phy_addr_t), ds18b20_compare_addr);

    for (uint8_t i = 0; i < scan_count; i++)
    {
        ds18b20_t *sensor = &list->ds18b20[i];

        memcpy(sensor->addr, scan[i], sizeof(ds18b20_phy_addr_t));
        sensor->endpoint = DS18B20_FIRST_ENDPOINT + i;
        sensor->value    = (int16_t)DS18B20_TEMPERATURE_UNKNOWN;
    }

    list->count = scan_count;
}

static uint8_t thermometer_free_endpoint(const thermometer_list_t *list)
{
    for (uint8_t endpoint = DS18B20_FIRST_ENDPOINT; endpoint < DS18B20_FIRST_ENDPOINT + DS18B20_MAX_DEVICES; endpoint++)
    {
        bool used = false;

        for (uint8_t i = 0; i < list->count && !used; i++)
        {
            used = list->ds18b20[i].endpoint == endpoint;
        }

        if (!used)
        {
            return endpoint;
        }
    }

    return 0;
}

static bool thermometer_list_reconcile(thermometer_list_t *list, ds18b20_phy_addr_t *scan, uint8_t scan_count)
{
    bool found[DS18B20_MAX_DEVICES]  = {false};
    bool absent[DS18B20_MAX_DEVICES] = {false};
    uint8_t absent_count             = 0;
    uint8_t new_count                = 0;
    bool changed                     = false;

    for (uint8_t i = 0; i < list->count; i++)
    {
        int index = thermometer_addr_index(scan, scan_count, list->ds18b20[i].addr);

        if (index >= 0)
        {
            found[index] = true;
        }
        else
        {
            absent[i] = true;
            absent_count++;
        }
    }

    for (uint8_t i = 0; i < scan_count; i++)
    {
        if (!found[i])
        {
            new_count++;
        }
    }

    if (absent_count == 1 && new_count == 1)
    {
        for (uint8_t i = 0; i < list->count; i++)
        {
            if (!absent[i])
            {
                continue;
            }

            for (uint8_t j = 0; j < scan_count; j++)
            {
                if (found[j])
                {
                    continue;
                }

                char old_addr[20];
                char new_addr[20];

                thermometer_addr_to_string(list->ds18b20[i].addr, old_addr, sizeof(old_addr));
                thermometer_addr_to_string(scan[j], new_addr, sizeof(new_addr));
                ESP_LOGI(TAG, "Replacing DS18B20 device %s with %s on endpoint %d", old_addr, new_addr, list->ds18b20[i].endpoint);

                memcpy(list->ds18b20[i].addr, scan[j], sizeof(ds18b20_phy_addr_t));
                list->ds18b20[i].value = (int16_t)DS18B20_TEMPERATURE_UNKNOWN;
                break;
            }
        }

        return true;
    }

    for (uint8_t j = 0; j < scan_count; j++)
    {
        if (found[j])
        {
            continue;
        }

        char addr[20];
        thermometer_addr_to_string(scan[j], addr, sizeof(addr));

        if (list->count >= DS18B20_MAX_DEVICES)
        {
            ESP_LOGW(TAG, "Endpoint registry is full, ignoring DS18B20 device %s", addr);
            break;
        }

        ds18b20_t *sensor = &list->ds18b20[list->count];

        memcpy(sensor->addr, scan[j], sizeof(ds18b20_phy_addr_t));
        sensor->endpoint = thermometer_free_endpoint(list);
        sensor->value    = (int16_t)DS18B20_TEMPERATURE_UNKNOWN;
        list->count++;
        changed = true;

        ESP_LOGI(TAG, "Registering DS18B20 device %s on endpoint %d", addr, sensor->endpoint);
    }

    return changed;
}

static uint8_t thermometer_scan(ds18b20_phy_addr_t *scan)
{
    uint8_t count = 0;

    while (count < DS18B20_MAX_DEVICES)
    {
        long result = ds18b20_search(&ds18b20_dev, scan[count]);

        if (result < 0)
        {
            ESP_LOGW(TAG, "Error while search DS18B20 devices: ERRNO %li", result);
            break;
        }

        if (result == 0)
        {
            ESP_LOGI(TAG, "All DS18B20 devices are found");
            break;
        }

        char addr[20];
        thermometer_addr_to_string(scan[count], addr, sizeof(addr));
        ESP_LOGI(TAG, "Found DS18B20 device %s", addr);
        count++;
    }

    if (count == DS18B20_MAX_DEVICES)
    {
        ESP_LOGW(TAG, "Bus limit of %d DS18B20 devices is reached", count);
    }

    return count;
}

void set_temperature_unknown(uint8_t ep)
{
    led_driver_set(0, 0xFF, 0xFF);
    ESP_LOGW(TAG, "Setting temperature to unknown for endpoint %d", ep);
    esp_zb_zcl_set_attribute_val(
        ep,
        ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
        &(int16_t){DS18B20_TEMPERATURE_UNKNOWN},  // 0x8000 is the Zigbee "invalid/measured value unknown" code, not IEEE NaN (z2m skips NaN values - bug)
        false);
}

void thermometer_update_values(void)
{
    led_driver_set(0, 0xFF, 0);
    ds18b20_requestTemperatures(&ds18b20_dev);
    led_driver_set(0, 0, 0);

    for (uint8_t i = 0; i < thermometer_list.count; i++)
    {
        ds18b20_t *ds18b20 = &thermometer_list.ds18b20[i];
        uint8_t ep         = ds18b20->endpoint;

        int32_t raw = ds18b20_getTemp(&ds18b20_dev, ds18b20->addr);
        if (raw == DEVICE_DISCONNECTED_RAW || raw == 0)
        {
            ESP_LOGW(TAG, "Failed to read temperature for endpoint %d", ep);

            if (ds18b20->read_attempts <= DS18B20_READ_FAILURE_ATTEMPTS)
            {
                ds18b20->read_attempts++;
            }

            if (ds18b20->read_attempts == DS18B20_READ_FAILURE_ATTEMPTS + 1)
            {
                ds18b20->value = (int16_t)DS18B20_TEMPERATURE_UNKNOWN;
                set_temperature_unknown(ep);
            }
            continue;
        }

        ds18b20->read_attempts = 0;

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
        uint8_t ep = thermometer_list.ds18b20[i].endpoint;
        char addr[20];

        thermometer_addr_to_string(thermometer_list.ds18b20[i].addr, addr, sizeof(addr));

        esp_zb_temperature_sensor_cfg_t temperature_sensor_cfg = ESP_ZB_DEFAULT_TEMPERATURE_SENSOR_CONFIG();
        esp_zb_cluster_list_t *esp_zb_cluster_list             = esp_zb_temperature_sensor_clusters_create(&temperature_sensor_cfg);

        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint           = ep,
            .app_profile_id     = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id      = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0,
        };
        esp_zb_ep_list_add_ep(ep_list, esp_zb_cluster_list, endpoint_config);

        ESP_LOGI(TAG, "Adding endpoint %d for DS18B20 device %s", ep, addr);
    }

    esp_zb_device_register(ep_list);

    temperature_update_handle = esp_zb_scheduler_user_alarm(temperature_update_callback, NULL, DS18B20_UPDATE_INTERVAL);
}

void thermometer_init(void)
{
    ds18b20_init(&ds18b20_dev, DS18B20_GPIO);

    ds18b20_phy_addr_t scan[DS18B20_MAX_DEVICES];
    uint8_t scan_count = thermometer_scan(scan);

    thermometer_list_load(&thermometer_list);

    if (thermometer_list.count == 0)
    {
        thermometer_list_seed(&thermometer_list, scan, scan_count);

        if (thermometer_list.count > 0)
        {
            ESP_LOGI(TAG, "Endpoint registry initialized with %d DS18B20 device(s)", thermometer_list.count);
            thermometer_list_save(&thermometer_list);
        }
    }
    else if (thermometer_list_reconcile(&thermometer_list, scan, scan_count))
    {
        qsort(thermometer_list.ds18b20, thermometer_list.count, sizeof(ds18b20_t), ds18b20_compare_endpoint);
        thermometer_list_save(&thermometer_list);
    }

    if (thermometer_list.count == 0)
    {
        ESP_LOGI(TAG, "No DS18B20 devices found");
        led_driver_set(0xFF, 0, 0xFF);
        return;
    }
}
