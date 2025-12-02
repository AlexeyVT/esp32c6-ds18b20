#include "led_driver.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdint.h>

#include "config.h"
#include "driver/rmt_tx.h"
#include "esp_check.h"
#include "esp_log.h"

#define MAX_LED_POWER 0x1F

static const char *TAG = "led_driver.c";

// Конфигурация RMT для адресных светодиодов
static rmt_channel_handle_t led_chan    = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

void led_driver_set(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t led_data[3] = {
        green & MAX_LED_POWER,
        red & MAX_LED_POWER,
        blue & MAX_LED_POWER,
    };  // Порядок GRB для WS2812

    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };

    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_data, sizeof(led_data), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

esp_err_t led_driver_init()
{
    ESP_LOGI(TAG, "Initialize RMT for RGB LED on GPIO %d", RGB_LED_GPIO);

    // Конфигурация канала RMT
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .gpio_num          = RGB_LED_GPIO,
        .mem_block_symbols = 64,
        .resolution_hz     = 10000000,  // 10 MHz
        .trans_queue_depth = 4,
        .flags.with_dma    = false,
    };
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx_chan_config, &led_chan), TAG, "create RMT channel failed");

    // Конфигурация энкодера для WS2812
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 =
            {
                .level0    = 1,
                .duration0 = 4,  // 0.4us
                .level1    = 0,
                .duration1 = 8,  // 0.8us
            },
        .bit1 =
            {
                .level0    = 1,
                .duration0 = 8,  // 0.8us
                .level1    = 0,
                .duration1 = 4,  // 0.4us
            },
        .flags.msb_first = 1};
    ESP_RETURN_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder), TAG, "create encoder failed");

    // Включить канал RMT
    ESP_RETURN_ON_ERROR(rmt_enable(led_chan), TAG, "enable RMT channel failed");

    led_driver_set(0xFF, 0xFF, 0);
    return ESP_OK;
}
