#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* light intensity level */
#define LIGHT_DEFAULT_ON 1
#define LIGHT_DEFAULT_OFF 0

/* LED strip configuration */
#define CONFIG_EXAMPLE_STRIP_LED_GPIO 8
#define CONFIG_EXAMPLE_STRIP_LED_NUMBER 1

    void led_driver_set(uint8_t red, uint8_t green, uint8_t blue);
    esp_err_t led_driver_init();

#ifdef __cplusplus
}  // extern "C"
#endif