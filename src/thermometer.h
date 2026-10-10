#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C"
{
#endif

    bool thermometer_add_endpoints(void);
    void thermometer_init(void);
    void thermometer_reporting_start(void);
    void thermometer_update_values(void);

#ifdef __cplusplus
}
#endif
