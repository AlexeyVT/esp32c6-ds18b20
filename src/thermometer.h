#pragma once

#include <stdint.h>

#include "esp_zigbee_core.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void thermometer_add_endpoints();
    void thermometer_init(void);
    void thermometer_update_values(void);

#ifdef __cplusplus
}
#endif
