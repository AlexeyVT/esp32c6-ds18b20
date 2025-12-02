#pragma once

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE false                                  /* enable the install code policy for security */
#define ED_AGING_TIMEOUT ESP_ZB_ED_AGING_TIMEOUT_64MIN                   /* aging timeout of device */
#define ED_KEEP_ALIVE 3000                                               /* 3000 millisecond */
#define ESP_ZB_PRIMARY_CHANNEL_MASK ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK /* Zigbee primary channel mask use in the example */

#define RGB_LED_GPIO GPIO_NUM_8 /* GPIO for RGB LED */

#define DS18B20_GPIO GPIO_NUM_1          /* GPIO for DS18B20 */
#define DS18B20_FIRST_ENDPOINT 1         /* First endpoint number for DS18B20 */
#define DS18B20_READ_FAILURE_ATTEMPTS 3  /* Maximum read failure attempts before setting temperature to unknown */
#define DS18B20_SKIP_UNCHANGED_UPDATE 60 /* Maximum number of skipping unchanged updates before force update value */
#define DS18B20_UPDATE_INTERVAL 5000     /* Update interval in milliseconds for DS18B20 */
