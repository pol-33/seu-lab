/* Simple state-machine (polling) example

   Functionality (per Lab requirement A):
   - Read a digital input on GPIO0.
   - On each rising edge (low->high) toggle an LED connected to GPIO4.
   - Only polling is used (no interrupts, no extra threads).
*/

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/* Fallback defines to help static analysis / builds outside full IDF SDK
    If the real build provides these via sdkconfig.h, these fallbacks are ignored.
*/
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif
#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 1000
#endif

static const char *TAG = "gpio_fsm";

/* Pins used for this lab exercise
    Using your external button on GPIO0 as requested. Adjust pull mode below if wired to 3V3.
*/
#undef INPUT_GPIO
#define INPUT_GPIO  GPIO_NUM_0
#ifndef OUTPUT_GPIO
#define OUTPUT_GPIO GPIO_NUM_4
#endif

/* Button wiring polarity
    1 = Active High (button -> 3V3). Uses internal pull-down. Rising edge occurs on press.
    0 = Active Low  (button -> GND). Uses internal pull-up.   Rising edge occurs on release.
*/
#ifndef INPUT_ACTIVE_HIGH
#define INPUT_ACTIVE_HIGH 1
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "Starting polling-based state machine: GPIO%d -> GPIO%d", INPUT_GPIO, OUTPUT_GPIO);

    /* Configure output pin (LED) */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << OUTPUT_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* Configure input pin (button/signal) */
    io_conf.pin_bit_mask = (1ULL << INPUT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    if (INPUT_ACTIVE_HIGH) {
        // Button to 3V3: idle=LOW, press=HIGH -> rising edge on press
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    } else {
        // Button to GND: idle=HIGH, press=LOW -> rising edge on release
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    int prev_level = gpio_get_level(INPUT_GPIO);
    ESP_LOGI(TAG, "Input idle level=%d (INPUT_ACTIVE_HIGH=%d)", prev_level, INPUT_ACTIVE_HIGH);
    bool led_state = false;
    gpio_set_level(OUTPUT_GPIO, (int)led_state);

    while (1) {
        int level = gpio_get_level(INPUT_GPIO);

        /* Detect rising edge: previous low (0) and current high (1) */
        if (prev_level == 0 && level == 1) {
            /* Toggle LED state on rising edge */
            led_state = !led_state;
            gpio_set_level(OUTPUT_GPIO, (int)led_state);
            ESP_LOGI(TAG, "Rising edge detected: LED %s", led_state ? "ON" : "OFF");
        }

        prev_level = level;

        /* Polling interval: 10 ms */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
