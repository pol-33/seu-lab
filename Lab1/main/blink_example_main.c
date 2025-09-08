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
   Note: On ESP32-C6 DevKit boards, the BOOT button is typically on GPIO9.
       We'll use GPIO9 by default on ESP32-C6, and GPIO0 otherwise.
       Override these with your own defines or via Kconfig if desired.
*/
#ifndef INPUT_GPIO
#ifdef CONFIG_IDF_TARGET_ESP32C6
#define INPUT_GPIO  GPIO_NUM_9
#else
#define INPUT_GPIO  GPIO_NUM_0
#endif
#endif
#ifndef OUTPUT_GPIO
#define OUTPUT_GPIO GPIO_NUM_4
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

     /* Configure input pin (button/signal) with internal pull-down disabled and pull-up enabled.
         If your button is wired to 3V3 instead (with pull-down), switch pull modes below.
     */
    io_conf.pin_bit_mask = (1ULL << INPUT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;    // common for buttons to ground
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);

    int prev_level = gpio_get_level(INPUT_GPIO);
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

            /* Simple debounce: wait 50 ms and re-read input */
            vTaskDelay(pdMS_TO_TICKS(50));
            level = gpio_get_level(INPUT_GPIO);
        }

        prev_level = level;

        /* Polling interval: 10 ms */
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
