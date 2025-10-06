/* Simple state-machine (polling) example
   B) Toggle LED only if the button press is long enough (> 0.5 s)
*/

#include <stdio.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

/* Fallback defines for static analysis */
#ifndef CONFIG_LOG_MAXIMUM_LEVEL
#define CONFIG_LOG_MAXIMUM_LEVEL 3
#endif
#ifndef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 1000
#endif

static const char *TAG = "gpio_fsm";

/* Pins */
#undef INPUT_GPIO
#define INPUT_GPIO  GPIO_NUM_0
#ifndef OUTPUT_GPIO
#define OUTPUT_GPIO GPIO_NUM_4
#endif

/* Button wiring polarity
   1 = Active High (button -> 3V3). Uses internal pull-down. Rising on press.
   0 = Active Low  (button -> GND). Uses internal pull-up.   Rising on release.
*/
#ifndef INPUT_ACTIVE_HIGH
#define INPUT_ACTIVE_HIGH 1
#endif

/* Timings (ms) */
#define POLL_MS        10
#define DEBOUNCE_MS    30
#define LONG_PRESS_MS  500

void app_main(void)
{
    ESP_LOGI(TAG, "Start FSM (long-press>=%d ms): GPIO%d -> GPIO%d, ACTIVE_HIGH=%d",
             LONG_PRESS_MS, INPUT_GPIO, OUTPUT_GPIO, INPUT_ACTIVE_HIGH);

    /* Configure output pin (LED) */
    gpio_config_t io_conf = (gpio_config_t){
        .pin_bit_mask = (1ULL << INPUT_GPIO) | (1ULL << OUTPUT_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // Output
    io_conf.pin_bit_mask = (1ULL << OUTPUT_GPIO);
    io_conf.mode = GPIO_MODE_OUTPUT;
    gpio_config(&io_conf);

    // Input
    io_conf.pin_bit_mask = (1ULL << INPUT_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    if (INPUT_ACTIVE_HIGH) {
        // Button to 3V3: idle=LOW, press=HIGH
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    } else {
        // Button to GND: idle=HIGH, press=LOW
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    }
    gpio_config(&io_conf);

    const int active_level = INPUT_ACTIVE_HIGH ? 1 : 0;

    // LED initial state
    bool led_state = false;
    gpio_set_level(OUTPUT_GPIO, (int)led_state);

    // Debounced input tracking
    int filtered_level = gpio_get_level(INPUT_GPIO);
    int debounced_level = filtered_level;
    bool pending_change = false;
    TickType_t change_start = 0;

    // Press tracking
    bool in_press = (debounced_level == active_level);
    TickType_t press_start = xTaskGetTickCount();

    ESP_LOGI(TAG, "Input initial level=%d", debounced_level);

    while (1) {
        // Read raw
        int raw = gpio_get_level(INPUT_GPIO);

        // Debounce: update debounced_level only if raw stays changed for DEBOUNCE_MS
        if (raw != filtered_level) {
            filtered_level = raw;
            pending_change = true;
            change_start = xTaskGetTickCount();
        } else if (pending_change) {
            if ((xTaskGetTickCount() - change_start) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
                debounced_level = filtered_level;
                pending_change = false;
            }
        }

        // FSM: measure press duration (active -> inactive)
        if (!in_press && debounced_level == active_level) {
            // Press started
            in_press = true;
            press_start = xTaskGetTickCount();
        } else if (in_press && debounced_level != active_level) {
            // Released: compute press length
            TickType_t press_ticks = xTaskGetTickCount() - press_start;
            if (press_ticks >= pdMS_TO_TICKS(LONG_PRESS_MS)) {
                led_state = !led_state;
                gpio_set_level(OUTPUT_GPIO, (int)led_state);
                ESP_LOGI(TAG, "Long press %u ms -> LED %s",
                         (unsigned)(press_ticks * 1000 / configTICK_RATE_HZ),
                         led_state ? "ON" : "OFF");
            } else {
                ESP_LOGI(TAG, "Short press %u ms -> ignored",
                         (unsigned)(press_ticks * 1000 / configTICK_RATE_HZ));
            }
            in_press = false;
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}