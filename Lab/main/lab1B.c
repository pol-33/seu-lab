#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"

#define INPUT_GPIO   GPIO_NUM_0
#define OUTPUT_GPIO  GPIO_NUM_4

static const char *TAG = "lab1B_longpress";

void app_main(void)
{
    // Configure LED output
    gpio_config_t io_out = {
        .pin_bit_mask = 1ULL << OUTPUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_out);

    // Configure input with internal pull-down (idle LOW, press -> HIGH)
    gpio_config_t io_in = {
        .pin_bit_mask = 1ULL << INPUT_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_in);

    bool led = false;
    gpio_set_level(OUTPUT_GPIO, led);

    int prev = gpio_get_level(INPUT_GPIO);

    while (1) {
        int cur = gpio_get_level(INPUT_GPIO);
        if (prev == 0 && cur == 1) {
            // button pressed: measure duration
            TickType_t start = xTaskGetTickCount();
            // wait until release or timeout checking every 20 ms
            while (gpio_get_level(INPUT_GPIO) == 1) {
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            TickType_t end = xTaskGetTickCount();
            float seconds = (end - start) * (configTICK_RATE_HZ > 0 ? (1.0f / configTICK_RATE_HZ) : 0);
            ESP_LOGI(TAG, "press duration: %.3f s", seconds);
            if (seconds > 0.5f) {
                led = !led;
                gpio_set_level(OUTPUT_GPIO, led);
                ESP_LOGI(TAG, "Long press detected: LED %s", led?"ON":"OFF");
            } else {
                ESP_LOGI(TAG, "Press too short, ignored");
            }
        }
        prev = cur;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
