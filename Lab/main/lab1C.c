#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BUTTON GPIO_NUM_0
#define LED GPIO_NUM_4

void app_main(void)
{
    // configure LED output
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&out);

    // configure button input with internal pull-down (idle = LOW, press -> HIGH)
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << BUTTON),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&in);

    int prev = gpio_get_level(BUTTON);
    int blinking = 0; // 0 = not blinking, 1 = blinking
    TickType_t blink_end = 0;

    while (1) {
        int cur = gpio_get_level(BUTTON);

        // detect rising edge (press)
        if (prev == 0 && cur == 1) {
            // measure press duration (simple loop)
            uint32_t ms = 0;
            while (gpio_get_level(BUTTON) == 1) {
                vTaskDelay(pdMS_TO_TICKS(50));
                ms += 50;
            }
            if (ms > 500) {
                if (!blinking) {
                    // start 10s fast blink
                    blinking = 1;
                    blink_end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
                } else {
                    // cancel blinking immediately
                    blinking = 0;
                    gpio_set_level(LED, 0);
                }
            }
        }

        if (blinking) {
            gpio_set_level(LED, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_set_level(LED, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
            if (xTaskGetTickCount() >= blink_end) {
                blinking = 0;
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        prev = cur;
    }
}
