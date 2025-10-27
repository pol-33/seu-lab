#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BUTTON GPIO_NUM_0
#define LED GPIO_NUM_4

void app_main(void)
{
    gpio_config_t out = { .pin_bit_mask = (1ULL<<LED), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask = (1ULL<<BUTTON), .mode = GPIO_MODE_INPUT, .pull_down_en = 1 };
    gpio_config(&in);

    uint32_t press_counter = 0;
    const uint32_t long_ms = 500;

    int bursting = 0;
    TickType_t burst_end = 0;

    TickType_t last = xTaskGetTickCount();

    while (1) {
        int raw = gpio_get_level(BUTTON);
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed = (now - last) * portTICK_PERIOD_MS;
        last = now;

        if (!bursting) {
            if (raw == 1) {
                press_counter += elapsed;
            } else {
                if (press_counter > long_ms) {
                    bursting = 1;
                    burst_end = xTaskGetTickCount() + pdMS_TO_TICKS(10000);
                }
                press_counter = 0;
            }
        } else {
            // in bursting mode: check for cancel long press
            if (raw == 1) {
                press_counter += elapsed;
            } else {
                if (press_counter > long_ms) {
                    // cancel
                    bursting = 0;
                    gpio_set_level(LED, 0);
                }
                press_counter = 0;
            }

            // blink using time checks
            if ((xTaskGetTickCount() & 0xFFFFFFFF) % pdMS_TO_TICKS(200) < pdMS_TO_TICKS(100)) {
                gpio_set_level(LED, 1);
            } else {
                gpio_set_level(LED, 0);
            }

            if (xTaskGetTickCount() >= burst_end) {
                bursting = 0;
                gpio_set_level(LED, 0);
            }
        }

        taskYIELD();
    }
}
