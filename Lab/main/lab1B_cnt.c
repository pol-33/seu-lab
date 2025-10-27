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

    int confirmed = gpio_get_level(BUTTON);
    uint32_t press_counter = 0;
    const uint32_t long_ms = 500;
    TickType_t last = xTaskGetTickCount();

    while (1) {
        int raw = gpio_get_level(BUTTON);
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed = (now - last) * portTICK_PERIOD_MS;
        last = now;

        if (raw == 1) {
            press_counter += elapsed;
        } else {
            if (press_counter > long_ms) {
                // long press detected -> toggle
                int led = gpio_get_level(LED);
                gpio_set_level(LED, !led);
            }
            press_counter = 0;
        }

        // update confirmed when stable (simple)
        confirmed = raw;
        taskYIELD();
    }
}
