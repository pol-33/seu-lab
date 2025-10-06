#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define BUTTON GPIO_NUM_0
#define LED GPIO_NUM_4

// Simple polling with debounce implemented by accumulating elapsed ms (no vTaskDelay)
void app_main(void)
{
    gpio_config_t out = { .pin_bit_mask = (1ULL<<LED), .mode = GPIO_MODE_OUTPUT };
    gpio_config(&out);
    gpio_config_t in = { .pin_bit_mask = (1ULL<<BUTTON), .mode = GPIO_MODE_INPUT, .pull_down_en = 1 };
    gpio_config(&in);

    int confirmed = gpio_get_level(BUTTON); // stable confirmed state
    uint32_t counter_ms = 0;
    const uint32_t debounce_ms = 30;

    TickType_t last_tick = xTaskGetTickCount();

    while (1) {
        int raw = gpio_get_level(BUTTON);
        TickType_t now = xTaskGetTickCount();
        uint32_t elapsed_ms = (now - last_tick) * portTICK_PERIOD_MS;
        last_tick = now;

        if (raw != confirmed) {
            counter_ms += elapsed_ms;
            if (counter_ms >= debounce_ms) {
                // state changed
                int prev_confirmed = confirmed;
                confirmed = raw;
                counter_ms = 0;
                if (prev_confirmed == 0 && confirmed == 1) {
                    // rising edge confirmed -> toggle LED
                    int led = gpio_get_level(LED);
                    gpio_set_level(LED, !led);
                }
            }
        } else {
            counter_ms = 0;
        }

        taskYIELD(); // give scheduler a chance; no delays used
    }
}
