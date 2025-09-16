#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h" 
#include "esp_log.h"

#define INPUT_GPIO   GPIO_NUM_0  // Botó a 3V3
#define OUTPUT_GPIO  GPIO_NUM_4  // LED a GND

void app_main(void)
{
    // LED com a sortida
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << OUTPUT_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);

    // Entrada amb pull-down intern (repòs=LOW, premer=HIGH)
    io.pin_bit_mask = 1ULL << INPUT_GPIO;
    io.mode = GPIO_MODE_INPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);

    bool led = false;
    gpio_set_level(OUTPUT_GPIO, led);
    int prev_level = gpio_get_level(INPUT_GPIO);

    while (1) {
        int level_now = gpio_get_level(INPUT_GPIO);
        if (prev_level == 0 && level_now == 1) {        // flanc de pujada en prémer
            led = !led;
            gpio_set_level(OUTPUT_GPIO, led);
        }
        prev_level = level_now;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}