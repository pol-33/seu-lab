#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define APP_TAG "LAB5_EX2"

// --- Configuració GPIO ---
#define GPIO_LED1           1
#define GPIO_LED2           2
#define GPIO_LED3           3

// --- Configuració Temps ---
#define LED1_DELAY_MS       300
#define LED23_DELAY_MS      1000

// --- Global ---
SemaphoreHandle_t xMutex = NULL;

/**
 * @brief Inicialitzem els GPIOs com a sortida
 */
static void configure_leds(void)
{
    gpio_reset_pin(GPIO_LED1);
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(GPIO_LED2);
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);
    
    gpio_reset_pin(GPIO_LED3);
    gpio_set_direction(GPIO_LED3, GPIO_MODE_OUTPUT);
    
    // Estat inicial
    gpio_set_level(GPIO_LED1, 0);
    gpio_set_level(GPIO_LED2, 1); // LED2 encès
    gpio_set_level(GPIO_LED3, 0); // LED3 apagat
}

/**
 * @brief Tasca 1: Intermitència LED1 (0.3s)
 */
void task_led1(void *pvParameters)
{
    uint8_t state = 0;
    while (1) {
        // --- INICI ZONA CRÍTICA ---
        // Intentem agafar el 'token' per accedir al recurs (LEDs/Consola)
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            
            state = !state;
            gpio_set_level(GPIO_LED1, state);
            ESP_LOGI(APP_TAG, "[T1] LED1 canviat a %d", state);
            
            xSemaphoreGive(xMutex);
        }
        // --- FI ZONA CRÍTICA ---

        vTaskDelay(pdMS_TO_TICKS(LED1_DELAY_MS));
    }
}

/**
 * @brief Tasca 2: Alternança LED2 i LED3 (1s)
 */
void task_led23(void *pvParameters)
{
    uint8_t toggle = 0;
    while (1) {
        // --- INICI ZONA CRÍTICA ---
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            
            toggle = !toggle;
            if (toggle) {
                gpio_set_level(GPIO_LED2, 0);
                gpio_set_level(GPIO_LED3, 1);
                ESP_LOGW(APP_TAG, "[T2] Alternança: LED2 OFF / LED3 ON");
            } else {
                gpio_set_level(GPIO_LED2, 1);
                gpio_set_level(GPIO_LED3, 0);
                ESP_LOGW(APP_TAG, "[T2] Alternança: LED2 ON / LED3 OFF");
            }

            xSemaphoreGive(xMutex);
        }
        // --- FI ZONA CRÍTICA ---

        vTaskDelay(pdMS_TO_TICKS(LED23_DELAY_MS));
    }
}

void app_main(void)
{
    configure_leds();

    // Creem un Mutex. Per defecte es crea "lliure" (unlocked).
    xMutex = xSemaphoreCreateMutex();

    if (xMutex != NULL) {
        // Creem les tasques
        // Nota: Hem posat la mateixa prioritat per a totes dues tasques, però es poden ajustar segons les necessitats.
        xTaskCreate(task_led1, "LED1_Task", 2048, NULL, 5, NULL);
        xTaskCreate(task_led23, "LED23_Task", 2048, NULL, 5, NULL);
    } else {
        ESP_LOGE(APP_TAG, "Error creant el semàfor/mutex");
    }
}