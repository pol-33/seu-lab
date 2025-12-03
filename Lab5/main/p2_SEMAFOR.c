#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define APP_TAG "LAB5_EX2_BIN"

// --- Configuració GPIO ---
#define GPIO_LED1           1
#define GPIO_LED2           2
#define GPIO_LED3           3

// --- Configuració Temps ---
#define LED1_DELAY_MS       300
#define LED23_DELAY_MS      1000

// Variable global per al semàfor
SemaphoreHandle_t xSemBinari = NULL;

/**
 * @brief Inicialitza els GPIOs com a sortida
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

// Tasca 1: Intermitència LED1 (0.3s)
void task_led1(void *pvParameters)
{
    uint8_t state = 0;
    while (1) {
        // Intentem agafar el semàfor
        if (xSemaphoreTake(xSemBinari, portMAX_DELAY) == pdTRUE) {
            
            // --- ZONA CRÍTICA ---
            state = !state;
            gpio_set_level(GPIO_LED1, state);
            ESP_LOGI(APP_TAG, "[T1] LED1: %d", state);
            
            // Retornem el semàfor perquè l'altra tasca pugui entrar
            xSemaphoreGive(xSemBinari);
            // --- FI ZONA CRÍTICA ---
        }

        vTaskDelay(pdMS_TO_TICKS(LED1_DELAY_MS));
    }
}

// Tasca 2: Alternança LED2/LED3 (1s)
void task_led23(void *pvParameters)
{
    uint8_t toggle = 0;
    while (1) {
        // Intentem agafar el semàfor
        if (xSemaphoreTake(xSemBinari, portMAX_DELAY) == pdTRUE) {
            
            // --- ZONA CRÍTICA ---
            toggle = !toggle;
            if (toggle) {
                gpio_set_level(GPIO_LED2, 0);
                gpio_set_level(GPIO_LED3, 1);
                ESP_LOGW(APP_TAG, "[T2] L2 OFF / L3 ON");
            } else {
                gpio_set_level(GPIO_LED2, 1);
                gpio_set_level(GPIO_LED3, 0);
                ESP_LOGW(APP_TAG, "[T2] L2 ON / L3 OFF");
            }

            // Retornem el semàfor
            xSemaphoreGive(xSemBinari);
            // --- FI ZONA CRÍTICA ---
        }

        vTaskDelay(pdMS_TO_TICKS(LED23_DELAY_MS));
    }
}

void app_main(void)
{
    configure_leds();

    // Creem un Semàfor Binari
    xSemBinari = xSemaphoreCreateBinary();

    if (xSemBinari != NULL) {
        // !!! PAS CRÍTIC !!!
        // A diferència del Mutex, el Semàfor Binari es crea "BUIT" (0).
        // Hem de fer un Give inicial per posar-lo a "1" i que la primera
        // tasca pugui entrar. Sinó, el programa es penja.
        xSemaphoreGive(xSemBinari);

        // Creació de tasques
        xTaskCreate(task_led1, "LED1_Task", 2048, NULL, 5, NULL);
        xTaskCreate(task_led23, "LED23_Task", 2048, NULL, 5, NULL);
        
        ESP_LOGI(APP_TAG, "Semàfor Binari creat i tasques iniciades.");
    } else {
        ESP_LOGE(APP_TAG, "Error creant el semàfor");
    }
}