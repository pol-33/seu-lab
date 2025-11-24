#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#define APP_TAG "OVERHEAD_TEST"

// --- Configuració GPIO ---
#define GPIO_LED1           1
#define GPIO_LED2           2

SemaphoreHandle_t xMutex = NULL;
volatile int64_t g_start_time = 0;

void task_low_priority(void *pvParameters)
{
    while (1) {
        // 1. Intentem agafar el Mutex
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            
            // Un cop tenim el mutex, NO el deixem anar de seguida.
            // Fem un delay DINS de la zona crítica.
            // Això obliga a que, si la Task High es desperta ara, 
            // es trobi la porta tancada i es BLOQUEGI realment.
            vTaskDelay(pdMS_TO_TICKS(20)); 

            // Quan tornem d'aquest delay, pot haver passat una de dues coses:
            // A) La High encara dorm: no passa res, tornarem a donar la volta.
            // B) La High s'ha despertat, ha intentat entrar i s'ha quedat bloquejada a la porta.
            //    En aquest cas (B) és quan mesurarem bé.

            gpio_set_level(GPIO_LED1, 1);
            
            // Agafem el temps just abans de l'alliberament
            g_start_time = esp_timer_get_time();

            // Alliberem el Mutex -> Si la High està esperant, SALTARÀ AQUÍ MATEIX
            xSemaphoreGive(xMutex);
            
            gpio_set_level(GPIO_LED1, 0);
        }
        // Petit descans perquè la CPU no es saturi si no hi ha ningú més
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void task_high_priority(void *pvParameters)
{
    int64_t end_time;
    
    while (1) {
        // Esperem 1 segon. Durant aquest temps, la Low va fent la seva vida (agafa, espera, deixa).
        vTaskDelay(pdMS_TO_TICKS(1000));

        // Ara ens despertem. Intentem agafar el Mutex.
        // Com que la Low té aquell 'vTaskDelay(20)' DINS del mutex,
        // és molt probable que l'enganxem tenint el mutex ocupat.
        // Ens bloquejarem aquí. El sistema farà context switch cap a la Low.
        // La Low acabarà el seu delay, agafarà el temps i farà el Give.
        if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
            
            // Just ens hem despertat i entrem
            end_time = esp_timer_get_time();
            
            int64_t overhead = end_time - g_start_time;

            // Filtre: Si l'overhead és gegant (ex: 20ms), és que no ens hem bloquejat
            // i hem agafat el mutex lliure (mala sort). Ignorem el valor.
            // Si l'overhead és petit (< 100us), és una mesura vàlida.
            if (overhead < 1000) {
                ESP_LOGI(APP_TAG, "VALID MESURA: Overhead = %lld us", overhead);
                
                // Parpelleig visible (si ho fem molt ràpid no es veu)
                gpio_set_level(GPIO_LED2, 1);
                esp_rom_delay_us(500000); // 50ms encès "hardcoded" per veure-ho segur
                gpio_set_level(GPIO_LED2, 0);
            } 
            else {
                // Això passa quan enganxem a la Low fora del mutex (no hauria de passar gairebé mai)
                ESP_LOGW(APP_TAG, "Sincronització fallida (Overhead fals: %lld us)", overhead);
            }

            xSemaphoreGive(xMutex);
        }
    }
}

void app_main(void)
{
    // Configura els teus GPIOs correctes aquí
    gpio_reset_pin(GPIO_LED1); 
    gpio_set_direction(GPIO_LED1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(GPIO_LED2); 
    gpio_set_direction(GPIO_LED2, GPIO_MODE_OUTPUT);

    xMutex = xSemaphoreCreateMutex();

    if (xMutex != NULL) {
        xTaskCreate(task_low_priority, "Low", 4096, NULL, 3, NULL);
        xTaskCreate(task_high_priority, "High", 4096, NULL, 5, NULL);
    }
}