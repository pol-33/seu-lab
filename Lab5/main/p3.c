#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#define TAG "EX3_MODIFICAT"

// --- Configuració ---
#define QUEUE_LEN 16
#define ITEM_SIZE sizeof(int)
#define CYCLE_DELAY_MS 1000 

// --- Variables Globals ---
QueueHandle_t xQueueData;

// Semàfors per sincronitzar Productors
SemaphoreHandle_t sem_prod_P1; // Torn de P1
SemaphoreHandle_t sem_prod_P2; // Torn de P2

// Semàfors per sincronitzar Consumidors (Roda de torns)
SemaphoreHandle_t sem_turn_C1;
SemaphoreHandle_t sem_turn_C2;
SemaphoreHandle_t sem_turn_C3;
SemaphoreHandle_t sem_turn_C4;

// --- TASQUES PRODUCTORES ---

// P1: Genera IMPARELLS (comença amb 1)
void task_P1_Odd(void *pv) {
    int count = 1;
    while (1) {
        xSemaphoreTake(sem_prod_P1, portMAX_DELAY);

        if (xQueueSend(xQueueData, &count, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "[P1] Generat IMPARELL: %d", count);
        }
        
        count += 2; 

        // Passa el torn a P2
        xSemaphoreGive(sem_prod_P2);
        
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// P2: Genera PARELLS (comença amb 2)
void task_P2_Even(void *pv) {
    int count = 2;
    while (1) {
        xSemaphoreTake(sem_prod_P2, portMAX_DELAY);

        if (xQueueSend(xQueueData, &count, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "[P2] Generat PARELL: %d", count);
        }

        count += 2; 

        // Passa el torn a P1
        xSemaphoreGive(sem_prod_P1);

        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// --- TASQUES CONSUMIDORES ---

// C1: Vol IMPARELLS (Comença ell!)
void task_C1(void *pv) {
    int rx_val;
    while (1) {
        xSemaphoreTake(sem_turn_C1, portMAX_DELAY); 
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C1) Rebut Imparell: %d", rx_val);
        }

        // El següent número a la cua serà PARELL (generat per P2).
        // Passem el torn a un consumidor de parells: C2.
        xSemaphoreGive(sem_turn_C2);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// C2: Vol PARELLS
void task_C2(void *pv) {
    int rx_val;
    while (1) {
        xSemaphoreTake(sem_turn_C2, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C2) Rebut Parell: %d", rx_val);
        }

        // El següent serà IMPARELL. Passem el torn a l'altre d'imparells: C3.
        xSemaphoreGive(sem_turn_C3);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// C3: Vol IMPARELLS
void task_C3(void *pv) {
    int rx_val;
    while (1) {
        xSemaphoreTake(sem_turn_C3, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C3) Rebut Imparell: %d", rx_val);
        }

        // El següent serà PARELL. Passem el torn a l'altre de parells: C4.
        xSemaphoreGive(sem_turn_C4);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// C4: Vol PARELLS
void task_C4(void *pv) {
    int rx_val;
    while (1) {
        xSemaphoreTake(sem_turn_C4, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C4) Rebut Parell: %d", rx_val);
        }

        // Tanca el cercle: El següent serà IMPARELL. Tornem a C1.
        xSemaphoreGive(sem_turn_C1);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void app_main(void)
{
    // 1. Crear Cua
    xQueueData = xQueueCreate(QUEUE_LEN, ITEM_SIZE);

    // 2. Crear Semàfors
    sem_prod_P1 = xSemaphoreCreateBinary();
    sem_prod_P2 = xSemaphoreCreateBinary();
    
    sem_turn_C1 = xSemaphoreCreateBinary();
    sem_turn_C2 = xSemaphoreCreateBinary();
    sem_turn_C3 = xSemaphoreCreateBinary();
    sem_turn_C4 = xSemaphoreCreateBinary();

    // 3. Inicialitzar Torns
    
    // PRODUCTORS: P1 comença (generarà l'1)
    xSemaphoreGive(sem_prod_P1);
    
    // CONSUMIDORS: C1 comença (llegirà l'1)
    xSemaphoreGive(sem_turn_C1);

    // 4. Crear Tasques
    xTaskCreate(task_P1_Odd,  "P1", 2048, NULL, 5, NULL);
    xTaskCreate(task_P2_Even, "P2", 2048, NULL, 5, NULL);

    xTaskCreate(task_C1, "C1", 2048, NULL, 3, NULL);
    xTaskCreate(task_C2, "C2", 2048, NULL, 3, NULL);
    xTaskCreate(task_C3, "C3", 2048, NULL, 3, NULL);
    xTaskCreate(task_C4, "C4", 2048, NULL, 3, NULL);
}