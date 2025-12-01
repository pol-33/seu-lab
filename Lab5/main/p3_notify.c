#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#define TAG "EX3_NOTIFY"

// --- Configuració ---
#define QUEUE_LEN 16
#define ITEM_SIZE sizeof(int)
#define CYCLE_DELAY_MS 1000 

// --- Variables Globals ---
QueueHandle_t xQueueData;

// Handles de les tasques (Necessaris per enviar-los notificacions)
TaskHandle_t h_P1 = NULL;
TaskHandle_t h_P2 = NULL;

TaskHandle_t h_C1 = NULL;
TaskHandle_t h_C2 = NULL;
TaskHandle_t h_C3 = NULL;
TaskHandle_t h_C4 = NULL;

// --- TASQUES PRODUCTORES ---

void task_P1_Odd(void *pv) {
    int count = 1; 
    while (1) {
        // 1. Esperar notificació (actua com xSemaphoreTake)
        // pdTRUE = Esborra el valor de la notificació al sortir (reset a 0)
        // portMAX_DELAY = Espera infinita
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xQueueSend(xQueueData, &count, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "[P1] Generat IMPARELL: %d", count);
        }
        
        count += 2; 

        // 2. Notificar a la següent tasca (actua com xSemaphoreGive)
        xTaskNotifyGive(h_P2);
        
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void task_P2_Even(void *pv) {
    int count = 2; 
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xQueueSend(xQueueData, &count, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "[P2] Generat PARELL: %d", count);
        }

        count += 2; 

        // Passa el torn a P1
        xTaskNotifyGive(h_P1);

        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

// --- TASQUES CONSUMIDORES ---

void task_C1(void *pv) {
    int rx_val;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Espera el seu torn
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C1) Rebut Imparell: %d", rx_val);
        }

        // Passa torn a C2 (Següent en la roda)
        xTaskNotifyGive(h_C2);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void task_C2(void *pv) {
    int rx_val;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C2) Rebut Parell: %d", rx_val);
        }

        // Passa torn a C3
        xTaskNotifyGive(h_C3);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void task_C3(void *pv) {
    int rx_val;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C3) Rebut Imparell: %d", rx_val);
        }

        // Passa torn a C4
        xTaskNotifyGive(h_C4);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void task_C4(void *pv) {
    int rx_val;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        
        if (xQueueReceive(xQueueData, &rx_val, portMAX_DELAY)) {
            ESP_LOGW(TAG, "\t\t(C4) Rebut Parell: %d", rx_val);
        }

        // Tanca el cercle: Passa torn a C1
        xTaskNotifyGive(h_C1);
        vTaskDelay(pdMS_TO_TICKS(CYCLE_DELAY_MS));
    }
}

void app_main(void)
{
    // 1. Crear Cua
    xQueueData = xQueueCreate(QUEUE_LEN, ITEM_SIZE);

    // 2. Crear Tasques i GUARDAR ELS HANDLES
    // L'últim paràmetre de xTaskCreate és on es guarda el handle.
    
    // Productors (Prioritat 5)
    xTaskCreate(task_P1_Odd,  "P1", 2048, NULL, 5, &h_P1);
    xTaskCreate(task_P2_Even, "P2", 2048, NULL, 5, &h_P2);

    // Consumidors (Prioritat 3)
    xTaskCreate(task_C1, "C1", 2048, NULL, 3, &h_C1);
    xTaskCreate(task_C2, "C2", 2048, NULL, 3, &h_C2);
    xTaskCreate(task_C3, "C3", 2048, NULL, 3, &h_C3);
    xTaskCreate(task_C4, "C4", 2048, NULL, 3, &h_C4);

    // 3. INICIALITZAR EL SISTEMA (KICKSTART)
    // Amb notificacions, primer s'han de crear les tasques (perquè existeixin els handles)
    // i després s'envia la primera notificació ("empenta inicial").
    
    ESP_LOGI(TAG, "Tasques creades. Iniciant seqüència...");
    
    // Comença P1
    if (h_P1 != NULL) xTaskNotifyGive(h_P1);
    
    // Comença C1
    if (h_C1 != NULL) xTaskNotifyGive(h_C1);
}