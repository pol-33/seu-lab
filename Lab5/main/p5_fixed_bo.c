#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <math.h>

SemaphoreHandle_t mutex;

/* PRIORITATS (de menor a major):
 * LOW : 3
 * MEDIUM : 5
 * HIGH : 7
 */
void task_low(void *pv)
{
    while (1)
    {
        printf("[LOW] Intentant agafar el mutex...\n");
        xSemaphoreTake(mutex, portMAX_DELAY);
        printf("[LOW] Tinc el mutex! (simulant treball llarg)\n");
        // Simula feina llarga que bloqueja el recurs
        double aux = 0;
        for (int i = 0; i < 10; i++)
        {
            aux = 0;
            for (long j = 0; j < 100000; j++) {
                aux = sqrt(j);
            }
            printf("[LOW] Treball lent... %d/10\n", i + 1);
        }
        printf("[LOW] Alliberant mutex\n");
        xSemaphoreGive(mutex);
        printf("[LOW] Mutex alliberat\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void task_high(void *pv)
{
    while (1)
    {
        printf(">>>> [HIGH] Intentant agafar el mutex...\n");
        // Aquí quedarà bloquejada si LOW té el mutex
        xSemaphoreTake(mutex, portMAX_DELAY);
        printf(">>>> [HIGH] Finalment he obtingut el mutex!\n");
        xSemaphoreGive(mutex);
        printf("[HIGH] Mutex alliberat\n");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void task_medium(void *pv)
{
    while (1)
    {
        // Aquesta tasca només consumeix CPU contínuament
        printf("[MEDIUM] Executant-se (interromp LOW!)\n");
        // Simula feina llarga que bloqueja el recurs
        for (int i = 0; i < 3; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(500)); // treball lent
            printf("[MEDIUM] Treball lent... %d/3\n", i + 1);
        }
        printf("[MEDIUM] Treball Finalitzat\n");
        vTaskDelay(50);
    }
}

void app_main()
{
    mutex = xSemaphoreCreateMutex();
    xTaskCreate(task_low, "LOW", 4096, NULL, 3, NULL);
    vTaskDelay(pdMS_TO_TICKS(100)); // retard perquè LOW ja tingui el mutex
    xTaskCreate(task_high, "HIGH", 4096, NULL, 7, NULL);
    xTaskCreate(task_medium, "MEDIUM", 4096, NULL, 5, NULL);
}