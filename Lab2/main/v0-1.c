// v0.c -> v0-1.c
// Afegim debugging per imprimir el buffer processat i el llindar de pics

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// --- Configuració de l'Algoritme ---
#define SAMPLING_FREQUENCY_HZ   50      // Mostrejar 50 vegades per segon
#define BUFFER_DURATION_S       5       // Emmagatzemar 5 segons de dades
#define BUFFER_SIZE             (SAMPLING_FREQUENCY_HZ * BUFFER_DURATION_S) // Total mostres = 250
#define SAMPLING_PERIOD_MS      (1000 / SAMPLING_FREQUENCY_HZ) // Període entre mostres (20ms)

// Llindar per a la detecció de pics (30% del valor màxim del buffer)
#define PEAK_THRESHOLD_RATIO    0.3

// --- Configuració de l'ADC (ESP32-C6) ---
#define ADC_UNIT        ADC_UNIT_1
// ADC1 Channel 4 és GPIO4 a l'ESP32-C6. Canvieu-ho si feu servir un altre pin.
#define ADC_CHANNEL     ADC_CHANNEL_4 
#define ADC_ATTEN       ADC_ATTEN_DB_6 // Atenuació de 6dB per llegir fins a ~2.5V (adequat per 1.4V)

// Tag per als missatges de logging
static const char *TAG = "HEART_RATE_MONITOR";

// Buffer per emmagatzemar les lectures de l'ADC
static int adc_buffer[BUFFER_SIZE];
// Buffer per a les dades processades (sense DC i filtrades)
static float processed_data[BUFFER_SIZE];


// Funció per inicialitzar el ADC en mode One-Shot
static void adc_init(adc_oneshot_unit_handle_t* adc_handle) {
    // --- Configuració de la unitat ADC ---
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE, // No fem servir el mode de baix consum
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, adc_handle));

    // --- Configuració del canal ADC ---
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Resolució per defecte (12 bits)
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(*adc_handle, ADC_CHANNEL, &config));
    
    ESP_LOGI(TAG, "ADC inicialitzat correctament.");
}

void heart_rate_task(void *pvParameter) {
    adc_oneshot_unit_handle_t adc_handle;
    adc_init(&adc_handle);

    // Variable per decidir si imprimim el buffer per a depuració
    bool enable_debug_plotting = true; 

    while (1) {
        // --- PAS 1: Adquisició de Dades ---
        ESP_LOGI(TAG, "Recopilant dades durant %d segons...", BUFFER_DURATION_S);
        for (int i = 0; i < BUFFER_SIZE; i++) {
            int adc_raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
            adc_buffer[i] = adc_raw;
            vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD_MS));
        }
        ESP_LOGI(TAG, "Dades recopilades. Iniciant processament.");

        // --- PAS 2: Eliminació del Component DC ---
        long sum = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            sum += adc_buffer[i];
        }
        float dc_offset = (float)sum / BUFFER_SIZE;

        for (int i = 0; i < BUFFER_SIZE; i++) {
            processed_data[i] = (float)adc_buffer[i] - dc_offset;
        }

        // --- PAS 3: Filtratge de Soroll (Mitjana Mòbil) ---
        float temp_buffer[BUFFER_SIZE];
        memcpy(temp_buffer, processed_data, sizeof(processed_data));
        for (int i = 1; i < BUFFER_SIZE - 1; i++) {
            processed_data[i] = (temp_buffer[i-1] + temp_buffer[i] + temp_buffer[i+1]) / 3.0f;
        }

        // --- PAS 4: Detecció de Pics ---
        int peak_count = 0;
        float max_value = 0;

        for (int i = 0; i < BUFFER_SIZE; i++) {
            if (processed_data[i] > max_value) {
                max_value = processed_data[i];
            }
        }
        float peak_threshold = max_value * PEAK_THRESHOLD_RATIO;
        
        // --- SECCIÓ DE DEPURACIÓ ---
        ESP_LOGI(TAG, "Valor DC calculat: %.2f", dc_offset);
        ESP_LOGI(TAG, "Valor màxim del senyal processat: %.2f", max_value);
        ESP_LOGI(TAG, "Llindar de pic calculat: %.2f", peak_threshold);
        
        if (enable_debug_plotting) {
            printf("--- INICI PLOT ---\n");
            for (int i = 0; i < BUFFER_SIZE; i++) {
                // Imprimim el senyal processat i el llindar per visualitzar-los
                printf("%.2f,%.2f\n", processed_data[i], peak_threshold);
            }
            printf("--- FI PLOT ---\n");
        }
        // -------------------------

        for (int i = 1; i < BUFFER_SIZE - 1; i++) {
            if (processed_data[i] > processed_data[i-1] && 
                processed_data[i] > processed_data[i+1] &&
                processed_data[i] > peak_threshold) 
            {
                peak_count++;
                // Opcional: Salt per evitar dobles deteccions en pics amples.
                // Si els pics són molt junts, comença amb un valor petit (ex: 5)
                // SAMPLING_FREQUENCY_HZ / 4 --> salta 250ms
                i += SAMPLING_FREQUENCY_HZ / 4; 
            }
        }
        ESP_LOGI(TAG, "Pics detectats: %d", peak_count);
        
        // --- PAS 5: Càlcul de BPM ---
        float bpm = ((float)peak_count / BUFFER_DURATION_S) * 60.0f;

        // --- PAS 6: Presentació del Resultat ---
        printf("---------------------------------------\n");
        printf("Freqüència Cardíaca: %.1f BPM\n", bpm);
        printf("---------------------------------------\n\n");

        // Desactivem el plotter després de la primera execució per no saturar la consola
        enable_debug_plotting = false; 
        // Esperem una mica abans del següent cicle complet
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    // Creem la tasca principal que farà tota la feina
    xTaskCreate(heart_rate_task, "heart_rate_task", 4096, NULL, 5, NULL);
}