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

// Llindar per a la detecció de pics (60% del valor màxim del buffer)
#define PEAK_THRESHOLD_RATIO    0.6 

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

    while (1) {
        // --- PAS 1: Adquisició de Dades ---
        ESP_LOGI(TAG, "Recopilant dades durant %d segons...", BUFFER_DURATION_S);
        for (int i = 0; i < BUFFER_SIZE; i++) {
            int adc_raw;
            ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw));
            adc_buffer[i] = adc_raw;
            // Esperem el temps necessari per mantenir la freqüència de mostreig
            vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD_MS));
        }
        ESP_LOGI(TAG, "Dades recopilades. Iniciant processament.");

        // --- PAS 2: Eliminació del Component DC ---
        long sum = 0;
        for (int i = 0; i < BUFFER_SIZE; i++) {
            sum += adc_buffer[i];
        }
        float dc_offset = (float)sum / BUFFER_SIZE;

        // Restem l'offset a cada mostra
        for (int i = 0; i < BUFFER_SIZE; i++) {
            processed_data[i] = (float)adc_buffer[i] - dc_offset;
        }

        // --- PAS 3: Filtratge de Soroll (Mitjana Mòbil simple de 3 punts) ---
        // Creem una còpia temporal ja que no podem modificar el buffer mentre el llegim
        float temp_buffer[BUFFER_SIZE];
        memcpy(temp_buffer, processed_data, sizeof(processed_data));

        for (int i = 1; i < BUFFER_SIZE - 1; i++) {
            // Mitjana de la mostra anterior, l'actual i la següent
            processed_data[i] = (temp_buffer[i-1] + temp_buffer[i] + temp_buffer[i+1]) / 3.0f;
        }

        // --- PAS 4: Detecció de Pics ---
        int peak_count = 0;
        float max_value = 0;

        // Trobem el valor màxim en el buffer processat per establir el llindar
        for (int i = 0; i < BUFFER_SIZE; i++) {
            if (processed_data[i] > max_value) {
                max_value = processed_data[i];
            }
        }
        float peak_threshold = max_value * PEAK_THRESHOLD_RATIO;
        ESP_LOGI(TAG, "Valor màxim del senyal: %.2f, Llindar de pic: %.2f", max_value, peak_threshold);
        
        // Comptem els pics que superen el llindar
        // Comencem a i=1 i acabem a BUFFER_SIZE-1 per poder comprovar els veïns
        for (int i = 1; i < BUFFER_SIZE - 1; i++) {
            // Un punt és un pic si és més alt que els seus veïns i supera el llindar
            if (processed_data[i] > processed_data[i-1] && 
                processed_data[i] > processed_data[i+1] &&
                processed_data[i] > peak_threshold) 
            {
                peak_count++;
                // Per evitar dobles deteccions en pics amples, podríem saltar algunes mostres
                // i += 5; // Opcional: saltar 5 mostres (100ms) per evitar comptar el mateix batec
            }
        }
        ESP_LOGI(TAG, "Pics detectats en %d segons: %d", BUFFER_DURATION_S, peak_count);
        
        // --- PAS 5: Càlcul de BPM (Batecs Per Minut) ---
        float bpm = ((float)peak_count / BUFFER_DURATION_S) * 60.0f;

        // --- PAS 6: Presentació del Resultat ---
        printf("---------------------------------------\n");
        printf("Freqüència Cardíaca: %.1f BPM\n", bpm);
        printf("---------------------------------------\n\n");

        // El bucle es repetirà automàticament. La durada de cada cicle és el temps de
        // recopilació de dades més el temps de processament (que és molt ràpid).
    }
}

void app_main(void)
{
    // Creem la tasca principal que farà tota la feina
    xTaskCreate(heart_rate_task, "heart_rate_task", 4096, NULL, 5, NULL);
}