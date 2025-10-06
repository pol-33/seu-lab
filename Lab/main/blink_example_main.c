#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

// === CONFIGURACIÓ DEL SISTEMA ===
#define ADC_CHANNEL         ADC_CHANNEL_0
#define ADC_ATTEN          ADC_ATTEN_DB_12        // 0-3.9V
#define ADC_WIDTH          ADC_BITWIDTH_12        // 12 bits (0-4095)
#define SAMPLE_RATE_HZ     100                    // Freqüència de mostreig 100 Hz (més eficient)
#define SAMPLE_PERIOD_MS   (1000/SAMPLE_RATE_HZ)  // 10ms entre mostres

// === PROCESSAMENT DIGITAL ===
#define BUFFER_SIZE        500                    // 5 segons de dades (100Hz * 5s)
#define MIN_PEAK_HEIGHT    3.0f                   // Altura mínima del pic (molt més baix)
#define MIN_INTERVAL_MS    400                    // Interval mínim entre batecs (150 BPM max)
#define MAX_INTERVAL_MS    2000                   // Interval màxim entre batecs (30 BPM min)
#define NOISE_THRESHOLD    1.0f                   // Llindar de soroll molt baix

// === PRESENTACIÓ RESULTATS ===
#define DISPLAY_INTERVAL_S 5                     // Mostrar BPM cada 5 segons

// === VARIABLES GLOBALS SIMPLIFICADES ===
static float dc_offset = 1700.0f;               // Offset DC
static float filtered_signal = 0;               // Senyal filtrada actual
static float signal_max = 0;                    // Màxim del senyal
static float signal_min = 0;                    // Mínim del senyal
static int sample_count = 0;                    // Comptador de mostres
static uint32_t last_peak_time = 0;             // Temps del darrer pic
static bool signal_valid = false;               // Indica si hi ha senyal vàlid

void app_main(void)
{
    printf("=== DETECTOR DE FREQÜÈNCIA CARDÍACA ===\n");
    printf("Freqüència de mostreig: %d Hz\n", SAMPLE_RATE_HZ);
    printf("Resolució ADC: 12 bits (0-4095)\n");
    printf("Rang de BPM detectat: 30-180\n\n");

    // === CONFIGURACIÓ ADC ===
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

    printf("Iniciant adquisició de dades...\n");

    // === BUCLE PRINCIPAL ===
    uint32_t last_display_time = xTaskGetTickCount();
    uint32_t sample_count = 0;
    
    while (1) {
        // === ADQUISICIÓ DE DADES ===
        int raw_adc = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &raw_adc));
        
        // Emmagatzemar mostra crua
        adc_buffer[buffer_index] = (float)raw_adc;
        
        // === PROCESSAMENT DIGITAL ===
        // 1. Eliminació de component DC
        float hp_signal = high_pass_filter((float)raw_adc);
        
        // 2. Filtratge passa-baix per eliminar soroll
        float filtered_signal = low_pass_filter(hp_signal);
        
        // Emmagatzemar senyal filtrada
        filtered_buffer[buffer_index] = filtered_signal;
        
        // Actualitzar índex del buffer circular
        buffer_index = (buffer_index + 1) % BUFFER_SIZE;
        if (!buffer_full && buffer_index == 0) {
            buffer_full = true;
        }
        
        sample_count++;
        
        // === ANÀLISI I PRESENTACIÓ DE RESULTATS ===
        uint32_t current_time = xTaskGetTickCount();
        if (pdTICKS_TO_MS(current_time - last_display_time) >= (DISPLAY_INTERVAL_S * 1000)) {
            
            if (buffer_full) {
                // Calcular estadístiques del senyal
                calculate_signal_stats();
                
                // Detectar pics
                int peak_positions[50];  // Màxim 50 pics en 5 segons
                int peak_count = detect_peaks(peak_positions, 50);
                
                // Calcular BPM
                float bpm = calculate_bpm(peak_positions, peak_count);
                
                // Mostrar resultats
                printf("=== RESULTATS ===\n");
                printf("Mostres processades: %d\n", (int)sample_count);
                printf("Variància del senyal: %.2f\n", signal_variance);
                printf("Pics detectats: %d en %ds\n", peak_count, BUFFER_SIZE/SAMPLE_RATE_HZ);
                
                if (signal_variance > NOISE_THRESHOLD && peak_count >= 2) {
                    printf("💓 FREQÜÈNCIA CARDÍACA: %.1f BPM\n", bpm);
                    
                    // Validar si el BPM està en rang fisiològic
                    if (bpm >= 30 && bpm <= 180) {
                        printf("✅ Mesura vàlida\n");
                    } else {
                        printf("⚠️  Mesura fora de rang normal\n");
                    }
                } else {
                    printf("❌ Senyal insuficient (variància: %.2f, pics: %d)\n", 
                           signal_variance, peak_count);
                    printf("   Verifiqueu la connexió del sensor\n");
                }
                
                printf("Valor ADC actual: %d\n", raw_adc);
                printf("Senyal filtrada: %.2f\n", filtered_signal);
                printf("------------------------\n\n");
                
                // Reset contador de mostres
                sample_count = 0;
            } else {
                printf("Omplint buffer... (%d%%)\n", 
                       (buffer_index * 100) / BUFFER_SIZE);
            }
            
            last_display_time = current_time;
        }
        
        // Esperar fins a la següent mostra
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}
