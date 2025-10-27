#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"

static const char *TAG = "SIMPLE_FILTER";

// === CONFIGURACIÓN SIMPLE ===
#define ADC_CHANNEL         ADC_CHANNEL_0
#define ADC_ATTEN          ADC_ATTEN_DB_0         // 0-1100mV range
#define ADC_WIDTH          ADC_BITWIDTH_12        // 12-bit resolution (0-4095)
#define SAMPLE_RATE_HZ     200                    // 200 Hz sampling
#define SAMPLE_PERIOD_MS   (1000/SAMPLE_RATE_HZ) // 5ms period

// === CONVERSIÓN ADC A VOLTIOS ===
#define ADC_MAX_VALUE      4095.0f               // 12-bit max
#define VREF_MV            3300.0f               // 3.3V reference voltage in mV
#define ADC_TO_MV(adc_val) ((adc_val * VREF_MV) / ADC_MAX_VALUE)

// === FILTRO PASA BAJOS SIMPLE ===
#define FILTER_ALPHA       0.1f                 // Factor de suavizado (0.0 a 1.0)
                                                // Menor valor = más suave pero más lento
                                                // Mayor valor = más rápido pero menos suave

// Variables globales del filtro
static float filtered_value = 0.0f;
static bool filter_initialized = false;

/**
 * Filtro pasa bajos exponencial simple
 * Fórmula: y[n] = α * x[n] + (1-α) * y[n-1]
 * Donde:
 * - x[n] = nueva muestra de entrada
 * - y[n] = salida filtrada
 * - α = factor de suavizado (FILTER_ALPHA)
 */
float simple_lowpass_filter(float input) {
    if (!filter_initialized) {
        // Inicializar el filtro con la primera muestra
        filtered_value = input;
        filter_initialized = true;
    } else {
        // Aplicar el filtro exponencial
        filtered_value = FILTER_ALPHA * input + (1.0f - FILTER_ALPHA) * filtered_value;
    }
    
    return filtered_value;
}

void app_main(void)
{
    ESP_LOGI(TAG, "🔧 Filtro Pasa Bajos Simple - ESP32-C6");
    ESP_LOGI(TAG, "📊 Factor de suavizado α = %.2f", FILTER_ALPHA);
    ESP_LOGI(TAG, "⚡ Frecuencia de muestreo: %d Hz", SAMPLE_RATE_HZ);
    
    // Configuración ADC
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_0,  // 0-1100mV range
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    
    ESP_LOGI(TAG, "✅ ADC configurado correctamente");
    ESP_LOGI(TAG, "🚀 Iniciando captura y filtrado de señal...");
    ESP_LOGI(TAG, "");
    
    // Variables para estadísticas
    int sample_count = 0;
    float sum_raw = 0, sum_filtered = 0;
    uint32_t last_display_time = 0;
    
    while (1) {
        // Leer ADC
        int adc_raw = 0;
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_raw));
        
        // Convertir a milivoltios
        float voltage_mv = ADC_TO_MV(adc_raw);
        
        // Aplicar filtro pasa bajos
        float filtered_mv = simple_lowpass_filter(voltage_mv);
        
        // Acumular estadísticas
        sum_raw += voltage_mv;
        sum_filtered += filtered_mv;
        sample_count++;
        
        // Mostrar resultados cada segundo
        uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (current_time - last_display_time >= 1000) {
            float avg_raw = sum_raw / sample_count;
            float avg_filtered = sum_filtered / sample_count;
            
            ESP_LOGI(TAG, "📈 Muestra actual:");
            ESP_LOGI(TAG, "   ADC: %d → %.1f mV (crudo)", adc_raw, voltage_mv);
            ESP_LOGI(TAG, "   Filtrado: %.1f mV", filtered_mv);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "📊 Promedios (últimas %d muestras):", sample_count);
            ESP_LOGI(TAG, "   Señal cruda: %.1f mV", avg_raw);
            ESP_LOGI(TAG, "   Señal filtrada: %.1f mV", avg_filtered);
            ESP_LOGI(TAG, "   Diferencia: %.1f mV", avg_raw - avg_filtered);
            ESP_LOGI(TAG, "");
            ESP_LOGI(TAG, "🔍 Análisis del filtro:");
            ESP_LOGI(TAG, "   Reducción de ruido: %.1f%%", 
                     fabsf(avg_raw - avg_filtered) / avg_raw * 100.0f);
            ESP_LOGI(TAG, "   Factor α = %.2f", FILTER_ALPHA);
            ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
            ESP_LOGI(TAG, "");
            
            // Reset estadísticas
            sum_raw = 0;
            sum_filtered = 0;
            sample_count = 0;
            last_display_time = current_time;
        }
        
        // Esperar según la frecuencia de muestreo
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}