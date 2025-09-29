#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"

// --- Configuration ---
#define ADC_UNIT                ADC_UNIT_1
#define ADC_CHANNEL             ADC_CHANNEL_0   // GPIO0 on ESP32-C6
#define ADC_ATTEN               ADC_ATTEN_DB_12
#define ADC_VREF_MV             3300            // Reference voltage in millivolts

// --- Heart Rate Algorithm Defines ---
#define SAMPLE_INTERVAL_MS      20      // Time between sensor readings (ms)
#define REPORT_INTERVAL_S       10      // Time between BPM reports (seconds)

// --- THRESHOLD IN MILLIVOLTS ---
// IMPORTANT: Tune this value based on your sensor's output.
// If your baseline is ~1600 mV, a pulse peak might be around 1750-1800 mV.
// A good starting point is a value between your observed baseline and peak.
#define PEAK_THRESHOLD_MV       1370

// --- Globals ---
static const char *TAG = "HEART_RATE_SENSOR";
static adc_oneshot_unit_handle_t adc1_handle;

/**
 * @brief Initialize the ADC for oneshot reading.
 */
static void configure_adc(void)
{
    // ADC1 initialization
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    // ADC channel configuration
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // Use 12-bit resolution
        .atten = ADC_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

    ESP_LOGI(TAG, "ADC Initialized Successfully on ADC_UNIT %d, Channel %d (GPIO0)", ADC_UNIT, ADC_CHANNEL);
}

/**
 * @brief Main task to read sensor data, detect peaks, and calculate BPM.
 */
void heart_rate_task(void *pvParameter)
{
    int beat_count = 0;
    bool is_peak = false;
    long last_report_time = 0;
    int adc_raw_value;

    while (1)
    {
        // Read raw ADC value
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw_value));
        
        // Convert raw value to millivolts
        int voltage_mv = (adc_raw_value * ADC_VREF_MV) / 4095;

        // --- Debugging output is now in mV ---
        //ESP_LOGI(TAG, "Sensor Value: %d mV", voltage_mv);

        // --- Peak Detection Logic now uses mV ---
        if (voltage_mv > PEAK_THRESHOLD_MV && !is_peak)
        {
            beat_count++;
            is_peak = true; // Mark that we are in a peak to avoid double counting
        }
        else if (voltage_mv < PEAK_THRESHOLD_MV && is_peak)
        {
            is_peak = false; // Peak is over, ready to detect the next one
        }

        // --- BPM Reporting every 10 seconds ---
        if ((pdTICKS_TO_MS(xTaskGetTickCount()) - last_report_time) > (REPORT_INTERVAL_S * 1000))
        {
            // Calculate BPM based on the beats counted in the last 10 seconds
            int bpm = beat_count * (60 / REPORT_INTERVAL_S);
            ESP_LOGI(TAG, "-------------------------");
            ESP_LOGI(TAG, "Heart Rate: %d BPM", bpm);
            ESP_LOGI(TAG, "-------------------------");

            // Reset for the next interval
            beat_count = 0;
            last_report_time = pdTICKS_TO_MS(xTaskGetTickCount());
        }

        // Wait for the next sample
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

void app_main(void)
{
    // Initialize ADC
    configure_adc();

    // Create the heart rate measurement task
    xTaskCreate(heart_rate_task, "heart_rate_task", 4096, NULL, 5, NULL);
}