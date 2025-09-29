#include <stdio.h>
#include <math.h> // Required for M_PI
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

// --- IIR Filter Configuration ---
// Define constants for the filter calculations
#define SAMPLE_TIME_S           (SAMPLE_INTERVAL_MS / 1000.0f)
#define TWO_PI                  (2.0f * M_PI)

// --- TUNABLE PARAMETERS ---
// Now you only need to change these two values
#define HPF_CUTOFF_FREQ_HZ      0.7f    // Cutoff for High-Pass Filter (removes DC)
#define LPF_CUTOFF_FREQ_HZ      4.0f    // Cutoff for Low-Pass Filter (removes noise)

// --- AUTOMATIC ALPHA CALCULATION ---
// The compiler calculates the correct alpha values before flashing.
// Formula: a = (2*PI*Ts*fc) / (2*PI*Ts*fc + 1)
#define HPF_ALPHA               ((TWO_PI * SAMPLE_TIME_S * HPF_CUTOFF_FREQ_HZ) / (TWO_PI * SAMPLE_TIME_S * HPF_CUTOFF_FREQ_HZ + 1.0f))
#define LPF_ALPHA               ((TWO_PI * SAMPLE_TIME_S * LPF_CUTOFF_FREQ_HZ) / (TWO_PI * SAMPLE_TIME_S * LPF_CUTOFF_FREQ_HZ + 1.0f))


// --- THRESHOLD FOR FILTERED SIGNAL (in mV) ---
#define PEAK_THRESHOLD_MV       4.0f

// --- Globals ---
static const char *TAG = "HEART_RATE_SENSOR";
static adc_oneshot_unit_handle_t adc1_handle;

/**
 * @brief Implements a first-order IIR high-pass filter.
 */
float highpass_filter(float input_value, float alpha)
{
    static float prev_input = 0.0f;
    static float prev_output = 0.0f;
    static bool first_run = true;
    if (first_run) {
        prev_input = input_value;
        prev_output = 0.0f;
        first_run = false;
    }
    float output_value = (1.0f - alpha) * (prev_output + input_value - prev_input);
    prev_input = input_value;
    prev_output = output_value;
    return output_value;
}

/**
 * @brief Implements a first-order IIR low-pass filter.
 */
float lowpass_filter(float input_value, float alpha)
{
    static float prev_output = 0.0f;
    static bool first_run = true;
    if (first_run) {
        prev_output = input_value;
        first_run = false;
    }
    float output_value = alpha * input_value + (1.0f - alpha) * prev_output;
    prev_output = output_value;
    return output_value;
}


/**
 * @brief Initialize the ADC for oneshot reading.
 */
static void configure_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config));

    ESP_LOGI(TAG, "ADC Initialized Successfully on ADC_UNIT %d, Channel %d (GPIO0)", ADC_UNIT, ADC_CHANNEL);
    // You can even print the calculated alpha values to verify them!
    ESP_LOGI(TAG, "Calculated HPF Alpha: %f, LPF Alpha: %f", HPF_ALPHA, LPF_ALPHA);
}

/**
 * @brief Main task to read, filter, and process the sensor signal.
 */
void heart_rate_task(void *pvParameter)
{
    int beat_count = 0;
    bool is_peak = false;
    long last_report_time = 0;
    int adc_raw_value;

    while (1)
    {
        // 1. Read raw ADC value and convert to millivolts
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw_value));
        int voltage_mv = (adc_raw_value * ADC_VREF_MV) / 4095;

        // 2. Apply the filters in sequence (now using the auto-calculated alphas)
        float hpf_output = highpass_filter((float)voltage_mv, HPF_ALPHA);
        float final_signal = lowpass_filter(hpf_output, LPF_ALPHA);
        
        // --- Debugging output shows raw and filtered values ---
         ESP_LOGI(TAG, "Raw: %d mV,  Filtered: %.2f mV", voltage_mv, final_signal);

        // 3. Peak Detection on the final, filtered signal
        if (final_signal > PEAK_THRESHOLD_MV && !is_peak)
        {
            beat_count++;
            is_peak = true; 
        }
        else if (final_signal < PEAK_THRESHOLD_MV && is_peak)
        {
            is_peak = false; 
        }

        // 4. BPM Reporting (every 10 seconds)
        if ((pdTICKS_TO_MS(xTaskGetTickCount()) - last_report_time) > (REPORT_INTERVAL_S * 1000))
        {
            int bpm = beat_count * (60 / REPORT_INTERVAL_S);
            ESP_LOGI(TAG, "-------------------------");
            ESP_LOGI(TAG, "Heart Rate: %d BPM", bpm);
            ESP_LOGI(TAG, "-------------------------");
            beat_count = 0;
            last_report_time = pdTICKS_TO_MS(xTaskGetTickCount());
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
    }
}

void app_main(void)
{
    configure_adc();
    xTaskCreate(heart_rate_task, "heart_rate_task", 4096, NULL, 5, NULL);
}