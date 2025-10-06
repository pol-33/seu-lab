#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include <limits.h>
#include <math.h>

// --- Configuration ---
#define ADC_UNIT                ADC_UNIT_1

// Set the ADC Channel to 0, which corresponds to GPIO0 on the ESP32-C6.
#define ADC_CHANNEL             ADC_CHANNEL_0 
#define ADC_ATTEN               ADC_ATTEN_DB_12

// --- Heart Rate Algorithm Defines ---
#define SAMPLE_INTERVAL_MS      20      // Time between sensor readings (ms)
#define REPORT_INTERVAL_S       10      // Time between BPM reports (seconds)

// IMPORTANT: This threshold needs to be tuned for your specific setup.
// It should be set to a value between the sensor's baseline reading and the peak of a pulse.
// A good starting point is around 2/3 of the maximum expected ADC value.
// Given a 1.4V peak on a 3.3V system, the signal might peak around 2.4V.
// With 12-bit resolution and 12dB attenuation (~3.1V range), this is approx (2.4/3.1)*4095 = 3170.
// Let's set a threshold slightly below that.
#define PEAK_THRESHOLD          800
// When using dynamic baseline detection we'll look for a rise above the smoothed
// baseline by this delta (ADC units). Tune this instead of the static threshold.
#define PEAK_DELTA              30

// Minimum time (ms) between recognized beats to avoid double counting
#define REFRACTORY_MS           300
// Reject obviously invalid ADC readings (sensor unplugged / saturated)
#define MIN_VALID_READING       10
#define MAX_VALID_READING       4090

// Use standard-deviation multiplier to raise the detection threshold when noise is high
#define STD_MULTIPLIER          2.0f
// If the ADC value jumps more than this between samples it's probably an artifact
#define JUMP_THRESHOLD          600

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

    ESP_LOGI(TAG, "ADC Initialized Successfully on ADC_UNIT %d, Channel %d", ADC_UNIT, ADC_CHANNEL);
}

/**
 * @brief Main task to read sensor data, detect peaks, and calculate BPM.
 */
void heart_rate_task(void *pvParameter)
{
    int beat_count = 0;
    bool is_peak = false; // retained for compatibility but not used as sole gate
    long last_report_time = pdTICKS_TO_MS(xTaskGetTickCount());
    int adc_raw_value;

    /* Debug helpers */
    int sample_count = 0;
    int max_raw = 0;
    int min_raw = INT_MAX;
    int prev_raw = -1;
    
    /* Dynamic baseline for adaptive peak detection */
    float ema = 0.0f; // exponential moving average baseline
    float ema2 = 0.0f; // EMA of squared samples for variance estimation
    bool ema_initialized = false;
    TickType_t last_beat_tick = 0; // last beat time for refractory

    while (1)
    {
        // Read ADC value
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_raw_value));
        // Track min/max for this run so we can tune the threshold
        if (adc_raw_value > max_raw) max_raw = adc_raw_value;
        if (adc_raw_value < min_raw) min_raw = adc_raw_value;
        sample_count++;

        // Occasionally print a compact summary so logs aren't flooded
        if ((sample_count % (500 / SAMPLE_INTERVAL_MS)) == 0) { // every ~500ms
            ESP_LOGI(TAG, "Raw ADC cur:%d min:%d max:%d samples:%d", adc_raw_value, min_raw, max_raw, sample_count);
        }

        // Reject invalid/saturated samples
        if (adc_raw_value < MIN_VALID_READING || adc_raw_value > MAX_VALID_READING) {
            ESP_LOGW(TAG, "Ignoring out-of-range ADC sample: %d", adc_raw_value);
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
            continue;
        }

        // Reject abrupt jumps (likely artifacts) so they don't corrupt EMA or cause false peaks
        if (prev_raw >= 0 && abs(adc_raw_value - prev_raw) > JUMP_THRESHOLD) {
            ESP_LOGW(TAG, "Ignoring abrupt ADC jump: prev=%d cur=%d", prev_raw, adc_raw_value);
            // don't update prev_raw to allow next sample to be compared to the last good one
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
            continue;
        }

        // --- Adaptive Peak Detection using EMA baseline + variance-based delta + refractory ---
        const float alpha = 0.02f; // smaller alpha -> slower baseline tracking
        if (!ema_initialized) {
            ema = (float)adc_raw_value;
            ema2 = (float)adc_raw_value * (float)adc_raw_value;
            ema_initialized = true;
        } else {
            ema = (1.0f - alpha) * ema + alpha * (float)adc_raw_value;
            ema2 = (1.0f - alpha) * ema2 + alpha * ((float)adc_raw_value * (float)adc_raw_value);
        }

        // Estimate variance/stddev from EMAs
        float var = ema2 - (ema * ema);
        if (var < 0.0f) var = 0.0f;
        float stddev = sqrtf(var);

        // Dynamic delta: at least PEAK_DELTA, but increase with noise
        float dynamic_delta = fmaxf((float)PEAK_DELTA, STD_MULTIPLIER * stddev);

        // Time since last beat in ms
        long ms_since_last_beat = (pdTICKS_TO_MS(xTaskGetTickCount()) - pdTICKS_TO_MS(last_beat_tick));

        bool peak_now = false;
        // Require a rising edge (prev_raw < cur) to avoid counting plateaus or noise
        if ((float)adc_raw_value > (ema + dynamic_delta) && ms_since_last_beat > REFRACTORY_MS && !is_peak && (prev_raw < adc_raw_value)) {
            peak_now = true;
        }

        if (peak_now) {
            beat_count++;
            last_beat_tick = xTaskGetTickCount();
            ESP_LOGI(TAG, "Peak detected! Raw:%d EMA:%.1f std:%.2f dyn_delta:%.1f Beat count:%d", adc_raw_value, ema, stddev, dynamic_delta, beat_count);
            is_peak = true;
        } else if ((float)adc_raw_value < (ema + dynamic_delta * 0.5f)) {
            // allow reset of is_peak when signal falls back toward baseline
            is_peak = false;
        }

        // --- BPM Reporting every 10 seconds ---
        if ((pdTICKS_TO_MS(xTaskGetTickCount()) - last_report_time) > (REPORT_INTERVAL_S * 1000))
        {
            /* Calculate BPM based on the beats counted in the last REPORT_INTERVAL_S seconds.
             * Use floating math to avoid integer truncation and log a little more info. */
            int bpm = (int)roundf((float)beat_count * (60.0f / (float)REPORT_INTERVAL_S));
            ESP_LOGI(TAG, "Heart Rate: %d BPM (beats:%d over %d s). ADC min:%d max:%d", bpm, beat_count, REPORT_INTERVAL_S, min_raw, max_raw);

            // Reset for the next interval
            beat_count = 0;
            sample_count = 0;
            max_raw = 0;
            min_raw = INT_MAX;
            last_report_time = pdTICKS_TO_MS(xTaskGetTickCount());
        }

        // Save for next-iteration comparisons
        prev_raw = adc_raw_value;

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