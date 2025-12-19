/*
 * LAB 6 - Single Device Distance Scanner
 * Targeted for: Redmi Note 10 Pro
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h> 

#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_ibeacon_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

static const char* TAG = "CALIBRATION";

// --- CONFIGURATION ---
// 1. MAC Address of your Redmi Note 10 Pro (from your logs)
uint8_t TARGET_MAC[6] = {0x78, 0xb1, 0x12, 0xed, 0x7e, 0x3a};

// --- GLOBAL VARIABLES FOR CALIBRATION ---
// Default values (will be overwritten by calibration)
float global_measured_power_at_1m = -74.0f;
float global_path_loss_exponent   = 3.0f;

// Calibration State Machine
typedef enum {
    STATE_IDLE_START,
    STATE_MEASURING_0_5M,
    STATE_WAITING_FOR_MOVE,
    STATE_MEASURING_1_0M,
    STATE_CALIBRATION_DONE,
    STATE_RUNNING
} app_state_t;

volatile app_state_t current_state = STATE_IDLE_START;

// Calibration accumulators
int32_t rssi_accumulator = 0;
int16_t sample_count = 0;
#define SAMPLES_NEEDED 50  // How many packets to average for precision

// Measured averages
float rssi_at_0_5m = 0;
float rssi_at_1_0m = 0;

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE 
};

// --- MATH FUNCTIONS ---

// Formula: Distance = 10 ^ ((Measured_Power - RSSI) / (10 * N))
float calculate_distance(int rssi) {
    if (rssi == 0) return -1.0; 
    float ratio = (global_measured_power_at_1m - rssi) / (10 * global_path_loss_exponent);
    return pow(10, ratio);
}

// Calculate Constants based on the two measurements
void perform_calibration_math() {
    // 1. Calculate Measured Power at 1m (A)
    // Since we measured exactly at 1m, this IS the value.
    global_measured_power_at_1m = rssi_at_1_0m;

    // 2. Calculate Path Loss Exponent (N)
    // Formula derived from: RSSI = A - 10 * N * log10(distance)
    // N = (RSSI_0.5m - RSSI_1.0m) / (10 * log10(1.0 / 0.5))
    // log10(2) approx 0.301
    // Denominator approx 3.01
    
    float rssi_diff = rssi_at_0_5m - rssi_at_1_0m;
    
    // Prevent division by zero or negative logic errors
    if (rssi_diff <= 0) {
        ESP_LOGE(TAG, "Calibration Error: RSSI at 0.5m (%f) was weaker than 1m (%f). Environment too noisy.", rssi_at_0_5m, rssi_at_1_0m);
        global_path_loss_exponent = 3.0; // Fallback
    } else {
        global_path_loss_exponent = rssi_diff / 3.01f;
    }

    ESP_LOGW(TAG, "------------------------------------------------");
    ESP_LOGW(TAG, "CALIBRATION COMPLETE RESULTS:");
    ESP_LOGW(TAG, "NEW Measured Power @ 1m: %.2f dBm", global_measured_power_at_1m);
    ESP_LOGW(TAG, "NEW Path Loss Exponent:  %.2f", global_path_loss_exponent);
    ESP_LOGW(TAG, "------------------------------------------------");
}

// --- TASKS ---

// Task to handle user input via Console
void console_task(void *pvParameters) {
    char input_char;
    
    // Step 1: Prompt for 0.5m
    printf("\n\n");
    printf("==================================================\n");
    printf("   AUTO-CALIBRATION WIZARD FOR IPHONE\n");
    printf("==================================================\n");
    printf("1. Place iPhone EXACTLY 0.5 meters (50cm) away.\n");
    printf("2. Press ENTER to start measuring...\n");
    
    while(1) {
        input_char = getchar();
        if(input_char == '\n' || input_char == '\r') break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    sample_count = 0;
    rssi_accumulator = 0;
    current_state = STATE_MEASURING_0_5M;
    printf("--> Measuring 0.5m signal... Please wait.\n");

    // Wait until measurement is done
    while(current_state == STATE_MEASURING_0_5M) { vTaskDelay(100 / portTICK_PERIOD_MS); }

    // Step 2: Prompt for 1.0m
    printf("\n");
    printf("SUCCESS! 0.5m average RSSI: %.2f\n", rssi_at_0_5m);
    printf("--------------------------------------------------\n");
    printf("3. Now Move iPhone EXACTLY 1.0 meter away.\n");
    printf("4. Press ENTER to start measuring...\n");

    while(1) {
        input_char = getchar();
        if(input_char == '\n' || input_char == '\r') break;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    sample_count = 0;
    rssi_accumulator = 0;
    current_state = STATE_MEASURING_1_0M;
    printf("--> Measuring 1.0m signal... Please wait.\n");

    // Wait until measurement is done
    while(current_state == STATE_MEASURING_1_0M) { vTaskDelay(100 / portTICK_PERIOD_MS); }

    printf("\n");
    printf("SUCCESS! 1.0m average RSSI: %.2f\n", rssi_at_1_0m);
    
    // Calculate
    perform_calibration_math();
    
    current_state = STATE_RUNNING;
    printf("--> STARTING NORMAL DISTANCE TRACKING...\n\n");

    vTaskDelete(NULL); // Kill this task
}

// --- BLUETOOTH CALLBACK ---

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
        
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(TAG, "BLE Scanner Started.");
        break;
    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            
            // Check TARGET MAC
            if (memcmp(scan_result->scan_rst.bda, TARGET_MAC, 6) == 0) {
                int rssi = scan_result->scan_rst.rssi;

                // --- STATE MACHINE HANDLER ---
                if (current_state == STATE_MEASURING_0_5M) {
                    rssi_accumulator += rssi;
                    sample_count++;
                    if (sample_count % 10 == 0) printf("."); // Progress indicator
                    
                    if (sample_count >= SAMPLES_NEEDED) {
                        rssi_at_0_5m = (float)rssi_accumulator / SAMPLES_NEEDED;
                        current_state = STATE_WAITING_FOR_MOVE; // Pause
                    }
                } 
                else if (current_state == STATE_MEASURING_1_0M) {
                    rssi_accumulator += rssi;
                    sample_count++;
                    if (sample_count % 10 == 0) printf("."); 

                    if (sample_count >= SAMPLES_NEEDED) {
                        rssi_at_1_0m = (float)rssi_accumulator / SAMPLES_NEEDED;
                        current_state = STATE_CALIBRATION_DONE; // Done
                    }
                }
                else if (current_state == STATE_RUNNING) {
                    // Normal Operation using NEW values
                    float dist = calculate_distance(rssi);
                    ESP_LOGI(TAG, "iPhone: %d dBm | Dist: %.2fm", rssi, dist);
                }
            }
        }
        break;
    }
    default: break;
    }
}

void app_main(void)
{
    // Init NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    
    // Init Bluetooth
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gap_set_scan_params(&ble_scan_params);

    // Start the Console Task for user interaction
    // We create a task because we can't block the main app_main or bluetooth callbacks
    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}