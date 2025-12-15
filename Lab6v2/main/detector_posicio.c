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

static const char* DEMO_TAG = "DISTANCE_FINAL";

// --- CONFIGURATION ---
// 1. MAC Address of your Redmi Note 10 Pro (from your logs)
uint8_t TARGET_MAC[6] = {0x78, 0xb1, 0x12, 0xed, 0x7e, 0x3a};

// 2. Calibration
#define MEASURED_POWER_AT_1M  -80.0f 
#define PATH_LOSS_EXPONENT    2.0f   

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE 
};

// float calculate_distance(int rssi) {
//     if (rssi == 0) return -1.0; 
//     float ratio = (MEASURED_POWER_AT_1M - rssi) / (10 * PATH_LOSS_EXPONENT);
//     return pow(10, ratio);
// }

float calculate_distance(int rssi) {
    if (rssi == 0) return -1.0; 
    float distance = 1 * sqrt(rssi / MEASURED_POWER_AT_1M);
    return distance;
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
        
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(DEMO_TAG, "Searching for Redmi Note 10 Pro...");
        break;
    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            
            // Check if MAC matches TARGET_MAC
            if (memcmp(scan_result->scan_rst.bda, TARGET_MAC, 6) == 0) {
                int rssi = scan_result->scan_rst.rssi;
                float dist = calculate_distance(rssi);
                
                ESP_LOGI(DEMO_TAG, "PHONE FOUND! RSSI: %d dBm | Distance: %.2f meters", rssi, dist);
            }
        }
        break;
    }
    default: break;
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gap_set_scan_params(&ble_scan_params);
}