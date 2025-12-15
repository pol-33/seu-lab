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

static const char* DEMO_TAG = "LAB6_SCANNER";

// --- CONFIG ---
#define IBEACON_RECEIVER    1
#define IBEACON_MODE        IBEACON_RECEIVER 
#define MEASURED_POWER_AT_1M  -60.0f 
#define PATH_LOSS_EXPONENT    2.5f   

// --- HELPER: Parse Name from Advertisement Data ---
// Returns true if a name is found, and copies it to 'name_buffer'
bool get_ble_name(uint8_t *adv_data, uint8_t adv_data_len, char *name_buffer, int buffer_len) {
    int pos = 0;
    while (pos < adv_data_len) {
        uint8_t length = adv_data[pos];
        if (length == 0) break; // End of data
        
        uint8_t type = adv_data[pos + 1];
        
        // 0x09 is Complete Local Name, 0x08 is Shortened Local Name
        if (type == 0x09 || type == 0x08) {
            int name_len = length - 1;
            if (name_len > buffer_len - 1) name_len = buffer_len - 1;
            
            memcpy(name_buffer, &adv_data[pos + 2], name_len);
            name_buffer[name_len] = '\0'; // Null-terminate
            return true;
        }
        pos += length + 1;
    }
    return false;
}

float calculate_distance(int rssi) {
    if (rssi == 0) return -1.0; 
    float ratio = (MEASURED_POWER_AT_1M - rssi) / (10 * PATH_LOSS_EXPONENT);
    return pow(10, ratio);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0); // Scan forever
        break;
        
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        ESP_LOGI(DEMO_TAG, "Scanning started...");
        break;
    
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        if (scan_result->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            
            // Only look at strong signals to reduce spam
            if (scan_result->scan_rst.rssi > -70) {
                
                char dev_name[32] = {0};
                bool has_name = get_ble_name(scan_result->scan_rst.ble_adv, scan_result->scan_rst.adv_data_len, dev_name, sizeof(dev_name));
                
                // If it has a name, print it clearly!
                if (has_name) {
                    float dist = calculate_distance(scan_result->scan_rst.rssi);
                    ESP_LOGW(DEMO_TAG, "NAME: %s | MAC: %02x:%02x:%02x:%02x:%02x:%02x | Dist: %.2fm", 
                             dev_name,
                             scan_result->scan_rst.bda[0], scan_result->scan_rst.bda[1], scan_result->scan_rst.bda[2],
                             scan_result->scan_rst.bda[3], scan_result->scan_rst.bda[4], scan_result->scan_rst.bda[5],
                             dist);
                }
            }
        }
        break;
    }
    default: break;
    }
}

// ... Init functions same as before ...
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_ble_gap_register_callback(esp_gap_cb);
    
    static esp_ble_scan_params_t ble_scan_params = {
        .scan_type = BLE_SCAN_TYPE_ACTIVE,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval = 0x50,
        .scan_window = 0x30,
        .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE
    };
    esp_ble_gap_set_scan_params(&ble_scan_params);
}