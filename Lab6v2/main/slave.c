/*
 * LAB 6 - SMART SLAVE (Auto-Configuring Proxy)
 * 1. Scans for Master (0xAA) to learn Target MAC.
 * 2. Scans for Target -> Gets RSSI.
 * 3. Advertises RSSI (0xEE) back to Master.
 */

 // Quan utilitzem la versió 3 del máster (amb 2 esclaus), cal canviar el nom del dispositiu a "SLAVE_2" per a un dels dispositius esclaus (esp_ble_gap_set_device_name("SLAVE_2");

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "SLAVE_PROXY";

// --- STATE ---
uint8_t TARGET_MAC[6] = {0}; // Initially empty
bool target_configured = false;
int8_t last_rssi = 0;

// --- ADVERTISING CONFIG ---
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data = NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = 0,
    .p_service_uuid = NULL,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE
};

// Send RSSI back to Master (Protocol 0xEE)
void advertise_rssi(int8_t rssi) {
    static uint8_t manu_data[2] = {0xEE, 0x00};
    manu_data[1] = (uint8_t)rssi;
    adv_data.manufacturer_len = 2;
    adv_data.p_manufacturer_data = manu_data;
    esp_ble_gap_config_adv_data(&adv_data);
}

static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
        
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = (esp_ble_gap_cb_param_t *)param;
        if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            
            uint8_t adv_name_len = 0;
            uint8_t *manu = esp_ble_resolve_adv_data(scan->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &adv_name_len);

            // 1. LISTEN FOR MASTER CONFIGURATION (Protocol 0xAA)
            if (manu != NULL && manu[0] == 0xAA && adv_name_len >= 7) {
                // Check if MAC changed
                if (memcmp(TARGET_MAC, &manu[1], 6) != 0) {
                    memcpy(TARGET_MAC, &manu[1], 6);
                    target_configured = true;
                    ESP_LOGW(TAG, "NEW TARGET RECEIVED: %02x:%02x:%02x:%02x:%02x:%02x",
                             TARGET_MAC[0], TARGET_MAC[1], TARGET_MAC[2],
                             TARGET_MAC[3], TARGET_MAC[4], TARGET_MAC[5]);
                }
            }

            // 2. LISTEN FOR TARGET PHONE
            if (target_configured && memcmp(scan->scan_rst.bda, TARGET_MAC, 6) == 0) {
                int rssi = scan->scan_rst.rssi;
                // Only update if RSSI changed significantly to save bandwidth
                if (abs(rssi - last_rssi) > 2) {
                    last_rssi = rssi;
                    advertise_rssi(rssi); // Report to master
                    ESP_LOGI(TAG, "Tracking Target... RSSI: %d", rssi);
                }
            }
        }
        break;
    }
    
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
        break;

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
    
    esp_ble_gap_set_device_name("SLAVE_1");
    esp_ble_gap_register_callback(esp_gap_cb);
    
    // Start with empty advertising
    advertise_rssi(0);
    
    esp_ble_gap_set_scan_params(&ble_scan_params);
}