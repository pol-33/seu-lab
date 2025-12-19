/*
 * LAB 6 - Device Selector, Auto-Calibration & Averaged Tracking
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

static const char* TAG = "LAB6_TRACKER";

// --- GLOBAL VARIABLES ---
uint8_t TARGET_MAC[6] = {0}; 
bool target_selected = false;

// Calibration Defaults
float measured_power_at_1m = -74.0f; 
float path_loss_exponent   = 3.0f;

// Tracking Accumulators (for the 1-second average)
volatile int32_t track_rssi_sum = 0;
volatile int16_t track_packet_count = 0;

// --- DATA STRUCTURES ---
#define MAX_DEVICES 20
typedef struct {
    uint8_t bda[6];
    char name[32];
    int rssi;
} found_device_t;

found_device_t device_list[MAX_DEVICES];
int device_count = 0;

// --- STATE MACHINE ---
typedef enum {
    STATE_SCANNING_LIST,
    STATE_SELECTING, 
    STATE_CALIB_0_5M,
    STATE_WAIT_MOVE,
    STATE_CALIB_1_0M,
    STATE_TRACKING
} app_state_t;

volatile app_state_t current_state = STATE_SCANNING_LIST;

// Calibration Variables
int32_t calib_accum = 0;
int16_t calib_cnt = 0;
float calib_rssi_0_5 = 0;
float calib_rssi_1_0 = 0;
#define CALIB_SAMPLES 40

static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50,
    .scan_window            = 0x30,
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE 
};

// --- HELPER: Parse Name ---
void parse_ble_name(uint8_t *adv_data, uint8_t adv_data_len, char *name_out) {
    int pos = 0;
    name_out[0] = '\0'; 
    while (pos < adv_data_len) {
        uint8_t length = adv_data[pos];
        if (length == 0) break;
        uint8_t type = adv_data[pos + 1];
        if (type == 0x09 || type == 0x08) { 
            int name_len = length - 1;
            if (name_len > 31) name_len = 31;
            memcpy(name_out, &adv_data[pos + 2], name_len);
            name_out[name_len] = '\0';
            return;
        }
        pos += length + 1;
    }
    strcpy(name_out, "Unknown");
}

// --- HELPER: Store Device ---
void add_device_to_list(uint8_t *bda, char *name, int rssi) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(device_list[i].bda, bda, 6) == 0) {
            device_list[i].rssi = rssi;
            if (strcmp(device_list[i].name, "Unknown") == 0 && strcmp(name, "Unknown") != 0) {
                strcpy(device_list[i].name, name);
            }
            return;
        }
    }
    if (device_count < MAX_DEVICES) {
        memcpy(device_list[device_count].bda, bda, 6);
        strcpy(device_list[device_count].name, name);
        device_list[device_count].rssi = rssi;
        device_count++;
    }
}

// --- HELPER: Wait for User Input ---
void wait_for_user_input() {
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); } // Clear buffer
    printf(" >> Press ENTER to measure... \n");
    while (1) {
        int c = getchar();
        if (c != EOF && c != 0xFF) break;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

// --- MATH ---
float calculate_distance(float rssi) {
    if (rssi == 0) return -1.0; 
    float ratio = (measured_power_at_1m - rssi) / (10 * path_loss_exponent);
    return pow(10, ratio);
}

void compute_calibration() {
    measured_power_at_1m = calib_rssi_1_0;
    // Calculate N:  (RSSI@0.5 - RSSI@1.0) / (10 * log10(1/0.5))
    // 10 * log10(2) = 3.01
    float diff = calib_rssi_0_5 - calib_rssi_1_0;
    
    if (diff <= 0) {
        path_loss_exponent = 3.0f; // Default if data is bad
        ESP_LOGE(TAG, "Bad calibration data (0.5m was weaker than 1m). Using default N=3.0");
    } else {
        path_loss_exponent = diff / 3.01f;
    }
    printf("\n>>> CALIBRATION RESULTS <<<\n");
    printf("   Reference Power (1m): %.2f dBm\n", measured_power_at_1m);
    printf("   Path Loss Exponent:   %.2f\n", path_loss_exponent);
    printf(">>> ------------------- <<<\n");
}

// --- UI & TRACKING LOOP TASK ---
void app_task(void *pvParameters) {
    // 1. SCANNING
    printf("\n*** SCANNING FOR 5 SECONDS... ***\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    esp_ble_gap_stop_scanning();
    current_state = STATE_SELECTING;

    // 2. SELECTION
    printf("\n------------------------------------------------\n");
    printf(" FOUND DEVICES:\n");
    printf("------------------------------------------------\n");
    for (int i = 0; i < device_count; i++) {
        printf("[%d] %-15s | %02x:%02x:%02x:%02x:%02x:%02x | RSSI: %d\n", 
               i, device_list[i].name,
               device_list[i].bda[0], device_list[i].bda[1], device_list[i].bda[2],
               device_list[i].bda[3], device_list[i].bda[4], device_list[i].bda[5],
               device_list[i].rssi);
    }
    printf("------------------------------------------------\n");
    printf("Type number of your phone: ");

    int selection = -1;
    while (selection < 0 || selection >= device_count) {
        char c = getchar();
        if (c >= '0' && c <= '9') selection = c - '0';
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    memcpy(TARGET_MAC, device_list[selection].bda, 6);
    target_selected = true;
    printf("\nSelected: %s\n", device_list[selection].name);
    
    // 3. CALIBRATION 0.5m
    printf("\n--- CALIBRATION STEP 1 ---\n");
    printf("Place phone at 0.5 METERS.\n");
    wait_for_user_input();

    calib_accum = 0; calib_cnt = 0;
    current_state = STATE_CALIB_0_5M;
    esp_ble_gap_start_scanning(0); 
    
    while(current_state == STATE_CALIB_0_5M) vTaskDelay(100/portTICK_PERIOD_MS);

    // 4. CALIBRATION 1.0m
    printf("\n--- CALIBRATION STEP 2 ---\n");
    printf("Place phone at 1.0 METER.\n");
    wait_for_user_input();

    calib_accum = 0; calib_cnt = 0;
    current_state = STATE_CALIB_1_0M;
    
    while(current_state == STATE_CALIB_1_0M) vTaskDelay(100/portTICK_PERIOD_MS);

    compute_calibration();
    
    // 5. TRACKING LOOP (1 Second Average)
    current_state = STATE_TRACKING;
    printf("\n*** TRACKING STARTED (Updates every 1s) ***\n");

    while (1) {
        // Reset accumulators
        track_rssi_sum = 0;
        track_packet_count = 0;

        // Wait 1 second to gather data
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if (track_packet_count > 0) {
            float avg_rssi = (float)track_rssi_sum / track_packet_count;
            float dist = calculate_distance(avg_rssi);
            
            ESP_LOGI(TAG, "Avg RSSI: %6.1f dBm | Avg Dist: %5.2f m | Packets: %d", 
                     avg_rssi, dist, track_packet_count);
        } else {
            ESP_LOGW(TAG, "No signal received...");
        }
    }
}

// --- BLE CALLBACK ---
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        esp_ble_gap_start_scanning(0);
        break;
        
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = (esp_ble_gap_cb_param_t *)param;
        if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
            
            // Phase 1: Scan List
            if (current_state == STATE_SCANNING_LIST) {
                char name[32];
                parse_ble_name(scan->scan_rst.ble_adv, scan->scan_rst.adv_data_len, name);
                add_device_to_list(scan->scan_rst.bda, name, scan->scan_rst.rssi);
            }
            // Check Target
            else if (target_selected && memcmp(scan->scan_rst.bda, TARGET_MAC, 6) == 0) {
                int rssi = scan->scan_rst.rssi;

                // Phase 2: Calibration 0.5m
                if (current_state == STATE_CALIB_0_5M) {
                    calib_accum += rssi;
                    calib_cnt++;
                    if (calib_cnt % 5 == 0) printf(".");
                    if (calib_cnt >= CALIB_SAMPLES) {
                        calib_rssi_0_5 = (float)calib_accum / CALIB_SAMPLES;
                        printf(" Done (%.1f)\n", calib_rssi_0_5);
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                // Phase 3: Calibration 1.0m
                else if (current_state == STATE_CALIB_1_0M) {
                    calib_accum += rssi;
                    calib_cnt++;
                    if (calib_cnt % 5 == 0) printf(".");
                    if (calib_cnt >= CALIB_SAMPLES) {
                        calib_rssi_1_0 = (float)calib_accum / CALIB_SAMPLES;
                        printf(" Done (%.1f)\n", calib_rssi_1_0);
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                // Phase 4: Tracking (Accumulate for Task)
                else if (current_state == STATE_TRACKING) {
                    track_rssi_sum += rssi;
                    track_packet_count++;
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
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gap_set_scan_params(&ble_scan_params);

    xTaskCreate(app_task, "app_task", 4096, NULL, 5, NULL);
}