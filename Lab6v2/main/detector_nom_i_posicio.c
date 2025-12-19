/*
 * LAB 6 - Device Selector & Auto-Calibrated Tracker
 * Fixed Input: Just press ENTER to continue steps.
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

static const char* TAG = "LAB6_APP";

// --- GLOBAL VARIABLES ---
uint8_t TARGET_MAC[6] = {0}; 
bool target_selected = false;

// Calibration Defaults
float measured_power_at_1m = -74.0f; 
float path_loss_exponent   = 3.0f;

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

// Calibration Accumulators
int32_t rssi_accum = 0;
int16_t sample_cnt = 0;
float calib_rssi_0_5 = 0;
float calib_rssi_1_0 = 0;
#define SAMPLES_NEEDED 40

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
    // Clear buffer
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); }
    
    printf(" >> Press ENTER (or any key) to start measuring... \n");
    while (1) {
        int c = getchar();
        // Check for any valid character (not EOF and not error)
        if (c != EOF && c != 0xFF) { 
            break;
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    // Small delay to clear newline characters
    vTaskDelay(200 / portTICK_PERIOD_MS);
}

// --- MATH ---
float calculate_distance(int rssi) {
    if (rssi == 0) return -1.0; 
    float ratio = (measured_power_at_1m - rssi) / (10 * path_loss_exponent);
    return pow(10, ratio);
}

void compute_calibration() {
    measured_power_at_1m = calib_rssi_1_0;
    float diff = calib_rssi_0_5 - calib_rssi_1_0;
    
    if (diff <= 0) {
        path_loss_exponent = 3.0f; 
        ESP_LOGE(TAG, "Calibration data unclear. Using default N=3.0");
    } else {
        path_loss_exponent = diff / 3.01f;
    }
    printf("\n>>> CALIBRATION SAVED: Power@1m=%.2f, N=%.2f <<<\n", measured_power_at_1m, path_loss_exponent);
}

// --- CONSOLE TASK (UI) ---
void console_task(void *pvParameters) {
    printf("\n*** SCANNING FOR 5 SECONDS... ***\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    
    esp_ble_gap_stop_scanning();
    current_state = STATE_SELECTING;

    printf("\n------------------------------------------------\n");
    printf(" FOUND DEVICES:\n");
    printf("------------------------------------------------\n");
    for (int i = 0; i < device_count; i++) {
        printf("[%d] Name: %-15s | MAC: %02x:%02x:%02x:%02x:%02x:%02x | RSSI: %d\n", 
               i, device_list[i].name,
               device_list[i].bda[0], device_list[i].bda[1], device_list[i].bda[2],
               device_list[i].bda[3], device_list[i].bda[4], device_list[i].bda[5],
               device_list[i].rssi);
    }
    printf("------------------------------------------------\n");
    printf("Type the number of your phone: ");

    int selection = -1;
    while (selection < 0 || selection >= device_count) {
        char c = getchar();
        if (c >= '0' && c <= '9') {
            selection = c - '0';
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    
    memcpy(TARGET_MAC, device_list[selection].bda, 6);
    target_selected = true;
    printf("\nSelected: %s\n", device_list[selection].name);
    
    // --- STEP 1 ---
    printf("\n--- CALIBRATION STEP 1 ---\n");
    printf("Place phone at 0.5 METERS.\n");
    wait_for_user_input();

    rssi_accum = 0; sample_cnt = 0;
    current_state = STATE_CALIB_0_5M;
    esp_ble_gap_start_scanning(0); 
    
    while(current_state == STATE_CALIB_0_5M) vTaskDelay(100/portTICK_PERIOD_MS);

    // --- STEP 2 ---
    printf("\n--- CALIBRATION STEP 2 ---\n");
    printf("Place phone at 1.0 METER.\n");
    wait_for_user_input();

    rssi_accum = 0; sample_cnt = 0;
    current_state = STATE_CALIB_1_0M;
    
    while(current_state == STATE_CALIB_1_0M) vTaskDelay(100/portTICK_PERIOD_MS);

    compute_calibration();
    current_state = STATE_TRACKING;
    printf("\n*** TRACKING STARTED ***\n");

    vTaskDelete(NULL);
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
            
            if (current_state == STATE_SCANNING_LIST) {
                char name[32];
                parse_ble_name(scan->scan_rst.ble_adv, scan->scan_rst.adv_data_len, name);
                add_device_to_list(scan->scan_rst.bda, name, scan->scan_rst.rssi);
            }
            else if (target_selected && memcmp(scan->scan_rst.bda, TARGET_MAC, 6) == 0) {
                int rssi = scan->scan_rst.rssi;

                if (current_state == STATE_CALIB_0_5M) {
                    rssi_accum += rssi;
                    sample_cnt++;
                    if (sample_cnt % 5 == 0) printf(".");
                    if (sample_cnt >= SAMPLES_NEEDED) {
                        calib_rssi_0_5 = (float)rssi_accum / SAMPLES_NEEDED;
                        printf(" Done (Avg: %.1f)\n", calib_rssi_0_5);
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                else if (current_state == STATE_CALIB_1_0M) {
                    rssi_accum += rssi;
                    sample_cnt++;
                    if (sample_cnt % 5 == 0) printf(".");
                    if (sample_cnt >= SAMPLES_NEEDED) {
                        calib_rssi_1_0 = (float)rssi_accum / SAMPLES_NEEDED;
                        printf(" Done (Avg: %.1f)\n", calib_rssi_1_0);
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                else if (current_state == STATE_TRACKING) {
                    float dist = calculate_distance(rssi);
                    ESP_LOGI(TAG, "RSSI: %d | Dist: %.2fm", rssi, dist);
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

    xTaskCreate(console_task, "console_task", 4096, NULL, 5, NULL);
}