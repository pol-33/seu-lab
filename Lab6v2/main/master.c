/*
 * LAB 6 - MASTER COMMANDER
 * 1. Selects Device.
 * 2. Broadcasts Device MAC to Slave (Protocol 0xAA).
 * 3. Triangulates.
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

static const char* TAG = "MASTER_CMD";

// --- CONFIGURATION ---
#define SLAVE_POS_X  2.0f 
#define SLAVE_NAME   "SLAVE_1"

// --- GLOBALS ---
uint8_t TARGET_MAC[6] = {0}; 
bool target_selected = false;
float measured_power_at_1m = -74.0f; 
float path_loss_exponent   = 3.0f;

// Triangulation Vars
volatile int32_t track_rssi_sum = 0;
volatile int16_t track_packet_count = 0;
volatile int8_t slave_read_rssi = 0;

// --- BLE PARAMS ---
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE 
};

// Adv params for broadcasting MAC
static esp_ble_adv_data_t master_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
static esp_ble_adv_params_t ble_adv_params = {
    .adv_int_min = 0x20, .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND, .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL, .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// --- DATA STRUCTS & HELPERS ---
#define MAX_DEVICES 20
typedef struct { uint8_t bda[6]; char name[32]; int rssi; } found_device_t;
found_device_t device_list[MAX_DEVICES];
int device_count = 0;

typedef enum { STATE_SCANNING_LIST, STATE_SELECTING, STATE_CALIB_0_5M, STATE_WAIT_MOVE, STATE_CALIB_1_0M, STATE_TRACKING } app_state_t;
volatile app_state_t current_state = STATE_SCANNING_LIST;

int32_t calib_accum = 0; int16_t calib_cnt = 0;
float calib_rssi_0_5 = 0; float calib_rssi_1_0 = 0;
#define CALIB_SAMPLES 30

void parse_ble_name(uint8_t *adv_data, uint8_t adv_data_len, char *name_out) {
    int pos = 0; name_out[0] = '\0'; 
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

void add_device_to_list(uint8_t *bda, char *name, int rssi) {
    for (int i = 0; i < device_count; i++) {
        if (memcmp(device_list[i].bda, bda, 6) == 0) {
            device_list[i].rssi = rssi;
            if (strcmp(device_list[i].name, "Unknown") == 0 && strcmp(name, "Unknown") != 0) strcpy(device_list[i].name, name);
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

int get_single_digit_input() {
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); }
    while (1) {
        int c = getchar();
        if (c != EOF && c >= '0' && c <= '9') return c - '0';
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void wait_for_enter() {
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); }
    printf(" >> Press ENTER to measure... \n"); fflush(stdout);
    while (1) {
        int c = getchar();
        if (c != EOF && c != 0xFF) break;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

float calculate_distance(float rssi) {
    if (rssi == 0) return 0.0; 
    float ratio = (measured_power_at_1m - rssi) / (10 * path_loss_exponent);
    return pow(10, ratio);
}

void compute_calibration() {
    measured_power_at_1m = calib_rssi_1_0;
    float diff = calib_rssi_0_5 - calib_rssi_1_0;
    if (diff <= 0) { path_loss_exponent = 3.0f; } else { path_loss_exponent = diff / 3.01f; }
    printf("\n>>> CALIBRATION SAVED: Power@1m=%.2f, N=%.2f <<<\n", measured_power_at_1m, path_loss_exponent);
}

// --- NEW: BROADCAST TARGET MAC TO SLAVE ---
void broadcast_target_to_slave() {
    // Protocol: [0xAA, MAC_BYTE_0 ... MAC_BYTE_5]
    static uint8_t manu_payload[7];
    manu_payload[0] = 0xAA; // ID
    memcpy(&manu_payload[1], TARGET_MAC, 6);

    master_adv_data.manufacturer_len = 7;
    master_adv_data.p_manufacturer_data = manu_payload;

    esp_ble_gap_config_adv_data(&master_adv_data);
    // Note: We will start advertising inside the GAP event handler
}

void print_position(float d1, float d2) {
    float D = SLAVE_POS_X; 
    if (d1 + d2 <= D) { printf("Pos: On Line | X=%.2f\n", d1); return; }
    float x = (pow(d1, 2) - pow(d2, 2) + pow(D, 2)) / (2 * D);
    float y_sq = pow(d1, 2) - pow(x, 2);
    float y = (y_sq > 0) ? sqrt(y_sq) : 0;
    printf("\n=== TRIANGULATION ===\nMaster: %.2fm | Slave: %.2fm\nCOORDS: X = %.2fm, Y = %.2fm\n=====================\n", d1, d2, x, y);
}

// --- MAIN TASK ---
void app_task(void *pvParameters) {
    printf("\n*** SCANNING FOR 5 SECONDS... ***\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    esp_ble_gap_stop_scanning();
    current_state = STATE_SELECTING;

    printf("\nFOUND DEVICES:\n");
    for (int i = 0; i < device_count; i++) {
        printf("[%d] %-15s | %02x:%02x:%02x:%02x:%02x:%02x | RSSI: %d\n", 
               i, device_list[i].name,
               device_list[i].bda[0], device_list[i].bda[1], device_list[i].bda[2],
               device_list[i].bda[3], device_list[i].bda[4], device_list[i].bda[5],
               device_list[i].rssi);
    }
    printf("Type number of your phone: "); fflush(stdout);

    int selection = -1;
    while (selection < 0 || selection >= device_count) { selection = get_single_digit_input(); }
    
    memcpy(TARGET_MAC, device_list[selection].bda, 6);
    target_selected = true;
    printf("\nSelected: %s\n", device_list[selection].name);
    
    // --- CALIBRATION ---
    printf("\n--- CALIBRATION STEP 1 (0.5m) ---\n");
    printf("Place phone at 0.5m. "); wait_for_enter();
    calib_accum = 0; calib_cnt = 0; current_state = STATE_CALIB_0_5M;
    esp_ble_gap_start_scanning(0); 
    while(current_state == STATE_CALIB_0_5M) vTaskDelay(100/portTICK_PERIOD_MS);

    printf("\n--- CALIBRATION STEP 2 (1.0m) ---\n");
    printf("Place phone at 1.0m. "); wait_for_enter();
    calib_accum = 0; calib_cnt = 0; current_state = STATE_CALIB_1_0M;
    while(current_state == STATE_CALIB_1_0M) vTaskDelay(100/portTICK_PERIOD_MS);

    compute_calibration();
    
    // --- START BROADCASTING TARGET TO SLAVE ---
    printf("\n>> CONFIGURING SLAVE WITH MAC... <<\n");
    broadcast_target_to_slave();
    
    // Wait a bit to ensure advertising starts and slave picks it up
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // --- TRACKING LOOP ---
    current_state = STATE_TRACKING;
    printf("\n*** TRACKING STARTED ***\n");

    while (1) {
        track_rssi_sum = 0; track_packet_count = 0;
        vTaskDelay(3000 / portTICK_PERIOD_MS);

        if (track_packet_count > 0) {
            float avg_rssi = (float)track_rssi_sum / track_packet_count;
            float d1 = calculate_distance(avg_rssi);
            if (slave_read_rssi != 0) {
                float d2 = calculate_distance((float)slave_read_rssi);
                print_position(d1, d2);
                slave_read_rssi = 0; 
            } else {
                ESP_LOGW(TAG, "Master D1: %.2fm | Waiting for Slave...", d1);
            }
        } else {
            ESP_LOGW(TAG, "Master lost signal...");
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
        
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&ble_adv_params);
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
                    calib_accum += rssi; calib_cnt++;
                    if (calib_cnt >= CALIB_SAMPLES) {
                        calib_rssi_0_5 = (float)calib_accum / CALIB_SAMPLES;
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                else if (current_state == STATE_CALIB_1_0M) {
                    calib_accum += rssi; calib_cnt++;
                    if (calib_cnt >= CALIB_SAMPLES) {
                        calib_rssi_1_0 = (float)calib_accum / CALIB_SAMPLES;
                        current_state = STATE_WAIT_MOVE;
                    }
                }
                else if (current_state == STATE_TRACKING) {
                    track_rssi_sum += rssi;
                    track_packet_count++;
                }
            }
            else if (current_state == STATE_TRACKING) {
                uint8_t *adv_name = NULL; uint8_t adv_name_len = 0;
                adv_name = esp_ble_resolve_adv_data(scan->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                if (adv_name != NULL && strncmp((char *)adv_name, SLAVE_NAME, adv_name_len) == 0) {
                    uint8_t manu_len;
                    uint8_t *manu = esp_ble_resolve_adv_data(scan->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manu_len);
                    if (manu != NULL && manu[0] == 0xEE) {
                        slave_read_rssi = (int8_t)manu[1];
                    }
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
    
    esp_ble_gap_set_device_name("MASTER_CMD");
    esp_ble_gap_register_callback(esp_gap_cb);
    esp_ble_gap_set_scan_params(&ble_scan_params);

    xTaskCreate(app_task, "app_task", 4096, NULL, 5, NULL);
}