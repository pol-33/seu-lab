// Aquesta versió del master fa visualització gràfica.
// S'assumeix un sol master i 2 esclaus. La distància entre ells és configurada per l'usuari.
// Atenció: Per a poder interacturar per consola, connecteu el cable a l'ESP32 a través de l'USB-C on hi diu UART, no a l'altre.

/*
 * LAB 6 - TRIANGULATION MASTER (1 Master + 2 Slaves)
 * 1. Selects Device.
 * 2. Setup Geometry (Coords of S1 and S2).
 * 3. Auto-Calibrates.
 * 4. Configures both Slaves.
 * 5. Visualizes 3-Point Trilateration.
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
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

static const char* TAG = "TRILATERATION";

// --- CONFIGURATION ---
// Refresh Rate (Seconds)
#define REFRESH_RATE_MS  3000 

// Names to look for
#define SLAVE_1_NAME "SLAVE_1"
#define SLAVE_2_NAME "SLAVE_2"

// --- GLOBALS ---
uint8_t TARGET_MAC[6] = {0}; 
bool target_selected = false;

// Geometry (Set by User at runtime)
// Master is always at (0,0)
float s1_x = 2.0f; // Slave 1 X
float s1_y = 0.0f; // Slave 1 Y (Usually 0)
float s2_x = 1.0f; // Slave 2 X
float s2_y = 2.0f; // Slave 2 Y

// Calibration Defaults
float measured_power_at_1m = -74.0f; 
float path_loss_exponent   = 3.0f;

// Tracking Data
volatile int32_t track_rssi_sum = 0;
volatile int16_t track_packet_count = 0;
volatile int8_t rssi_s1 = 0; // Last seen RSSI from Slave 1
volatile int8_t rssi_s2 = 0; // Last seen RSSI from Slave 2

// --- DATA STRUCTURES ---
#define MAX_DEVICES 50 
typedef struct {
    uint8_t bda[6];
    char name[32];
    int rssi;
} found_device_t;

found_device_t device_list[MAX_DEVICES];
int device_count = 0;

typedef enum {
    STATE_SCANNING_LIST,
    STATE_SELECTING, 
    STATE_CONFIG_GEO,
    STATE_CALIB_0_5M,
    STATE_WAIT_MOVE,
    STATE_CALIB_1_0M,
    STATE_TRACKING
} app_state_t;

volatile app_state_t current_state = STATE_SCANNING_LIST;

// Calibration Vars
int32_t calib_accum = 0; int16_t calib_cnt = 0;
float calib_rssi_0_5 = 0; float calib_rssi_1_0 = 0;
#define CALIB_SAMPLES 30

// --- BLE PARAMS ---
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 0x50,
    .scan_window = 0x30,
    .scan_duplicate = BLE_SCAN_DUPLICATE_DISABLE 
};

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

// --- VISUAL MAP DRAWER ---
void draw_visual_map(float obj_x, float obj_y, float d_m, float d_s1, float d_s2) {
    printf("\033[2J\033[H"); // Clear Screen

    printf("================= 3-POINT TRILATERATION =================\n");
    printf("DISTANCES: Master: %.2fm | S1: %.2fm | S2: %.2fm\n", d_m, d_s1, d_s2);
    printf("LOCATION:  X = %.2fm | Y = %.2fm\n", obj_x, obj_y);
    printf("---------------------------------------------------------\n");

    int grid_w = 60; 
    int grid_h = 25;
    
    // Auto-scale map based on slave positions
    float max_x = (s1_x > s2_x) ? s1_x : s2_x;
    float max_y = (s1_y > s2_y) ? s1_y : s2_y;
    if (obj_x > max_x) max_x = obj_x;
    if (obj_y > max_y) max_y = obj_y;

    float map_width_m  = max_x + 1.5f; // Add margin
    float map_height_m = max_y + 1.0f; // Add margin
    
    int m_col = 5; // Master fixed at left
    int m_row = grid_h - 2; // Master fixed at bottom

    // Helper: Coord to Col/Row
    auto int to_col(float x) { return m_col + (int)((x / map_width_m) * grid_w); }
    auto int to_row(float y) { return m_row - (int)((y / map_height_m) * grid_h); }

    int s1_c = to_col(s1_x); int s1_r = to_row(s1_y);
    int s2_c = to_col(s2_x); int s2_r = to_row(s2_y);
    int obj_c = to_col(obj_x); int obj_r = to_row(obj_y);

    for (int r = 0; r < grid_h; r++) {
        printf("  |");
        for (int c = 0; c < grid_w; c++) {
            if (r == obj_r && c == obj_c) printf("O"); // Phone
            else if (r == m_row && c == m_col) printf("M"); // Master
            else if (r == s1_r && c == s1_c) printf("1"); // Slave 1
            else if (r == s2_r && c == s2_c) printf("2"); // Slave 2
            else if (r == m_row) printf("_"); // Floor
            else printf(" "); 
        }
        printf("|\n");
    }
    printf("  ------------------------------------------------------------\n");
    printf("  M=(0,0)  S1=(%.1f, %.1f)  S2=(%.1f, %.1f)\n", s1_x, s1_y, s2_x, s2_y);
}

// --- INPUT HELPERS ---
int get_multidigit_int() {
    char buffer[16]; int idx = 0;
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); }
    while (1) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                if (idx > 0) { buffer[idx] = '\0'; printf("\n"); return atoi(buffer); }
            } else if (c >= '0' && c <= '9' && idx < 10) {
                buffer[idx++] = (char)c; printf("%c", c); fflush(stdout);
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void wait_for_enter() {
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); }
    printf(" >> Press ENTER to continue... \n"); fflush(stdout);
    while (1) {
        int c = getchar();
        if (c != EOF && c != 0xFF) break;
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

// --- CORE LOGIC ---

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
    ESP_LOGI(TAG, "Calibration updated.");
}

void broadcast_target_to_slaves() {
    static uint8_t manu_payload[7];
    manu_payload[0] = 0xAA; 
    memcpy(&manu_payload[1], TARGET_MAC, 6);
    master_adv_data.manufacturer_len = 7;
    master_adv_data.p_manufacturer_data = manu_payload;
    esp_ble_gap_config_adv_data(&master_adv_data);
}

// --- TRIANGULATION MATH ---
void calculate_position(float d_m, float d_s1, float d_s2) {
    float x = 0, y = 0;

    // 1. Calculate Intersection of Master (0,0) and Slave 1 (s1_x, s1_y)
    // Simplified assumption: Slave 1 is on X-Axis (y=0) for primary X calculation
    // x = (d_m^2 - d_s1^2 + s1_x^2) / (2 * s1_x)
    
    // Safety check for division by zero
    if (s1_x == 0) s1_x = 0.1f;

    // Estimate X based on Master and Slave 1
    float x_est = (pow(d_m, 2) - pow(d_s1, 2) + pow(s1_x, 2)) / (2 * s1_x);
    
    // Estimate Y based on Master Radius: y = sqrt(d_m^2 - x^2)
    float y_sq = pow(d_m, 2) - pow(x_est, 2);
    float y_est = (y_sq > 0) ? sqrt(y_sq) : 0;

    // Slave 2 helps us determine if Y is positive or negative, or refine accuracy
    // Simple approach: Use S2 to validate. If S2 is "North" (y > 0), and d_s2 matches y_est, keep it.
    // For this lab, we just assume +Y (In front of sensors).
    
    // Optional: Average with S2 calculation if needed, but M+S1 is usually robust enough for X.
    
    x = x_est;
    y = y_est;

    draw_visual_map(x, y, d_m, d_s1, d_s2);
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
    printf("Type number of your phone + ENTER: "); fflush(stdout);

    int selection = -1;
    while (selection < 0 || selection >= device_count) { selection = get_multidigit_int(); }
    
    memcpy(TARGET_MAC, device_list[selection].bda, 6);
    target_selected = true;
    printf("\nSelected: %s\n", device_list[selection].name);
    
    // --- GEOMETRY SETUP ---
    current_state = STATE_CONFIG_GEO;
    printf("\n--- SLAVE SETUP ---\n");
    printf("Recommended: Place SLAVE 1 to the RIGHT of Master.\n");
    printf("Recommended: Place SLAVE 2 in FRONT (Triangle).\n\n");
    
    printf(">> Enter SLAVE 1 Distance (cm) from Master: ");
    int cm = get_multidigit_int();
    s1_x = (float)cm / 100.0f; s1_y = 0.0f;

    printf(">> Enter SLAVE 2 Distance X (cm) (Right of Master): ");
    cm = get_multidigit_int();
    s2_x = (float)cm / 100.0f;

    printf(">> Enter SLAVE 2 Distance Y (cm) (Forward of Master): ");
    cm = get_multidigit_int();
    s2_y = (float)cm / 100.0f;

    printf("\nGeometry Saved: M(0,0), S1(%.2f,0), S2(%.2f,%.2f)\n", s1_x, s2_x, s2_y);

    // --- CALIBRATION ---
    printf("\n--- CALIBRATION STEP 1 (0.5m) ---\n");
    printf("Place phone 0.5m from MASTER. "); wait_for_enter();
    calib_accum = 0; calib_cnt = 0; current_state = STATE_CALIB_0_5M;
    esp_ble_gap_start_scanning(0); 
    while(current_state == STATE_CALIB_0_5M) vTaskDelay(100/portTICK_PERIOD_MS);

    printf("\n--- CALIBRATION STEP 2 (1.0m) ---\n");
    printf("Place phone 1.0m from MASTER. "); wait_for_enter();
    calib_accum = 0; calib_cnt = 0; current_state = STATE_CALIB_1_0M;
    while(current_state == STATE_CALIB_1_0M) vTaskDelay(100/portTICK_PERIOD_MS);

    compute_calibration();
    
    printf("\n>> BROADCASTING TARGET TO SLAVES... <<\n");
    broadcast_target_to_slaves();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    current_state = STATE_TRACKING;

    while (1) {
        track_rssi_sum = 0; track_packet_count = 0;
        
        vTaskDelay(REFRESH_RATE_MS / portTICK_PERIOD_MS); 

        if (track_packet_count > 0) {
            float avg_rssi = (float)track_rssi_sum / track_packet_count;
            float d_m = calculate_distance(avg_rssi);
            
            // Check Slaves
            float d_s1 = 0, d_s2 = 0;
            if (rssi_s1 != 0) d_s1 = calculate_distance((float)rssi_s1);
            if (rssi_s2 != 0) d_s2 = calculate_distance((float)rssi_s2);

            // Need at least Master + 1 Slave to compute, preferably all 3
            if (d_s1 > 0) {
                calculate_position(d_m, d_s1, d_s2);
                
                // Reset flags to check liveness next cycle
                rssi_s1 = 0; 
                rssi_s2 = 0;
            } else {
                printf("Waiting for Slaves... (M:%.2fm)\n", d_m);
            }
        } else {
            printf("Master lost signal...\n");
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
            // LISTENING FOR SLAVES
            else if (current_state == STATE_TRACKING) {
                uint8_t *adv_name = NULL; uint8_t adv_name_len = 0;
                adv_name = esp_ble_resolve_adv_data(scan->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
                
                if (adv_name != NULL) {
                    uint8_t manu_len;
                    uint8_t *manu = esp_ble_resolve_adv_data(scan->scan_rst.ble_adv, ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, &manu_len);
                    
                    if (manu != NULL && manu[0] == 0xEE) {
                        // Check Name to distinguish slaves
                        if (strncmp((char *)adv_name, SLAVE_1_NAME, adv_name_len) == 0) {
                            rssi_s1 = (int8_t)manu[1];
                        }
                        else if (strncmp((char *)adv_name, SLAVE_2_NAME, adv_name_len) == 0) {
                            rssi_s2 = (int8_t)manu[1];
                        }
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