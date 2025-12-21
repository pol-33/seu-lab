/*
 * LAB 6 - MASTER NODE (Triangulator + Centered Map)
 * 1. Selects Device.
 * 2. Asks user for Slave Distance.
 * 3. Auto-Calibrates.
 * 4. Draws Centered Map (3s refresh).
 */

 // Aquesta versió del master fa visualització gràfica.
// S'assumeix un sol master i un sol esclau. La distància entre ells és configurada per l'usuari.
// Atenció: Per a poder interacturar per consola, connecteu el cable a l'ESP32 a través de l'USB-C on hi diu UART, no a l'altre.


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

static const char* TAG = "MASTER_VISUAL";

// --- CONFIGURATION ---
#define SLAVE_NAME   "SLAVE_1"

// --- GLOBALS ---
uint8_t TARGET_MAC[6] = {0}; 
bool target_selected = false;

// Geometry (Set by User)
float slave_dist_m = 2.0f; // Default

// Calibration Defaults
float measured_power_at_1m = -74.0f; 
float path_loss_exponent   = 3.0f;

// Tracking Accumulators
volatile int32_t track_rssi_sum = 0;
volatile int16_t track_packet_count = 0;
volatile int8_t slave_read_rssi = 0;

// --- DATA STRUCTURES ---
#define MAX_DEVICES 50 
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

// --- VISUAL MAP DRAWER (CENTERED) ---
void draw_visual_map(float x, float y, float d1, float d2) {
    // Clear screen
    printf("\033[2J\033[H"); 

    printf("================= REAL TIME TRACKING =================\n");
    printf("Master Dist: %5.2fm  |  Slave Dist: %5.2fm\n", d1, d2);
    printf("Position:    X=%5.2fm |  Y=%5.2fm\n", x, y);
    printf("------------------------------------------------------\n");

    int grid_w = 60; // Wider grid for better centering
    int grid_h = 20; 
    
    // --- CENTERING LOGIC ---
    // We add a virtual margin of 1.0 meter to the left of Master and right of Slave
    float margin_m = 1.0f; 
    float total_width_m = slave_dist_m + (margin_m * 2);
    
    // Master is at 0.0. In our logical map, it starts at -margin.
    // logical_x ranges from [-margin] to [slave_dist + margin]
    
    // Convert Real coords to Grid coords
    // Col = ( (X - min_x) / total_width ) * grid_w
    // min_x is -margin_m. So X - (-margin) = X + margin
    int p_col = (int)( (x + margin_m) / total_width_m * grid_w );
    
    // Y Axis (0 to 4m depth)
    float range_y = 4.0;
    int p_row = grid_h - (int)((y / range_y) * grid_h);

    // Calculate fixed positions for M and S
    int m_col = (int)( (0.0f + margin_m) / total_width_m * grid_w );
    int s_col = (int)( (slave_dist_m + margin_m) / total_width_m * grid_w );
    int base_row = grid_h; 

    // Bounds check
    if (p_col < 0) p_col = 0; 
    if (p_col >= grid_w) p_col = grid_w - 1;
    if (p_row < 0) p_row = 0; 
    if (p_row >= grid_h) p_row = grid_h - 1;

    // Draw
    for (int r = 0; r <= grid_h; r++) {
        printf("  |");
        for (int c = 0; c < grid_w; c++) {
            
            if (r == p_row && c == p_col) {
                printf("O"); // Phone
            } 
            else if (r == base_row && c == m_col) {
                printf("M"); // Master
            }
            else if (r == base_row && c == s_col) {
                printf("S"); // Slave
            }
            else if (r == base_row && c > m_col && c < s_col) {
                printf("_"); // Line connecting them
            }
            else if (r == base_row) {
                printf(" "); // Floor outside zone
            }
            else {
                printf("."); // Air
            }
        }
        printf("|\n");
    }
    printf("  ------------------------------------------------------------\n");
    printf("         [M](0,0) <--- %.2fm ---> [S](%.2f,0)\n", slave_dist_m, slave_dist_m);
}

// --- INPUT HELPERS ---

int get_multidigit_input() {
    char buffer[16];
    int idx = 0;
    while (getchar() != EOF) { vTaskDelay(10/portTICK_PERIOD_MS); } // Flush

    while (1) {
        int c = getchar();
        if (c != EOF) {
            if (c == '\n' || c == '\r') {
                if (idx > 0) {
                    buffer[idx] = '\0';
                    printf("\n");
                    return atoi(buffer);
                }
            } else if (c >= '0' && c <= '9' && idx < 10) {
                buffer[idx++] = (char)c;
                printf("%c", c); fflush(stdout);
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

// --- LOGIC ---

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

void broadcast_target_to_slave() {
    static uint8_t manu_payload[7];
    manu_payload[0] = 0xAA; 
    memcpy(&manu_payload[1], TARGET_MAC, 6);
    master_adv_data.manufacturer_len = 7;
    master_adv_data.p_manufacturer_data = manu_payload;
    esp_ble_gap_config_adv_data(&master_adv_data);
}

void calculate_and_draw_position(float d1, float d2) {
    float D = slave_dist_m; 
    float x = 0, y = 0;

    if (d1 + d2 <= D) {
        x = d1; y = 0; // On line
    } else {
        x = (pow(d1, 2) - pow(d2, 2) + pow(D, 2)) / (2 * D);
        float y_sq = pow(d1, 2) - pow(x, 2);
        y = (y_sq > 0) ? sqrt(y_sq) : 0;
    }
    draw_visual_map(x, y, d1, d2);
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
    while (selection < 0 || selection >= device_count) { 
        selection = get_multidigit_input(); 
    }
    
    memcpy(TARGET_MAC, device_list[selection].bda, 6);
    target_selected = true;
    printf("\nSelected: %s\n", device_list[selection].name);
    
    // --- GEOMETRY SETUP ---
    current_state = STATE_CONFIG_GEO;
    printf("\n------------------------------------------------\n");
    printf(">> Enter distance between Master and Slave (in cm): ");
    fflush(stdout);
    
    int dist_cm = get_multidigit_input();
    if (dist_cm <= 0) dist_cm = 200; 
    slave_dist_m = (float)dist_cm / 100.0f;
    printf(">> Set Slave Distance to %.2f meters.\n", slave_dist_m);
    printf("------------------------------------------------\n");

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
    
    printf("\n>> CONFIGURING SLAVE... <<\n");
    broadcast_target_to_slave();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    current_state = STATE_TRACKING;

    while (1) {
        track_rssi_sum = 0; track_packet_count = 0;
        
        // Wait 3 seconds (Refresh Rate)
        vTaskDelay(3000 / portTICK_PERIOD_MS); 

        if (track_packet_count > 0) {
            float avg_rssi = (float)track_rssi_sum / track_packet_count;
            float d1 = calculate_distance(avg_rssi);
            
            if (slave_read_rssi != 0) {
                float d2 = calculate_distance((float)slave_read_rssi);
                calculate_and_draw_position(d1, d2);
                slave_read_rssi = 0; 
            }
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