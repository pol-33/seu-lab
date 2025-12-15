#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#define TAG "BLE_DISTANCE"

// --- CONFIGURACIÓ ---
// Posa aquí el nom que emet el teu mòbil (usa nRF Connect -> Advertiser)
#define TARGET_DEVICE_NAME "MIO_SEU" 

// Constant ambiental (N) i potència a 1 metre (MeasuredPower)
// Ajusta aquests valors fent proves a 1 metre real.
#define MEASURED_POWER     -60 
#define ENV_FACTOR         2.5f 

// Paràmetres d'escaneig
static esp_ble_scan_params_t ble_scan_params = {
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval          = 0x50, // 50ms
    .scan_window            = 0x30, // 30ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE // Volem rebre múltiples lectures per veure com canvia
};

/**
 * @brief Calcula la distància en metres a partir de l'RSSI
 */
float calculate_distance(int rssi) {
    // Fórmula: Distance = 10 ^ ((MeasuredPower - RSSI) / (10 * N))
    return pow(10, ((float)(MEASURED_POWER - rssi) / (10 * ENV_FACTOR)));
}

/**
 * @brief Funció auxiliar per buscar el nom del dispositiu dins el paquet raw d'advertisement
 */
static bool adv_name_find(uint8_t *adv_data, uint8_t adv_data_len, const char *target_name) {
    uint8_t *ptr = adv_data;
    uint8_t len;
    uint8_t type;
    
    while (ptr < adv_data + adv_data_len) {
        len = *ptr++; // Longitud del camp
        if (len == 0) break;
        
        type = *ptr++; // Tipus de dada (0x09 = Complete Local Name, etc.)
        
        // Comprovem si és un nom (0x08 = Shortened Name, 0x09 = Complete Name)
        if (type == 0x08 || type == 0x09) {
            // Comprovem si coincideix amb el nostre objectiu
            if (len - 1 == strlen(target_name) && memcmp(ptr, target_name, len - 1) == 0) {
                return true;
            }
        }
        ptr += (len - 1);
    }
    return false;
}

/**
 * @brief Callback d'esdeveniments GAP (Generic Access Profile)
 */
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {
        // Un cop configurats els paràmetres, iniciem l'escaneig infinit
        uint32_t duration = 0; // 0 = infinit
        esp_ble_gap_start_scanning(duration);
        break;
    }
    
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Error iniciant escaneig");
        } else {
            ESP_LOGI(TAG, "Escaneig iniciat...");
        }
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;
        
        switch (scan_result->scan_rst.search_evt) {
            case ESP_GAP_SEARCH_INQ_RES_EVT:
                // Hem trobat un dispositiu. Mirem si és el nostre.
                // scan_result->scan_rst.ble_adv conté les dades raw
                if (adv_name_find(scan_result->scan_rst.ble_adv, 
                                  scan_result->scan_rst.adv_data_len, 
                                  TARGET_DEVICE_NAME)) {
                    
                    int rssi = scan_result->scan_rst.rssi;
                    float dist = calculate_distance(rssi);
                    
                    ESP_LOGI(TAG, "TARGET TROBAT! [%s] RSSI: %d dBm | Distància est: %.2f m", 
                             TARGET_DEVICE_NAME, rssi, dist);
                }
                break;
            default:
                break;
        }
        break;
    }

    default:
        break;
    }
}

void app_main(void) {
    // 1. Inicialitzar NVS (Necessari per WiFi/BT)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Alliberar memòria de Bluetooth Clàssic (l'ESP32-C6 només té BLE, però és bona pràctica general)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // 3. Inicialitzar controlador BT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_init fallit: %s", esp_err_to_name(ret));
        return;
    }

    // 4. Habilitar controlador en mode BLE
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "esp_bt_controller_enable fallit: %s", esp_err_to_name(ret));
        return;
    }

    // 5. Inicialitzar Bluedroid (Stack software)
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_init fallit: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "esp_bluedroid_enable fallit: %s", esp_err_to_name(ret));
        return;
    }

    // 6. Registrar callback i configurar escaneig
    ESP_LOGI(TAG, "Inicialització BLE completada. Configurant escàner...");
    esp_ble_gap_register_callback(esp_gap_cb);
    
    // Això dispararà l'esdeveniment ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT
    // que al seu torn iniciarà l'escaneig.
    esp_ble_gap_set_scan_params(&ble_scan_params);
}