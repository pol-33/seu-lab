/*
 * Codi corregit per a l'eco del bus CAN (TWAI) a l'ESP32-C6.
 *
 * Correcció principal: S'elimina la funció inexistent `twai_node_receive`
 * i s'implementa el sistema de recepció correcte basat en callbacks,
 * tal com exigeix l'API oficial de l'ESP-IDF v5.5.1.
 */

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

// Inclusions per a la nova API del driver TWAI
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define EXAMPLE_TAG "TWAI Echo C6 Corrected"

// Variable global per al handle del node, accessible des del callback
static twai_node_handle_t node_hdl = NULL;

/**
 * @brief Funció Callback que s'executa quan es rep un missatge CAN.
 *
 * Aquesta funció s'executa en el context d'una interrupció (ISR).
 * NO ES POT utilitzar ESP_LOGI ni altres funcions de logging aquí!
 * Aquí és on es gestiona la recepció i l'eco del missatge.
 */
static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    esp_err_t ret;
    uint8_t recv_buff[8];  // Buffer per rebre les dades
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    // Llegeix el missatge rebut des del buffer intern del driver
    ret = twai_node_receive_from_isr(handle, &rx_frame);
    if (ret == ESP_OK) {
        // Envia el mateix missatge de tornada (eco)
        // No fem logging aquí perquè estem en context ISR
        twai_node_transmit(handle, &rx_frame, 0);
    }

    return false; // No necessitem despertar cap tasca de major prioritat
}

void app_main(void)
{
    // 1. Configuració del node TWAI
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = 4, // Pin TX per a l'ESP32-C6
            .rx = 5  // Pin RX per a l'ESP32-C6
        },
        .bit_timing = {
            .bitrate = 500000 // 500 kbps, com demana la pràctica
        },
        .tx_queue_depth = 10,
        .flags = {
             .enable_listen_only = false,  // Normal mode - can receive and transmit
             .enable_loopback = false,
             .enable_self_test = false,    // Disabled - communicating with external device
        }
    };
    
    ESP_LOGI(EXAMPLE_TAG, "Creant node TWAI...");
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    
    // 2. Registre de la funció callback per a la recepció
    twai_event_callbacks_t cbs = {
        .on_rx_done = twai_rx_callback,
    };
    ESP_LOGI(EXAMPLE_TAG, "Registrant callbacks...");
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &cbs, NULL));

    // 3. Inici del node TWAI
    ESP_LOGI(EXAMPLE_TAG, "Iniciant node TWAI...");
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));

    ESP_LOGI(EXAMPLE_TAG, "El node està actiu i esperant missatges...");

    // La lògica principal ja no necessita fer res, ja que tot es gestiona
    // per interrupcions a través del callback. Deixem la tasca 'main'
    // en un bucle infinit per evitar que el programa finalitzi.
    while(1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
