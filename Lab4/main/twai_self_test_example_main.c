/*
 * Laboratori 4 - Joc del Gat i la Rata amb Bus CAN
 * 
 * Implementació: Opció A (un sol microcontrolador ESP32-C6)
 * 
 * Descripció:
 * - Rep comandes WASD de dos PCs via bus CAN
 * - PC1 (ID 0x100-0x103): Controla el Gat (Jugador 1)
 * - PC2 (ID 0x200-0x203): Controla la Rata (Jugador 2)
 * - Mostra ambdós punts a l'oscil·loscopi via PWM (mode X-Y)
 * - Detecta quan el gat atrapa la rata
 * 
 * Connexions:
 * - GPIO 4: CAN TX
 * - GPIO 5: CAN RX
 * - GPIO 21: PWM X -> RC Filter -> Oscil·loscopi CH1
 * - GPIO 22: PWM Y -> RC Filter -> Oscil·loscopi CH2
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_err.h"
#include "rom/ets_sys.h"

// Inclusions per a la API del driver TWAI
#include "esp_twai.h"
#include "esp_twai_onchip.h"

#define EXAMPLE_TAG "CAN Cat&Mouse Game"

// --- Configuració CAN ---
#define CAN_TX_GPIO         4
#define CAN_RX_GPIO         5
#define CAN_BITRATE         500000

// --- Configuració PWM ---
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_GPIO_X         21
#define LEDC_CHANNEL_X      LEDC_CHANNEL_0
#define LEDC_GPIO_Y         22
#define LEDC_CHANNEL_Y      LEDC_CHANNEL_1
#define LEDC_RESOLUTION     LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY      250000

// --- Paràmetres del joc ---
#define SCREEN_SIZE         255    // Mida de la pantalla (0-255)
#define MOVEMENT_STEP       10     // Píxels de moviment per cada tecla
#define CAPTURE_DISTANCE    15     // Distància per considerar captura
#define REFRESH_RATE_MS     100    // 10 Hz refresh rate (100ms per frame)
#define POINT_HOLD_TIME_MS  40     // Temps que es manté cada punt visible (ms)

// IDs de missatges CAN
#define CAN_ID_P1_W         0x100
#define CAN_ID_P1_A         0x101
#define CAN_ID_P1_S         0x102
#define CAN_ID_P1_D         0x103
#define CAN_ID_P2_W         0x200
#define CAN_ID_P2_A         0x201
#define CAN_ID_P2_S         0x202
#define CAN_ID_P2_D         0x203

// Estructura per la posició dels jugadors
typedef struct {
    uint8_t x;
    uint8_t y;
} Position;

// Variables globals
static twai_node_handle_t node_hdl = NULL;
static Position cat_pos = {50, 128};    // Gat (Jugador 1) inicia a l'esquerra
static Position mouse_pos = {200, 128}; // Rata (Jugador 2) inicia a la dreta
static bool game_over = false;
static QueueHandle_t can_rx_queue;

/**
 * @brief Inicialitza el perifèric LEDC per PWM
 */
static void ledc_init(void)
{
    // Configuració del temporitzador PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_RESOLUTION,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Canal PWM per X
    ledc_channel_config_t ledc_channel_x = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_X,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_GPIO_X,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_x));

    // Canal PWM per Y
    ledc_channel_config_t ledc_channel_y = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_Y,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_GPIO_Y,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_y));
    
    ESP_LOGI(EXAMPLE_TAG, "PWM inicialitzat: X=GPIO%d, Y=GPIO%d", LEDC_GPIO_X, LEDC_GPIO_Y);
}

/**
 * @brief Actualitza la posició d'un jugador segons la tecla premuda
 */
static void update_position(Position *pos, uint32_t can_id)
{
    switch (can_id) {
        case CAN_ID_P1_W:
        case CAN_ID_P2_W: // W - Amunt
            if (pos->x < SCREEN_SIZE - MOVEMENT_STEP) pos->x += MOVEMENT_STEP;
            else pos->x = SCREEN_SIZE;
            break;
        case CAN_ID_P1_S:
        case CAN_ID_P2_S: // S - Avall
            if (pos->x > MOVEMENT_STEP) pos->x -= MOVEMENT_STEP;
            else pos->x = 0;
            break;
        case CAN_ID_P1_A:
        case CAN_ID_P2_A: // A - Esquerra
            if (pos->y > MOVEMENT_STEP) pos->y -= MOVEMENT_STEP;
            else pos->y = 0;
            break;
        case CAN_ID_P1_D:
        case CAN_ID_P2_D: // D - Dreta
            if (pos->y < SCREEN_SIZE - MOVEMENT_STEP) pos->y += MOVEMENT_STEP;
            else pos->y = SCREEN_SIZE;
            break;
    }
}

/**
 * @brief Comprova si el gat ha capturat la rata
 */
static bool check_capture(void)
{
    int dx = (int)cat_pos.x - (int)mouse_pos.x;
    int dy = (int)cat_pos.y - (int)mouse_pos.y;
    float distance = sqrtf(dx*dx + dy*dy);
    return distance < CAPTURE_DISTANCE;
}

/**
 * @brief Callback per rebre missatges CAN (executat en ISR)
 */
static bool twai_rx_callback(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    uint8_t recv_buff[8] = {0};
    twai_frame_t rx_frame = {
        .buffer = recv_buff,
        .buffer_len = sizeof(recv_buff),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
        // Envia l'ID del missatge a la cua per processar-lo en la tasca principal
        uint32_t msg_id = rx_frame.header.id;
        xQueueSendFromISR(can_rx_queue, &msg_id, NULL);
    }

    return false;
}

/**
 * @brief Dibuixa un punt a l'oscil·loscopi
 * 
 * Manté el punt fix durant POINT_HOLD_TIME_MS per permetre que l'oscil·loscopi
 * mostri múltiples mostres al mateix punt (important amb sampling rate baix)
 */
static void draw_point(uint8_t x, uint8_t y)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, x));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, y));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
    
    // Manté el punt fix per permetre múltiples mostres de l'oscil·loscopi
    vTaskDelay(pdMS_TO_TICKS(POINT_HOLD_TIME_MS));
}

/**
 * @brief Tasca de renderització (dibuixa el gat i la rata)
 * 
 * Alterna entre mostrar el gat i la rata. Cada punt es manté
 * durant POINT_HOLD_TIME_MS per ser visible a l'oscil·loscopi.
 */
static void render_task(void *arg)
{
    ESP_LOGI(EXAMPLE_TAG, "Tasca de renderització iniciada");
    
    while (1) {
        if (game_over) {
            // Animació de final del joc: parpelleig al centre
            for (int i = 0; i < 5; i++) {
                draw_point(128, 128);
                draw_point(0, 0);
            }
            ESP_LOGW(EXAMPLE_TAG, "===================================");
            ESP_LOGW(EXAMPLE_TAG, "    EL GAT HA ATRAPAT LA RATA!");
            ESP_LOGW(EXAMPLE_TAG, "         JOC FINALITZAT");
            ESP_LOGW(EXAMPLE_TAG, "===================================");
            game_over = false; // Reset per jugar de nou
            
            // Reinicia posicions
            cat_pos.x = 50;
            cat_pos.y = 128;
            mouse_pos.x = 200;
            mouse_pos.y = 128;
        }
        
        // Alterna entre mostrar el gat i la rata
        // Això crea un efecte de dos punts parpellejants a l'oscil·loscopi
        draw_point(cat_pos.x, cat_pos.y);    // Mostra el gat
        draw_point(mouse_pos.x, mouse_pos.y); // Mostra la rata
        
        // Petit delay addicional entre frames si cal
        vTaskDelay(pdMS_TO_TICKS(REFRESH_RATE_MS - 2*POINT_HOLD_TIME_MS));
    }
}

/**
 * @brief Tasca de processament de missatges CAN
 */
static void can_process_task(void *arg)
{
    uint32_t msg_id;
    
    ESP_LOGI(EXAMPLE_TAG, "Tasca de processament CAN iniciada");
    
    while (1) {
        if (xQueueReceive(can_rx_queue, &msg_id, portMAX_DELAY)) {
            // Determina quin jugador i actualitza posició
            if (msg_id >= CAN_ID_P1_W && msg_id <= CAN_ID_P1_D) {
                // Jugador 1 (Gat)
                update_position(&cat_pos, msg_id);
                ESP_LOGI(EXAMPLE_TAG, "Gat moviment: ID=0x%lx -> Pos(%d,%d)", msg_id, cat_pos.x, cat_pos.y);
            } 
            else if (msg_id >= CAN_ID_P2_W && msg_id <= CAN_ID_P2_D) {
                // Jugador 2 (Rata)
                update_position(&mouse_pos, msg_id);
                ESP_LOGI(EXAMPLE_TAG, "Rata moviment: ID=0x%lx -> Pos(%d,%d)", msg_id, mouse_pos.x, mouse_pos.y);
            }
            
            // Comprova captura
            if (check_capture() && !game_over) {
                game_over = true;
                ESP_LOGW(EXAMPLE_TAG, "CAPTURA! Distància: %.1f", 
                         sqrtf(powf(cat_pos.x - mouse_pos.x, 2) + powf(cat_pos.y - mouse_pos.y, 2)));
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(EXAMPLE_TAG, "========================================");
    ESP_LOGI(EXAMPLE_TAG, "   JOC DEL GAT I LA RATA - CAN + PWM");
    ESP_LOGI(EXAMPLE_TAG, "========================================");
    
    // Crea la cua per missatges CAN
    can_rx_queue = xQueueCreate(10, sizeof(uint32_t));
    if (can_rx_queue == NULL) {
        ESP_LOGE(EXAMPLE_TAG, "Error creant la cua CAN");
        return;
    }
    
    // Inicialitza PWM
    ledc_init();
    
    // Configuració del node TWAI (CAN)
    twai_onchip_node_config_t node_config = {
        .io_cfg = {
            .tx = CAN_TX_GPIO,
            .rx = CAN_RX_GPIO
        },
        .bit_timing = {
            .bitrate = CAN_BITRATE
        },
        .tx_queue_depth = 10,
        .flags = {
             .enable_listen_only = false,
             .enable_loopback = false,
             .enable_self_test = false,
        }
    };
    
    ESP_LOGI(EXAMPLE_TAG, "Creant node CAN...");
    ESP_ERROR_CHECK(twai_new_node_onchip(&node_config, &node_hdl));
    
    // Registra callback de recepció
    twai_event_callbacks_t cbs = {
        .on_rx_done = twai_rx_callback,
    };
    ESP_LOGI(EXAMPLE_TAG, "Registrant callbacks CAN...");
    ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &cbs, NULL));

    // Inicia el node CAN
    ESP_LOGI(EXAMPLE_TAG, "Iniciant node CAN...");
    ESP_ERROR_CHECK(twai_node_enable(node_hdl));

    ESP_LOGI(EXAMPLE_TAG, "Sistema inicialitzat correctament!");
    ESP_LOGI(EXAMPLE_TAG, "  - Gat (P1): Posició inicial (%d,%d)", cat_pos.x, cat_pos.y);
    ESP_LOGI(EXAMPLE_TAG, "  - Rata (P2): Posició inicial (%d,%d)", mouse_pos.x, mouse_pos.y);
    ESP_LOGI(EXAMPLE_TAG, "  - IDs CAN P1: 0x100-0x103 (WASD)");
    ESP_LOGI(EXAMPLE_TAG, "  - IDs CAN P2: 0x200-0x203 (WASD)");
    ESP_LOGI(EXAMPLE_TAG, "Esperant comandes...");
    
    // Crea les tasques
    xTaskCreate(render_task, "render", 4096, NULL, 5, NULL);
    xTaskCreate(can_process_task, "can_proc", 4096, NULL, 10, NULL);
}
