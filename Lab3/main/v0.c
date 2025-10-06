#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "rom/ets_sys.h" // For ets_delay_us

/*
 ==============================================================================
 Laboratori 3 - Sortides amb PWM: Dibuix vectorial en oscil·loscopi
 ==============================================================================

 Aquest codi genera un quadrat en un oscil·loscopi configurat en mode X-Y.
 Utilitza dos canals del perifèric LEDC (PWM) de l'ESP32 per controlar
 la deflexió horitzontal (eix X) i vertical (eix Y) del feix d'electrons.

 Connexions:
 - GPIO 21 -> Filtre RC -> Canal X de l'oscil·loscopi (CH1)
 - GPIO 22 -> Filtre RC -> Canal Y de l'oscil·loscopi (CH2)

 Paràmetres clau segons les recomanacions de la pràctica:
 - Freqüència PWM (F_PWM): 250 kHz (recomanat 200-300 kHz)
 - Resolució del cicle de treball: 8 bits (valors de 0 a 255)
 - Freqüència de refresc de punts (Fref): 4 kHz (recomanat 2-5 kHz)
 - Temps total de dibuix: 40 ms (recomanat < 20-40 ms)
*/

// --- Configuració dels pins i canals PWM ---
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_GPIO_X         (21) // Pin per a l'eix X
#define LEDC_CHANNEL_X      LEDC_CHANNEL_0
#define LEDC_GPIO_Y         (22) // Pin per a l'eix Y
#define LEDC_CHANNEL_Y      LEDC_CHANNEL_1

// --- Paràmetres del senyal PWM ---
#define LEDC_RESOLUTION     LEDC_TIMER_8_BIT // Resolució de 8 bits (0-255)
#define LEDC_FREQUENCY      (250000)         // Freqüència de 250 kHz

// --- Paràmetres de la figura (quadrat) ---
// S'utilitza un rang de duty cycle més petit que 0-255 per centrar la figura
#define DUTY_MIN            30  // Valor mínim del cicle de treball
#define DUTY_MAX            225 // Valor màxim del cicle de treball
#define POINTS_PER_SIDE     40  // Nombre de punts per dibuixar cada costat del quadrat

// --- Càlcul del retard per complir el temps de refresc ---
// Temps total = 4 costats * POINTS_PER_SIDE * DELAY_US
// Per a 40ms (40000µs): 40000 / (4 * 40) = 250µs
// Fref = 1 / 250µs = 4 kHz
#define DELAY_US            250 // Retard en microsegons entre cada punt

/**
 * @brief Inicialitza el temporitzador i els dos canals PWM (X i Y).
 */
static void ledc_init(void)
{
    // 1. Configuració del temporitzador PWM (comú per als dos canals)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_RESOLUTION,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. Configuració del canal PWM per a l'eix X
    ledc_channel_config_t ledc_channel_x = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_X,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_GPIO_X,
        .duty           = 0, // Cicle de treball inicial al 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_x));

    // 3. Configuració del canal PWM per a l'eix Y
    ledc_channel_config_t ledc_channel_y = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_Y,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_GPIO_Y,
        .duty           = 0, // Cicle de treball inicial al 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_y));
}

void app_main(void)
{
    // Inicialitza el perifèric LEDC
    ledc_init();

    uint32_t x_duty, y_duty;

    while (1) {
        // Bucle infinit per redibuixar el quadrat contínuament

        // Costat 1: Inferior (esquerra -> dreta)
        // Y es manté a DUTY_MIN, X augmenta
        y_duty = DUTY_MIN;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, y_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
        for (int i = 0; i <= POINTS_PER_SIDE; i++) {
            x_duty = DUTY_MIN + (i * (DUTY_MAX - DUTY_MIN)) / POINTS_PER_SIDE;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, x_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));
            ets_delay_us(DELAY_US);
        }

        // Costat 2: Dret (baix -> dalt)
        // X es manté a DUTY_MAX, Y augmenta
        x_duty = DUTY_MAX;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, x_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));
        for (int i = 0; i <= POINTS_PER_SIDE; i++) {
            y_duty = DUTY_MIN + (i * (DUTY_MAX - DUTY_MIN)) / POINTS_PER_SIDE;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, y_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
            ets_delay_us(DELAY_US);
        }

        // Costat 3: Superior (dreta -> esquerra)
        // Y es manté a DUTY_MAX, X disminueix
        y_duty = DUTY_MAX;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, y_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
        for (int i = 0; i <= POINTS_PER_SIDE; i++) {
            x_duty = DUTY_MAX - (i * (DUTY_MAX - DUTY_MIN)) / POINTS_PER_SIDE;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, x_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));
            ets_delay_us(DELAY_US);
        }

        // Costat 4: Esquerre (dalt -> baix)
        // X es manté a DUTY_MIN, Y disminueix
        x_duty = DUTY_MIN;
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_X, x_duty));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_X));
        for (int i = 0; i <= POINTS_PER_SIDE; i++) {
            y_duty = DUTY_MAX - (i * (DUTY_MAX - DUTY_MIN)) / POINTS_PER_SIDE;
            ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_Y, y_duty));
            ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_Y));
            ets_delay_us(DELAY_US);
        }
    }
}