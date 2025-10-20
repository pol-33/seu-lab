#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "driver/twai.h"
#include "driver/gpio.h"

// ATENCIÓ: Modifica aquests pins segons el teu muntatge!
#define TX_GPIO_NUM GPIO_NUM_21 // Canvia pel teu pin de TX
#define RX_GPIO_NUM GPIO_NUM_22 // Canvia pel teu pin de RX

void app_main(void)
{
    // 1. Configuració general del driver TWAI
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

    // 2. Configuració del timing per a 500 Kbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();

    // 3. Configuració del filtre per acceptar tots els missatges
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // 4. Instal·lar i iniciar el driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        printf("Driver TWAI instal·lat correctament\n");
    } else {
        printf("Error en instal·lar el driver TWAI\n");
        return;
    }

    if (twai_start() == ESP_OK) {
        printf("Driver TWAI iniciat\n");
    } else {
        printf("Error en iniciar el driver TWAI\n");
        return;
    }

    printf("Preparat per fer eco de missatges CAN...\n");

    // 5. Bucle per rebre i fer l'eco
    while (1) {
        twai_message_t message;
        // Espera fins a rebre un missatge
        if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
            printf("Missatge rebut! ID: 0x%lx, DLC: %d\n", message.identifier, message.data_length_code);
            
            // Reenvia el mateix missatge (eco)
            if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
                printf("Eco del missatge enviat correctament.\n");
            } else {
                printf("Error en enviar l'eco.\n");
            }
        } else {
            // Si no es rep res en 10 segons, el bucle continua
        }
    }
}