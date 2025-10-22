#include "comm.h"
#include "process.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

#include "stm32f3xx_hal.h"

char USBuffer[50];

void comm(void *pvParameters)
{
    while (1) {

        process_positions *pos = &positions[positions_active];
        xSemaphoreTake(pos->mutex, portMAX_DELAY);

        for (uint8_t i = 0; i < MAX_PEAKS; i ++) {
            process_token *token = &(pos->token[i]);

            if (token->x == infinityf() || token->y == infinityf())
                continue;
            
            sprintf(USBuffer, "%2.1f %2.1f %i,", token->x - 15.5, -(token->y - 15.5), token->order);
            CDC_Transmit_FS((uint8_t *) USBuffer, strlen(USBuffer));
            HAL_Delay(1);
        }
        sprintf(USBuffer, "\n\r");
        CDC_Transmit_FS((uint8_t *) USBuffer, strlen(USBuffer));
        HAL_Delay(1);

        xSemaphoreGive(pos->mutex);

        vTaskDelay(15/portTICK_PERIOD_MS);

    }
}