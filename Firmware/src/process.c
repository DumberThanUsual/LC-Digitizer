#include "process.h"
#include "measure.h"
#include "comm.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "stdio.h"

char buffer[50];

extern measure_cell grid[32*32];

void process(void *pvParameters)
{

    uint8_t peak_index;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        process_positions *pos = &positions[positions_active];
        positions_active ^= 1;
        xSemaphoreTake(pos->mutex, portMAX_DELAY);

        peak_index = 0;
        for (uint8_t i = 0; i < MAX_PEAKS; i ++) {
            process_token token = {
                .amplitude = 0,
                .order = 0,
                .x = 0,
                .y = 0,
            };
            pos->token[i] = token;
        }

        for (int8_t x = 0; x < 32; x ++) {
            for (int8_t y = 0; y < 32; y ++) {

                measure_cell cell = grid[x + (32 * y)];

                if ((cell.order == 127) || (cell.amplitude < 600))
                    continue;

                uint8_t is_max = 1;
                for (int8_t i = (x - SPREAD); i <= (x + SPREAD); i ++) {
                    for (int8_t j = (y - SPREAD); j <= (y + SPREAD); j ++) {

                        if ((i == x) && (j == y))
                            continue;
                        
                        if ((i < 0) || (i >= 32) || (j < 0) || (j >= 32))
                            continue;

                        if (grid[i  + (32 * j)].amplitude > cell.amplitude) {
                            is_max = 0;
                            break;
                        } else if (grid[i  + (32 * j)].amplitude == cell.amplitude) {
                            ///TODO: Scary
                        }
                    }

                    if (!is_max)
                        break;

                }

                if (is_max && (peak_index < MAX_PEAKS)) {
                    process_token token = {
                        .amplitude = cell.amplitude,
                        .order = cell.order,
                        .x = x,
                        .y = y,
                    };
                    pos->token[peak_index] = token;
                    peak_index ++;
                }
                
            }
        }

        for (uint8_t i = 0; i < MAX_PEAKS; i ++) {
            process_token *token = &(pos->token[i]);

            float xp = 0;
            float xm = 0;
            float yp = 0;
            float ym = 0;

            if (token->x < 32)
                xp = grid[(uint16_t)token->x + 32 * (uint16_t)token->y  + 1].amplitude;

            if (token->x >= 1)
                xm = grid[(uint16_t)token->x + 32 * (uint16_t)token->y  - 1].amplitude;

            if (token->y < 32)
                yp = grid[(uint16_t)token->x + 32 * (uint16_t)token->y  + 32].amplitude;

            if (token->y >= 1)
                ym = grid[(uint16_t)token->x + 32 * (uint16_t)token->y  - 32].amplitude;

            float dx = (xp - xm)/(token->amplitude);
            float dy = (yp - ym)/(token->amplitude);

            token->x += dx;
            token->y += dy;
        }
        
        /*
        ssd1306_Fill(White);
        sprintf(buffer, "Peaks: %u  %2.1f %2.1f", peak_index, peaks[0].x, peaks[0].y);
        ssd1306_SetCursor(6, 20);
        ssd1306_WriteString(buffer, Font_6x8, Black);

        sprintf(buffer, "   %5.f  %2.1f %2.1f", peaks[1].amplitude, peaks[1].x, peaks[1].y);
        ssd1306_SetCursor(6, 32);
        ssd1306_WriteString(buffer, Font_6x8, Black);

        ssd1306_UpdateScreen();
        */

        //vTaskDelay(1000/portTICK_PERIOD_MS);
        
        xSemaphoreGive(pos->mutex);
        xTaskNotifyGive(measure_task);
    }
    
    vTaskDelete(NULL);
}

void process_init()
{
    for (uint8_t i = 0; i < 2; i ++) {
        positions[i].mutex = xSemaphoreCreateMutex();
    }
}