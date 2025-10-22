/*
 * measure.c
 *
 *  Created on: Jun 9, 2025
 *      Author: yuki
 */

#include "measure.h"
#include "process.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "arm_math.h"
#include "math.h"

#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "ssd1306_conf.h"
#include "stdio.h"

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_cdc_if.h"

#define CLK_TO_ADC_RATIO    28  
#define IC_PSC              8   // Prescaler on input capture channels
#define CLK_KHZ             72000
#define FS_KHZ              (float)(CLK_KHZ / CLK_TO_ADC_RATIO)

/*
#define F_C_0 750
#define F_C_1 693
#define F_C_2 653
#define F_C_3 591
#define F_C_4 533
#define F_C_5 482
#define F_C_6 422
#define F_C_7 370
#define F_C_8 320

#define F_0_0 800
#define F_0_1 750
#define F_1_2 675
#define F_2_3 620
#define F_3_4 560
#define F_4_5 505
#define F_5_6 452
#define F_6_7 395
#define F_7_8 345
#define F_8_8 280
*/

#define F_C_0 610
#define F_C_1 563
#define F_C_2 516
#define F_C_3 487
#define F_C_4 452
#define F_C_5 411
#define F_C_6 367
#define F_C_7 333
#define F_C_8 300

#define F_0_0 650
#define F_0_1 585
#define F_1_2 539
#define F_2_3 501
#define F_3_4 469
#define F_4_5 431
#define F_5_6 389
#define F_6_7 350
#define F_7_8 315
#define F_8_8 280


#define SIZE_X 32 //32
#define SIZE_Y 8 //8

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern COMP_HandleTypeDef hcomp3;
extern COMP_HandleTypeDef hcomp4;
extern COMP_HandleTypeDef hcomp5;
extern COMP_HandleTypeDef hcomp6;

extern TaskHandle_t measure_task;

measure_cell grid[SIZE_X * SIZE_Y * 4];

uint8_t drive_state = 0;
measure_group measurements;

void measure(void *pvParameters)
{
    uint16_t freq_khz;
    uint32_t diff;
    int8_t ident;

    float amplitude;

    int16_t *ADC;
    uint16_t *IC;

    int16_t cell_index;
    uint32_t notify_bits;

    xTaskNotifyGive(measure_task);

    while (1) {
        for (uint8_t cell_y = 0; cell_y < 8; cell_y ++) {
            row_to_mux(cell_y);

            for (uint16_t j = 0; j < 200; j ++) {
                __ASM("nop");
            }

            for (uint8_t cell_x = 0; cell_x < 32; cell_x ++){

                measure_collect(&measurements, cell_x, cell_y);

                while ((notify_bits & EV_ALL) != EV_ALL) {
                    xTaskNotifyWait
                    (
                        0,
                        0,
                        &notify_bits,
                        1000/portTICK_PERIOD_MS
                    );
                }
                xTaskNotifyStateClear(measure_task);

                HAL_TIM_IC_Stop_DMA(&htim16, TIM_CHANNEL_1);
                HAL_TIM_IC_Stop_DMA(&htim4,  TIM_CHANNEL_2);
                HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1);
                HAL_TIM_IC_Stop_DMA(&htim17, TIM_CHANNEL_1);

                for (uint8_t channel = 0; channel < 4; channel ++) {
                    amplitude = 0;
                    
                    cell_index = cell_x + (cell_y * SIZE_X) + (channel * 256);
                    ADC = measurements.samples[channel].ADC_buffer;
                    IC  = measurements.samples[channel].IC_buffer; 
    
                    if (IC[1] == IC[0]) {
                        freq_khz = 0;
                    }else if (IC[1] > IC[0]) {
                        diff = (IC[1] - IC[0]);
                        freq_khz = (CLK_KHZ * IC_PSC / diff);
                    } else {
                        diff = (0x10000 + IC[1] - IC[0]);
                        freq_khz = (CLK_KHZ * IC_PSC / diff);
                    }

                    amplitude = measure_amplitude(ADC, freq_khz);

                    if ((freq_khz < F_8_8) || (freq_khz > F_0_0)) {
                        ident = 127;
                    } else if ((freq_khz >= F_8_8) & (freq_khz < F_7_8)) {
                        ident = -4;
                    } else if ((freq_khz >= F_7_8) & (freq_khz < F_6_7)) {
                        ident = -3;
                    } else if ((freq_khz >= F_6_7) & (freq_khz < F_5_6)) {
                        ident = -2;
                    } else if ((freq_khz >= F_5_6) & (freq_khz < F_4_5)) {
                        ident = -1;
                    } else if ((freq_khz >= F_4_5) & (freq_khz < F_3_4)) {
                        ident = 4;
                    } else if ((freq_khz >= F_3_4) & (freq_khz < F_2_3)) {
                        ident = 3;
                    } else if ((freq_khz >= F_2_3) & (freq_khz < F_1_2)) {
                        ident = 2;
                    } else if ((freq_khz >= F_1_2) & (freq_khz < F_0_1)) {
                        ident = 1;
                    } else if ((freq_khz >= F_0_1) & (freq_khz < F_0_0)) {
                        ident = 0;
                    } else {
                        ident = 127;
                    }

                    //float scaled = (amplitude * 255.0f) / 1500000.0f;
                    //if (scaled < 0.0f)      scaled = 0.0f;
                    //else if (scaled > 255.0f) scaled = 255.0f;
                    //grid[cell_index].amplitude = (uint8_t)(scaled);

                    grid[cell_index].amplitude = amplitude;
                    grid[cell_index].order = ident;
                    //grid[cell_index].amplitude = amplitude;
                }
                
            }
        }

        
        /*
        float max = 0;
        volatile uint16_t max_index = 0;
        for (uint16_t i = 0; i < 1024; i ++) {
            if ((grid[i].amplitude > max) && (grid[i].order != 127)) {
                max_index = i;
                max = grid[i].amplitude;
            }
        }

        ssd1306_Fill(White);
        sprintf(buffer, "Pos: %u %u", max_index % 32, max_index / 32);
        ssd1306_SetCursor(6, 20);
        ssd1306_WriteString(buffer, Font_6x8, Black);
        sprintf(buffer, "ID: %*u  %*g", 4, grid[max_index].frequency, 6, grid[max_index].amplitude);
        ssd1306_SetCursor(6, 32);
        ssd1306_WriteString(buffer, Font_6x8, Black);
        ssd1306_UpdateScreen();
        */

        xTaskNotifyGive(process_task);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }
    vTaskDelete(NULL);
}

float measure_amplitude(int16_t *data, uint16_t freq_khz)
{
    //HAL_GPIO_WritePin(TP906_GPIO_Port, TP906_Pin, SET);

    //Goertzel Algorithm
    float w = 2 * PI * ((float)freq_khz / FS_KHZ);
    float c_r = arm_cos_f32(w);
    float c_i = arm_sin_f32(w);
    float s_prev_1 = 0;
    float s_prev_2 = 0;
    float s;

    float X_r;
    float X_i;

    float res;

    for (uint16_t i = 0; i < N_SAMPLES; i ++) {
        s = (float)data[i] + (2 * c_r * s_prev_1) - s_prev_2;
        s_prev_2 = s_prev_1;
        s_prev_1 = s; 
    }

    X_r = (s_prev_1 * c_r) - s_prev_2;
    X_i = s_prev_1 * c_i;

    res = sqrtf(X_r*X_r + X_i*X_i) / N_SAMPLES;

    HAL_GPIO_WritePin(TP906_GPIO_Port, TP906_Pin, RESET);
    return res;
}

void measure_collect(measure_group *measurements, uint8_t cell_x, uint8_t cell_y)
{
    uint32_t frame = 1U << cell_x;

    taskENTER_CRITICAL();
    HAL_SPI_Transmit(&hspi1, (uint8_t *) &frame, 4, 1000);
    __ASM("nop");
    HAL_GPIO_WritePin(TX_RCLK_GPIO_Port, TX_RCLK_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TX_RCLK_GPIO_Port, TX_RCLK_Pin, GPIO_PIN_RESET);

    drive_state = 0;
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim8, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(TX_SLEEP_GPIO_Port, TX_SLEEP_Pin, GPIO_PIN_SET);

/*
    for (uint8_t j = 0; j < 60; j ++) {
        __ASM("nop");
    }
*/

    //HAL_GPIO_WritePin(TX_SLEEP_GPIO_Port, TX_SLEEP_Pin, GPIO_PIN_RESET);

    /*
    for (uint16_t j = 0; j < 200; j ++) {
        __ASM("nop");
    }
        */

    /*
    RX1 - ADC4 - COMP6 - TIM16
    RX2 - ADC3 - COMP4 - TIM4
    RX3 - ADC2 - COMP5 - TIM17
    RX4 - ADC1 - COMP3 - TIM15

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) measurements->samples[3].ADC_buffer, N_SAMPLES);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *) measurements->samples[2].ADC_buffer, N_SAMPLES);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *) measurements->samples[1].ADC_buffer, N_SAMPLES);
    HAL_ADC_Start_DMA(&hadc4, (uint32_t *) measurements->samples[0].ADC_buffer, N_SAMPLES);

    HAL_TIM_IC_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*) measurements->samples[0].IC_buffer, 2);
    HAL_TIM_IC_Start_DMA(&htim4 , TIM_CHANNEL_2, (uint32_t*) measurements->samples[1].IC_buffer, 2);
    HAL_TIM_IC_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t*) measurements->samples[2].IC_buffer, 2);
    HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t*) measurements->samples[3].IC_buffer, 2);
    */

    taskEXIT_CRITICAL();
}

void TIM8_delay()
{
    if (drive_state == 0) {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        HAL_GPIO_WritePin(TX_SLEEP_GPIO_Port, TX_SLEEP_Pin, GPIO_PIN_RESET);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        HAL_TIM_OC_Stop(&htim8, TIM_CHANNEL_1);
    } else if (drive_state == 1) {
        UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
        /*
        RX1 - ADC4 - COMP6 - TIM16
        RX2 - ADC3 - COMP4 - TIM4
        RX3 - ADC2 - COMP5 - TIM17
        RX4 - ADC1 - COMP3 - TIM15
        */

        HAL_ADC_Start_DMA(&hadc1, (uint32_t *) measurements.samples[3].ADC_buffer, N_SAMPLES);
        HAL_ADC_Start_DMA(&hadc2, (uint32_t *) measurements.samples[2].ADC_buffer, N_SAMPLES);
        HAL_ADC_Start_DMA(&hadc3, (uint32_t *) measurements.samples[1].ADC_buffer, N_SAMPLES);
        HAL_ADC_Start_DMA(&hadc4, (uint32_t *) measurements.samples[0].ADC_buffer, N_SAMPLES);

        HAL_TIM_IC_Start_DMA(&htim16, TIM_CHANNEL_1, (uint32_t*) measurements.samples[0].IC_buffer, 2);
        HAL_TIM_IC_Start_DMA(&htim4 , TIM_CHANNEL_2, (uint32_t*) measurements.samples[1].IC_buffer, 2);
        HAL_TIM_IC_Start_DMA(&htim17, TIM_CHANNEL_1, (uint32_t*) measurements.samples[2].IC_buffer, 2);
        HAL_TIM_IC_Start_DMA(&htim15, TIM_CHANNEL_1, (uint32_t*) measurements.samples[3].IC_buffer, 2);
        taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
        HAL_TIM_OC_Stop(&htim8, TIM_CHANNEL_2);
    }
    drive_state ++;
}


void row_to_mux(uint8_t row)
{
    switch (row)
    {
    case 0:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_RESET);
        break;
    case 1:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_RESET);
        break;
    case 2:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_RESET);
        break;
    case 3:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_RESET);
        break;
    case 4:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_SET);
        break;
    case 5:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_SET);
        break;
    case 6:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_SET);
        break;
    case 7:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, GPIO_PIN_RESET);
        break;
    }
    __ASM("nop");
    __ASM("nop");
    __ASM("nop");
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    BaseType_t woken = pdFALSE;

    if (hadc->Instance == ADC1) {
        HAL_ADC_Stop_DMA(&hadc1);
        xTaskNotifyFromISR(measure_task, EV_ADC0, eSetBits, &woken);
    } else if (hadc->Instance == ADC2) {
        HAL_ADC_Stop_DMA(&hadc2);
        xTaskNotifyFromISR(measure_task, EV_ADC1, eSetBits, &woken);
    } else if (hadc->Instance == ADC3) {
        HAL_ADC_Stop_DMA(&hadc3);
        xTaskNotifyFromISR(measure_task, EV_ADC2, eSetBits, &woken);
    } else if (hadc->Instance == ADC4) {
        HAL_ADC_Stop_DMA(&hadc4);
        xTaskNotifyFromISR(measure_task, EV_ADC3, eSetBits, &woken);
    }
    portYIELD_FROM_ISR(woken);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    BaseType_t woken = pdFALSE;
    if (htim->Instance == TIM16) {
        HAL_TIM_IC_Stop_DMA(&htim16, TIM_CHANNEL_1);
    } else if (htim->Instance == TIM4) {
        HAL_TIM_IC_Stop_DMA(&htim4, TIM_CHANNEL_2);
    } else if (htim->Instance == TIM15) {
        HAL_TIM_IC_Stop_DMA(&htim15, TIM_CHANNEL_1);
    } else if (htim->Instance == TIM17) {
        HAL_TIM_IC_Stop_DMA(&htim17, TIM_CHANNEL_1);
    }
    portYIELD_FROM_ISR(woken);
}
