#ifndef MEASURE_H
#define MEASURE_H

#include "stm32f3xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "arm_math.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern ADC_HandleTypeDef hadc4;
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc2;
extern DMA_HandleTypeDef hdma_adc3;
extern DMA_HandleTypeDef hdma_adc4;

extern SPI_HandleTypeDef hspi1;

extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define N_SAMPLES 64

#define EV_ADC0   (1UL<<0)
#define EV_IC0    (1UL<<1)
#define EV_ADC1   (1UL<<2)
#define EV_IC1    (1UL<<3)
#define EV_ADC2   (1UL<<4)
#define EV_IC2    (1UL<<5)
#define EV_ADC3   (1UL<<6)
#define EV_IC3    (1UL<<7)

#define EV_ALL    (EV_ADC0|EV_ADC1|EV_ADC2|EV_ADC3)

typedef struct {
    int16_t ADC_buffer[N_SAMPLES];
    uint16_t IC_buffer[2];
} measure_sample;

typedef struct {
    uint8_t status;
	measure_sample samples[4];
} measure_group;

typedef struct {
    uint8_t order;
    float amplitude;
} measure_cell;

TaskHandle_t measure_task;

void measure(void *pvParameters);

void measure_collect(measure_group *buffer, uint8_t cell_x, uint8_t cell_y);
void row_to_mux(uint8_t row);
float measure_amplitude(int16_t *data, uint16_t freq_khz);

void TIM8_update();
void TIM8_delay();

#endif
