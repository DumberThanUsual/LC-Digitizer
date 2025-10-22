#ifndef PROCESS_H
#define PROCESS_H

#include "measure.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define MAX_PEAKS 8
#define SPREAD 2

typedef struct {
    float x;          // Q5.3
    float y;          // Q5.3
    int8_t  order;  
    float   amplitude;  
} process_token;

typedef struct {
    SemaphoreHandle_t mutex;
    process_token token[MAX_PEAKS];
} process_positions;

uint8_t positions_active;
process_positions positions[2];

TaskHandle_t process_task;

void process(void *pvParameters);
void process_init();

#endif