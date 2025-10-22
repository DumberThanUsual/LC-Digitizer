#ifndef COMM_H
#define COMM_H

#include "stm32f3xx_hal.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define MAX_ORDER_SUM 4

xTaskHandle comm_task;

void comm(void *pvParameters);

#endif