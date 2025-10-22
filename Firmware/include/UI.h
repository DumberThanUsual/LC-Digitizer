#ifndef UI_H
#define UI_H

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

TaskHandle_t UI_task;

void UI(void *pvParameters);

#endif