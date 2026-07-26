#ifndef __APP_H__
#define __APP_H__

#include "config.h"
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t leg_cl_Task_handle;
void leg_cl(void *pvParameters);

extern TaskHandle_t get_data_Task_handle;
void get_data(void *pvParameters);

void Task_Init(void);

#endif /* __APP_H__ */
