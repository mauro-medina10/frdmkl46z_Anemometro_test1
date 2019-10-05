/*
 * The Clear BSD License
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "board_dsi.h"
#include "stdio.h"
#include "pin_mux.h"
#include "key.h"
#include "led_rtos.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Blinkeo led */
typedef struct
{
    board_ledId_enum idLed;
    board_swId_enum idSW;
    uint32_t semiPeriodo;
}paramBlinked_t;

/* Task priorities. */
#define blinky_task_PRIORITY ( 1 )
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void blinky_task(void *pvParameters);

led_conf_enum redLed = {BOARD_LED_ID_ROJO ,LED_MSG_HEARTBEAT, 600 , 3};
led_conf_enum geenLed = {BOARD_LED_ID_VERDE,LED_MSG_PULSE_TRAIN , 500 , 6};


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    board_init();

    key_init();

    if (xTaskCreate(blinky_task, "red_blinky_task", configMINIMAL_STACK_SIZE,(void * const) &redLed, blinky_task_PRIORITY, NULL) != pdPASS)
    {
        printf("Error creacion task 1");
        while (1)
            ;
    }
    if (xTaskCreate(blinky_task, "green_blinky_task", configMINIMAL_STACK_SIZE, (void * const) &geenLed, blinky_task_PRIORITY, NULL) != pdPASS)
    {
        printf("Error creacion task 2");
        while (1)
            ;
    }

    vTaskStartScheduler();
    for (;;)
        ;
}
/* Cuando se pulsa un sw empieza a parpadear el led o se apaga*/
static void blinky_task(void *pvParameters)
{
    led_conf_enum* paramBlinked;
    uint8_t ledSetted = 0;
    paramBlinked = (led_conf_enum*) pvParameters;

    for (;;)
    {
        if(ledSetted == 0)
        {
            led_setConf(paramBlinked);
            ledSetted = 1;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void vApplicationTickHook(void)
{
    key_periodicTask1ms();
    led_periodicTask1ms();
}

extern void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    printf("Overflow stack: %s",pcTaskName);
    while(1);
}
