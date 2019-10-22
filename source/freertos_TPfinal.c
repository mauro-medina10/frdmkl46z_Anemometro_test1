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
#include "semphr.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "board_dsi.h"
#include "stdio.h"
#include "pin_mux.h"
#include "adc.h"
#include "led_rtos.h"
#include "mma8451_rtos.h"
#include "uart_rtos.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define LUZ_SAMPLE_TIME 1
#define LUZ_THR 3000

#define ACC_THR 20
#define ACC_CNT 5

#define LED_ON "LED:ON"
#define LED_OFF "LED:OFF"

#define MMAmesagge "[%6u] ACC:X=%3d;Y=%3d;Z=%3d \r\n"
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void luz_task(void *pvParameters);
static void mma8451_task(void *pvParameters);

led_conf_enum redLedOn = {BOARD_LED_ID_ROJO ,LED_MSG_ON, 0 , 0};
led_conf_enum redLedOff = {BOARD_LED_ID_ROJO,LED_MSG_OFF , 0 , 0};

/*******************************************************************************
 * Declarations
 ******************************************************************************/
SemaphoreHandle_t xSemaphoreLuz;

typedef enum
{
    PRENDIDO = 0,
    APAGADO
}led_state_enum;

typedef enum
{
    ESPERANDO = 0,
    WORKING
}mma_state_enum;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    /* Init board hardware. */
    board_init();


    if (xTaskCreate(luz_task, "red_led_task", configMINIMAL_STACK_SIZE * 2,NULL, 1 , NULL) != pdPASS)
    {
        printf("Error creacion task red led");
        while (1)
            ;
    }
    if (xTaskCreate(mma8451_task, "mma8451_task", configMINIMAL_STACK_SIZE * 2, NULL, 1 , NULL) != pdPASS)
    {
        printf("Error creacion MMA8451 task");
        while (1)
            ;
    }

    vTaskStartScheduler();
    for (;;)
        ;
}

static void mma8451_task(void *pvParameters)
{
    volatile TickType_t tickTime = 0;
    mma_state_enum mma8451 = ESPERANDO;
    int16_t accXaux = 0, accYaux = 0,accZaux = 0;
    int16_t accX = 0, accY = 0,accZ = 0;
    uint8_t message[33];

    mma8451_init();

    while(1)
    {
        accXaux = mma8451_getAcX();
        accYaux = mma8451_getAcY();
        accZaux = mma8451_getAcZ();

        switch(mma8451){
        case ESPERANDO:
            if(abs(accXaux - accX) > ACC_THR || abs(accYaux - accY) > ACC_THR || abs(accYaux - accY) > ACC_THR)
            {
                accX = accXaux;
                accY = accYaux;
                accZ = accZaux;
                tickTime = xTaskGetTickCount();
                mma8451 = WORKING;
                sprintf((char*)message,MMAmesagge, tickTime/portTICK_PERIOD_MS,accX,accY,accZ);
                uart_rtos_envDatos(message,33,portMAX_DELAY);
            }
            break;
        case WORKING:
            for(int i = 0; i < (ACC_CNT-1); i++)
            {
                accX = mma8451_getAcX();
                accY = mma8451_getAcY();
                accZ = mma8451_getAcZ();
                tickTime = xTaskGetTickCount();
                sprintf((char*)message,MMAmesagge, tickTime/portTICK_PERIOD_MS,accX,accY,accZ);
                uart_rtos_envDatos(message,33,portMAX_DELAY);
            }
            mma8451 = ESPERANDO;
            break;
        default: mma8451 = ESPERANDO;
        break;
        }
    }
}

static void luz_task(void *pvParameters)
{
    volatile TickType_t tickTime = 0;

    int32_t samples = 20;
    int32_t prom = 0;
    uint8_t message[17] = "                 ";
    led_state_enum ledRojo = APAGADO;

    adc_init(LUZ_SAMPLE_TIME);

    while(1)
    {
        prom = adc_getProm_nonbloq(samples);
        tickTime = xTaskGetTickCount();
        switch(ledRojo)
        {
        case APAGADO:
            if(prom > LUZ_THR)
            {
                led_setConf(&redLedOn);
                ledRojo = PRENDIDO;
                sprintf((char*)message,"[%6u]%s \n\r", tickTime/portTICK_PERIOD_MS,LED_ON );
                uart_rtos_envDatos(message,17,portMAX_DELAY);
                vTaskDelay(50 / portTICK_PERIOD_MS);
            }
            break;
        case PRENDIDO:
            if(prom < LUZ_THR)
            {
                led_setConf(&redLedOff);
                ledRojo = APAGADO;
                sprintf((char*)message,"[%6u]%s \n\r", tickTime/portTICK_PERIOD_MS,LED_OFF );
                uart_rtos_envDatos(message,17,portMAX_DELAY);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            break;
        default:
            led_setConf(&redLedOff);
            ledRojo = APAGADO;
            break;
        }
    }
}

void vApplicationTickHook(void)
{
    led_periodicTask1ms();
}

extern void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    printf("Overflow stack: %s",pcTaskName);
    while(1);
}

