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
#include "board_dsi.h"
#include "key.h"
#include "led_rtos.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define PWM_PULSE_CANT 10

#define ADC_REF_H 3150
#define ADC_REF_L 3050
#define ADC_REF 3100
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void pwm_test_task(void *pvParameters);
//static void pwm_task(void *pvParameters);

/*******************************************************************************
 * Declarations
 ******************************************************************************/
typedef enum{
    FIRST_TRIGGER = 0,
    SECOND_TRIGGER
}acd_state_enum;

typedef enum
{
    PRENDIDO = 0,
    APAGADO
}led_state_enum;

typedef enum
{
    IDLE = 0,
    WORKING
}mma_state_enum;

led_conf_enum ledToggle = {BOARD_LED_ID_ROJO, LED_MSG_TOGGLE, 0, 0};

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

    if (xTaskCreate(pwm_test_task, "pwm_test_task", configMINIMAL_STACK_SIZE * 2,NULL, 2 , NULL) != pdPASS)
    {
        printf("Error creacion task pwm test");
        while (1)
            ;
    }
//    if (xTaskCreate(pwm_task, "pwm_task", configMINIMAL_STACK_SIZE * 3,NULL, 2 , NULL) != pdPASS)
//    {
//        printf("Error creacion task pwm");
//        while (1)
//            ;
//    }

    vTaskStartScheduler();
    for (;;)
        ;
}
static void pwm_task(void *pvParameters){
    int32_t adcLect = 0;
    acd_state_enum adcState = FIRST_TRIGGER;
    bool detectionFlag = false;
    bool adcRefFlag = false;

    while(1){
        if(key_waitForPressEv(BOARD_SW_ID_3, portMAX_DELAY)){

            pwm_updateDutycycle(50);

            TPM_StartTimer(TPM1, kTPM_SystemClock);

            for(uint32_t i = 0; i < 12000; i++);

            ADC_IniciarConv();

            detectionFlag = true;

            while(detectionFlag){
                switch(adcState){
                case FIRST_TRIGGER:
                    adc_getValueBlocking(&adcLect, portMAX_DELAY);

                    if(adcRefFlag && adcLect <= ADC_REF){
                        adcState = SECOND_TRIGGER;
                        adcRefFlag = false;
                    }
                    else if(!adcRefFlag && adcLect > ADC_REF_H){
                        adcRefFlag = true;
                    }
                    break;

                case SECOND_TRIGGER:
                    if(adc_getValueBlocking(&adcLect, portMAX_DELAY) && adcLect < ADC_REF_L){
                         /*PULSO DETECTADO*/
                        detectionFlag = false;

                        adcState = FIRST_TRIGGER;
                    }
                    break;

                default: adcState = FIRST_TRIGGER;
                        break;
                }
            }
        }
    }
}


static void pwm_test_task(void *pvParameters){

    while(1){

        if(key_waitForPressEv(BOARD_SW_ID_1, portMAX_DELAY)){

            led_setConf(&ledToggle);

            pwm_updateDutycycle(50);

            vTaskDelay(10000 / portTICK_PERIOD_MS);
            //for(uint32_t i = 0; i < 12000; i++);  //~10 ciclos 43kHz

            pwm_updateDutycycle(0);

            led_setConf(&ledToggle);
        }
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

