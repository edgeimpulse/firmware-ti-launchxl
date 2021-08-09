/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Include ----------------------------------------------------------------- */
#include <stdint.h>
#include <stddef.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>
#include "ti/drivers/Timer.h"
#include "ti/drivers/I2C.h"
#include <stdlib.h>
#include <stdio.h>
/* Driver configuration */
#include "ti_drivers_config.h"
#include "bmi160_config.h"


/* Extern CPP functions ---------------------------------------------------- */
extern int ei_main();
extern void timer_led_callback(void);

/* Forward declerations ---------------------------------------------------- */
void Serial_Out(char *string, int length);
uint8_t Serial_In(void);
static int uart_init(void);
static int init_timer(void);
static int init_inertial_sensor(void);

/* Private variables ------------------------------------------------------- */
static UART2_Handle uart;
static uint64_t timer_count = 0;
I2C_Handle      i2c;
I2C_Params      i2cParams;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    I2C_init();

    /* Configure the LED pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    if(uart_init()) {
        while(1); /* Uart failed */
    }

    if(init_timer()) {
        Serial_Out("Timer init error\r\n", 18);
    }

    if(init_inertial_sensor()) {
        Serial_Out("Inertial sensor init error\r\n", 28);
    }

    ei_main();

    return NULL;
}

/**
 * @brief Write data to the serial output
 *
 * @param string
 * @param length
 */
void Serial_Out(char *string, int length)
{
    size_t bytes_written;
    UART2_write(uart, string, length, &bytes_written);
}

/**
 * @brief Read 1 character from serial
 *
 * @return uint8_t 0 if rx is empty
 */
uint8_t Serial_In(void)
{
    uint8_t character;
    size_t bytes_read;
    UART2_read(uart, &character, 1, &bytes_read);

    return bytes_read == 0 ? 0 : character;
}

/**
 * @brief Get current time in ms
 *
 * @return uint64_t
 */
uint64_t Timer_getMs(void)
{
    return timer_count;
}

/**
 * @brief Write to LED pins, called from EI SDK
 *
 * @param led
 * @param on_off
 */
void Led_control(uint32_t led, bool on_off)
{

    if(led == 0) {
        GPIO_write(CONFIG_GPIO_LED_0, on_off);
        GPIO_write(CONFIG_GPIO_LED_1, !on_off);
    }
    else if(led == 1) {
        GPIO_write(CONFIG_GPIO_LED_0, !on_off);
        GPIO_write(CONFIG_GPIO_LED_1, on_off);
    }
    else {
        GPIO_write(CONFIG_GPIO_LED_0, on_off);
        GPIO_write(CONFIG_GPIO_LED_1, on_off);
    }

}

/**
 * @brief Called by timer interrupt
 *
 * @param myHandle
 * @param status
 */
static void timer_Callback(Timer_Handle myHandle, int_fast16_t status)
{
    static int32_t ms200 = 0;

    /* Led control */
    if(++ms200 > 200) {
        timer_led_callback();
        ms200 = 0;
    }

    timer_count++;
}

/**
 * @brief Init the UART driver (115200 / Non blocking)
 *
 * @return int, 0 on success
 */
static int uart_init(void)
{
    UART2_Params uartParams;

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.readMode = UART2_Mode_NONBLOCKING;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    return uart == NULL ? -1 : 0;
}

/**
 * @brief Setup and start timer
 *
 * @return Error if unequal to 0
 */
static int init_timer(void)
{
    Timer_Handle timer_handle;
    Timer_Params timer_params;

    Timer_init();

    /* Create a timer that fire's the callback every 1000 microseconds */
    Timer_Params_init(&timer_params);
    timer_params.period = 1000;
    timer_params.periodUnits = Timer_PERIOD_US;
    timer_params.timerMode = Timer_CONTINUOUS_CALLBACK;
    timer_params.timerCallback = timer_Callback;

    timer_handle = Timer_open(CONFIG_TIMER_0, &timer_params);

    if(timer_handle == NULL) {
        return -1;
    }

    if(Timer_start(timer_handle) == Timer_STATUS_ERROR) {
        return -2;
    }
    else {
        return 0;
    }
}

/**
 * @brief Setup I2C and init bmi160
 *
 * @return int
 */
static int init_inertial_sensor(void)
{
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.transferCallbackFxn = NULL;
    i2c = I2C_open(CONFIG_I2C_BMI, &i2cParams);
    if (i2c == NULL) {
        return -1;
    }

    return init_bmi160(i2c);
}
