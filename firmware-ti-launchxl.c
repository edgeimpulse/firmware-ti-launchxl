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
#include "ti/drivers/I2S.h"
#include <stdlib.h>
#include <stdio.h>
/* Driver configuration */
#include "ti_drivers_config.h"
#include "bmi160_config.h"
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)


/* Extern CPP functions ---------------------------------------------------- */
extern int ei_main();
extern void ei_microphone_init();
extern void timer_led_callback(void);

/* Forward declerations ---------------------------------------------------- */
void Serial_Out(char *string, int length);
uint8_t Serial_In(void);
static int uart_init(void);
static void uart_disable(uint32_t ui32Base);
static void uart_enable(uint32_t ui32Base);
static void uart_set_baudrate(uint32_t ui32Base, uint32_t ui32UARTClk, uint32_t ui32Baud);
static int init_timer(void);
static int init_inertial_sensor(void);
static int init_i2c(void);

/* Private variables ------------------------------------------------------- */
static UART2_Handle uart;
static uint64_t timer_count = 0;
I2C_Handle      i2cHandle;
I2C_Params      i2cParams;

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    I2C_init();
    I2S_init();

    /* Configure the LED pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_1, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    if(uart_init()) {
        while(1); /* Uart failed */
    }

    if(init_timer()) {
        Serial_Out("Timer init error\r\n", 18);
    }

    if(init_i2c()) {
        Serial_Out("I2C setup error\r\n", 28);
    }

    if(init_inertial_sensor()) {
        Serial_Out("Inertial sensor init error\r\n", 28);
    }

    ei_microphone_init();

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
 * @brief Switch baudrate for fast data transfer
 *
 * @param lowHigh false low
 */
void Serial_set_baudrate(uint32_t baud)
{
    const uint32_t uart_base = 0x40001000;

    uart_set_baudrate(uart_base, SysCtrlClockGet(), baud);
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
 * @brief Stop UART
 *
 * @param ui32Base UART0 base address
 */
static void uart_disable(uint32_t ui32Base)
{
    int timeout = 10000;
    // Check the arguments.
    ASSERT(UARTBaseValid(ui32Base));

    // Wait for end of TX.
    while(HWREG(ui32Base + UART_O_FR) & UART_FR_BUSY && timeout--)
    {
    }

    // Disable the FIFO.
    HWREG(ui32Base + UART_O_LCRH) &= ~(UART_LCRH_FEN);

    // Disable the UART.
    HWREG(ui32Base + UART_O_CTL) &= ~(UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/**
 * @brief Enable UART
 *
 * @param ui32Base UART0 base address
 */
static void uart_enable(uint32_t ui32Base)
{
    // Check the arguments.
    ASSERT(UARTBaseValid(ui32Base));

    // Disable the FIFO.
    HWREG(ui32Base + UART_O_LCRH) |= (UART_LCRH_FEN);

    // Disable the UART.
    HWREG(ui32Base + UART_O_CTL) |= (UART_CTL_UARTEN | UART_CTL_TXE | UART_CTL_RXE);
}

/**
 * @brief Calculate and set baudrate
 *
 * @param ui32Base UART0 base address
 * @param ui32UARTClk Processor control clock
 * @param ui32Baud Baudrate to set
 */
static void uart_set_baudrate(uint32_t ui32Base, uint32_t ui32UARTClk, uint32_t ui32Baud)
{
    uint32_t ui32Div;

    // Check the arguments.
    ASSERT(UARTBaseValid(ui32Base));
    ASSERT(ui32Baud != 0);

    // Stop the UART.
    uart_disable(ui32Base);

    // Compute the fractional baud rate divider.
    ui32Div = (((ui32UARTClk * 8) / ui32Baud) + 1) / 2;

    // Set the baud rate.
    HWREG(ui32Base + UART_O_IBRD) = ui32Div / 64;
    HWREG(ui32Base + UART_O_FBRD) = ui32Div % 64;

    uart_enable(ui32Base);
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
 * @brief Setup I2C
 *
 * @return int
 */
static int init_i2c(void)
{
    // Initialize the I2C pins
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2cParams.transferMode = I2C_MODE_BLOCKING;
    i2cParams.transferCallbackFxn = NULL;
    i2cHandle = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2cHandle == NULL) {
        return -1;
    }
    return 0;
}

/**
 * @brief Setup I2C and init bmi160
 *
 * @return int
 */
static int init_inertial_sensor(void)
{
    return init_bmi160(i2cHandle);
}
