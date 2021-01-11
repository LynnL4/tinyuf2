/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Ha Thach for Adafruit Industries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef BOARD_H_
#define BOARD_H_
#include "stm32h7xx_hal.h"
//--------------------------------------------------------------------+
// LED
//--------------------------------------------------------------------+

#define LED_PORT GPIOG
#define LED_PIN GPIO_PIN_1
#define LED_STATE_ON 1

#define BUTTON_PORT GPIOD
#define BUTTON_PIN GPIO_PIN_9
#define BUTTON_STATE_ACTIVE 0

//--------------------------------------------------------------------+
// Neopixel
//--------------------------------------------------------------------+

//// Number of neopixels
#define NEOPIXEL_NUMBER 0

//#define NEOPIXEL_PORT         GPIOC
//#define NEOPIXEL_PIN          GPIO_PIN_0
//
//// Brightness percentage from 1 to 255
//#define NEOPIXEL_BRIGHTNESS   0x10

//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+

// Flash size of the board
#define BOARD_FLASH_SIZE (1024 * 1024 * 16)

//--------------------------------------------------------------------+
// USB UF2
//--------------------------------------------------------------------+

#define USB_VID           0x2886
#define USB_PID           0x0040
#define USB_MANUFACTURER  "SeeedStudio"
#define USB_PRODUCT       "Seeeduino ST-AI"

#define UF2_PRODUCT_NAME  USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID      "STM32H7-ST-AI-rev1"
#define UF2_VOLUME_LABEL  "STAIBOOT"
#define UF2_INDEX_URL     "https://www.seeedstudio.com"

//--------------------------------------------------------------------+
// UART
//--------------------------------------------------------------------+

// #define UART_DEV              USART2
// #define UART_CLOCK_ENABLE     __HAL_RCC_USART2_CLK_ENABLE
// #define UART_GPIO_PORT        GPIOA
// #define UART_GPIO_AF          GPIO_AF7_USART2
// #define UART_TX_PIN           GPIO_PIN_2
// #define UART_RX_PIN           GPIO_PIN_3

void Error_Handler(void);
void clock_init(void);
void dfu_init(void);

#endif
