/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Ha Thach (tinyusb.org)
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
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#ifdef __cplusplus
extern "C"
{
#endif

#define LED1 BSP_IO_PORT_00_PIN_11 // D11
#define LED_STATE_ON 0

#define SW1 BSP_IO_PORT_00_PIN_01 // D1
#define BUTTON_STATE_ACTIVE 0

    static const ioport_pin_cfg_t board_pin_cfg[] = {
        {.pin = LED1, .pin_cfg = IOPORT_CFG_PORT_DIRECTION_OUTPUT},
        {.pin = SW1, .pin_cfg = IOPORT_CFG_PORT_DIRECTION_INPUT},
        // USB FS D+, D-, VBus
        {.pin = BSP_IO_PORT_04_PIN_07, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS},
        {.pin = BSP_IO_PORT_09_PIN_14, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS},
        {.pin = BSP_IO_PORT_09_PIN_15, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_USB_FS},
        // UART
        {.pin = BSP_IO_PORT_03_PIN_02, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI0_2_4_6_8},
        {.pin = BSP_IO_PORT_03_PIN_01, .pin_cfg = IOPORT_CFG_PERIPHERAL_PIN | IOPORT_PERIPHERAL_SCI0_2_4_6_8},
    };

#define USB_VID 0x2886
#define USB_PID 0x0049
#define USB_MANUFACTURER "Seeed"
#define USB_PRODUCT "XIAO R4"

#define UF2_PRODUCT_NAME USB_MANUFACTURER " " USB_PRODUCT
#define UF2_BOARD_ID "XIAO R4"
#define UF2_VOLUME_LABEL "XIAO_R4BOOT"
#define UF2_INDEX_URL "https://www.seeedstudio.com/"

#define TINYUF2_DBL_TAP_REG (*((volatile uint32_t *)&R_SYSTEM->VBTBKR[0]))
#define TINYUF2_DBL_TAP_REG_SIZE 32

#define BOARD_FLASH_SIZE (256 * 1024)

#ifdef __cplusplus
}
#endif

#endif
