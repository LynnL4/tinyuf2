/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Ha Thach (tinyusb.org) for Adafruit Industries
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

#ifndef BOARDS_H_
#define BOARDS_H_

#ifdef __cplusplus
extern "C"
{
#endif
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstrict-prototypes"
#pragma GCC diagnostic ignored "-Wundef"

// extra push due to https://github.com/renesas/fsp/pull/278
#pragma GCC diagnostic push
#endif

#include "bsp_api.h"
#include "r_ioport.h"
#include "r_ioport_api.h"
#include "r_sci_uart.h"
#include "r_flash_lp.h"
#include "renesas.h"

#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif

#include "board.h"
#define flash_instance_ctrl_t flash_lp_instance_ctrl_t
#define flash_apis g_flash_on_flash_lp
#define PROGRAM_BLOCK_SIZE (2 * 1024)
#define SKETCH_FLASH_OFFSET (32 * 1024)

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_KEY (0xA500U)
#define BSP_PRV_PRCR_PRC1_UNLOCK ((BSP_PRV_PRCR_KEY) | 0x2U)
#define BSP_PRV_PRCR_LOCK ((BSP_PRV_PRCR_KEY) | 0x0U)

#ifndef BOARD_FLASH_APP_START
#define BOARD_FLASH_APP_START (SKETCH_FLASH_OFFSET)
#endif

#define TINYUF2_DBL_TAP_DFU 1

#ifdef __cplusplus
}
#endif

#endif /* BOARDS_H_ */
