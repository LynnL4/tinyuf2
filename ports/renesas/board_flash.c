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

#include "board_api.h"

#ifndef BUILD_NO_TINYUSB
#include "tusb.h"
#endif

#include "boards.h"

#define FLASH_CACHE_SIZE PROGRAM_BLOCK_SIZE
#define FLASH_CACHE_INVALID_ADDR 0xffffffff

// FLASH
#define NO_CACHE 0xffffffff

#define BLOCK_SIZE (FLASH_CACHE_SIZE)

static uint32_t _flash_block_addr = NO_CACHE;
static uint8_t _flash_cache[BLOCK_SIZE] __attribute__((aligned(4)));

flash_instance_ctrl_t g_flash_ctrl;
const flash_cfg_t g_flash_cfg =
    {
        .data_flash_bgo = false,
        .p_callback = NULL,
        .p_context = NULL,
        .irq = 11,
        .err_irq = 12,
        .err_ipl = (2),
        .ipl = (2),
};

/* Instance structure to use this module. */
const flash_instance_t g_flash = {.p_ctrl = &g_flash_ctrl, .p_cfg = &g_flash_cfg, .p_api = &flash_apis};

//--------------------------------------------------------------------+
// Flash
//--------------------------------------------------------------------+
void board_flash_init(void)
{
    R_FLASH_LP_Open(g_flash.p_ctrl, g_flash.p_cfg);
    R_FLASH_LP_Reset(g_flash.p_ctrl);
}

uint32_t board_flash_size(void)
{
    return BOARD_FLASH_SIZE;
}

void board_flash_read(uint32_t addr, void *buffer, uint32_t len)
{
    memcpy(buffer, (void const *)addr, len);
}

void board_flash_flush(void)
{
    fsp_err_t err = 0;

    if (_flash_block_addr == NO_CACHE)
    {
        return;
    }

    if (memcmp(_flash_cache, (void const *)_flash_block_addr, BLOCK_SIZE) == 0)
    {
        _flash_block_addr = NO_CACHE;
        return;
    }

    __disable_irq();
    err = R_FLASH_LP_Erase(g_flash.p_ctrl, _flash_block_addr, 1);
    __enable_irq();

    if (err != FSP_SUCCESS)
    {
        TUF2_LOG1("Erase failed: status = %d!\r\n", err);
        return;
    }

    __disable_irq();
    err = R_FLASH_LP_Write(g_flash.p_ctrl, (uint32_t)_flash_cache, _flash_block_addr, BLOCK_SIZE);
    __enable_irq();

    if (err != FSP_SUCCESS)
    {
        TUF2_LOG1("Write failed: status = %d!\r\n", err);
        return;
    }

    _flash_block_addr = NO_CACHE;
    return;
}

bool board_flash_write(uint32_t addr, void const *data, uint32_t len)
{
    uint32_t const block_addr = addr & ~(BLOCK_SIZE - 1);

    if (addr + len > BOARD_FLASH_SIZE)
    {
        return false;
    }

    if (block_addr != _flash_block_addr)
    {
        board_flash_flush();
        _flash_block_addr = block_addr;
        memcpy(_flash_cache, (void const *)block_addr, BLOCK_SIZE);
    }

    memcpy(_flash_cache + (addr & (BLOCK_SIZE - 1)), data, len);

    return true;
}

void board_flash_erase_app(void)
{
    memset(_flash_cache, 0xff, BLOCK_SIZE);
    for (int addr = BOARD_FLASH_APP_START; addr < BOARD_FLASH_SIZE; addr += PROGRAM_BLOCK_SIZE)
    {
        if (memcmp((void const *)addr, _flash_cache, PROGRAM_BLOCK_SIZE) == 0)
        {
            continue;
        }
        __disable_irq();
        R_FLASH_LP_Erase(g_flash.p_ctrl, addr, 1);
        __enable_irq();
    }
}

bool board_flash_protect_bootloader(bool protect)
{
    // TODO implement later
    (void)protect;
    return false;
}

#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t *bootloader_bin, uint32_t bootloader_len)
{
    (void)bootloader_bin;
    (void)bootloader_len;
}
#endif
