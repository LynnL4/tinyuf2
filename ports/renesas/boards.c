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

#include <stdio.h>

#include "board_api.h"
#include "board.h"

#ifndef BUILD_NO_TINYUSB
#include "tusb.h"
#endif

/* Key code for writing PRCR register. */
#define BSP_PRV_PRCR_KEY (0xA500U)

extern void sci_uart_txi_isr(void);
extern void sci_uart_tei_isr(void);
extern void sci_uart_rxi_isr(void);
extern void sci_uart_eri_isr(void);

static const ioport_cfg_t family_pin_cfg = {
    .number_of_pins = sizeof(board_pin_cfg) / sizeof(ioport_pin_cfg_t),
    .p_pin_cfg_data = board_pin_cfg,
};
static ioport_instance_ctrl_t port_ctrl;
static sci_uart_instance_ctrl_t uart_ctrl;
static volatile bool tx_done = false;

//--------------------------------------------------------------------+
// Vector Data
//--------------------------------------------------------------------+
BSP_DONT_REMOVE BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS)
    const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] = {
        [0] = usbfs_interrupt_handler, /* USBFS INT (USBFS interrupt) */
        [1] = usbfs_resume_handler,    /* USBFS RESUME (USBFS resume interrupt) */

#ifndef BSP_MCU_GROUP_RA2A1
        [2] = usbfs_d0fifo_handler, /* USBFS FIFO 0 (DMA transfer request 0) */
        [3] = usbfs_d1fifo_handler, /* USBFS FIFO 1 (DMA transfer request 1) */
#endif

#ifdef BOARD_HAS_USB_HIGHSPEED
        [4] = usbhs_interrupt_handler, /* USBHS INT (USBHS interrupt) */
        [5] = usbhs_d0fifo_handler,    /* USBHS FIFO 0 (DMA transfer request 0) */
        [6] = usbhs_d1fifo_handler,    /* USBHS FIFO 1 (DMA transfer request 1) */
#endif
#if TUF2_LOG
        [7] = sci_uart_txi_isr,  /* SCI UART TXI (SCI UART transmit interrupt) */
        [8] = sci_uart_tei_isr,  /* SCI UART TXE (SCI UART transmit buffer empty interrupt) */
        [9] = sci_uart_rxi_isr,  /* SCI UART RXI (SCI UART receive interrupt) */
        [10] = sci_uart_eri_isr, /* SCI UART RXI (SCI UART receive error interrupt) */
#endif
};

const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] = {
    [0] = BSP_PRV_IELS_ENUM(EVENT_USBFS_INT),    /* USBFS INT (USBFS interrupt) */
    [1] = BSP_PRV_IELS_ENUM(EVENT_USBFS_RESUME), /* USBFS RESUME (USBFS resume interrupt) */

#ifndef BSP_MCU_GROUP_RA2A1
    [2] = BSP_PRV_IELS_ENUM(EVENT_USBFS_FIFO_0), /* USBFS FIFO 0 (DMA transfer request 0) */
    [3] = BSP_PRV_IELS_ENUM(EVENT_USBFS_FIFO_1), /* USBFS FIFO 1 (DMA transfer request 1) */
#endif

#ifdef BOARD_HAS_USB_HIGHSPEED
    [4] = BSP_PRV_IELS_ENUM(EVENT_USBHS_USB_INT_RESUME), /* USBHS USB INT RESUME (USBHS interrupt) */
    [5] = BSP_PRV_IELS_ENUM(EVENT_USBHS_FIFO_0),         /* USBHS FIFO 0 (DMA transfer request 0) */
    [6] = BSP_PRV_IELS_ENUM(EVENT_USBHS_FIFO_1),         /* USBHS FIFO 1 (DMA transfer request 1) */
#endif
#if TUF2_LOG
    [7] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TXI),  /* SCI UART TXI (SCI UART transmit interrupt) */
    [8] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TEI),  /* SCI UART TXE (SCI UART transmit buffer empty interrupt) */
    [9] = BSP_PRV_IELS_ENUM(EVENT_SCI2_RXI),  /* SCI UART RXI (SCI UART receive interrupt) */
    [10] = BSP_PRV_IELS_ENUM(EVENT_SCI2_ERI), /* SCI UART RXI (SCI UART receive error interrupt) */
#endif
};

void uart_callback(uart_callback_args_t *p_args)
{
    switch (p_args->event)
    {
    case UART_EVENT_TX_DATA_EMPTY:
        tx_done = true;
        break;
    default:
        break;
    }
    return;
}

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
void board_init(void)
{
    // Enable global interrupts in CPSR register since board with bootloader such as Arduino Uno R4
    // can transfer CPU control with CPSR.I bit set to 0 (disable IRQ)
    __enable_irq();

    /* Configure pins. */
    R_IOPORT_Open(&port_ctrl, &family_pin_cfg);

#ifdef TRACE_ETM
    // TRCKCR is protected by PRCR bit0 register
    R_SYSTEM->PRCR = (uint16_t)(BSP_PRV_PRCR_KEY | 0x01);

    // Enable trace clock (max 100Mhz). Since PLL/CPU is 200Mhz, clock div = 2
    R_SYSTEM->TRCKCR = R_SYSTEM_TRCKCR_TRCKEN_Msk | 0x01;

    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_KEY;
#endif

#if CFG_TUSB_OS == OPT_OS_NONE
    SysTick_Config(SystemCoreClock / 1000);
#endif

#if TUF2_LOG
    board_uart_init(BOARD_UART_BAUDRATE);
#endif
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_PRC1_UNLOCK;
}

void board_teardown(void)
{
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_LOCK;
#if TUF2_LOG
    R_BSP_IrqDisable(7);
    R_BSP_IrqDisable(8);
    R_BSP_IrqDisable(9);
    R_BSP_IrqDisable(10);
#endif
}

void board_dfu_init(void)
{

#if CFG_TUSB_OS == OPT_OS_FREERTOS
    // If freeRTOS is used, IRQ priority is limit by max syscall ( smaller is higher )
    NVIC_SetPriority(USBFS_INT_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USBFS_RESUME_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USBFS_FIFO_0_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_SetPriority(USBFS_FIFO_1_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
#endif
}

void board_reset(void)
{
    NVIC_SystemReset();
}

void board_dfu_complete(void)
{
    TUF2_LOG1("DFU complete");
    // Mostly reset
    NVIC_SystemReset();
}

bool board_app_valid(void)
{
    return (((*(uint32_t *)BOARD_FLASH_APP_START) & 0xFF000000) == 0x20000000);
}

void board_app_jump(void)
{
    // Jump to application code
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_PRC1_UNLOCK;
    R_BSP_MODULE_STOP(FSP_IP_USBFS, 0);
    R_BSP_MODULE_STOP(FSP_IP_USBHS, 0);
    R_SYSTEM->PRCR = (uint16_t)BSP_PRV_PRCR_LOCK;

    /* Disable MSP monitoring. */
#if BSP_FEATURE_TZ_HAS_TRUSTZONE
    __set_MSPLIM(0);
#else
    R_MPU_SPMON->SP[0].CTL = 0;
#endif

    __disable_irq(); // Note: remember to enable IRQ in application
    __DSB();
    __ISB();

    // Disable SysTick
    SysTick->CTRL = 0;

    SCB->VTOR = BOARD_FLASH_APP_START;

    uint32_t mainStackPointer = *(volatile uint32_t *)(BOARD_FLASH_APP_START);
    __set_MSP(mainStackPointer);
    uint32_t programResetHandlerAddress = *(volatile uint32_t *)(BOARD_FLASH_APP_START + 4);
    void (*programResetHandler)(void) = (void (*)(void))programResetHandlerAddress;
    programResetHandler();
}

uint8_t board_usb_get_serial(uint8_t serial_id[16])
{
    (void)serial_id;
    const bsp_unique_id_t *t = R_BSP_UniqueIdGet();
    memcpy(serial_id, t->unique_id_bytes, 16);
    return 0;
}

//--------------------------------------------------------------------+
// LED pattern
//--------------------------------------------------------------------+

void board_led_write(uint32_t state)
{
    R_IOPORT_PinWrite(&port_ctrl, LED1, state ? LED_STATE_ON : !LED_STATE_ON);
}

void board_rgb_write(uint8_t const rgb[])
{
    (void)rgb;
}

//--------------------------------------------------------------------+
// Timer
//--------------------------------------------------------------------+

void board_timer_start(uint32_t ms)
{
    (void)ms;
    SysTick_Config((SystemCoreClock / 1000) * ms);
}

void board_timer_stop(void)
{
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
    board_timer_handler();
}

void board_uart_init(uint32_t baudrate)
{
    fsp_err_t err;
    const bool bit_mod = true;
    const uint32_t err_rate = 3000; // means 3%
    uart_cfg_t uart_cfg;
    baud_setting_t uart_baud;
    sci_uart_extended_cfg_t uart_cfg_extend;

    uart_baud.semr_baudrate_bits_b.abcse = 0;
    uart_baud.semr_baudrate_bits_b.abcs = 0;
    uart_baud.semr_baudrate_bits_b.bgdm = 1;
    uart_baud.cks = 0;
    uart_baud.brr = 25;
    uart_baud.mddr = (uint8_t)256;
    uart_baud.semr_baudrate_bits_b.brme = false;

    uart_cfg_extend.clock = SCI_UART_CLOCK_INT;
    uart_cfg_extend.rx_edge_start = SCI_UART_START_BIT_FALLING_EDGE;
    uart_cfg_extend.noise_cancel = SCI_UART_NOISE_CANCELLATION_DISABLE;
    uart_cfg_extend.rx_fifo_trigger = SCI_UART_RX_FIFO_TRIGGER_MAX;
    uart_cfg_extend.p_baud_setting = &uart_baud;
    uart_cfg_extend.flow_control = SCI_UART_FLOW_CONTROL_RTS;
    uart_cfg_extend.flow_control_pin = (bsp_io_port_pin_t)UINT16_MAX;

    uart_cfg_extend.rs485_setting.enable = SCI_UART_RS485_DISABLE;
    uart_cfg_extend.rs485_setting.polarity = SCI_UART_RS485_DE_POLARITY_HIGH;
    uart_cfg_extend.rs485_setting.de_control_pin = (bsp_io_port_pin_t)UINT16_MAX;

    uart_cfg.txi_irq = 7;
    uart_cfg.tei_irq = 8;
    uart_cfg.rxi_irq = 9;
    uart_cfg.eri_irq = 10;

    uart_cfg.txi_ipl = 12;
    uart_cfg.rxi_ipl = 12;
    uart_cfg.tei_ipl = 12;
    uart_cfg.eri_ipl = 12;

    R_BSP_IrqEnable(uart_cfg.txi_irq);
    R_BSP_IrqEnable(uart_cfg.tei_irq);
    R_BSP_IrqEnable(uart_cfg.rxi_irq);
    R_BSP_IrqEnable(uart_cfg.eri_irq);

    uart_cfg.channel = 2;
    uart_cfg.p_context = NULL;
    uart_cfg.p_extend = &uart_cfg_extend;
    uart_cfg.p_transfer_tx = NULL;
    uart_cfg.p_transfer_rx = NULL;

    // 8N1
    uart_cfg.data_bits = UART_DATA_BITS_8;
    uart_cfg.parity = UART_PARITY_OFF;
    uart_cfg.stop_bits = UART_STOP_BITS_1;
    uart_cfg.p_callback = uart_callback;

    err = R_SCI_UART_BaudCalculate(baudrate, bit_mod, err_rate, &uart_baud);
    if (uart_baud.mddr == 0)
    {
        err = R_SCI_UART_BaudCalculate(baudrate, false, err_rate, &uart_baud);
    }
    err = R_SCI_UART_Open(&uart_ctrl, &uart_cfg);
    if (err != FSP_SUCCESS)
        while (1)
            ;
    err = R_SCI_UART_BaudSet(&uart_ctrl, (void *)&uart_baud);
    if (err != FSP_SUCCESS)
        while (1)
            ;
}

int board_uart_write(void const *buf, int len)
{
    tx_done = false;
    R_SCI_UART_Write(&uart_ctrl, buf, len);
    while (!tx_done)
    {
        ;
    }

    return 0;
}

//--------------------------------------------------------------------+
// USB Interrupt Handler
//--------------------------------------------------------------------+

//------------- USB0 FullSpeed -------------//
void usbfs_interrupt_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    tud_int_handler(0);
}

void usbfs_resume_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    tud_int_handler(0);
}

void usbfs_d0fifo_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    // TODO not used yet
}

void usbfs_d1fifo_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    // TODO not used yet
}

//------------- USB1 HighSpeed -------------//
#ifdef BOARD_HAS_USB_HIGHSPEED

void usbhs_interrupt_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    tud_int_handler(1);
}

void usbhs_d0fifo_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    // TODO not used yet
}

void usbhs_d1fifo_handler(void)
{
    IRQn_Type irq = R_FSP_CurrentIrqGet();
    R_BSP_IrqStatusClear(irq);
    // TODO not used yet
}

#endif