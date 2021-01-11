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
#include "tusb.h" // for logging
#include "W25Q128JVSIQ.h"

OSPI_HandleTypeDef hospi2;

#define FLASH_CACHE_SIZE 4096
#define FLASH_CACHE_INVALID_ADDR 0xffffffff

// FLASH
#define NO_CACHE 0xffffffff

#define APP_LOAD_ADDRESS OCTOSPI2_BASE

#define SECTOR_SIZE (4 * 1024)
#define FLASH_PAGE_SIZE 256
#define FILESYSTEM_BLOCK_SIZE 256

static uint32_t _flash_page_addr = NO_CACHE;
static uint8_t _flash_cache[SECTOR_SIZE] __attribute__((aligned(4)));

/**
  * @brief OCTOSPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI2_Init(void)
{

	/* USER CODE BEGIN OCTOSPI2_Init 0 */

	/* USER CODE END OCTOSPI2_Init 0 */

	OSPIM_CfgTypeDef sOspiManagerCfg = {0};

	/* USER CODE BEGIN OCTOSPI2_Init 1 */

	/* USER CODE END OCTOSPI2_Init 1 */
	/* OCTOSPI2 parameter configuration*/
	hospi2.Instance = OCTOSPI2;
	hospi2.Init.FifoThreshold = 4;
	hospi2.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
	hospi2.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
	hospi2.Init.DeviceSize = 24;
	hospi2.Init.ChipSelectHighTime = 1;
	hospi2.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
	hospi2.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
	hospi2.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
	hospi2.Init.ClockPrescaler = 3;
	hospi2.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
	hospi2.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
	hospi2.Init.ChipSelectBoundary = 0;
	hospi2.Init.ClkChipSelectHighTime = 0;
	hospi2.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
	hospi2.Init.MaxTran = 0;
	hospi2.Init.Refresh = 0;
	if (HAL_OSPI_Init(&hospi2) != HAL_OK)
	{
		Error_Handler();
	}
	sOspiManagerCfg.ClkPort = 2;
	sOspiManagerCfg.NCSPort = 2;
	sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_2_LOW;
	if (HAL_OSPIM_Config(&hospi2, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN OCTOSPI2_Init 2 */

	/* USER CODE END OCTOSPI2_Init 2 */
}

// /**
//   * @brief OCTOSPI2 Initialization Function
//   * @param None
//   * @retval None
//   */
// static void MX_OCTOSPI2_DeInit(void)
// {
// 	HAL_OSPI_DeInit(&hospi2);
// }

/**
* @brief OSPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hospi: OSPI handle pointer
* @retval None
*/
void HAL_OSPI_MspInit(OSPI_HandleTypeDef *hospi)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (hospi->Instance == OCTOSPI2)
	{
		/* USER CODE BEGIN OCTOSPI2_MspInit 0 */

		/* USER CODE END OCTOSPI2_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_OSPI2_CLK_ENABLE();

		__HAL_RCC_GPIOF_CLK_ENABLE();
		__HAL_RCC_GPIOG_CLK_ENABLE();
		/**OCTOSPI2 GPIO Configuration
    	PF1     ------> OCTOSPIM_P2_IO1
    	PF3     ------> OCTOSPIM_P2_IO3
    	PG12    ------> OCTOSPIM_P2_NCS
    	PF0     ------> OCTOSPIM_P2_IO0
    	PF2     ------> OCTOSPIM_P2_IO2
    	PF4     ------> OCTOSPIM_P2_CLK
    	*/
		GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P2;
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF3_OCTOSPIM_P2;
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

		/* USER CODE BEGIN OCTOSPI2_MspInit 1 */

		/* USER CODE END OCTOSPI2_MspInit 1 */
	}
}

/**
* @brief OSPI MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hospi: OSPI handle pointer
* @retval None
*/
void HAL_OSPI_MspDeInit(OSPI_HandleTypeDef *hospi)
{
	if (hospi->Instance == OCTOSPI2)
	{
		/* USER CODE BEGIN OCTOSPI2_MspDeInit 0 */

		/* USER CODE END OCTOSPI2_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_OSPI2_CLK_DISABLE();

		/**OCTOSPI2 GPIO Configuration
    	PF1     ------> OCTOSPIM_P2_IO1
    	PF3     ------> OCTOSPIM_P2_IO3
    	PG12    ------> OCTOSPIM_P2_NCS
    	PF0     ------> OCTOSPIM_P2_IO0
    	PF2     ------> OCTOSPIM_P2_IO2
    	PF4     ------> OCTOSPIM_P2_CLK
    	*/
		HAL_GPIO_DeInit(GPIOF, GPIO_PIN_1 | GPIO_PIN_3 | GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_4);

		HAL_GPIO_DeInit(GPIOG, GPIO_PIN_12);

		/* USER CODE BEGIN OCTOSPI2_MspDeInit 1 */

		/* USER CODE END OCTOSPI2_MspDeInit 1 */
	}
}

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
uint8_t txBuffer[256] = {0};
uint8_t rxBuffer[256] = {0};
void flash_init(void)
{
	MX_OCTOSPI2_Init();
	W25Q128JVSIQ_Init(&hospi2);
	W25Q128JVSIQ_QuadWriteEnable(&hospi2);
}

void board_flash_memory_mapped(void)
{
	W25Q128JVSIQ_EnableQuadMemoryMappedMode(&hospi2);
}

void board_flash_init(void)
{

}

uint32_t board_flash_size(void)
{
	return W25Q128JVSIQ_FLASH_SIZE;
}

void board_flash_read(uint32_t addr, void *buffer, uint32_t len)
{
	uint32_t const phy_addr = (addr - BOARD_FLASH_APP_START);
	TU_LOG1("board_flash_read addr:%lu len:%lu\n\r", phy_addr, len);
	W25Q128JVSIQ_ReadQuad(&hospi2, buffer, phy_addr, len);
}

void board_flash_flush(void)
{
	uint32_t status;

	if (_flash_page_addr == NO_CACHE)
		return;
	TU_LOG1("Erase and Write at address = 0x%08lX\r\n", _flash_page_addr);

	// Skip if data is the same
	if (1)
	{
		uint32_t const sector_addr = (_flash_page_addr - BOARD_FLASH_APP_START);

		__disable_irq();
		W25Q128JVSIQ_WriteEnable(&hospi2);
		status = W25Q128JVSIQ_BlockErase(&hospi2, sector_addr, W25Q128JVSIQ_ERASE_4K);
		if (status != W25Q128JVSIQ_OK)
		{
			TU_LOG1("Page program failed: status = %ld!\r\n", status);
			return;
		}
		status = W25Q128JVSIQ_AutoPollingMemReady(&hospi2);
		__enable_irq();

		if (status != W25Q128JVSIQ_OK)
		{
			TU_LOG1("Erase failed: status = %ld!\r\n", status);
			return;
		}

		for (int i = 0; i < SECTOR_SIZE / FLASH_PAGE_SIZE; ++i)
		{
			uint32_t const page_addr = sector_addr + i * FLASH_PAGE_SIZE;
			void *page_data = _flash_cache + i * FLASH_PAGE_SIZE;

			__disable_irq();
			W25Q128JVSIQ_WriteEnable(&hospi2);
			status = W25Q128JVSIQ_PageProgramQuad(&hospi2, page_data, page_addr, FLASH_PAGE_SIZE);
			if (status != W25Q128JVSIQ_OK)
			{
				TU_LOG1("Page program failed: status = %ld!\r\n", status);
				return;
			}
			status = W25Q128JVSIQ_AutoPollingMemReady(&hospi2);
			__enable_irq();

			if (status != W25Q128JVSIQ_OK)
			{
				TU_LOG1("Page program failed: status = %ld!\r\n", status);
				return;
			}
		}
	}

	_flash_page_addr = NO_CACHE;
}

void board_flash_write(uint32_t addr, void const *data, uint32_t len)
{
	uint32_t const page_addr = addr & ~(SECTOR_SIZE - 1);
	if (page_addr != _flash_page_addr)
	{
		// Write out anything in cache before overwriting it.
		board_flash_flush();
		_flash_page_addr = page_addr;
	}
	// Overwrite part or all of the page cache with the src data.
	memcpy(_flash_cache + (addr & (SECTOR_SIZE - 1)), data, len);
}

#ifdef TINYUF2_SELF_UPDATE
void board_self_update(const uint8_t *bootloader_bin, uint32_t bootloader_len)
{
	(void)bootloader_bin;
	(void)bootloader_len;
}
#endif
