/** 
 * The MIT License (MIT) 
 * 
 * Author: Jiahua.zhang (1196466552@qq.com) 
 * 
 * Copyright (C) 2021 Seeed Technology Co.,Ltd. 
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

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
#include "board_api.h"
#include "tusb.h" // for logging

OSPI_HandleTypeDef hospi1;

/* OCTOSPI1 init function */
void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef sOspiManagerCfg = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 4;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_APMEMORY;
  hospi1.Init.DeviceSize = 23;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
  hospi1.Init.ClockPrescaler = 2;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_ENABLE;
  hospi1.Init.ChipSelectBoundary = 10;
  hospi1.Init.ClkChipSelectHighTime = 1;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_USED;
  hospi1.Init.MaxTran = 0;
  hospi1.Init.Refresh = 0;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  sOspiManagerCfg.ClkPort = 1;
  sOspiManagerCfg.DQSPort = 1;
  sOspiManagerCfg.NCSPort = 1;
  sOspiManagerCfg.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  sOspiManagerCfg.IOHighPort = HAL_OSPIM_IOPORT_1_HIGH;
  if (HAL_OSPIM_Config(&hospi1, &sOspiManagerCfg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

}

void DLYB_OCTOSPI1_Calibration(uint8_t phase)
{
	int flag = 0;
	uint32_t LNGG = 0;
	uint32_t LNG[3];
	uint32_t TimeOut = 0;

	/*Preparation before calibration*/
	CLEAR_BIT(OCTOSPI1->DCR1,OCTOSPI_DCR1_DLYBYP);
	SET_BIT(OCTOSPI1->DCR1,OCTOSPI_DCR1_FRCK);

	/*Configure delay line length to 1 full input clock cycle*/
	DLYB_OCTOSPI1->CR 	|= 0x03;				
	DLYB_OCTOSPI1->CFGR &= ( ~ (0xf) );			
	DLYB_OCTOSPI1->CFGR |= 12;

	for(uint8_t i = 0;i < 128;i ++)
	{
		DLYB_OCTOSPI1->CFGR &= ( ~ (0x7f<<8) ); 	
		DLYB_OCTOSPI1->CFGR |= (i << 8);			
		while( !( (DLYB_OCTOSPI1->CFGR>>31) & 0x01) )		
		{
			TimeOut ++;
			if(TimeOut > 0xffff)
			{
				break;
			}
		}
		if( ( (DLYB_OCTOSPI1->CFGR>>31) & 0x01) )			
		{
			flag = 1;
		}

		if(flag)
		{
			LNGG = (DLYB_OCTOSPI1->CFGR >> 16) & 0xfff;
			LNG[0] = LNGG & 0x7ff;
			LNG[1] = (LNGG >> 10) & 0x01;
			LNG[2] = (LNGG >> 11) & 0x01;
			if( (LNG[0] > 0) && ( (LNG[1] == 0) || (LNG[2] == 0) ) )	//Determine whether the delay line length is reasonable
			{
				TU_LOG1("The Delay Line is set one input clock period\r\n");
				break;
			}
			else
			{
				flag = 0;
			}
		}
		else
		{	
			TU_LOG1("The Delay Line is set err\r\n");
		}
	}


	/*Determine how many Unit delays are required for an input clock cycle*/
	for(int8_t i = 10;i >= 0;i --)
	{
		if( (LNGG >> i) & 0x01 )
		{
			TU_LOG1("UnitDelayNum is %d\r\n",i);
			break;
		}
	}

	/*Sets the output clock phase*/
	DLYB_OCTOSPI1->CFGR &= ( ~ (0xf) );
	DLYB_OCTOSPI1->CFGR |= phase;

	DLYB_OCTOSPI1->CR 	&= ( ~ ( 1 << 1 ) );

	SET_BIT(OCTOSPI1->CR,OCTOSPI_CR_ABORT);
	CLEAR_BIT(OCTOSPI1->DCR1,OCTOSPI_DCR1_FRCK);
}


void PsramRegWrite(uint8_t *reg, uint32_t addr)
{
		OSPI_RegularCmdTypeDef sCommand;

		sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
		sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
		sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
		sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
		sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
		sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
		sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
		sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
		sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
		sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
		sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
		sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
		sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
		sCommand.Instruction        = 0xC0;
		sCommand.Address            = addr;
		sCommand.NbData             = 2;
		sCommand.DummyCycles        = 0;

		if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            TU_LOG1("reg write command err\r\n");
		    __set_FAULTMASK(1);
	  		NVIC_SystemReset();
		  }

		if (HAL_OSPI_Transmit(&hospi1, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            TU_LOG1("reg write value err\r\n");
			__set_FAULTMASK(1);
			NVIC_SystemReset();
		  }
}


void PsramRegRead(uint8_t *reg, uint32_t addr)
{
		OSPI_RegularCmdTypeDef sCommand;

		sCommand.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
	    sCommand.FlashId            = HAL_OSPI_FLASH_ID_1;
	    sCommand.InstructionMode    = HAL_OSPI_INSTRUCTION_8_LINES;
	    sCommand.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
	    sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	    sCommand.AddressMode        = HAL_OSPI_ADDRESS_8_LINES;
	    sCommand.AddressSize        = HAL_OSPI_ADDRESS_32_BITS;
	    sCommand.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_ENABLE;
	    sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	    sCommand.DataMode           = HAL_OSPI_DATA_8_LINES;
	    sCommand.DataDtrMode        = HAL_OSPI_DATA_DTR_ENABLE;
	    sCommand.DQSMode            = HAL_OSPI_DQS_ENABLE;
	    sCommand.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;
	    sCommand.Instruction        = 0x40;
	    sCommand.Address            = addr;
	    sCommand.NbData             = 2;
	    sCommand.DummyCycles        = 5;

	    if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            TU_LOG1("reg read command err\r\n");
	    	__set_FAULTMASK(1);
	  		NVIC_SystemReset();
	    }

	    if (HAL_OSPI_Receive(&hospi1, reg, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
            TU_LOG1("reg read value err\r\n");
	    	__set_FAULTMASK(1);
	  		NVIC_SystemReset();
	    }
}


/* This function enables memory-mapped mode for Read and Write operations */
void EnableMemMapped(void)
{
        OSPI_RegularCmdTypeDef sCommand;
        OSPI_MemoryMappedTypeDef sMemMappedCfg;
        sCommand.FlashId = HAL_OSPI_FLASH_ID_1;
        sCommand.InstructionMode = HAL_OSPI_INSTRUCTION_8_LINES;
        sCommand.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
        sCommand.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
        sCommand.AddressMode = HAL_OSPI_ADDRESS_8_LINES;
        sCommand.AddressSize = HAL_OSPI_ADDRESS_32_BITS;
        sCommand.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_ENABLE;
        sCommand.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
        sCommand.DataMode = HAL_OSPI_DATA_8_LINES;
        sCommand.DataDtrMode = HAL_OSPI_DATA_DTR_ENABLE;
        sCommand.DQSMode = HAL_OSPI_DQS_ENABLE;
        sCommand.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;
        sCommand.Address = 0;
        sCommand.NbData = 1;

        /* Memory-mapped mode configuration for Linear burst write operations */
        sCommand.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
        sCommand.Instruction = 0xA0;
        sCommand.DummyCycles = 4;
        if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            Error_Handler();
        }

        /* Memory-mapped mode configuration for Linear burst read operations */
        sCommand.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
        sCommand.Instruction = 0x20;
        sCommand.DummyCycles = 5;
        if (HAL_OSPI_Command(&hospi1, &sCommand, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
        {
            Error_Handler();
        }

        /*Disable timeout counter for memory mapped mode*/
        sMemMappedCfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;
        /*Enable memory mapped mode*/
        if (HAL_OSPI_MemoryMapped(&hospi1, &sMemMappedCfg) != HAL_OK)
        {
            Error_Handler();
        }
}


static inline void psram_delay(uint32_t delay)
{
    for(uint32_t i = 0;i < 66000;i ++)
        for(uint32_t j = 0;j < delay;j ++)
            ;
}

void psram_init(void)
{
    TU_LOG1("psram init\r\n");
    MX_OCTOSPI1_Init(); 

	/* Calibration delay module */                
    DLYB_OCTOSPI1_Calibration(1);		
	psram_delay(100);

    /*Calibration PSRAM*/
    uint8_t regs;
	regs = 0x08;
	PsramRegWrite(&regs,0);
    psram_delay(500);
    
    psram_delay(500);

	/* Memory Map */
    EnableMemMapped();
    psram_delay(500);
}
