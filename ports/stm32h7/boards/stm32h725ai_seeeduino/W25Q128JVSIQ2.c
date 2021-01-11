/**
 ******************************************************************************
 * @file    W25Q128JVSIQ.c
 * @modify
 * @brief   This file provides the W25Q128JVSIQ OSPI drivers.
 ******************************************************************************
 */
#include "W25Q128JVSIQ.h"
#include "tusb.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

/* Function by commands combined */
int32_t W25Q128JVSIQ_GetFlashInfo(W25Q128JVSIQ_Info_t *pInfo) {
	/* Configure the structure with the memory configuration */
	pInfo->FlashSize = W25Q128JVSIQ_FLASH_SIZE;
	pInfo->EraseSectorSize = W25Q128JVSIQ_SECTOR_64K;
	pInfo->EraseSectorsNumber = (W25Q128JVSIQ_FLASH_SIZE
			/ W25Q128JVSIQ_SECTOR_64K );
	pInfo->EraseSubSectorSize = W25Q128JVSIQ_SUBSECTOR_4K;
	pInfo->EraseSubSectorNumber = (W25Q128JVSIQ_FLASH_SIZE
			/ W25Q128JVSIQ_SUBSECTOR_4K );
	pInfo->EraseSubSector1Size = W25Q128JVSIQ_SUBSECTOR_4K;
	pInfo->EraseSubSector1Number = (W25Q128JVSIQ_FLASH_SIZE
			/ W25Q128JVSIQ_SUBSECTOR_4K );
	pInfo->ProgPageSize = W25Q128JVSIQ_PAGE_SIZE;
	pInfo->ProgPagesNumber = (W25Q128JVSIQ_FLASH_SIZE / W25Q128JVSIQ_PAGE_SIZE);

	return W25Q128JVSIQ_OK;
}

int32_t W25Q128JVSIQ_AutoPollingMemReady(OSPI_HandleTypeDef *Ctx) {

	OSPI_RegularCmdTypeDef s_command = { 0 };
	OSPI_AutoPollingTypeDef s_config = { 0 };

	/* Configure automatic polling mode to wait for memory ready */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_READ_STATUS_REG1_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
	s_command.Address = 0U;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_1_LINE;
	s_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = 0U;
	s_command.NbData = 1U;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	s_config.Match = 0U;
	s_config.Mask = W25Q128JVSIQ_STATUS_REG1_BUSY;
	s_config.MatchMode = HAL_OSPI_MATCH_MODE_AND;
	s_config.Interval = W25Q128JVSIQ_AUTOPOLLING_INTERVAL_TIME;
	s_config.AutomaticStop = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	if (HAL_OSPI_AutoPolling(Ctx, &s_config, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	return W25Q128JVSIQ_OK;
}

int32_t W25Q128JVSIQ_Init(OSPI_HandleTypeDef *Ctx) {
	if (W25Q128JVSIQ_ResetEnable(Ctx) != W25Q128JVSIQ_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	if (W25Q128JVSIQ_ResetDevice(Ctx) != W25Q128JVSIQ_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	uint8_t JEDECID[W25Q128JVSIQ_JEDEC_ID_SIZE] = { 0 };

	W25Q128JVSIQ_ReadJEDECID(Ctx, JEDECID);
	TU_LOG1("JEDEC ID: %02x%02x%02x\n\r", JEDECID[0], JEDECID[1], JEDECID[2]);

	uint8_t UniqueID[W25Q128JVSIQ_UNIQUE_ID_SIZE] = { 0 };
	W25Q128JVSIQ_ReadUniqueID(Ctx, UniqueID);
	TU_LOG1("UNIQUE ID: ");
	for (int i = 0; i < W25Q128JVSIQ_UNIQUE_ID_SIZE; i++) {
		TU_LOG1("%02x", UniqueID[i]);
	}
	TU_LOG1("\n\r");

	if (W25Q128JVSIQ_ResetEnable(Ctx) != W25Q128JVSIQ_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	if (W25Q128JVSIQ_ResetDevice(Ctx) != W25Q128JVSIQ_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	return W25Q128JVSIQ_OK;
}

int32_t W25Q128JVSIQ_QuadWriteEnable(OSPI_HandleTypeDef *Ctx)
{
	unsigned char reg;
	W25Q128JVSIQ_ReadStatusRegister(Ctx, 2, &reg);
	TU_LOG1("REG2: %02x\n\r", reg);
	if(reg != 0x02)
	{
		reg = 0x02;
		W25Q128JVSIQ_WriteEnable(Ctx);
		W25Q128JVSIQ_WriteStatusRegister(Ctx, 2, reg);
    W25Q128JVSIQ_ReadStatusRegister(Ctx, 2, &reg);
	  TU_LOG1("REG2: %02x\n\r", reg);
	}


	return W25Q128JVSIQ_OK;
}

/* Reset Commands *************************************************************/
/**
 * @brief  Flash reset enable command
 *         SPI/OPI
 * @param  Ctx Component object pointer
 * @retval error status
 */
int32_t W25Q128JVSIQ_ResetEnable(OSPI_HandleTypeDef *Ctx) {
	OSPI_RegularCmdTypeDef s_command = { 0 };

	/* Initialize the reset enable command */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_RESET_ENABLE_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_NONE;
	s_command.DummyCycles = 0U;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	return W25Q128JVSIQ_OK;
}

/**
 * @brief  Flash reset device command
 *         SPI/OPI
 * @param  Ctx Component object pointer
 * @retval error status
 */
int32_t W25Q128JVSIQ_ResetDevice(OSPI_HandleTypeDef *Ctx) {
	OSPI_RegularCmdTypeDef s_command = { 0 };

	/* Initialize the reset enable command */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_RESET_DEVICE_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_NONE;
	s_command.DummyCycles = 0U;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	return W25Q128JVSIQ_OK;
}

/**
 * @brief  Flash reset device command
 *         SPI/OPI
 * @param  Ctx Component object pointer
 * @retval error status
 */
int32_t W25Q128JVSIQ_EnterPowerDown(OSPI_HandleTypeDef *Ctx) {
	OSPI_RegularCmdTypeDef s_command = { 0 };

	/* Initialize the reset enable command */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_POWER_DOWN_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_NONE;
	s_command.DummyCycles = 0U;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Send the command */
	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	return W25Q128JVSIQ_OK;
}

/* ID Commands ****************************************************************/
/**
 * @brief  Read Flash 16 Byte Unique ID.
 * @param  Ctx Component object pointer
 * @param  ID 16 bytes IDs pointer
 * @retval error status
 */
int32_t W25Q128JVSIQ_ReadUniqueID(OSPI_HandleTypeDef *Ctx, uint8_t *ID) {

	if (ID == NULL) {
		return W25Q128JVSIQ_ERROR;
	}

	OSPI_RegularCmdTypeDef s_command = { 0 };

	uint8_t uniqueID[W25Q128JVSIQ_UNIQUE_ID_SIZE
			+ W25Q128JVSIQ_UNIQUE_ID_DUMMY_BYTES] = { 0 };

	/* Initialize the read ID command */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_READ_UNIQUE_ID_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
	s_command.Address = 0U;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_1_LINE;
	s_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = 0U;
	s_command.NbData = W25Q128JVSIQ_UNIQUE_ID_SIZE
			+ W25Q128JVSIQ_UNIQUE_ID_DUMMY_BYTES;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	/* Reception of the data */
	if (HAL_OSPI_Receive(Ctx, uniqueID, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	memcpy(ID, uniqueID + W25Q128JVSIQ_UNIQUE_ID_DUMMY_BYTES,
	W25Q128JVSIQ_UNIQUE_ID_SIZE);

	return W25Q128JVSIQ_OK;
}

/**
  * @brief  Flash no operation command
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface select
  * @param  Rate Transfer rate STR or DTR
  * @retval error status
  */
int32_t W25Q128JVSIQ_NoOperation(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the no operation command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_NOP_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/* ID Commands ****************************************************************/
/**
 * @brief  Read Flash 3 Byte JEDEC ID.
 * @param  Ctx Component object pointer
 * @param  ID 3 bytes IDs pointer
 * @retval error status
 */
int32_t W25Q128JVSIQ_ReadJEDECID(OSPI_HandleTypeDef *Ctx, uint8_t *ID) {

	if (ID == NULL) {
		return W25Q128JVSIQ_ERROR;
	}

	OSPI_RegularCmdTypeDef s_command = { 0 };

	uint8_t JEDECID[W25Q128JVSIQ_JEDEC_ID_SIZE] = { 0 };

	/* Initialize the read ID command */
	s_command.OperationType = HAL_OSPI_OPTYPE_COMMON_CFG;
	s_command.FlashId = HAL_OSPI_FLASH_ID_1;
	s_command.InstructionMode = HAL_OSPI_INSTRUCTION_1_LINE;
	s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
	s_command.InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS;
	s_command.Instruction = W25Q128JVSIQ_READ_JEDEC_ID_CMD;
	s_command.AddressMode = HAL_OSPI_ADDRESS_NONE;
	s_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
	s_command.AddressSize = HAL_OSPI_ADDRESS_24_BITS;
	s_command.Address = 0U;
	s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
	s_command.DataMode = HAL_OSPI_DATA_1_LINE;
	s_command.DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE;
	s_command.DummyCycles = 0U;
	s_command.NbData = W25Q128JVSIQ_JEDEC_ID_SIZE;
	s_command.DQSMode = HAL_OSPI_DQS_DISABLE;
	s_command.SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD;

	/* Configure the command */
	if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	/* Reception of the data */
	if (HAL_OSPI_Receive(Ctx, JEDECID, HAL_OSPI_TIMEOUT_DEFAULT_VALUE)
			!= HAL_OK) {
		return W25Q128JVSIQ_ERROR;
	}

	memcpy(ID, JEDECID, W25Q128JVSIQ_JEDEC_ID_SIZE);

	return W25Q128JVSIQ_OK;
}

/* Read/Write Array Commands (3/4 Byte Address Command Set) *********************/
/**
  * @brief  Reads an amount of data from the OSPI memory on STR mode.
  *         SPI/OPI; 1-1-1/4-4-4
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start address
  * @param  Size Size of data to read
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_ReadSTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* OPI mode and 3-bytes address size not supported by memory */

  /* Initialize the read command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_FAST_READ_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = ReadAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = W25Q128JVSIQ_DUMMY_CYCLES_READ;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}
/**
  * @brief  Reads an amount of data from the OSPI memory on DTR mode.
  *         OPI
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be read
  * @param  ReadAddr Read start addressS
  * @param  Size Size of data to read
  * @note   Only OPI mode support DTR transfer rate
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_ReadQuad(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the read command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_FAST_READ_QUAD_IO_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = ReadAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
  s_command.AlternateBytes 	   = 0;
  s_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.DataMode           = HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = W25Q128JVSIQ_DUMMY_CYCLES_READ_QUAD;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Writes an amount of data to the OSPI memory.
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ W25Q128JVSIQ_PAGE_SIZE
  * @note   Address size is forced to 3 Bytes when the 4 Bytes address size
  *         command is not available for the specified interface mode
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_PageProgram(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_PAGE_PROG_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = WriteAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Transmission of the data */
  if (HAL_OSPI_Transmit(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Writes an amount of data to the OSPI memory on DTR mode.
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @param  pData Pointer to data to be written
  * @param  WriteAddr Write start address
  * @param  Size Size of data to write. Range 1 ~ W25Q128JVSIQ_PAGE_SIZE
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_PageProgramQuad(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the program command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_PAGE_PROG_QUAD_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = WriteAddr;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_4_LINES;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = Size;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Transmission of the data */
  if (HAL_OSPI_Transmit(Ctx, pData, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Erases the specified block of the OSPI memory.
  *         W25Q128JVSIQ support 4K, 64K size block erase commands.
  *         SPI/OPI; 1-1-1/4-4-4
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @param  BlockAddress Block address to erase
  * @param  BlockSize Block size to erase
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_BlockErase(OSPI_HandleTypeDef *Ctx, uint32_t BlockAddress, W25Q128JVSIQ_Erase_t BlockSize)
{
  OSPI_RegularCmdTypeDef s_command = {0};


  /* Initialize the erase command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = BlockAddress;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  switch(BlockSize)
  {
  case W25Q128JVSIQ_ERASE_4K:
	  s_command.Instruction = W25Q128JVSIQ_ERASE_SECTOR_CMD;
	  break;
  case W25Q128JVSIQ_ERASE_32K:
	  s_command.Instruction = W25Q128JVSIQ_ERASE_BLOCK_32K_CMD;
	  break;
  case W25Q128JVSIQ_ERASE_64K:
	  s_command.Instruction = W25Q128JVSIQ_ERASE_BLOCK_64K_CMD;
	  break;
  default:
	  s_command.Instruction = W25Q128JVSIQ_ERASE_SECTOR_CMD;
	  break;
  }

  /* Send the command */
  if(HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Whole chip erase.
  *         SPI/OPI; 1-0-0/4-0-0
  * @param  Ctx Component object pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_ChipErase(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef s_command = {0};


  /* Initialize the erase command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_ERASE_CHIP_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if(HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Enable memory mapped mode for the OSPI memory on STR mode.
  *         SPI; 1-1-1/1-1-4
  * @param  Ctx Component object pointer
  * @param  Mode Interface mode
  * @param  AddressSize Address size
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_EnableSTRMemoryMappedMode(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef      s_command = {0};
  OSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

  /* Initialize the read command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_READ_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_FAST_READ_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_1_LINE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = W25Q128JVSIQ_DUMMY_CYCLES_READ;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the read command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Initialize the program command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_WRITE_CFG;
  s_command.Instruction        = W25Q128JVSIQ_PAGE_PROG_CMD;
  s_command.DummyCycles        = 0U;

  /* Send the write command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(Ctx, &s_mem_mapped_cfg) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}
/**
  * @brief  Enable memory mapped mode for 1-1-4
  * @param  Ctx Component object pointer
  * @note   Only OPI mode support DTR transfer rate
  * @retval OSPI memory status
  */
int32_t W25Q128JVSIQ_EnableQuadMemoryMappedMode(OSPI_HandleTypeDef *Ctx)
{

  OSPI_RegularCmdTypeDef      s_command = {0};
  OSPI_MemoryMappedTypeDef s_mem_mapped_cfg = {0};

  /* Initialize the read command */
   s_command.OperationType      = HAL_OSPI_OPTYPE_READ_CFG;
   s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
   s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
   s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
   s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
   s_command.Instruction        = W25Q128JVSIQ_FAST_READ_QUAD_IO_CMD;
   s_command.AddressMode        = HAL_OSPI_ADDRESS_4_LINES;
   s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
   s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
   s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
   s_command.AlternateBytes 	= 0;
   s_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
   s_command.DataMode           = HAL_OSPI_DATA_4_LINES;
   s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
   s_command.DummyCycles        = W25Q128JVSIQ_DUMMY_CYCLES_READ_QUAD;
   s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
   s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Initialize the program command */
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.AlternateBytes 	= 0;
  s_command.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
  s_command.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
  s_command.Instruction   = W25Q128JVSIQ_PAGE_PROG_QUAD_CMD;
  s_command.DummyCycles   = 0;
  s_command.DQSMode       = HAL_OSPI_DQS_DISABLE;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Configure the memory mapped mode */
  s_mem_mapped_cfg.TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_OSPI_MemoryMapped(Ctx, &s_mem_mapped_cfg) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Flash suspend program or erase command
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_Suspend(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef s_command = {0};



  /* Initialize the suspend command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_PROG_ERASE_SUSPEND_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}

/**
  * @brief  Flash resume program or erase command
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_Resume(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef s_command = {0};


  /* Initialize the resume command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_PROG_ERASE_RESUME_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}


/* Register/Setting Commands **************************************************/
/**
  * @brief  This function send a Write Enable and wait it is effective.
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_WriteEnable(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef     s_command = {0};
  OSPI_AutoPollingTypeDef s_config = {0};


  /* Initialize the write enable command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_WRITE_ENABLE_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  s_command.Instruction    = W25Q128JVSIQ_READ_STATUS_REG1_CMD;
  s_command.AddressMode    = HAL_OSPI_ADDRESS_NONE;
  s_command.AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize    = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address        = 0U;
  s_command.DataMode       = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode    = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles    = 0U ;
  s_command.NbData         = 1U;
  s_command.DQSMode        = HAL_OSPI_DQS_DISABLE;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  s_config.Match           = W25Q128JVSIQ_STATUS_REG1_WEL;
  s_config.Mask            = W25Q128JVSIQ_STATUS_REG1_WEL;
  s_config.MatchMode       = HAL_OSPI_MATCH_MODE_AND;
  s_config.Interval        = W25Q128JVSIQ_AUTOPOLLING_INTERVAL_TIME;
  s_config.AutomaticStop   = HAL_OSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_OSPI_AutoPolling(Ctx, &s_config, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}


/**
  * @brief  This function reset the (WEN) Write Enable Latch bit.
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_WriteDisable(OSPI_HandleTypeDef *Ctx)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  /* Initialize the write disable command */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.Instruction        = W25Q128JVSIQ_WRITE_DISABLE_CMD;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_NONE;
  s_command.DummyCycles        = 0U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}


/**
  * @brief  Read Flash Status register value
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @param  Status index
  * @param  Value Status register value pointer
  * @retval error status
  */
int32_t W25Q128JVSIQ_ReadStatusRegister(OSPI_HandleTypeDef *Ctx, uint8_t Index, uint8_t *Value)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  if (Index < 1 || Index > 3)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Initialize the reading of status register */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = 0U;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = 1U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;


  switch(Index)
  {
  case 1:
	  s_command.Instruction = W25Q128JVSIQ_READ_STATUS_REG1_CMD; break;
  case 2:
	  s_command.Instruction = W25Q128JVSIQ_READ_STATUS_REG2_CMD; break;
  case 3:
	  s_command.Instruction = W25Q128JVSIQ_READ_STATUS_REG3_CMD; break;
  default:
 	  break;
  }

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  /* Reception of the data */
  if (HAL_OSPI_Receive(Ctx, Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}


/**
  * @brief  Write Flash Status register
  *         SPI/OPI
  * @param  Ctx Component object pointer
  * @param  Status index
  * @param  Value Value to write to Status register
  * @retval error status
  */
int32_t W25Q128JVSIQ_WriteStatusRegister(OSPI_HandleTypeDef *Ctx, uint8_t Index, uint8_t Value)
{
  OSPI_RegularCmdTypeDef s_command = {0};

  if (Index < 1 || Index > 3)
  {
    return W25Q128JVSIQ_ERROR;
  }


  /* Initialize the writing of status register */
  s_command.OperationType      = HAL_OSPI_OPTYPE_COMMON_CFG;
  s_command.FlashId            = HAL_OSPI_FLASH_ID_1;
  s_command.InstructionMode    = HAL_OSPI_INSTRUCTION_1_LINE;
  s_command.InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE;
  s_command.InstructionSize    = HAL_OSPI_INSTRUCTION_8_BITS;
  s_command.AddressMode        = HAL_OSPI_ADDRESS_NONE;
  s_command.AddressDtrMode     = HAL_OSPI_ADDRESS_DTR_DISABLE;
  s_command.AddressSize        = HAL_OSPI_ADDRESS_24_BITS;
  s_command.Address            = 0U;
  s_command.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode           = HAL_OSPI_DATA_1_LINE;
  s_command.DataDtrMode        = HAL_OSPI_DATA_DTR_DISABLE;
  s_command.DummyCycles        = 0U;
  s_command.NbData             = 1U;
  s_command.DQSMode            = HAL_OSPI_DQS_DISABLE;
  s_command.SIOOMode           = HAL_OSPI_SIOO_INST_EVERY_CMD;

  switch(Index)
  {
  case 1:
	  s_command.Instruction = W25Q128JVSIQ_WRITE_STATUS_REG1_CMD; break;
  case 2:
	  s_command.Instruction = W25Q128JVSIQ_WRITE_STATUS_REG2_CMD; break;
  case 3:
	  s_command.Instruction = W25Q128JVSIQ_WRITE_STATUS_REG2_CMD; break;
  default:
 	  break;
  }

  /* Send the command */
  if (HAL_OSPI_Command(Ctx, &s_command, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  if (HAL_OSPI_Transmit(Ctx, &Value, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return W25Q128JVSIQ_ERROR;
  }

  return W25Q128JVSIQ_OK;
}


