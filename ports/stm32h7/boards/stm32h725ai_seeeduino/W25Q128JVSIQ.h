/*!<*
 ******************************************************************************
 * @file    W25Q128JVSIQ.h
 * @author  SeeedStduio
 * @brief   This file contains all the description of the
 *          W25Q128JVSIQ OSPI memory.
 ******************************************************************************
 */
#ifndef __W25Q128JVSIQ__H
#define __W25Q128JVSIQ__H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

/*!< Private define ------------------------------------------------------------*/
#define W25Q128JVSIQ_ID  0XEF4018  /*!< W25Q128JVSIQ JEDEC ID */

/**
 * @brief  W25Q128JVSIQ Info
 */

#define W25Q128JVSIQ_FLASH_SIZE                  (uint32_t)(128*1024*1024/16)  /*!< 128Mbits => 16MBytes */
#define W25Q128JVSIQ_PAGE_SIZE                   256                          /*!< 65536 pages of 256Bytes */
#define W25Q128JVSIQ_SECTOR_64K                  (uint32_t)(16 * 1024)        /*!< 1024 sectors of 64KBytes     */
#define W25Q128JVSIQ_SUBSECTOR_4K                (uint32_t)(1  * 1024)        /*!< 4087 subsectors of 4KBytes  */

#define W25Q128JVSIQ_DUMMY_CYCLES_READ           8
#define W25Q128JVSIQ_DUMMY_CYCLES_READ_QUAD      4

/**
 * @brief  W25Q128JVSIQ Timing configuration
 */

#define W25Q128JVSIQ_BULK_ERASE_MAX_TIME          460000U
#define W25Q128JVSIQ_SECTOR_ERASE_MAX_TIME        1000U
#define W25Q128JVSIQ_SUBSECTOR_4K_ERASE_MAX_TIME  400U
#define W25Q128JVSIQ_WRITE_REG_MAX_TIME           40U

#define W25Q128JVSIQ_RESET_MAX_TIME               100U                 /*!< when SWreset during erase operation */

#define W25Q128JVSIQ_AUTOPOLLING_INTERVAL_TIME    0x10U

/* QSPI Error codes */
#define  W25Q128JVSIQ_OK            ((uint8_t)0x00)
#define  W25Q128JVSIQ_ERROR         ((uint8_t)0x01)
#define  W25Q128JVSIQ_BUSY          ((uint8_t)0x02)
#define  W25Q128JVSIQ_NOT_SUPPORTED ((uint8_t)0x04)
#define  W25Q128JVSIQ_SUSPENDED     ((uint8_t)0x08)

/**
 * @brief  W25Q128 Command
 */
/* Reset & Power  */
#define W25Q128JVSIQ_NOP_CMD                              0x00U   /*!<!< No operation */
#define W25Q128JVSIQ_RESET_ENABLE_CMD                     0x66
#define W25Q128JVSIQ_RESET_DEVICE_CMD                     0x99
#define W25Q128JVSIQ_POWER_DOWN_CMD                       0xB9

/* QPI CMD */
#define  W25Q128JVSIQ_ENTER_QPI_MODE_CMD                  0x38
#define  W25Q128JVSIQ_EXIT_QPI_MODE_CMD                   0xFF

/* ID CMD */
#define W25Q128JVSIQ_READ_UNIQUE_ID_CMD                   0x4B
#define W25Q128JVSIQ_READ_DEVICE_ID_CMD                   0x90
#define W25Q128JVSIQ_READ_DEVICE_ID_DUAL_CMD              0x92
#define W25Q128JVSIQ_READ_DEVICE_ID_QUAD_CMD              0x94
#define W25Q128JVSIQ_READ_JEDEC_ID_CMD                    0x9F

#define  W25Q128JVSIQ_UNIQUE_ID_SIZE 16          /*!<!<  64Bits => 16Bytes */
#define  W25Q128JVSIQ_UNIQUE_ID_DUMMY_BYTES 4    /*!<!<  4Bytes */
#define  W25Q128JVSIQ_DEVICE_ID_SIZE 16          /*!<!<  64Bits => 16Bytes */
#define  W25Q128JVSIQ_JEDEC_ID_SIZE 3            /*!<!<  3Bytes */

/* Erase */
#define W25Q128JVSIQ_ERASE_SECTOR_CMD              0x20
#define W25Q128JVSIQ_ERASE_BLOCK_32K_CMD           0x52
#define W25Q128JVSIQ_ERASE_BLOCK_64K_CMD           0xD8
#define W25Q128JVSIQ_ERASE_CHIP_CMD                0xC7

/* Write */
#define W25Q128JVSIQ_WRITE_ENABLE_CMD                          0x06
#define W25Q128JVSIQ_WRITE_VOLATILE_STATUS_REG_ENABLE_CMD      0x50
#define W25Q128JVSIQ_WRITE_DISABLE_CMD                         0x04

/* Read */
#define W25Q128JVSIQ_READ_CMD                      0x03
#define W25Q128JVSIQ_FAST_READ_CMD                 0x0B
#define W25Q128JVSIQ_FAST_READ_DUAL_O_CMD          0x3B
#define W25Q128JVSIQ_FAST_READ_QUAD_O_CMD          0x6B
#define W25Q128JVSIQ_FAST_READ_DUAL_IO_CMD         0xBB
#define W25Q128JVSIQ_FAST_READ_QUAD_IO_CMD         0xEB
#define W25Q128JVSIQ_FAST_READ_QUAD_IO_DTR_CMD     0xED
#define W25Q128JVSIQ_PAGE_PROG_CMD                 0x02
#define W25Q128JVSIQ_PAGE_PROG_QUAD_CMD            0x32

#define W25Q128JVSIQ_PROG_ERASE_SUSPEND_CMD        0x75
#define W25Q128JVSIQ_PROG_ERASE_RESUME_CMD         0x7A

/* Register */
#define W25Q128JVSIQ_READ_STATUS_REG1_CMD          0x05
#define W25Q128JVSIQ_READ_STATUS_REG2_CMD          0x35
#define W25Q128JVSIQ_READ_STATUS_REG3_CMD          0x15

#define W25Q128JVSIQ_WRITE_STATUS_REG1_CMD         0x01
#define W25Q128JVSIQ_WRITE_STATUS_REG2_CMD         0x31
#define W25Q128JVSIQ_WRITE_STATUS_REG3_CMD         0x11

#define W25Q128JVSIQ_STATUS_REG1_BUSY    (1 << 0ul)  /*!< Erase/Write In Progress R */
#define W25Q128JVSIQ_STATUS_REG1_WEL     (1 << 1ul)  /*!< Write Enable Latch  R */
#define W25Q128JVSIQ_STATUS_REG1_BP0     (1 << 2ul)  /*!< Block Protect Bits 0 R/W */
#define W25Q128JVSIQ_STATUS_REG1_BP1     (1 << 3ul)  /*!< Block Protect Bits 1 R/W */
#define W25Q128JVSIQ_STATUS_REG1_BP2     (1 << 4ul)  /*!< Block Protect Bits 2 R/W */
#define W25Q128JVSIQ_STATUS_REG1_TB      (1 << 5ul)  /*!< Top/Bottom Block Protect R/W */
#define W25Q128JVSIQ_STATUS_REG1_SEC     (1 << 6ul)  /*!< Sector/Block Protect Bit R/W */
#define W25Q128JVSIQ_STATUS_REG1_SRP     (1 << 7ul)  /*!< Complement Protect R/W */

#define W25Q128JVSIQ_STATUS_REG2_SRL     (1 << 0ul)  /*!< Status Register Lock R/W */
#define W25Q128JVSIQ_STATUS_REG2_QE      (1 << 1ul)  /*!< Quad Enable R/W */
#define W25Q128JVSIQ_STATUS_REG2_R0      (1 << 2ul)  /*!< Reserved */
#define W25Q128JVSIQ_STATUS_REG2_LB1     (1 << 3ul)  /*!< Security Register Lock Bits 1 R/W */
#define W25Q128JVSIQ_STATUS_REG2_LB2     (1 << 4ul)  /*!< Security Register Lock Bits 2 R/W */
#define W25Q128JVSIQ_STATUS_REG2_LB3     (1 << 5ul)  /*!< Security Register Lock Bits 3 R/W */
#define W25Q128JVSIQ_STATUS_REG2_CMP     (1 << 6ul)  /*!< Complement protect R/W */
#define W25Q128JVSIQ_STATUS_REG2_SUS     (1 << 7ul)  /*!< Erase/Program Suspend Status R */

#define W25Q128JVSIQ_STATUS_REG3_R0      (1 << 0ul)  /*!< Reserved */
#define W25Q128JVSIQ_STATUS_REG3_R1      (1 << 1ul)  /*!< Reserved */
#define W25Q128JVSIQ_STATUS_REG3_WPS     (1 << 2ul)  /*!< Write Protect Selection R/W */
#define W25Q128JVSIQ_STATUS_REG3_R2      (1 << 3ul)  /*!< Reserved */
#define W25Q128JVSIQ_STATUS_REG3_R3      (1 << 4ul)  /*!< Reserved */
#define W25Q128JVSIQ_STATUS_REG3_DRV0    (1 << 5ul)  /*!< Output Driver Strength Bits0 R/W */
#define W25Q128JVSIQ_STATUS_REG3_DRV1    (1 << 6ul)  /*!< Output Driver Strength Bits1 R/W */
#define W25Q128JVSIQ_STATUS_REG3_R4      (1 << 7ul)  /*!< Reserved */

#define W25Q128JVSIQ_STATUS_REG3_DRV_Pos  5
#define W25Q128JVSIQ_STATUS_REG3_DRV_Msk  (_Ul(0x03) << W25Q128JVSIQ_STATUS_REG2_DRV_Pos)  /*!<  Output Driver Strength */
#define W25Q128JVSIQ_STATUS_REG3_DRV_25   (_Ul(0x03) << W25Q128JVSIQ_STATUS_REG2_DRV_Pos)  /*!< 25% default */
#define W25Q128JVSIQ_STATUS_REG3_DRV_50   (_Ul(0x02) << W25Q128JVSIQ_STATUS_REG2_DRV_Pos)  /*!< 50% */
#define W25Q128JVSIQ_STATUS_REG3_DRV_75   (_Ul(0x01) << W25Q128JVSIQ_STATUS_REG2_DRV_Pos)  /*!< 75% */
#define W25Q128JVSIQ_STATUS_REG3_DRV_100  (_Ul(0x00) << W25Q128JVSIQ_STATUS_REG2_DRV_Pos)  /*!< 100% */

/**
 * @}
 */

/** @defgroup W25Q128JVSIQ_Exported_Types W25Q128JVSIQ Exported Types
 * @{
 */
typedef struct {
	uint32_t FlashSize; /*!<!< Size of the flash                             */
	uint32_t EraseSectorSize; /*!<!< Size of sectors for the erase operation       */
	uint32_t EraseSectorsNumber; /*!<!< Number of sectors for the erase operation     */
	uint32_t EraseSubSectorSize; /*!<!< Size of subsector for the erase operation     */
	uint32_t EraseSubSectorNumber; /*!<!< Number of subsector for the erase operation   */
	uint32_t EraseSubSector1Size; /*!<!< Size of subsector 1 for the erase operation   */
	uint32_t EraseSubSector1Number; /*!<!< Number of subsector 1 for the erase operation */
	uint32_t ProgPageSize; /*!<!< Size of pages for the program operation       */
	uint32_t ProgPagesNumber; /*!<!< Number of pages for the program operation     */
} W25Q128JVSIQ_Info_t;

typedef enum {
  W25Q128JVSIQ_ERASE_4K = 0,                 /*!< 4K size Sector erase                          */
  W25Q128JVSIQ_ERASE_32K,                    /*!< 32K size Block erase                          */
  W25Q128JVSIQ_ERASE_64K,                    /*!< 64K size Block erase                          */
  W25Q128JVSIQ_ERASE_BULK                    /*!< Whole bulk erase                              */
} W25Q128JVSIQ_Erase_t;

/* Function by commands combined */
int32_t W25Q128JVSIQ_GetFlashInfo(W25Q128JVSIQ_Info_t *pInfo);
int32_t W25Q128JVSIQ_AutoPollingMemReady(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_Init(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_QuadWriteEnable(OSPI_HandleTypeDef *Ctx);

/* Reset & Power Commands **************************************************/
int32_t W25Q128JVSIQ_ResetEnable(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_ResetDevice(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_NoOperation(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_EnterPowerDown(OSPI_HandleTypeDef *Ctx);

/* ID Commands *************************************************************/
int32_t W25Q128JVSIQ_ReadUniqueID(OSPI_HandleTypeDef *Ctx, uint8_t *ID);
int32_t W25Q128JVSIQ_ReadJEDECID(OSPI_HandleTypeDef *Ctx, uint8_t *ID);

/* Read/Write Array Commands **************************************************/
int32_t W25Q128JVSIQ_ReadSTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t W25Q128JVSIQ_ReadDTR(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t W25Q128JVSIQ_ReadQuad(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t ReadAddr, uint32_t Size);
int32_t W25Q128JVSIQ_PageProgram(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t W25Q128JVSIQ_PageProgramQuad(OSPI_HandleTypeDef *Ctx, uint8_t *pData, uint32_t WriteAddr, uint32_t Size);
int32_t W25Q128JVSIQ_BlockErase(OSPI_HandleTypeDef *Ctx, uint32_t BlockAddress, W25Q128JVSIQ_Erase_t BlockSize);
int32_t W25Q128JVSIQ_ChipErase(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_EnableSTRMemoryMappedMode(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_EnableQuadMemoryMappedMode(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_Suspend(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_Resume(OSPI_HandleTypeDef *Ctx);

/* Register/Setting Commands **************************************************/
int32_t W25Q128JVSIQ_WriteEnable(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_WriteDisable(OSPI_HandleTypeDef *Ctx);
int32_t W25Q128JVSIQ_ReadStatusRegister(OSPI_HandleTypeDef *Ctx, uint8_t Index, uint8_t *Value);
int32_t W25Q128JVSIQ_WriteStatusRegister(OSPI_HandleTypeDef *Ctx, uint8_t Index, uint8_t Value);



#ifdef __cplusplus
}
#endif
#endif
