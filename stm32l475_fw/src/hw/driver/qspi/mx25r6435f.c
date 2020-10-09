/*
 * mx25r6435f.c
 *
 *  Created on: 2020. 10. 9.
 *      Author: baram
 */




#include "qspi/mx25r6435f.h"


#if defined(_USE_HW_QSPI) && HW_QSPI_DRIVER == MX25R6435F

#define _FLASH_SIZE       0x800000    /* 64    MBits => 64MBytes */
#define _BLOCK_SIZE       0x10000     /* 128   blocks of 64KBytes */
#define _SECTOR_SIZE      0x1000      /* 2048  subsectors of 4kBytes */
#define _PAGE_SIZE        0x100       /* 32768 pages of 256 bytes */


#define QSPI_OK                 ((uint8_t)0x00)
#define QSPI_ERROR              ((uint8_t)0x01)
#define QSPI_BUSY               ((uint8_t)0x02)
#define QSPI_NOT_SUPPORTED      ((uint8_t)0x04)
#define QSPI_SUSPENDED          ((uint8_t)0x08)

#define QSPI_QUAD_DISABLE       0x0
#define QSPI_QUAD_ENABLE        0x1

#define QSPI_HIGH_PERF_DISABLE  0x0
#define QSPI_HIGH_PERF_ENABLE   0x1




static bool init(void);
static bool getID(qspi_info_t *p_info);
static bool getInfo(qspi_info_t *p_info);
static bool reset(void);
static bool read(uint8_t *p_data, uint32_t addr, uint32_t length);
static bool write(uint8_t *p_data, uint32_t addr, uint32_t length);
static bool eraseBlock(uint32_t block_addr);
static bool eraseSector(uint32_t sector_addr);
static bool eraseChip(void);
static bool getStatus(void);
static bool enableMemoryMappedMode(void);
static bool disableMemoryMappedMode(void);

static uint32_t getFlashSize(void);
static uint32_t getSectorSize(void);
static uint32_t getBlockSize(void);
static uint32_t getPageSize(void);


static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi_);
static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi_);
static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi_, uint32_t Timeout);
static uint8_t QSPI_QuadMode(QSPI_HandleTypeDef *hqspi_, uint8_t Operation);
static uint8_t QSPI_HighPerfMode(QSPI_HandleTypeDef *hqspi_, uint8_t Operation);


static QSPI_HandleTypeDef hqspi;



bool mx25r6435fInitDriver(qspi_driver_t *p_driver)
{
  p_driver->init = init;
  p_driver->getID = getID;
  p_driver->getInfo = getInfo;
  p_driver->reset = reset;
  p_driver->read = read;
  p_driver->write = write;
  p_driver->eraseBlock = eraseBlock;
  p_driver->eraseSector = eraseSector;
  p_driver->eraseChip = eraseChip;
  p_driver->getStatus = getStatus;
  p_driver->enableMemoryMappedMode = enableMemoryMappedMode;
  p_driver->disableMemoryMappedMode = disableMemoryMappedMode;
  p_driver->getFlashSize = getFlashSize;
  p_driver->getBlockSize = getBlockSize;
  p_driver->getSectorSize = getSectorSize;
  p_driver->getPageSize = getPageSize;

  return true;
}

bool init(void)
{
  bool ret = true;


  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 2; /* QSPI clock = 80MHz / (ClockPrescaler+1) = 26.67MHz */
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = POSITION_VAL(_FLASH_SIZE) - 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }


  if (QSPI_ResetMemory(&hqspi) != QSPI_OK)
  {
    return false;
  }

  if (QSPI_QuadMode(&hqspi, QSPI_QUAD_ENABLE) != QSPI_OK)
  {
    return false;
  }

  if (QSPI_HighPerfMode(&hqspi, QSPI_HIGH_PERF_ENABLE) != QSPI_OK)
  {
    return false;
  }

  /* Re-configure the clock for the high performance mode */
  hqspi.Init.ClockPrescaler = 1; /* QSPI clock = 80MHz / (ClockPrescaler+1) = 40MHz */

  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    return false;
  }

  return ret;
}


bool getID(qspi_info_t *p_info)
{
  bool ret = true;

  if (getInfo(p_info) != true)
  {
    return false;
  }

  if (p_info->device_id[0] != 0xC2 || p_info->device_id[1] != 0x28)
  {
    ret = false;
  }

  return ret;
}

bool getInfo(qspi_info_t *p_info)
{
  bool ret = true;


  p_info->FlashSize          = _FLASH_SIZE;
  p_info->EraseSectorSize    = _SECTOR_SIZE;
  p_info->EraseSectorsNumber = (_FLASH_SIZE/_SECTOR_SIZE);
  p_info->ProgPageSize       = _PAGE_SIZE;
  p_info->ProgPagesNumber    = (_FLASH_SIZE/_PAGE_SIZE);


  QSPI_CommandTypeDef sCommand;

  /* Initialize the read command */
  sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction        = READ_ID_CMD;
  sCommand.AddressMode        = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode           = QSPI_DATA_1_LINE;
  sCommand.DummyCycles        = 0;
  sCommand.NbData             = 2;
  sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  memset(p_info->device_id, 0, sizeof(p_info->device_id));

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, p_info->device_id, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }


  return ret;
}

bool reset(void)
{
  bool ret = true;
  QSPI_CommandTypeDef sCommand;


  //-- Reset Enable
  //
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;



  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return false;
  }

  delay(40);

  return ret;
}

bool read(uint8_t *p_data, uint32_t addr, uint32_t length)
{
  bool ret = true;
  QSPI_CommandTypeDef sCommand;


  /* Initialize the read command */
  sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction        = QUAD_INOUT_READ_CMD;
  sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
  sCommand.Address            = addr;
  sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.DataMode           = QSPI_DATA_4_LINES;
  sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  sCommand.NbData             = length;
  sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, 100) != HAL_OK)
  {
    return false;
  }

  /* Reception of the data */
  if (HAL_QSPI_Receive(&hqspi, p_data, 100) != HAL_OK)
  {
    return false;
  }

  return ret;
}

bool writePage(uint32_t addr, uint32_t data_addr)
{
  bool ret = true;
  QSPI_CommandTypeDef sCommand;


  /* Initialize the program command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = QUAD_PAGE_PROG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_4_LINES;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sCommand.Address = addr;
  sCommand.NbData  = _PAGE_SIZE;

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return false;
  }

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Transmission of the data */
  if (HAL_QSPI_Transmit(&hqspi, (uint8_t *)data_addr, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Configure automatic polling mode to wait for end of program */
  if (QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return false;
  }

  return ret;
}

bool write(uint8_t *p_data, uint32_t addr, uint32_t length)
{
  bool ret = true;
  uint32_t index;
  uint32_t write_length;
  uint32_t write_addr;
  uint32_t buf_mem[_PAGE_SIZE];
  uint8_t  *buf;
  uint32_t offset;


  index = 0;
  buf = (uint8_t *)buf_mem;
  offset = addr%_PAGE_SIZE;

  if (offset != 0 || length < _PAGE_SIZE)
  {
    write_addr = addr - offset;
    //memcpy(&buf[0], (void *)write_addr, _PAGE_SIZE);
    read(&buf[0], write_addr, _PAGE_SIZE);
    memcpy(&buf[offset], &p_data[0], constrain(_PAGE_SIZE-offset, 0, length));

    ret = writePage(write_addr, (uint32_t)buf);
    if (ret != true)
    {
      return false;
    }

    if (length < _PAGE_SIZE)
    {
      index += length;
    }
    else
    {
      index += (_PAGE_SIZE - offset);
    }
  }


  while(index < length)
  {
    write_length = constrain(length - index, 0, _PAGE_SIZE);

    ret = writePage(addr + index, (uint32_t)&p_data[index]);
    if (ret != true)
    {
      ret = false;
      break;
    }

    index += write_length;

    if ((length - index) > 0 && (length - index) < _PAGE_SIZE)
    {
      offset = length - index;
      write_addr = addr + index;
      //memcpy(&buf[0], (void *)write_addr, _PAGE_SIZE);
      read(&buf[0], write_addr, _PAGE_SIZE);
      memcpy(&buf[0], &p_data[index], offset);

      ret = writePage(write_addr, (uint32_t)buf);
      if (ret != true)
      {
        return false;
      }
      break;
    }
  }

  return ret;
}

bool eraseBlock(uint32_t block_addr)
{
  bool ret = true;

  QSPI_CommandTypeDef sCommand;

  /* Initialize the erase command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = BLOCK_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = block_addr;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return false;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(&hqspi, MX25R6435F_BLOCK_ERASE_MAX_TIME) != QSPI_OK)
  {
    return false;
  }

  return ret;
}

bool eraseSector(uint32_t sector_addr)
{
  bool ret = true;
  QSPI_CommandTypeDef sCommand;


  /* Initialize the erase command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = SECTOR_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_1_LINE;
  sCommand.AddressSize       = QSPI_ADDRESS_24_BITS;
  sCommand.Address           = sector_addr;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return false;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  return ret;
}

bool eraseChip(void)
{
  bool ret = true;
  QSPI_CommandTypeDef sCommand;

  /* Initialize the erase command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = CHIP_ERASE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return false;
  }

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return false;
  }

  /* Configure automatic polling mode to wait for end of erase */
  if (QSPI_AutoPollingMemReady(&hqspi, MX25R6435F_CHIP_ERASE_MAX_TIME) != QSPI_OK)
  {
    return false;
  }

  return ret;
}

bool getStatus(void)
{
  bool ret = true;

  return ret;
}

bool enableMemoryMappedMode(void)
{
  bool ret = true;
  QSPI_CommandTypeDef      sCommand;
  QSPI_MemoryMappedTypeDef sMemMappedCfg;

  /* Configure the command for the read instruction */
  sCommand.InstructionMode    = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction        = QUAD_INOUT_READ_CMD;
  sCommand.AddressMode        = QSPI_ADDRESS_4_LINES;
  sCommand.AddressSize        = QSPI_ADDRESS_24_BITS;
  sCommand.AlternateByteMode  = QSPI_ALTERNATE_BYTES_4_LINES;
  sCommand.AlternateBytesSize = QSPI_ALTERNATE_BYTES_8_BITS;
  sCommand.AlternateBytes     = MX25R6435F_ALT_BYTES_NO_PE_MODE;
  sCommand.DataMode           = QSPI_DATA_4_LINES;
  sCommand.DummyCycles        = MX25R6435F_DUMMY_CYCLES_READ_QUAD;
  sCommand.DdrMode            = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle   = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode           = QSPI_SIOO_INST_EVERY_CMD;

  /* Configure the memory mapped mode */
  sMemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;

  if (HAL_QSPI_MemoryMapped(&hqspi, &sCommand, &sMemMappedCfg) != HAL_OK)
  {
    return false;
  }


  return ret;
}

bool disableMemoryMappedMode(void)
{
  bool ret = true;

  return ret;
}

uint32_t getFlashSize(void)
{
  return _FLASH_SIZE;
}

uint32_t getSectorSize(void)
{
  return _SECTOR_SIZE;
}

uint32_t getBlockSize(void)
{
  return _BLOCK_SIZE;
}

uint32_t getPageSize(void)
{
  return _PAGE_SIZE;
}


static uint8_t QSPI_ResetMemory(QSPI_HandleTypeDef *hqspi_)
{
  QSPI_CommandTypeDef sCommand;

  /* Initialize the reset enable command */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = RESET_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  /* Send the command */
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Send the reset memory command */
  sCommand.Instruction = RESET_MEMORY_CMD;
  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait the memory is ready */
  if (QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_AutoPollingMemReady(QSPI_HandleTypeDef *hqspi_, uint32_t Timeout)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Configure automatic polling mode to wait for memory ready */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  sConfig.Match           = 0;
  sConfig.Mask            = MX25R6435F_SR_WIP;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, Timeout) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_QuadMode(QSPI_HandleTypeDef *hqspi_, uint8_t Operation)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg;

  /* Read status register */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_QUAD_ENABLE)
  {
    SET_BIT(reg, MX25R6435F_SR_QE);
  }
  else
  {
    CLEAR_BIT(reg, MX25R6435F_SR_QE);
  }

  sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Wait that memory is ready */
  if (QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the configuration has been correctly done */
  sCommand.Instruction = READ_STATUS_REG_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, &reg, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if ((((reg & MX25R6435F_SR_QE) == 0) && (Operation == QSPI_QUAD_ENABLE)) ||
      (((reg & MX25R6435F_SR_QE) != 0) && (Operation == QSPI_QUAD_DISABLE)))
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_WriteEnable(QSPI_HandleTypeDef *hqspi_)
{
  QSPI_CommandTypeDef     sCommand;
  QSPI_AutoPollingTypeDef sConfig;

  /* Enable write operations */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = WRITE_ENABLE_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_NONE;
  sCommand.DummyCycles       = 0;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Configure automatic polling mode to wait for write enabling */
  sConfig.Match           = MX25R6435F_SR_WEL;
  sConfig.Mask            = MX25R6435F_SR_WEL;
  sConfig.MatchMode       = QSPI_MATCH_MODE_AND;
  sConfig.StatusBytesSize = 1;
  sConfig.Interval        = 0x10;
  sConfig.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  sCommand.Instruction    = READ_STATUS_REG_CMD;
  sCommand.DataMode       = QSPI_DATA_1_LINE;

  if (HAL_QSPI_AutoPolling(&hqspi, &sCommand, &sConfig, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}

static uint8_t QSPI_HighPerfMode(QSPI_HandleTypeDef *hqspi_, uint8_t Operation)
{
  QSPI_CommandTypeDef sCommand;
  uint8_t reg[3];

  /* Read status register */
  sCommand.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  sCommand.Instruction       = READ_STATUS_REG_CMD;
  sCommand.AddressMode       = QSPI_ADDRESS_NONE;
  sCommand.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  sCommand.DataMode          = QSPI_DATA_1_LINE;
  sCommand.DummyCycles       = 0;
  sCommand.NbData            = 1;
  sCommand.DdrMode           = QSPI_DDR_MODE_DISABLE;
  sCommand.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  sCommand.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Read configuration registers */
  sCommand.Instruction = READ_CFG_REG_CMD;
  sCommand.NbData      = 2;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, &(reg[1]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Enable write operations */
  if (QSPI_WriteEnable(&hqspi) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Activate/deactivate the Quad mode */
  if (Operation == QSPI_HIGH_PERF_ENABLE)
  {
    SET_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }
  else
  {
    CLEAR_BIT(reg[2], MX25R6435F_CR2_LH_SWITCH);
  }

  sCommand.Instruction = WRITE_STATUS_CFG_REG_CMD;
  sCommand.NbData      = 3;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  /* Wait that memory is ready */
  if (QSPI_AutoPollingMemReady(&hqspi, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != QSPI_OK)
  {
    return QSPI_ERROR;
  }

  /* Check the configuration has been correctly done */
  sCommand.Instruction = READ_CFG_REG_CMD;
  sCommand.NbData      = 2;

  if (HAL_QSPI_Command(&hqspi, &sCommand, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, &(reg[0]), HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return QSPI_ERROR;
  }

  if ((((reg[1] & MX25R6435F_CR2_LH_SWITCH) == 0) && (Operation == QSPI_HIGH_PERF_ENABLE)) ||
      (((reg[1] & MX25R6435F_CR2_LH_SWITCH) != 0) && (Operation == QSPI_HIGH_PERF_DISABLE)))
  {
    return QSPI_ERROR;
  }

  return QSPI_OK;
}


void HAL_QSPI_MspInit(QSPI_HandleTypeDef* qspiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspInit 0 */

  /* USER CODE END QUADSPI_MspInit 0 */
    /* QUADSPI clock enable */
    __HAL_RCC_QSPI_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**QUADSPI GPIO Configuration
    PE10     ------> QUADSPI_CLK
    PE11     ------> QUADSPI_NCS
    PE12     ------> QUADSPI_BK1_IO0
    PE13     ------> QUADSPI_BK1_IO1
    PE14     ------> QUADSPI_BK1_IO2
    PE15     ------> QUADSPI_BK1_IO3
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* USER CODE BEGIN QUADSPI_MspInit 1 */

  /* USER CODE END QUADSPI_MspInit 1 */
  }
}

void HAL_QSPI_MspDeInit(QSPI_HandleTypeDef* qspiHandle)
{

  if(qspiHandle->Instance==QUADSPI)
  {
  /* USER CODE BEGIN QUADSPI_MspDeInit 0 */

  /* USER CODE END QUADSPI_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_QSPI_CLK_DISABLE();

    /**QUADSPI GPIO Configuration
    PE10     ------> QUADSPI_CLK
    PE11     ------> QUADSPI_NCS
    PE12     ------> QUADSPI_BK1_IO0
    PE13     ------> QUADSPI_BK1_IO1
    PE14     ------> QUADSPI_BK1_IO2
    PE15     ------> QUADSPI_BK1_IO3
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN QUADSPI_MspDeInit 1 */

  /* USER CODE END QUADSPI_MspDeInit 1 */
  }
}

#endif
