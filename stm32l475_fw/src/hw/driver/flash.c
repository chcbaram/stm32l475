/*
 * flash.c
 *
 *  Created on: 2020. 10. 9.
 *      Author: baram
 */




#include "flash.h"
#include "cmdif.h"
#ifdef _USE_HW_QSPI
#include "qspi.h"
#endif


#define FLASH_MAX_SECTOR          256
#define FLASH_WRITE_BLOCK_LEN     8


typedef struct
{
  uint32_t bank;
  uint32_t addr;
  uint32_t length;
} flash_tbl_t;



const flash_tbl_t flash_tbl_bank1 = {FLASH_BANK_1, 0x08000000, 2*1024};
const flash_tbl_t flash_tbl_bank2 = {FLASH_BANK_2, 0x08080000, 2*1024};




void flashCmdifInit(void);
void flashCmdif(void);





bool flashInit(void)
{

#ifdef _USE_HW_CMDIF
  flashCmdifInit();
#endif

  return true;
}

bool flashErase(uint32_t addr, uint32_t length)
{
  bool ret = false;

  int32_t start_sector = -1;
  int32_t end_sector = -1;
  uint32_t banks;
  const flash_tbl_t *flash_tbl;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiErase(addr - qspiGetAddr(), length);
    return ret;
  }
#endif


  HAL_FLASH_Unlock();

  for (banks = 0; banks < 2; banks++)
  {
    start_sector = -1;
    end_sector = -1;

    if (banks == 0)
    {
      flash_tbl = &flash_tbl_bank1;
    }
    else
    {
      flash_tbl = &flash_tbl_bank2;
    }

    for (int i=0; i<FLASH_MAX_SECTOR; i++)
    {
      bool update = false;
      uint32_t start_addr;
      uint32_t end_addr;


      start_addr = flash_tbl->addr + (flash_tbl->length * i);
      end_addr   = start_addr      +  flash_tbl->length - 1;

      if (start_addr >= addr && start_addr < (addr+length))
      {
        update = true;
      }
      if (end_addr >= addr && end_addr < (addr+length))
      {
        update = true;
      }

      if (addr >= start_addr && addr <= end_addr)
      {
        update = true;
      }
      if ((addr+length-1) >= start_addr && (addr+length-1) <= end_addr)
      {
        update = true;
      }


      if (update == true)
      {
        if (start_sector < 0)
        {
          start_sector = i;
        }
        end_sector = i;
      }
    }

    if (start_sector >= 0)
    {
      FLASH_EraseInitTypeDef EraseInit;
      uint32_t SectorError;
      HAL_StatusTypeDef status;


      EraseInit.Page         = start_sector;
      EraseInit.NbPages      = (end_sector - start_sector) + 1;
      EraseInit.TypeErase    = FLASH_TYPEERASE_PAGES;
      EraseInit.Banks        = flash_tbl->bank;

      status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
      if (status == HAL_OK)
      {
        ret = true;
      }
    }
  }

  HAL_FLASH_Lock();

  return ret;
}

bool flashWrite(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint32_t index;
  uint32_t write_length;
  uint32_t write_addr;
  uint8_t *buf;
  uint64_t buf_data;
  uint32_t offset;
  HAL_StatusTypeDef status;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiWrite(addr - qspiGetAddr(), p_data, length);
    return ret;
  }
#endif


  HAL_FLASH_Unlock();

  index = 0;
  offset = addr%FLASH_WRITE_BLOCK_LEN;
  buf = (uint8_t *)&buf_data;

  if (offset != 0 || length < FLASH_WRITE_BLOCK_LEN)
  {
    write_addr = addr - offset;
    memcpy(&buf[0], (void *)write_addr, FLASH_WRITE_BLOCK_LEN);
    memcpy(&buf[offset], &p_data[0], constrain(FLASH_WRITE_BLOCK_LEN-offset, 0, length));

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr, buf_data);
    if (status != HAL_OK)
    {
      return false;
    }

    if (length < FLASH_WRITE_BLOCK_LEN)
    {
      index += length;
    }
    else
    {
      index += (FLASH_WRITE_BLOCK_LEN - offset);
    }
  }


  while(index < length)
  {
    write_length = constrain(length - index, 0, FLASH_WRITE_BLOCK_LEN);

    memcpy(&buf[0], (void *)&p_data[index], FLASH_WRITE_BLOCK_LEN);

    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + index, buf_data);
    if (status != HAL_OK)
    {
      ret = false;
      break;
    }

    index += write_length;

    if ((length - index) > 0 && (length - index) < FLASH_WRITE_BLOCK_LEN)
    {
      offset = length - index;
      write_addr = addr + index;
      memcpy(&buf[0], (void *)write_addr, FLASH_WRITE_BLOCK_LEN);
      memcpy(&buf[0], &p_data[index], offset);

      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, write_addr, buf_data);
      if (status != HAL_OK)
      {
        return false;
      }
      break;
    }
  }

  HAL_FLASH_Lock();

  return ret;
}

bool flashRead(uint32_t addr, uint8_t *p_data, uint32_t length)
{
  bool ret = true;
  uint8_t *p_byte = (uint8_t *)addr;


#ifdef _USE_HW_QSPI
  if (addr >= qspiGetAddr() && addr < (qspiGetAddr() + qspiGetLength()))
  {
    ret = qspiRead(addr - qspiGetAddr(), p_data, length);
    return ret;
  }
#endif

  for (int i=0; i<length; i++)
  {
    p_data[i] = p_byte[i];
  }

  return ret;
}





#ifdef _USE_HW_CMDIF
void flashCmdifInit(void)
{
  cmdifAdd("flash", flashCmdif);
}

void flashCmdif(void)
{
  bool ret = true;
  uint32_t i;
  uint32_t addr;
  uint32_t length;
  uint8_t  data;
  uint32_t pre_time;
  bool flash_ret;


  if (cmdifGetParamCnt() == 1)
  {
    if(cmdifHasString("info", 0) == true)
    {
      cmdifPrintf("flash addr  : 0x%X\n", 0x8000000);
      cmdifPrintf("qspi  addr  : 0x%X\n", 0x90000000);
    }
    else
    {
      ret = false;
    }
  }
  else if (cmdifGetParamCnt() == 3)
  {
    if(cmdifHasString("read", 0) == true)
    {
      addr   = (uint32_t)cmdifGetParam(1);
      length = (uint32_t)cmdifGetParam(2);

      for (i=0; i<length; i++)
      {
        flash_ret = flashRead(addr+i, &data, 1);

        if (flash_ret == true)
        {
          cmdifPrintf( "addr : 0x%X\t 0x%02X\n", addr+i, data);
        }
        else
        {
          cmdifPrintf( "addr : 0x%X\t Fail\n", addr+i);
        }
      }
    }
    else if(cmdifHasString("erase", 0) == true)
    {
      addr   = (uint32_t)cmdifGetParam(1);
      length = (uint32_t)cmdifGetParam(2);

      pre_time = millis();
      flash_ret = flashErase(addr, length);

      cmdifPrintf( "addr : 0x%X\t len : %d %d ms\n", addr, length, (millis()-pre_time));
      if (flash_ret)
      {
        cmdifPrintf("OK\n");
      }
      else
      {
        cmdifPrintf("FAIL\n");
      }
    }
    else if(cmdifHasString("write", 0) == true)
    {
      addr = (uint32_t)cmdifGetParam(1);
      data = (uint8_t )cmdifGetParam(2);

      pre_time = millis();
      flash_ret = flashWrite(addr, &data, 1);

      cmdifPrintf( "addr : 0x%X\t 0x%02X %dms\n", addr, data, millis()-pre_time);
      if (flash_ret)
      {
        cmdifPrintf("OK\n");
      }
      else
      {
        cmdifPrintf("FAIL\n");
      }
    }
    else
    {
      ret = false;
    }
  }
  else
  {
    ret = false;
  }


  if (ret == false)
  {
    cmdifPrintf( "flash info\n");
    cmdifPrintf( "flash read  [addr] [length]\n");
    cmdifPrintf( "flash erase [addr] [length]\n");
    cmdifPrintf( "flash write [addr] [data]\n");
  }

}
#endif
