/*
 * Zsy_FlashOP.c
 *
 *  Created on: Dec 6, 2022
 *      Author: Administrator
 *      //https://blog.csdn.net/lwb450921/article/details/125022862?spm=1001.2014.3001.5502
 */

#include "Zsy_FlashOP.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash.h"
static FLASH_EraseInitTypeDef  tEraseInitStructure;

int ZsyFlashErase(void)
{
  uint32_t tPageErr;
  tEraseInitStructure.TypeErase=FLASH_TYPEERASE_PAGES;
  tEraseInitStructure.PageAddress=FLASH_USER_ADDR_START;
  tEraseInitStructure.NbPages=1;
  if(HAL_FLASHEx_Erase(&tEraseInitStructure, &tPageErr)!=HAL_OK)
   {
      //error.
      HAL_FLASH_Lock();
      return -1;
    }
  return 0;
}
int ZsyFlashWrite(unsigned int *pdata, unsigned int data_size)
{
  unsigned int i=0;
  unsigned int addr=0;
  HAL_FLASH_Unlock();
  if(ZsyFlashErase()<0)
   {
      //error.
      return -1;
    }
  while(addr<data_size)
    {
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_USER_ADDR_START+addr, pdata[i])==HAL_OK)
	{
	     addr+=4; //move to next address.
	     i++;
	}else{
	    //error occurred.
	    HAL_FLASH_Lock();
	    return -1;
	    break;
	}
    }
  HAL_FLASH_Lock();
  return 0;
}
int ZsyFlashRead(unsigned int *pdata, unsigned int data_size)
{
  unsigned int i=0;
  unsigned int addr=0;
  while(addr<data_size)
    {
	pdata[i++]=*(__IO unsigned int *)(FLASH_USER_ADDR_START+addr);
	addr+=4;
    }
  return 0;
}
