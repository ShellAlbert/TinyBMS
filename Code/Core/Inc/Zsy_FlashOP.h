/*
 * Zsy_FlashOP.h
 *
 *  Created on: Dec 6, 2022
 *      Author: Administrator
 */

#ifndef INC_ZSY_FLASHOP_H_
#define INC_ZSY_FLASHOP_H_

//STM32F103CBT6, 128KB Flash, 20KB RAM
#define SECTOR_SIZE 1024 //1KB

#define FLASH_BASE_ADDR	0x08000000 //base address

#define FLASH_USER_ADDR_START		(FLASH_BASE_ADDR+SECTOR_SIZE*127)
#define FLASH_USER_ADDR_END		(FLASH_BASE_ADDR+SECTOR_SIZE*127)

extern int ZsyFlashErase(void);
extern int ZsyFlashWrite(unsigned int *pdata, unsigned int data_size);
extern int ZsyFlashRead(unsigned int *pdata, unsigned int data_size);

#endif /* INC_ZSY_FLASHOP_H_ */
