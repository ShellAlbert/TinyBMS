/*
 * Zsy_Utils.h
 *
 *  Created on: Dec 6, 2022
 *      Author: Administrator
 */

#ifndef INC_ZSY_UTILS_H_
#define INC_ZSY_UTILS_H_
#include "stm32f1xx_hal.h"
extern int Zsy_MapVol2SoC(float voltage);

extern float Zsy_GetPT100Resistance(uint8_t num);
extern float Zsy_PT100_to_Temperature(float fPT100Res);
#endif /* INC_ZSY_UTILS_H_ */
