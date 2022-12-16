/*
 * Zsy_Utils.c
 *
 *  Created on: Dec 6, 2022
 *      Author: Administrator
 */
#include "main.h"
#include "Zsy_Utils.h"
extern ADC_HandleTypeDef hadc1;
int
Zsy_MapVol2SoC (float voltage)
{
  int iSoC;
  if (voltage >= 4.2)
    {
      iSoC = 1000;
    }
  else if (voltage >= 4.06 && voltage < 4.2)
    {
      iSoC = 900;
    }
  else if (voltage >= 3.98 && voltage < 4.06)
    {
      iSoC = 800;
    }
  else if (voltage >= 3.92 && voltage < 3.98)
    {
      iSoC = 700;
    }
  else if (voltage >= 3.87 && voltage < 3.92)
    {
      iSoC = 600;
    }
  else if (voltage >= 3.82 && voltage < 3.87)
    {
      iSoC = 500;
    }
  else if (voltage >= 3.79 && voltage < 3.82)
    {
      iSoC = 400;
    }
  else if (voltage >= 3.77 && voltage < 3.79)
    {
      iSoC = 300;
    }
  else if (voltage >= 3.74 && voltage < 3.77)
    {
      iSoC = 200;
    }
  else if (voltage >= 3.68 && voltage < 3.74)
    {
      iSoC = 100;
    }
  else if (voltage >= 3.45 && voltage < 3.68)
    {
      iSoC = 50;
    }
  else if (voltage >= 3.0 && voltage < 3.45)
    {
      iSoC = 0;
    }
  else
    {
      iSoC = 0;
    }
  return iSoC;
}
float
Zsy_GetPT100Resistance (uint8_t num)
{
  float fResistance;
  switch (num)
    {
    case 0:
      //SELA=0, SELB=0, X0.
      HAL_GPIO_WritePin (GPIOB, SELA_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (GPIOB, SELB_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      //SELA=1, SELB=0, X1.
      HAL_GPIO_WritePin (GPIOB, SELA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin (GPIOB, SELB_Pin, GPIO_PIN_RESET);
      break;
    case 2:
      //SELA=0, SELB=1, X2.
      HAL_GPIO_WritePin (GPIOB, SELA_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin (GPIOB, SELB_Pin, GPIO_PIN_SET);
      break;
    case 3:
      //SELA=1, SELB=1, X3.
      HAL_GPIO_WritePin (GPIOB, SELA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin (GPIOB, SELB_Pin, GPIO_PIN_SET);
      break;
    default:
      break;
    }
  HAL_Delay (500);
  HAL_ADCEx_Calibration_Start (&hadc1);
  HAL_ADC_Start (&hadc1);
  HAL_ADC_PollForConversion (&hadc1, 50);
  if (HAL_IS_BIT_SET(HAL_ADC_GetState (&hadc1), HAL_ADC_STATE_REG_EOC))
    {
      float fVoltage = 0.0f;
      uint32_t adcValue = HAL_ADC_GetValue (&hadc1);
      fVoltage = adcValue / 4096.0f * 3.3f;
      if (fVoltage > 0)
	{
	  float fDifference;
	  //ADC=2.08V
	  //2.08V-1.25V(DC offset)=0.83V
	  //0.83V/23.7 (Gain) = 0.035V
	  //0.035V+0.110V(fixed voltage of another arm of the bridge)= 0.145V
	  //R=V/I=0.145V/[(2.5V-0.145V)/2.15K]=132R.
	  //since the typical Ron of RS2255XN is 24R,
	  //so the real PT100 resistor is R-24R.
	  fDifference = (fVoltage - 1.25f) / 23.7f + 0.110f;
	  fResistance = fDifference / ((2.5f - fDifference) / 2150);
	  fResistance -= 24;
	}
    }
  return fResistance;
}

//According to PT100 Resistance Table
//0 degree Celsius, PT100=100.00R
//1 degree Celsius, PT100=99.61R
//So from 0 to 1 the Ratio=100.00-99.61=0.39

//10.0 degree Celsius, PT100=103.90R
//10.1 degree Celsius, PT100=104.29R
//So from 10.0 to 10.1 the Ratio=104.29-103.9=0.39R

//So A Fast way to convert resistance to temperature is:
//when temperature increase 1.0 degree Celsius, the PT100 resistance increase 0.39R.

//-10.0 degree Celsius, PT100=96.09R
//-11.0 degree Celsius, PT100=95.69R
//So the Ratio is 96.09R-95.69R=0.4R
//For temperature below zero,
//when temperature decrease -1 degree Celsius, PT100 resistance decrease 0.4R.
float Zsy_PT100_to_Temperature(float fPT100Res)
{
  float fTempC;
  fTempC=(fPT100Res-100.0f)/0.39f;
  return fTempC;
}
