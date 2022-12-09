/*
 * Zsy_Utils.c
 *
 *  Created on: Dec 6, 2022
 *      Author: Administrator
 */

#include "Zsy_Utils.h"
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
