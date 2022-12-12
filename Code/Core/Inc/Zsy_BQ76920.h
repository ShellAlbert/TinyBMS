/*
 * Zsy_BQ76920.h
 *
 *  Created on: Dec 7, 2022
 *      Author: Administrator
 */

#ifndef INC_ZSY_BQ76920_H_
#define INC_ZSY_BQ76920_H_
#include "stm32f1xx_hal.h"
#include <stdio.h>

#define RSENSE				1.5// mOhm

typedef struct
{
  float cellVoltage[4]; // Cells voltage in mV unit.
  int adcGain; 		// ADCGain in uV/LSB unit.
  int adcOffset; 	// ADCOFFSET in mV unit.

  float batVoltage;   // The total voltage of battery pack in mV unit.
  float batCurrent; // The current of battery pack in mA unit.

  float dieTemp; // die Temperature.

  //State of Charge.
  //range:0~1000, means 0%~100.0%
  uint32_t iSoC;

  //Coulomb Counter, accumulated AMP Second.
  uint32_t iAMPSec;
} BQ76920_DEV;
extern BQ76920_DEV gBQ76920Dev;

//begin to initial variables.
extern int
BQ76920_init (void);

// limit settings (for battery protection)
// minimum temperature degree Celsius at Discharge.
//maximum temperature degree Celsius at Discharge.
//minimum temperature degree Celsius at Charge.
//maximum temperature degree Celsius at Charge.
// °C
extern int
setTemperatureLimits (int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC);

//set short circuit protection current value with action latency at Discharge.
extern int
setShortCircuitProtection (long current_mA, int delay_us);

//set over current protection current value with action latency at Discharge.
extern int
setOvercurrentDischargeProtection (long current_mA, int delay_ms);

//set Cell Under voltage protection with latency.
extern int
setCellUndervoltageProtection (int voltage_mV, int delay_s);

//set Cell Over voltage protection with latency.
extern int
setCellOvervoltageProtection (int voltage_mV, int delay_s);

//Charging control
extern int
enableCharging (void);
extern int
disableCharging (void);

//Discharging control.
extern int
enableDischarging (void);
extern int
disableDischarging (void);

//Enable Balancing.
extern int
enableBalancing(void);
extern void
disableBalancing(void);

//get Die Temperature.
extern int
getDieTemperature(void);

//Enter SHIP mode.
extern int
enterSHIPMode (void);

//dump All registers to UART.
extern int
dumpRegisters (void);

//check Status Register to see if BMS has errors.
extern int
checkStatus(void);

//update current.
//if bIgnoreCCReadyFlag=1, the current is read independent of an interrupt.
//indicating the availability of a new CC reading.
extern int
updateCurrent(int8_t bIgnoreCCReadyFlag);

//update voltage.
extern int
updateVoltages(void);


#endif /* INC_ZSY_BQ76920_H_ */
