/*
 * Zsy_BQ76920.c
 *
 *  Created on: Dec 7, 2022
 *      Author: Administrator
 *      https://blog.csdn.net/qq_43225938/article/details/84142147
 */
#include "Zsy_BQ76920.h"
#include "Zsy_Registers.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
//BQ7692003PW, 3~5 Cells, I2C Address(7-Bit)=0x08, LDO=3.3V, CRC=YES
#define BQ76920_ADDR		(0x08<<1)
//For I2C Read, LSB must be 1.  BQ76920_ADDR|0x01
//For I2C Write, LSB must be 0.  BQ76920_ADDR
#define BQ76920_ADDR_RD	(BQ76920_ADDR|0x01)
#define BQ76920_ADDR_WR	(BQ76920_ADDR)
// maps for settings in protection registers
const int SCD_delay_setting[4] =
  { 70, 100, 200, 400 }; // us
const int SCD_threshold_setting[8] =
  { 44, 67, 89, 111, 133, 155, 178, 200 }; // mV

const int OCD_delay_setting[8] =
  { 8, 20, 40, 80, 160, 320, 640, 1280 }; // ms
const int OCD_threshold_setting[16] =
  { 17, 22, 28, 33, 39, 44, 50, 56, 61, 67, 72, 78, 83, 89, 94, 100 };  // mV

const int UV_delay_setting[4] =
  { 1, 4, 8, 16 };  // s
const int OV_delay_setting[4] =
  { 1, 2, 4, 8 };   // s

//global variable.
BQ76920_DEV gBQ76920Dev;

//SCL:PB6, SDA:PB9
#define GPIO_PORT_I2C 	GPIOB
#define I2C_SCL_PIN		GPIO_PIN_6
#define I2C_SDA_PIN		GPIO_PIN_9

//SCL outputs 1 and 0.
#define I2C_SCL_1()		GPIO_PORT_I2C->BSRR=I2C_SCL_PIN //SCL=1
#define I2C_SCL_0()		GPIO_PORT_I2C->BSRR=(uint32_t)I2C_SCL_PIN<<16u //SCL=0

//SDA outputs 1 and 0.
#define I2C_SDA_1()		GPIO_PORT_I2C->BSRR=GPIO_PIN_9 //SDA=1
#define I2C_SDA_0()		GPIO_PORT_I2C->BSRR=(uint32_t)GPIO_PIN_9<<16u //SDA=0

//input direction to read.
#define I2C_SDA_READ()	(GPIO_PORT_I2C->IDR&GPIO_PIN_9) //read SDA pin.
#define I2C_SCL_READ()	(GPIO_PORT_I2C->IDR&I2C_SCL_PIN) //read SCL pin.

static void
Zsy_i2c_delay (void)
{
  uint8_t i;
  for (i = 0; i < 255; i++)
    {
      ;
    }
}
static void
Zsy_i2c_start (void)
{
  //SDA 1->0 generate falling edge while SCL=1.
  I2C_SDA_1();
  I2C_SCL_1();
  Zsy_i2c_delay ();
  I2C_SDA_0();
  Zsy_i2c_delay ();

  I2C_SCL_0();
  Zsy_i2c_delay ();
}
static void
Zsy_i2c_stop (void)
{
  //SDA 0->1 to generate a raising edge while SCL=1.
  I2C_SDA_0();
  I2C_SCL_1();
  Zsy_i2c_delay ();
  I2C_SDA_1();
  Zsy_i2c_delay ();
}
static void
Zsy_i2c_txbyte (uint8_t txByte)
{
  uint8_t i;
  for (i = 0; i < 8; i++)
    {
      if (txByte & 0x80)
	I2C_SDA_1();
      else
	I2C_SDA_0();
      /////////////////////////////
      Zsy_i2c_delay ();
      I2C_SCL_1();
      Zsy_i2c_delay ();
      I2C_SCL_0();
      //////////////////////////////////////
      if (i == 7)
	{
	  I2C_SDA_1(); // release bus.
	}
      /////////////////////////////////////
      txByte <<= 1; //left shift 1 bit.
      Zsy_i2c_delay ();
    }
}
static uint8_t
Zsy_i2c_rxByte (void)
{
  uint8_t i;
  uint8_t rdByte = 0;
  for (i = 0; i < 8; i++)
    {
      rdByte <<= 1; //left shift 1 bit.
      I2C_SCL_1();
      Zsy_i2c_delay ();
      if (I2C_SDA_READ())
	{
	  rdByte++;
	}
      I2C_SCL_0();
      Zsy_i2c_delay ();
    }
  return rdByte;
}
static uint8_t
Zsy_i2c_waitAck (void)
{
  uint8_t ret;
  I2C_SDA_1(); //release SDA line.
  Zsy_i2c_delay ();
  I2C_SCL_1(); //CPU drive SCL=1,then slave device will give ACK.
  Zsy_i2c_delay ();

  ///////////////////////////////
  if (I2C_SDA_READ())  //CPU read SDA line.
    {
      ret = 1;
    }
  else
    {
      ret = 0;
    }
  I2C_SCL_0();
  Zsy_i2c_delay ();
  return ret;
}
static void
Zsy_i2c_Ack (void)
{
  I2C_SDA_0();  //CPU drive SDA=1.
  Zsy_i2c_delay ();
  /////////////////////////////CPU generate one clk.
  I2C_SCL_1();
  Zsy_i2c_delay ();
  /////////////////////////////
  I2C_SCL_0();
  Zsy_i2c_delay ();
  /////////////////////////////
  I2C_SDA_1();
  return;
}
static void
Zsy_i2c_NoAck (void)
{
  I2C_SDA_1(); //CPU drive SDA=1.
  Zsy_i2c_delay ();
  //////////////////////////CPU generate one clk.
  I2C_SCL_1();
  Zsy_i2c_delay ();
  //////////////////////////
  I2C_SCL_0();
  Zsy_i2c_delay ();
  return;
}

//CRC8=x8+x2+x+1
//https://blog.csdn.net/Btefuliycomputer/article/details/101391042
uint8_t
Zsy_CRC8 (uint8_t *pstr, uint32_t length)
{
  uint8_t *p = pstr;
  int len = length;
  uint8_t crc = 0x0; //the initial value is 0 in BQ76920 Datasheet.
  uint8_t i = 0;
  while (len--)
    {
      crc ^= *p++;
      for (i = 8; i > 0; --i)
	{
	  if (crc & 0x80)
	    {
	      crc = (crc << 1) ^ 0x07;
	    }
	  else
	    {
	      crc = (crc << 1);
	    }
	}
    }
  return crc;
}

//Single byte write transaction.
static void
Zsy_BQ76920_SingleByte_Write (uint8_t txRegister, uint8_t txData)
{
  //0. calculate CRC.
  uint8_t crc;
  uint8_t buffer[3];
  buffer[0] = BQ76920_ADDR_WR;
  buffer[1] = txRegister;
  buffer[2] = txData;
  crc = Zsy_CRC8 (buffer, sizeof(buffer));

  //1. Start.
  Zsy_i2c_start ();
  //2.Slave Address.
  Zsy_i2c_txbyte (BQ76920_ADDR_WR);
  //3.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //4.Register Address.
  Zsy_i2c_txbyte (txRegister);
  //5.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //6. Data.
  Zsy_i2c_txbyte (txData);
  //7.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //8.CRC.
  Zsy_i2c_txbyte (crc);
  //9.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //10.Stop.
  Zsy_i2c_stop ();
}
//Single byte read transaction.
static uint8_t
Zsy_BQ76920_SingleByte_Read (uint8_t rdRegister)
{
  uint8_t rdData = 0;
  uint8_t rdCRC = 0;
  //1. Start.
  Zsy_i2c_start ();
  //2.Slave Address.
  Zsy_i2c_txbyte (BQ76920_ADDR_RD);
  //3.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //4.Register Address.
  Zsy_i2c_txbyte (rdRegister);
  //5.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();

  //6. Repeated Start.
  Zsy_i2c_start ();
  //7. Slave Address.
  Zsy_i2c_txbyte (BQ76920_ADDR_RD);
  //8.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //9.Slave Drives Data.
  rdData = Zsy_i2c_rxByte ();
  //10.Wait slave ACK (should be 0).
  Zsy_i2c_waitAck ();
  //11.Slave Drives CRC.
  rdCRC = Zsy_i2c_rxByte ();
  //12.Master Drives NACK.
  Zsy_i2c_NoAck ();
  return rdData;
}
//begin to initial variables.
int
BQ76920_init (void)
{
  int8_t adcGain1, adcGain2;
  uint8_t i;
  for (i = 0; i < 4; i++)
    {
      gBQ76920Dev.cellVoltage[i] = 0;
    }
  //test I2C communication.
  //should be set to 0x19 according to the data sheet.
  Zsy_BQ76920_SingleByte_Write (CC_CFG, 0x19);
  if (Zsy_BQ76920_SingleByte_Read (CC_CFG) != 0x19)
    {
      //I2C communication failed.
      return -1;
    }
  //start to initial registers.
  //SYS_CTRL1:
  //TEMP_SEL: [3]=1, Store thermistor reading in TSx_HI and TSx_LO (all thermistor ports).
  //ADC_EN: [4]=1, Enable voltage and temperature ADC readings (also enables OV protection).
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL1, (0x1 << 3) | (0x1 << 4));

  //SYS_CTRL2:  Coulomb counter continuous operation enable command.
  //CC_EN: [6]=1, Enable CC continuous readings and ignore [CC_ONESHOT] state.
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL2, (0x1 << 6));

  //Read ADCOFFSET.
  //store in global variable for later use.
  gBQ76920Dev.adcOffset = Zsy_BQ76920_SingleByte_Read (ADCOFFSET);

  //Read ADCGain1, ADCGain2.
  //ADCGain1:[3],[2],  0000,1100=0xC0
  //ADCGain2:[7],[6],[5], 1110,0000=0xE0
  adcGain1 = (Zsy_BQ76920_SingleByte_Read (ADCGAIN1) & 0x0C) << 1;
  adcGain2 = (Zsy_BQ76920_SingleByte_Read (ADCGAIN2) & 0xE0) >> 5;
  //store in global variable for later use.
  gBQ76920Dev.adcGain = 365 + (int8_t) (adcGain1 | adcGain2);

  //https://github.com/LLL2542/BQ76920/blob/main/BQ76920.c
  //Set Protect1 register.
  //[7]:RSNS, 1 = OCD and SCD thresholds at upper input range.
  //[4:3]: SCD_D1, SCD_D0: Short circuit in discharge delay setting.
  //0x0=70us, 0x1=100us, 0x2=200us, 0x3=400us.
  //[2:0]: SCD_T2, SCD_T1, SCD_T0: Short circuit in discharge threshold setting.
  //0x0, 44mV (RSNS=1), 22mV (RSNS=0)
  //0x1, 67mV (RSNS=1), 33mV (RSNS=0)
  //0x2, 89mV (RSNS=1), 44mV (RSNS=0)
  //0x3, 111mV (RSNS=1), 56mV (RSNS=0)
  //0x4, 133mV (RSNS=1), 67mV (RSNS=0)
  //0x5, 155mV (RSNS=1), 78mV (RSNS=0)
  //0x6, 178mV (RSNS=1), 89mV (RSNS=0)
  //0x7, 200mV (RSNS=1), 100mV (RSNS=0)

  //Since maximum current of payload is 100A, so short-circuit current we choose 110A.
  //we use a 1.5mR shunt, so V=I*R=110A*1.5mR=165mV.
  //(0x1<<7): RSNS=1
  //(0x1<<3): 100us
  //(0x5<<0): 155mV
  Zsy_BQ76920_SingleByte_Write (PROTECT1, (0x1 << 7) | (0x1 << 3) | (0x5 << 0));

  //Set Protect2 register.
  //[6:4], OCD_D2, OCD_D1, OCD_D0: Overcurrent in discharge delay setting.
  //0x0=8ms
  //0x1=20ms
  //0x2=40ms
  //0x3=80ms
  //0x4=160ms
  //0x5=320ms
  //0x6=640ms
  //0x7=1280ms

  //[3:0], OCD_T3, OCD_T2, OCD_T1, OCD_T0: Overcurrent in discharge threshold setting.
  //0x0, 17mV(RSNS=1), 8mV(RSNS=0)
  //0x1, 22mV(RSNS=1), 11mV(RSNS=0)
  //0x2, 28mV(RSNS=1), 14mV(RSNS=0)
  //0x3, 33mV(RSNS=1), 17mV(RSNS=0)
  //0x4, 39mV(RSNS=1), 19mV(RSNS=0)
  //0x5, 44mV(RSNS=1), 22mV(RSNS=0)
  //0x6, 50mV(RSNS=1), 25mV(RSNS=0)
  //0x7, 56mV(RSNS=1), 28mV(RSNS=0)
  //0x8, 61mV(RSNS=1), 31mV(RSNS=0)
  //0x9, 67mV(RSNS=1), 33mV(RSNS=0)
  //0xA, 72mV(RSNS=1), 36mV(RSNS=0)
  //0xB, 78mV(RSNS=1), 39mV(RSNS=0)
  //0xC, 83mV(RSNS=1), 42mV(RSNS=0)
  //0xD, 89mV(RSNS=1), 44mV(RSNS=0)
  //0xE, 94mV(RSNS=1), 47mV(RSNS=0)
  //0xF, 100mV(RSNS=1), 50mV(RSNS=0)

  //(0xF<<0): 100mV,  100mV/1.5mR=66.67A
  //The shunt should be changed to 1mR, then 100mV/1mR=100A
  Zsy_BQ76920_SingleByte_Write (PROTECT2, (0x4 << 4) | (0xF << 0));

  //Set Protect3 register.
  //[7:6], UV_D1, UV_D0: Undervoltage delay setting.
  //0x0=1s
  //0x1=4s
  //0x2=8s
  //0x3=16s
  //[5:4], OV_D1, OV_D0: Overvoltage delay setting.
  //0x0=1s
  //0x1=2s
  //0x2=4s
  //0x3=8s
  Zsy_BQ76920_SingleByte_Write (PROTECT3, (0x1 << 6) | (0x2 << 4));

  //https://github.com/LLL2542/BQ76920/blob/main/BQ76920.c
  //Set OV_TRIP OverVoltage.
  Zsy_BQ76920_SingleByte_Write (OV_TRIP, 0xF9); //4.3V depend on ADC.

  //Set UV_TRIP UnderVoltage.
  Zsy_BQ76920_SingleByte_Write (UV_TRIP, 0x91); //2.5V depend on ADC.
  return 0;
}
// limit settings (for battery protection)
// minimum temperature degree Celsius at Discharge.
//maximum temperature degree Celsius at Discharge.
//minimum temperature degree Celsius at Charge.
//maximum temperature degree Celsius at Charge.
// °C
int
setTemperatureLimits (int minDischarge_degC, int maxDischarge_degC, int minCharge_degC, int maxCharge_degC)
{
  return 0;
}

//set short circuit protection current value with action latency at Discharge.
int
setShortCircuitProtection (long current_mA, int delay_us)
{
  return 0;
}

//set over current protection current value with action latency at Discharge.
int
setOvercurrentDischargeProtection (long current_mA, int delay_ms)
{
  return 0;
}

//set Cell Under voltage protection with latency.
int
setCellUndervoltageProtection (int voltage_mV, int delay_s)
{
  return 0;
}

//set Cell Over voltage protection with latency.
int
setCellOvervoltageProtection (int voltage_mV, int delay_s)
{
  return 0;
}

//Charging control.
//At default, CHG/DSG are controlled by MCU GPIO.
int
enableCharging (void)
{
  //Read - Modify - Write.
  uint8_t rdData;
  rdData = Zsy_BQ76920_SingleByte_Read (SYS_CTRL2);
  //[0]=1, CHG_ON.
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL2, rdData | (0x1 << 0));
  return 0;
}
int
disableCharging (void)
{
  return 0;
}

//Discharging control.
//At default, CHG/DSG are controlled by MCU GPIO.
int
enableDischarging (void)
{
  //Read - Modify - Write.
  uint8_t rdData;
  rdData = Zsy_BQ76920_SingleByte_Read (SYS_CTRL2);
  //[1]=1, DSG_ON.
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL2, rdData | (0x1 << 1));
  return 0;
}
int
disableDischarging (void)
{
  return 0;
}
//get Die Temperature.
int
getDieTemperature (void)
{
  //https://github.com/LLL2542/BQ76920/blob/main/BQ76920.c
  uint8_t ts1_high, ts1_low;
  uint16_t ts1_val;
  ts1_high = Zsy_BQ76920_SingleByte_Read (TS1_HI_BYTE);
  ts1_low = Zsy_BQ76920_SingleByte_Read (TS1_LO_BYTE);
  ts1_val = (ts1_high << 8) | ts1_low;
  //V25 = 1.200 V (nominal)
  //VTSX = (ADC in Decimal) x 382 µV/LSB
  //TEMPDIE = 25° – ((VTSX – V25) ÷ 0.0042)

  //uV->mV-V, 1uV=1/1000mV=1/1000,000V
  gBQ76920Dev.dieTemp = 25.0f - (((ts1_val * 0.000328f) - 1.20f) / 0.0042f);
  return 0;
}
//Enable Balancing.
#define	BalanceThreshold		0.05f	  //V,50mV.
int
enableBalancing (void)
{
  uint8_t i;
  float minVoltage = 4.2f;
  uint8_t balancingFlag = 0, balancingFlagNew = 0;
  uint8_t adjacentCellCollision = 0;
  //find min voltage among 4 cells.
  for (i = 0; i < 4; i++)
    {
      if (gBQ76920Dev.cellVoltage[i] - minVoltage < 0)
	{
	  minVoltage = gBQ76920Dev.cellVoltage[i];
	}
    }
  for (i = 0; i < 4; i++)
    {
      if ((gBQ76920Dev.cellVoltage[i] - minVoltage) >= BalanceThreshold)
	{
	  //try to enable balancing of current cell.
	  balancingFlagNew = balancingFlag | (0x1 << i);

	  //check if attempting to balance adjacent cells.

	  //cell[0], if true, balancingFlag=0x0, balancingFlagNew=0|(0x1<<0)=0x1, then
	  //((balancingFlagNew << 1) & balancingFlag)=0,
	  //((balancingFlag << 1) & balancingFlagNew)=0,
	  //adjacentCellCollision=0, so balancingFlag=balancingFlagNew=0x1.

	  //cell[1], if true, balancingFlag=0x1, balancingFlagNew=0x1|(0x1<<1)=B11, then
	  //((balancingFlagNew << 1) & balancingFlag)=110&1=110,
	  //((balancingFlag << 1) & balancingFlagNew)=10&11=10.
	  //adjacentCellCollision!=0, so balancingFlag keep original value 0x1.

	  //cell[2], if true, balancingFlag=0x1, balancingFlagNew=0x1|(0x1<<2)=B101, then
	  //((balancingFlagNew << 1) & balancingFlag)=1010&1=0,
	  //((balancingFlag << 1) & balancingFlagNew)=10&101=0.
	  //adjacentCellCollision=0, so balancingFlag=balancingFlagNew=B101.

	  //cell[3], if true, balancingFlag=B101, balancingFlagNew=B101|(0x1<<3)=B1101, then
	  //((balancingFlagNew << 1) & balancingFlag)=B11010&B101=B11000,
	  //((balancingFlag << 1) & balancingFlagNew)=B1010&B1101=B1000.
	  //adjacentCellCollision!=0, so balancingFlag keep original value B101.

	  //Since balancingFlag=B101, so only [0] and [2] Cells will be balanced.

	  //adjacentCellCollision=0, so balancingFlag=balancingFlagNew=B101.
	  adjacentCellCollision = ((balancingFlagNew << 1) & balancingFlag) || ((balancingFlag << 1) & balancingFlagNew);
	  if (!adjacentCellCollision)
	    {
	      balancingFlag = balancingFlagNew;
	    }
	}
    }
  //CELLBAL1, CB[5:1].
  //VC1: Controlled by CB1.
  //VC2: Controlled by CB2.
  //VC3: Controlled by CB3.
  //VC4: Controlled by CB4.
  //VC5: Controlled by CB5.
  Zsy_BQ76920_SingleByte_Write (CELLBAL1, balancingFlag);
  return 0;
}
void
disableBalancing (void)
{
  Zsy_BQ76920_SingleByte_Write (CELLBAL1, 0x0);
}
//Enter SHIP mode.
int
enterSHIPMode (void)
{
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL1, 0x0);
  //Write 1#: [SHUT_A]=0, [SHUT_B]=1, =01=0x1.
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL1, 0x1);
  //Write 2#: [SHUT_A]=1, [SHUT_B]=0, =10=0x2.
  Zsy_BQ76920_SingleByte_Write (SYS_CTRL1, 0x2);
  return 0;
}
//dump All registers to UART.
int
dumpRegisters (void)
{
  uint8_t rdData;
  char buffer[32];
  rdData = Zsy_BQ76920_SingleByte_Read (SYS_STAT);
  sprintf (buffer, "SYS_STAT:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CELLBAL1);
  sprintf (buffer, "CELLBAL1:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CELLBAL2);
  sprintf (buffer, "CELLBAL2:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CELLBAL3);
  sprintf (buffer, "CELLBAL3:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (SYS_CTRL1);
  sprintf (buffer, "SYS_CTRL1:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (SYS_CTRL2);
  sprintf (buffer, "SYS_CTRL2:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (PROTECT1);
  sprintf (buffer, "PROTECT1:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (PROTECT2);
  sprintf (buffer, "PROTECT2:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (PROTECT3);
  sprintf (buffer, "PROTECT3:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (OV_TRIP);
  sprintf (buffer, "OV_TRIP:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (UV_TRIP);
  sprintf (buffer, "UV_TRIP:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CC_CFG);
  sprintf (buffer, "CC_CFG:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (BAT_HI_BYTE);
  sprintf (buffer, "BAT_HI:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (BAT_LO_BYTE);
  sprintf (buffer, "BAT_LO:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CC_HI_BYTE);
  sprintf (buffer, "CC_HI:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (CC_LO_BYTE);
  sprintf (buffer, "CC_LO:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (ADCGAIN1);
  sprintf (buffer, "ADCGAIN1:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (ADCOFFSET);
  sprintf (buffer, "ADCOFFSET:%0x\r\n", rdData);

  rdData = Zsy_BQ76920_SingleByte_Read (ADCGAIN2);
  sprintf (buffer, "ADCGAIN2:%0x\r\n", rdData);
  return 0;
}
//check Status Register to see if BMS has errors.
int
checkStatus (void)
{
  uint8_t rdData;
  //Read back SYS_STAT register.
  rdData = Zsy_BQ76920_SingleByte_Read (SYS_STAT);
  //bit[7]: CC Ready.
  if (rdData & (0x1 << 7))
    {
      //automatically clears CC ready flag.
      updateCurrent (1);
    }

  //bit[5]:DEVICE_XREADY.
  if (rdData & (0x1 << 5))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 5));
    }
  //bit[4]:Alert.
  if (rdData & (0x1 << 4))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 4));
    }
  //bit[3]:UV.
  if (rdData & (0x1 << 3))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 3));
    }
  //bit[2]:OV.
  if (rdData & (0x1 << 2))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 2));
    }
  //bit[1]:SCD.
  if (rdData & (0x1 << 1))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 1));
    }
  //bit[0]: OCD.
  if (rdData & (0x1 << 0))
    {
      //write 1 to clear flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 0));
    }
  return 0;
}
//update current.
//if bIgnoreCCReadyFlag=1, the current is read independent of an interrupt.
//indicating the availability of a new CC reading.
int
updateCurrent (int8_t bIgnoreCCReadyFlag)
{
  //https://github.com/LLL2542/BQ76920/blob/main/BQ76920.c
  uint8_t rdData;
  int8_t cc_high, cc_low;
  int32_t cc_val;
  float Vs;
  //Read back SYS_STAT register to check CC_READY flag.
  rdData = Zsy_BQ76920_SingleByte_Read (SYS_STAT);
  if (bIgnoreCCReadyFlag || rdData & (0x1 << 7))
    {
      //Read Coulomb Counter registers.
      cc_high = Zsy_BQ76920_SingleByte_Read (CC_HI_BYTE);
      cc_low = Zsy_BQ76920_SingleByte_Read (CC_LO_BYTE);
      cc_val = (cc_high << 8) | cc_low;
      //convert negative to positive.
      if (cc_val > ((0x1 << 15) - 1))
	{
	  cc_val = -((~cc_val) + 1);
	}
      //uV->mV->V, 1uV=1/1000mV=1/1000,000V.
      Vs = cc_val * 8.44 / 1000000.0f; //unit is V.

      //V->mV,1V=1000mV
      //Rsense=5mR
      //mV/mR=A.
      gBQ76920Dev.batCurrent = Vs * 1000.0f / (RSENSE);

      //write 1 to clear CC_READY flag.
      Zsy_BQ76920_SingleByte_Write (SYS_STAT, (0x1 << 7));
    }
  return 0;
}
//update voltage.
int
updateVoltages (void)
{
  //https://github.com/LLL2542/BQ76920/blob/main/BQ76920.c
  uint8_t val_high, val_low;
  uint16_t val_data;
  //read BAT_HI & BAT_LI.
  val_high = Zsy_BQ76920_SingleByte_Read (BAT_HI_BYTE);
  val_low = Zsy_BQ76920_SingleByte_Read (BAT_LO_BYTE);
  val_data = (val_high << 8) | val_low;
  //According to data sheet, the equation as below.
  // #Cells represents how many cells we have in connection, here is 4.
  //V(BAT) = 4 x GAIN x ADC(cell) + (#Cells x OFFSET)
  //GAIN is stored in units of µV/LSB, while OFFSET is stored in mV units.
  //uV->mV->V,1uV=1/1000mV=1/1000,000V.
  //mV->V, 1mV=1/1000V.
  //store in global variable for later use.
  gBQ76920Dev.batVoltage = (float) (4 * (gBQ76920Dev.adcGain / 1000000.0f) * val_data) + (4 * (gBQ76920Dev.adcOffset) / 1000.0f);

  //get Cells Voltages: VC1_HI & VC1_LO.
  val_high = Zsy_BQ76920_SingleByte_Read (VC1_HI_BYTE);
  val_low = Zsy_BQ76920_SingleByte_Read (VC1_LO_BYTE);
  val_data = (val_high << 8) | val_low;
  //V(cell) = GAIN x ADC(cell) + OFFSET
  //GAIN is stored in units of µV/LSB, while OFFSET is stored in mV units.
  //uV->mV, 1uV=1/1000mV.
  gBQ76920Dev.cellVoltage[0] = (gBQ76920Dev.adcGain * val_data) / 1000.0f + gBQ76920Dev.adcOffset;

  //get Cells Voltages: VC2_HI & VC2_LO.
  val_high = Zsy_BQ76920_SingleByte_Read (VC2_HI_BYTE);
  val_low = Zsy_BQ76920_SingleByte_Read (VC2_LO_BYTE);
  val_data = (val_high << 8) | val_low;
  //V(cell) = GAIN x ADC(cell) + OFFSET
  //GAIN is stored in units of µV/LSB, while OFFSET is stored in mV units.
  //uV->mV, 1uV=1/1000mV.
  gBQ76920Dev.cellVoltage[1] = (gBQ76920Dev.adcGain * val_data) / 1000.0f + gBQ76920Dev.adcOffset;

  //get Cells Voltages: VC3_HI & VC3_LO.
  val_high = Zsy_BQ76920_SingleByte_Read (VC3_HI_BYTE);
  val_low = Zsy_BQ76920_SingleByte_Read (VC3_LO_BYTE);
  val_data = (val_high << 8) | val_low;
  //V(cell) = GAIN x ADC(cell) + OFFSET
  //GAIN is stored in units of µV/LSB, while OFFSET is stored in mV units.
  //uV->mV, 1uV=1/1000mV.
  gBQ76920Dev.cellVoltage[2] = (gBQ76920Dev.adcGain * val_data) / 1000.0f + gBQ76920Dev.adcOffset;

  //get Cells Voltages: VC4_HI & VC4_LO.
  val_high = Zsy_BQ76920_SingleByte_Read (VC4_HI_BYTE);
  val_low = Zsy_BQ76920_SingleByte_Read (VC4_LO_BYTE);
  val_data = (val_high << 8) | val_low;
  //V(cell) = GAIN x ADC(cell) + OFFSET
  //GAIN is stored in units of µV/LSB, while OFFSET is stored in mV units.
  //uV->mV, 1uV=1/1000mV.
  gBQ76920Dev.cellVoltage[3] = (gBQ76920Dev.adcGain * val_data) / 1000.0f + gBQ76920Dev.adcOffset;
  return 0;
}
