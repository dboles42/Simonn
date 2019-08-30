/*
 * adxl375.h
 *
 *  Created on: Jun 6, 2019
 *      Author: dboles@sdsu.edu
 */

#ifndef ADXL375_H_
#define ADXL375_H_



#endif /* ADXL375_H_ */


#include "stm32wb55xx.h"
#include "main.h"
/************************************************/
/* 			ADXL375 REGISTER MAP
All registers in ADXL375 are 8 bits in length
	  A5:A0 are the address Bits
*/
/************************************************/
#define REG_DEVID				(0x00) //device ID
#define REG_THRESH_SHOCK 		(0x1D) //Shock threshold
#define REG_OFSX 				(0x1E) //X-axis offset
#define REG_OFSXY 				(0x1F) //Y-axis offset
#define REG_OFSXZ 				(0x20) //Z-axis offset
#define REG_DUR 				(0x21) //Shock duration
#define REG_LATENT 				(0x22) //Shock latency
#define REG_WINDOW 				(0x23) //Shock window
#define REG_THRESH_ACT 			(0x24) //Activity threshold
#define REG_THRESH_INACT 		(0x25) //Inactivity threshold
#define REG_TIME_INACT 			(0x26) //Inactivity time
#define REG_ACT_INACT_CTL 		(0x27) //Axis enable control for activity and inactivity detection
#define REG_SHOCK_AXES 			(0x2A) //Axis control for single shock/double shock
#define REG_ACT_SHOCK_STATUS	(0x2B) //Source of single shock/double shock
#define REG_BW_RATE 			(0x2C) //Data rate and power mode control
#define REG_POWER_CTRL 			(0x2D) //Power saving features control
#define REG_INT_ENABLE 			(0x2E) //Interrupt enable control
#define REG_INT_MAP 			(0x2F) //Interrupt mapping control
#define REG_INT_SOURCE 			(0x30) //Interrupt source
#define REG_DATA_FORMAT 		(0x31) //Data format control
#define REG_DATAX0 				(0X32) //X-Axis Data LH
#define REG_DATAX1 				(0X33) //X-Axis Data UH
#define REG_DATAY0 				(0X34) //Y-Axis Data LH
#define REG_DATAY1 				(0X35) //Y-Axis Data UH
#define REG_DATAZ0 				(0X36) //Z-Axis Data LH
#define REG_DATAZ1 				(0X37) //Z-Axis Data UH
#define REG_FIFO_CTL			(0X38) //FIFO control
#define FIFO_STATUS 			(0X39) //FIFO status

/************************************************/
/* 		ADXL375 COMMON REGISTER VALUES
/************************************************/

//Sampling Rate values
#define BW_3200HZ      (0x0F) //3200HZ
#define BW_1600HZ      (0x0E) //1600HZ
#define BW_800HZ       (0x0D) //800HZ
#define BW_400HZ       (0x0C) //400HZ
#define BW_200HZ       (0x0B) //200HZ
#define BW_100HZ       (0x0A) //100HZ
#define BW_50HZ        (0x09) //50HZ
#define BW_25HZ        (0x08) //25HZ
#define BW_12HZ5       (0x07) //12.5HZ
#define BW_6HZ25       (0x06) //6.25HZ

//A7 is R/W Bit for address
#define SPI_READ    (0x80) //Toggle Read
#define SPI_WRITE   (0x00) //Toggle Write
#define MULTI_BYTE  (0x40) //A6 is to set Multibyte R/W

//FIFO Modes
#define FIFO_BYPASS 	(0x00) //FIFO is bypassed
#define FIFO_ON 		(0x20) //FIFO collects samples until max then stops
#define FIFO_STREAM 	(0x40) //FIFO holds samples until buffer is full and overwrites
#define FIFO_TRIGGER	(0xC0) //FIFO holds samples till trigger

//ADXL 375  Constants
#define SHOCK_THRESH (0x40) //~50g's
#define SHOCK_WINDOW (0xA0) //~200ms Shock window (period of impact ~500hz)
#define SINGLE_SHOCK (0x40) //Enable Single Shock Interrupt
#define THRESH_SHOCK_SCALE (780) //780mg per LSB
#define THRESHOLD_ACTIVITY (5*1000)/THRESH_SHOCK_SCALE

//Power Modes TBD

//I2C Addresses
#define ADXL_ADDRESS_I2C 0xA6

/*Function Prototypes*/

void adxlWriteI2C (uint8_t reg, uint8_t value);
void adxlMultiByteReadI2C(uint8_t reg, uint8_t *array);
uint8_t adxlReadRegI2C(uint8_t reg);
void adxlInitTestI2C(void);
void adxlInitInterruptTest(void);



// I/O Handles
I2C_HandleTypeDef hi2c1;

//Private Variables




