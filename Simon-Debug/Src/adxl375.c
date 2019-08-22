/*
 * adxl375.c
 *
 *  Created on: Jun 6, 2019
 *      Author: dboles42@sdsu.edu
 */
#include "adxl375.h"
#include "main.h"
#include <stdio.h>


void adxlWriteI2C(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
	HAL_I2C_Master_Transmit (&hi2c1, ADXL_ADDRESS_I2C, data, 2, 100);
}

void adxlMultiByteReadI2C(uint8_t reg, uint8_t *array)
{
	HAL_I2C_Mem_Read (&hi2c1, ADXL_ADDRESS_I2C, reg, 1, (uint8_t *)array, 6, 100);
}

uint8_t adxlReadRegI2C(uint8_t reg)
{
	uint8_t value = 0;
	HAL_I2C_Mem_Read (&hi2c1, ADXL_ADDRESS_I2C, reg, 1, &value, 1, 100);
	return value;
}

void adxlInitTestI2C(void)
{

	printf("ADXL Configured For Real Time Measurements\n");
	adxlWriteI2C(REG_POWER_CTRL, 0x00);  // Put in standby mode for init
	adxlWriteI2C(REG_INT_ENABLE, 0x00);

	uint8_t regValue = 0;
	regValue = adxlReadRegI2C(REG_DEVID); // read the DEVID Register
	printf("Device ID:0x%x\n", regValue);

	adxlWriteI2C(REG_BW_RATE, BW_800HZ); //set Bandwidth to 800Hz
	regValue = adxlReadRegI2C(REG_BW_RATE); // read the BandWidth Register
	printf("BandWidth Rate:0x%x\n", regValue);

	adxlWriteI2C(REG_POWER_CTRL, 0x00);  // reset all bits
	adxlWriteI2C(REG_POWER_CTRL, 0x08);  // power_cntl measure and wake up 8hz
	regValue = adxlReadRegI2C(REG_POWER_CTRL); // read the Power Control Register
	printf("Power Control:0x%x\n", regValue);


}

void adxlInitInterruptTest(void)
{

	uint8_t regValue = 0;
	printf("ADXL Configured For Interrupts\n");

	adxlWriteI2C(REG_POWER_CTRL, 0x00);  // Put in standby mode for init

	adxlWriteI2C(REG_BW_RATE, BW_800HZ);
	regValue = adxlReadRegI2C(REG_BW_RATE);
	printf("BandWidth Rate:0x%x\n", regValue);

	adxlWriteI2C(REG_INT_ENABLE, 0x00); //Disable Interrupts Before Configuring
	//adxlWriteI2C(REG_INT_MAP, 0x00); //Trigger Interrupt on INT1 when DATA_READY
	adxlWriteI2C(REG_INT_MAP, 0x10); //Trigger Interrupt on INT2 when DATA_READY
	adxlWriteI2C(REG_INT_ENABLE, 0x10); //Enable Interrupt when DATA_READY and ACTIVITY occurs
	regValue = adxlReadRegI2C(REG_INT_ENABLE);
	printf("Interrupt Enabled:0x%x\n", regValue);

	adxlWriteI2C(REG_THRESH_ACT, THRESHOLD_ACTIVITY); //Set Activity THRESHOLD level for activity
	regValue = adxlReadRegI2C(REG_THRESH_ACT);
	printf("Activity Threshold:0x%x\n", regValue);
	adxlWriteI2C(REG_POWER_CTRL, 0x08);  // power_cntl measure and wake up 8hz

}


