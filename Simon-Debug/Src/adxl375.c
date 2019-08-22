/*
 * adxl375.c
 *
 *  Created on: Jun 6, 2019
 *      Author: dboles42@sdsu.edu
 */
#include "adxl375.h"
#include "main.h"
#include <stdio.h>

void adxlWriteSPI (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|MULTI_BYTE;  // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
}

void adxlReadSPI(uint8_t address, uint8_t *ptr, uint8_t size)
{
	address |= SPI_READ;  // read operation
	address |= MULTI_BYTE;  // multibyte read
	uint8_t data_rec;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);
	HAL_SPI_Receive (&hspi1, data_rec, 6, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
}

void adxlInitSPI(void)
{
	adxlWriteSPI(0x2d, 0x00);  // reset all bits
	adxlWriteSPI(0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
uint8_t adxlReadIDSPI(void)
{
	uint8_t address = REG_DEVID;
	uint8_t value;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);
	HAL_SPI_Receive (&hspi1, &value, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
	return value;
}

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

void adxlInitI2C(void)
{
	uint8_t regValue = 0;
	regValue = adxlReadRegI2C(REG_DEVID); // read the DEVID Register
	printf("Device ID:0x%x\n", regValue);

	adxlWriteI2C(REG_BW_RATE, 0x0D);
	regValue = adxlReadRegI2C(REG_BW_RATE); // read the BandWidth Register
	printf("BandWidth Rate:0x%x\n", regValue);

	adxlWriteI2C(REG_POWER_CTRL, 0x00);  // reset all bits
	adxlWriteI2C(REG_POWER_CTRL, 0x08);  // power_cntl measure and wake up 8hz
	regValue = adxlReadRegI2C(REG_POWER_CTRL); // read the Power Control Register
	printf("Power Control:0x%x\n", regValue);


}




