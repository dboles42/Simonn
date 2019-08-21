/*
 * adxl375.c
 *
 *  Created on: Jun 6, 2019
 *      Author: dboles42@sdsu.edu
 */
#include "adxl375.h"
#include "main.h"

void adxlWrite (uint8_t address, uint8_t value)
{
	uint8_t data[2];
	data[0] = address|MULTI_BYTE;  // multibyte write
	data[1] = value;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, data, 2, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
}

void adxlRead(uint8_t address, uint8_t *ptr, uint8_t size)
{
	address |= SPI_READ;  // read operation
	address |= MULTI_BYTE;  // multibyte read
	uint8_t data_rec;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);
	HAL_SPI_Receive (&hspi1, data_rec, 6, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
}

void adxlInit(void)
{
	adxlWrite (0x2d, 0x00);  // reset all bits
	adxlWrite (0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
uint8_t adxlReadID (void)
{
	uint8_t address = REG_DEVID;
	uint8_t value;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET); //PULL SPI CS PIN LOW;
	HAL_SPI_Transmit (&hspi1, &address, 1, 100);
	HAL_SPI_Receive (&hspi1, &value, 1, 100);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET); //PULL SPI CS PIN HIGH;
	return value;
}
