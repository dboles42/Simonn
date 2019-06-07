/*
 * adxl375.h
 *
 *  Created on: Jun 6, 2019
 *      Author: dboles@sdsu.edu
 */

#ifndef ADXL375_H_
#define ADXL375_H_



#endif /* ADXL375_H_ */


#include "main.h"
#include "stm32wb55xx.h"

/*Function Prototypes*/

void adxlWrite (uint8_t address, uint8_t value);
void adxlRead(uint8_t address, uint8_t *ptr, uint8_t size);
void adxlIinit(void);
uint8_t adxlReadID (void);

//Handle for SPI Channel 1
SPI_HandleTypeDef spi1;



