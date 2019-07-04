/*
 * spi.h
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef SPI_H_
#define SPI_H_

#include "stm32f10x_spi.h"
#include "stm32f10x_gpio.h"

void SPI_Config();
u8 SPI_Read_Write_Byte(uint8_t value);
u8 SPI_ReadByte(u8 Addr);
u8 SPI_ReadBytes(u8 Addr, u8* pBuffer, u8 readAddr, u16 bytesToRead);
u8 SPI_WriteByte(u8 Addr, u8 data);

#define SPI_CS_LOW()   GPIO_ResetBits(GPIOC, GPIO_Pin_1)
#define SPI_CS_HIGH()  GPIO_SetBits(GPIOC, GPIO_Pin_1)

extern void _delay(u32 value);

#endif /* SPI_H_ */
