/*
 * i2c.h
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */

#ifndef I2C_H_
#define I2C_H_

#include "stm32f10x_i2c.h"

void I2C_Config();
u8 I2C_Read(u8 Addr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);
u8 I2C_Read2(u8 addr, u8 *data, u8 reg, u8 num);
u8 I2C_ByteWrite(u8 Addr, u8 writeAddr, u8 value);

#define I2C_VLOGIC()  GPIO_SetBits(GPIOC, GPIO_Pin_1)
extern void _delay(u32 value);

#endif /* I2C_H_ */
