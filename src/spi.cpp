/*
 * spi.cpp
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */


#include "spi.h"

void SPI_Config()
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	// MOSI and SCK
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// MISO as input floating
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// CS output
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//RCC_APB1PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);

	SPI_CS_HIGH();
	_delay(10000);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
}

u8 SPI_Read_Write_Byte(uint8_t value)
{
	u8 retry = 0;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	//{
	//	retry++;
	//	if(retry > 250)	return 0;
	//}
	SPI_I2S_SendData(SPI1, value);
	retry = 0;

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	//{
	//	retry++;
	//	if(retry > 250) return 0;
	//}
	return SPI_I2S_ReceiveData(SPI1);
}

u8 SPI_ReadByte(u8 Addr)
{
	u8 status, byte;
	SPI_CS_LOW(); // MPU6000_CS 0;

	_delay(10000);

	status = SPI_Read_Write_Byte(Addr | 0x80);
	byte = SPI_Read_Write_Byte(0x00);

	_delay(1000);

	SPI_CS_HIGH(); // MPU6000_CS 1;
	return byte;
}

u8 SPI_ReadBytes(u8 Addr, u8* pBuffer, u8 readAddr, u16 bytesToRead)
{
	uint32_t i;

	for(i=readAddr; i<readAddr+bytesToRead; i++)
	{
		SPI_CS_LOW(); // MPU6000_CS 0;

		// Send Read from Addr
		SPI_Read_Write_Byte(Addr | 0x80);

		// Read a byte
		*pBuffer = SPI_Read_Write_Byte(0xff);
		pBuffer++;

		SPI_CS_HIGH(); // MPU6000_CS 1;
	}
	return 0x00;
}

u8 SPI_WriteByte(u8 Addr, u8 data)
{
	u8 status;
	SPI_CS_LOW(); // MPU6000_CS 0;

	status = SPI_Read_Write_Byte(Addr);
	SPI_Read_Write_Byte(data);

	SPI_CS_HIGH(); // MPU6000_CS 1;
	return status;
}

void SPI_write(uint8_t address, uint16_t data)
{
	uint8_t * txdata = (uint8_t*)&data;
	// Wait for SPIy Tx buffer empty
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	SPI_CS_LOW(); // MPU6000_CS 0;
	// Send SPI address
	SPI_I2S_SendData(SPI1, address);
	for (int8_t x = 1; x >= 0 ; x-- ) {
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
		// Send SPI data high
		SPI_I2S_SendData(SPI1, txdata[x]);
	}
	SPI_I2S_ReceiveData(SPI1); // Clear pending RXNE
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	SPI_CS_HIGH(); // MPU6000_CS 1;
}

u8 spiReadWrite(uint8_t *rbuf, const uint8_t *tbuf, int cnt)
{
	int i;
	for (i = 0; i < cnt; i++){
		if (tbuf) {
			SPI_I2S_SendData(SPI1, *tbuf++);
		} else {
			SPI_I2S_SendData(SPI1, 0xff);
		}
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
		if (rbuf) {
			*rbuf++ = SPI_I2S_ReceiveData(SPI1);
		} else {
			SPI_I2S_ReceiveData(SPI1);
		}
	}
	return i;
}

uint8_t WriteByte(uint8_t Data)
{
	// Wait until the transmit buffer is empty
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	// Send the byte
	SPI_I2S_SendData(SPI1, Data);
	// Wait to receive a byte
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	// Return the byte read from the SPI bus
	return SPI_I2S_ReceiveData(SPI1);
}

uint8_t ReadByte(void)
{
	// Wait until the transmit buffer is empty
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	// Send the byte
	SPI_I2S_SendData(SPI1, 0xff);
	// Wait until a data is received
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	// Return the byte read from the SPI bus
	return SPI_I2S_ReceiveData(SPI1);
}
