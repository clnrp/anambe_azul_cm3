/*
 * i2c.cpp
 *
 *  Created on: 29 de ago de 2018
 *      Author: cleoner
 */


#include "i2c.h"

void I2C_Config() {
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Forcing clean Busy Flag
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
	_delay(0xffff);
	GPIO_ResetBits(GPIOB, GPIO_Pin_6);
	GPIO_ResetBits(GPIOB, GPIO_Pin_7);
	_delay(0xffff);

	// Configuration
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// VLOGIC
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	I2C_VLOGIC();

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE); //Enable the peripheral clock of I2C

	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 300000;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0xD0;

	I2C_DeInit(I2C1);
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

u8 I2C_Read(u8 Addr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
	u8 retry = 0;
	//left align address
	Addr = Addr<<1;

	//re-enable ACK bit incase it was disabled last call
	I2C_AcknowledgeConfig(I2C1, ENABLE);

    // While the bus is busy
    while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send START condition
    I2C_GenerateSTART(I2C1, ENABLE);

    // Test on EV5 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send address for write
    I2C_Send7bitAddress(I2C1, Addr, I2C_Direction_Transmitter);

    // Test on EV6 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Clear EV6 by setting again the PE bit
    I2C_Cmd(I2C1, ENABLE);

    // Send the internal address to write to
    I2C_SendData(I2C1, readAddr);

    // Test on EV8 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send STRAT condition a second time
    I2C_GenerateSTART(I2C1, ENABLE);

    // Test on EV5 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send address for read
    I2C_Send7bitAddress(I2C1, Addr, I2C_Direction_Receiver);

    // Test on EV6 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // While there is data to be read
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            // Disable Acknowledgement
            I2C_AcknowledgeConfig(I2C1, DISABLE);

            // Send STOP Condition
            I2C_GenerateSTOP(I2C1, ENABLE);
        }

        // Test on EV7 and clear it
        if (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            // Read a byte
            *pBuffer = I2C_ReceiveData(I2C1);

            // Point to the next location where the byte read will be saved
            pBuffer++;

            // Decrement the read bytes counter
            NumByteToRead--;
        }
    }

    // Enable Acknowledgement to be ready for another reception
    I2C_AcknowledgeConfig(I2C1, ENABLE);
    return 0x01;
}

u8 I2C_Read2(u8 addr, u8 *data, u8 reg, u8 num)
{
	u8 i, retry = 0;
    // left align address
    addr = addr<<1;

    I2C_AcknowledgeConfig(I2C1, ENABLE);
    // Test on BUSY Flag
    while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }

    I2C_GenerateSTART(I2C1, ENABLE);
    // Test on start flag
    while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_SB)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }
    // Send device address for write
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Transmitter);
    // Test on master Flag
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }
    // Send the device's internal address to write to
    I2C_SendData(I2C1, reg);
    // Test on TXE FLag
    while (!I2C_GetFlagStatus(I2C1,I2C_FLAG_TXE)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }
    // Send START condition a second time (Re-Start)
    I2C_GenerateSTART(I2C1, ENABLE);
    // Test start flag
    while (!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }
    // Send address for read
    I2C_Send7bitAddress(I2C1, addr, I2C_Direction_Receiver);
    // Test Receive mode Flag
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }

    for(i=0; i<num; i++){
    	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)){
        	retry++;
        	_delay(100);
        	if(retry > 250)	return 0;
        }
    	data[i] = I2C_ReceiveData(I2C1);
    }

    // enable NACK bit
    //I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
    I2C_AcknowledgeConfig(I2C1, DISABLE);

    // Send STOP Condition
    I2C_GenerateSTOP(I2C1, ENABLE);
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF)){
    	retry++;
    	_delay(100);
    	if(retry > 250)	return 0;
    }

    return 0x01;
}


u8 I2C_ByteWrite(u8 Addr, u8 writeAddr, u8 value)
{
	u8 retry = 0;
	//left align address
	Addr = Addr<<1;

    // Send START condition
    I2C_GenerateSTART(I2C1, ENABLE);

    // Test on EV5 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send address for write
    I2C_Send7bitAddress(I2C1, Addr, I2C_Direction_Transmitter);

    // Test on EV6 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send the internal address to write to
    I2C_SendData(I2C1, writeAddr);

    // Test on EV8 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send the byte to be written
    I2C_SendData(I2C1, value);

    // Test on EV8 and clear it
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)){
    	retry++;
    	_delay(10000);
    	if(retry > 250)	return 0;
    }

    // Send STOP condition
    I2C_GenerateSTOP(I2C1, ENABLE);
    return 0x01;
}
