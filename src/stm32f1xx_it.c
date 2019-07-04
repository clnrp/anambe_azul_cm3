/**
  ******************************************************************************
  * @file    stm32f1xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    11-February-2014
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <global.h>
#include "stm32f1xx_it.h"


/** @addtogroup IO_Toggle
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M Processor Exceptions Handlers                          */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{

}

/**
  * @brief  This function handles USART1.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	u8 byte = 0;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_IT_RXNE);

	}
}

/**
  * @brief  This function handles USART2.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	u8 byte = 0;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		byte = USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_IT_RXNE);

		if(byte == '@'){
			cntRxBuffer=0;
			rxBuffer[cntRxBuffer]=byte;
		}else{
			rxBuffer[++cntRxBuffer]=byte;
			if(byte == '!'){
				rxBuffer[cntRxBuffer+1]='\0';
				Process(rxBuffer);
			}
		}
	}
}

/**
  * @brief  This function handles TIM2.
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler()
{
	// T = (360*2)/72e6 = 10us
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	TimerInc();

	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1))
		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	else
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
}
