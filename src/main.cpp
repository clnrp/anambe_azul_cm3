#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_rcc.h"

#define VAR_GLOBAL
#include "global.h"
#include "i2c.h"
#include <mpu6000.h>
#include "kalman.h"
#include "pid.h"
#include "timer.h"

#define RAD_TO_DEG  57.29577951308232f // from radians to degrees
#define DEG_TO_RAD  0.017453292519943295f // from degrees to radians

void Hardware_Config(void);
void RCC_Config(void);
void USART2_Config(void);
void TIM2_Config(void);
void PWM_TIM3_Config(void);
void PWM_TIM4_Config(void);
void usart_sendData(USART_TypeDef* usart, char* buffer);
void _delay(u32 value);
int adjustMinMax(float value, float min, float max);


//PID pid_roll(1.0,0.8,0.05,0);
//PID pid_pitch(1.0,0.8,0.05,0);
//PID pid_yaw(1.0,0.8,0.05,0);
float throttle;
float factor_m1=1;
float factor_m2=1;
float factor_m3=1;
float factor_m4=1;

void updatePid(int type, float kp, float ki, float kd)
{
	/*switch(type){ // Update PID parameters
	case 1:{
		pid_roll.SetK(kp,ki,kd); // roll PID
	}break;
	case 2:{
		pid_pitch.SetK(kp,ki,kd); // pitch PID
	}break;
	case 3:{
		pid_yaw.SetK(kp,ki,kd); // yaw PID
	}break;
	}*/
}

void updatePidSetPoint(int type, float value)
{
	/*switch(type){ // Update PID SetPoint
	case 1:{
		pid_roll.SetPoint(value);
	}break;
	case 2:{
		pid_pitch.SetPoint(value);
	}break;
	case 3:{
		pid_yaw.SetPoint(value);
	}break;
	}*/
}

void updateThro(float value) // Change throttle
{
	if(value > 500){
		throttle = 500;
	}else if(value < 0){
		value = 0;
	}else{
		throttle = value;
	}
}

void updateMotorFactor(float m1, float m2, float m3, float m4)
{
	factor_m1 = m1;
	factor_m2 = m2;
	factor_m3 = m3;
	factor_m4 = m4;
}

int main(void)
{
	u8 result;
	float ax=0,ay=0,az=0;
	float gx=0,gy=0,gz=0;
	float tem;
	float roll, pitch, yaw;
	float k_roll, k_pitch;
	float gyr_x=0, gyr_y=0, gyr_z=0;
	float pwm_m1=0,pwm_m2=0,pwm_m3=0,pwm_m4=0;
	float dt;
	int cnt = 0;
	char * str = new char(300);
	MPU6050 mpu6050;
	Kalman kalman_roll, kalman_pitch;
	PID pid_roll(1.0,0.8,0.05,0);
	PID pid_pitch(1.0,0.8,0.05,0);
	PID pid_yaw(1.0,0.8,0.05,0);

    speed = 200;
    throttle = 4;

	Hardware_Config();

	//_delay(1000000);
	mpu6050.Start();
	//mpu6050.Calibration(1000); // Automatic calibration

	mpu6050.SetOffSetAccel(0.415688485,-0.133357018,-1.15943706); // Manual calibration
	mpu6050.SetOffSetGyro(-3.5935421,0.57714504,-1.57878625);

	TIMER::ResetTime();
	while (1)
	{
		TIMER::Delay_ms(1);
		//mpu6050.Get(&ax,&ay,&az,&gx,&gy,&gz,&tem);
		result = mpu6050.GetAverage(4,&ax,&ay,&az,&gx,&gy,&gz,&tem);
		dt = TIMER::GetElapsedTime();

		if(result){
			gyr_x += gx*dt;
			gyr_y += gy*dt;
			gyr_z += gz*dt;

			yaw = 0; // Manter yaw em zero
			//roll = asin(ay / sqrt(ax * ax + ay * ay + az * az))*RAD_TO_DEG;
			//pitch = asin(ax / sqrt(ax * ax + ay * ay + az * az))*RAD_TO_DEG;

			//roll = atan2(ay, abs(az))*RAD_TO_DEG;
			//pitch = atan2(ax, abs(az))*RAD_TO_DEG;
			//pitch = atan(-ax / sqrt(ay * ay + az * az))*RAD_TO_DEG;

			// Calcular roll, pitch
			roll = atan2(-ax, abs(az))*RAD_TO_DEG; // OK
			//pitch = atan2(ax, abs(az))*RAD_TO_DEG;
			pitch = atan(ay / sqrt(ax * ax + az * az))*RAD_TO_DEG;

			// Filtro de Kalman do roll e pitch
			kalman_roll.Compute(roll, -gy, dt);
			kalman_pitch.Compute(pitch, gx, dt);
			k_roll = kalman_roll.GetAngle();
			k_pitch = kalman_pitch.GetAngle();

			// Calcular PID do roll e pitch
			pid_roll.ComputeIntPid(k_roll, 1);
			pid_pitch.ComputeIntPid(k_pitch, 1);
			pid_yaw.ComputeIntPid(yaw, 1);

			// PWM for X-frame
			// PWM1 = throttle + roll + pitch - yaw
			pwm_m1 = throttle + pid_roll.GetOut() + pid_pitch.GetOut() - pid_yaw.GetOut();
			pwm_m1 = factor_m1*pwm_m1;

			// PWM4 = throttle - roll + pitch + yaw
			pwm_m4 = throttle - pid_roll.GetOut() + pid_pitch.GetOut() + pid_yaw.GetOut();
			pwm_m4 = factor_m4*pwm_m4;

			// PWM2 = throttle + roll - pitch + yaw
			pwm_m2 = throttle + pid_roll.GetOut() - pid_pitch.GetOut() + pid_yaw.GetOut();
			pwm_m2 = factor_m2*pwm_m2;

			// PWM3 = throttle - roll - pitch - yaw
			pwm_m3 = throttle - pid_roll.GetOut() - pid_pitch.GetOut() - pid_yaw.GetOut();
			pwm_m3 = factor_m3*pwm_m3;
		}else{
			pwm_m1 = 0;
			pwm_m4 = 0;
			pwm_m2 = 0;
			pwm_m3 = 0;
		}

    	TIM_SetCompare1(TIM3, adjustMinMax(pwm_m1,0,500)); // PWM1 PC6_CH1 front left clockwise
    	TIM_SetCompare4(TIM3, adjustMinMax(pwm_m4,0,500)); // PWM4 PC7_CH4 front right counter-clockwise
    	TIM_SetCompare2(TIM3, adjustMinMax(pwm_m2,0,500)); // PWM2 PC8_CH2 rear left clockwise
    	TIM_SetCompare3(TIM3, adjustMinMax(pwm_m3,0,500)); // PWM3 PC9_CH3 rear right counter-clockwise

    	if(cnt++ >= 200){ // print debug
        	cnt=0;
        	//sprintf(str,"%i,%i,%i,%i,%i,%i|",(int)ax,(int)ay,(int)az,(int)gx,(int)gy,(int)gz);
        	//sprintf(str+strlen(str),"%i,%i,%i,%i,%i|",(int)roll,(int)k_roll,(int)pitch,(int)k_pitch,(int)yaw);
        	//sprintf(str+strlen(str),"%i,%i,%i|",(int)pid_roll.GetOut(),(int)pid_pitch.GetOut(),(int)pid_yaw.GetOut());
        	sprintf(str,"%i,%i,%i,%i,%i\r\n",(int)throttle,(int)pwm_m1,(int)pwm_m2,(int)pwm_m3,(int)pwm_m4);
        	usart_sendData(USART2, str);
        }
	}
}

void Hardware_Config(void)
{
	RCC_Config(); // Peripherals clock
	I2C_Config(); // I2C Configuration
	//SPI_Config();
	USART2_Config(); // UART 2 Configuration
	TIM2_Config(); // Timer 2
	PWM_TIM3_Config();

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void RCC_Config(void)
{
	/* CLOCK */
	RCC_DeInit();                                            // Start with the clocks in their expected state.
	RCC_HSEConfig(RCC_HSE_ON);                               // Enable HSE (high speed external clock).

	RCC_ClockSecuritySystemCmd(ENABLE);

	while(RCC_GetFlagStatus(RCC_FLAG_HSERDY) == RESET ){}    // Wait till HSE is ready.

	RCC_HCLKConfig(RCC_SYSCLK_Div1);                         // HCLK = SYSCLK
	RCC_PCLK2Config(RCC_HCLK_Div1);                          // PCLK2 = HCLK
	RCC_PCLK1Config(RCC_HCLK_Div2);                          // PCLK1 = HCLK/2
	RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);     // PLLCLK = 6MHz * 12 = 72 MHz.

	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	RCC_PLLCmd(ENABLE);                                      // Enable PLL.

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source. */
	while(RCC_GetSYSCLKSource() != 0x08)
	{
	}

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3 | RCC_APB1Periph_I2C1 | RCC_APB2Periph_SPI1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
}

void TIM2_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /*
     *         TIM_Period * (TIM_Prescaler+1)
     * time = -------------------------------
     *                  72000000
     */


    TIM_DeInit(TIM2);

    /* TIM2 configuration */
    TIM_TimeBaseStructure.TIM_Period = 360;
    TIM_TimeBaseStructure.TIM_Prescaler = 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(TIM2, ENABLE);

    TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

    /* Immediate load of TIM2 Precaler value */
    //TIM_PrescalerConfig(TIM2, ((SystemCoreClock/1200) - 1), TIM_PSCReloadMode_Immediate);

    /* Clear TIM2 update pending flag */
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);

    /* TIM2 Interrupt Config */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* Enable TIM2 Update interrupt */
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}

void PWM_TIM3_Config()
{
	uint16_t TimerPeriod = 500;
	uint16_t ChannelPulse = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7  | GPIO_Pin_8  | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_DeInit(TIM3);
    TIM_TimeBaseStructure.TIM_Prescaler = 400;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM3, ENABLE);

	ChannelPulse = 0; // 0%

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM3, &TIM_OCInitStructure);
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC4Init(TIM3, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void PWM_TIM4_Config(void)
{
	uint16_t TimerPeriod = 500;
	uint16_t ChannelPulse = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_DeInit(TIM4);
    TIM_TimeBaseStructure.TIM_Prescaler = 400;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = TimerPeriod;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM4, ENABLE);

	ChannelPulse = 250; // 50%

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = ChannelPulse;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	//TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	//TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_Structure;
	USART_InitTypeDef USART_Structure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Usart */
	/* Configure PA.9 (usart1_tx), PA.10 (usart1_rx)  --------------------------*/
	// Usart1 TX
	GPIO_Structure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	// Usart1 RX
	GPIO_Structure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	USART_DeInit(USART1);
	USART_Structure.USART_BaudRate = 9600;
	USART_Structure.USART_WordLength = USART_WordLength_8b;
	USART_Structure.USART_StopBits = USART_StopBits_1;
	USART_Structure.USART_Parity = USART_Parity_No;
	USART_Structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_StructInit(&USART_Structure);
	USART_Init(USART1,&USART_Structure);

	// USART Interrupt Config
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);

	USART_Cmd(USART1, ENABLE);
}

void USART2_Config(void){
	GPIO_InitTypeDef GPIO_Structure;
	USART_InitTypeDef USART_Structure;
	NVIC_InitTypeDef NVIC_InitStructure;

	//------------------------- USART2
	// usart2 tx
	GPIO_Structure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	// usart2 rx
	GPIO_Structure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Structure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Structure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_Structure);

	USART_DeInit(USART2);
	USART_Structure.USART_BaudRate = 9600;
	USART_Structure.USART_WordLength = USART_WordLength_8b;
	USART_Structure.USART_StopBits = USART_StopBits_1;
	USART_Structure.USART_Parity = USART_Parity_No;
	USART_Structure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Structure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_StructInit(&USART_Structure);
	USART_Init(USART2,&USART_Structure);

	// USART Interrupt Config
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USARTy Receive and Transmit interrupts */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART2, USART_IT_TXE, DISABLE);

	USART_Cmd(USART2, ENABLE);
}

void usart_sendData(USART_TypeDef* usart, char* buffer)
{
	u16 cnt, length, time_out=0;

	length = strlen(buffer);
	for (cnt = 0; cnt < length; cnt++){
		USART_SendData(usart, buffer[cnt]);
		time_out = 0;
		while((USART_GetFlagStatus(usart, USART_FLAG_TXE) == RESET ) && (time_out < 50) ){ // wait for the data to be sent
			_delay(1000);
			time_out++;
		}
	}

}

void _delay(u32 value)
{
	for(; value!=0; value--)
		asm("nop");
}

int adjustMinMax(float value, float min, float max) // limits the maximum and minimum value
{
    if (value > max) {
        value = max;
    } else if (value < min) {
        value = min;
    }
    return (int)value;
}

