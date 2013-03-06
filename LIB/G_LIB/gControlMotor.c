/* Private Includes ----------------------------------------------------------*/
#include "gControlMotor.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Function config*/
void MotorConfig(void){
	/* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) and CH3 (PC8) and TIM3 CH4 (PC9) */
	gpioSetMode(GPIO_Pin_8 | GPIO_Pin_9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP, GPIOC);
  gpioSetMode(GPIO_Pin_4 | GPIO_Pin_5, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP, GPIOB);
  /* Connect TIM3 pins to AF2 */  
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM3);
	/* set mode of TIM3*/
	setModePwm(TIM3, 4135, 84000000*Hz);
	setModePwmChannels(TIM3,1);
	setModePwmChannels(TIM3,2);
	setModePwmChannels(TIM3,3);
	setModePwmChannels(TIM3,4);

		/* GPIOA Configuration: TIM3 CH1 (PA1) and TIM1 CH3 (PA0)*/ 
		gpioSetMode(GPIO_Pin_0|GPIO_Pin_1, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP, GPIOA);

		/* Connect TIM3 pins to AF2 */  
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
		setModePwm(TIM2, 4135, 84000000*Hz);
		setModePwmChannels(TIM2,1);
		setModePwmChannels(TIM2,2);
	
}
/* Function support*/
void controlMotor(MOTOR M){
	if(M.id == md){
		if(M.dir == CW){
			TIM2->CCR1 = M.pwm;
			TIM2->CCR2 = 0;
		}else{
			TIM2->CCR2 = M.pwm;
			TIM2->CCR1 = 0;
		}
	}else if(M.id == msl){
		if(M.dir == CCW){
			TIM3->CCR1 = M.pwm;
			TIM3->CCR2 = 0;
		}else{
			TIM3->CCR2 = M.pwm;
			TIM3->CCR1 = 0;
		}
	}else if(M.id == msr){
		if(M.dir == CW){
			TIM3->CCR3 = M.pwm;
			TIM3->CCR4 = 0;
		}else{
			TIM3->CCR4 = M.pwm;
			TIM3->CCR3 = 0;
		}
	}
}
/* Function math */
/* Function interrupt*/

