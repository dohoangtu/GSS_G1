
#include "gTIM.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t CCR_Val[15][5];
__IO uint16_t PrescalerValue = 0;
/* Private function prototypes -----------------------------------------------*/
void TimInitMode(void){
  /* TIM4 clock enable */
  	setModeTimeBase(TIM4_IRQn, TIM4);
  	setModeTimeChannels(TIM4,1,1000*Hz);
		TIM_Cmd(TIM4, DISABLE);

		/* GPIOB Configuration: TIM11 CH1 (PB9) ---------*/ 
		gpioSetMode(GPIO_Pin_9, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_UP, GPIOB);
		/* Connect TIM11 pins to AF2 */  
		GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);
		/* set mode of TIM11*/
		setModePwm(TIM11, 4421, 80000000*Hz);
		setModePwmChannels(TIM11,1);
		TIM11->CCR1 = 4300;
	
		/* set mode encoder cua tim8-------------------------------------*/
	  setModeEncoder();
		setModeCapture();
	
		TIM_SetCounter(TIM8,30000);
}

void RCC_APB_TIM(TIM_TypeDef* TIMx){
	if(TIMx == TIM1)				{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	}
	else if(TIMx == TIM2)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	}
	else if(TIMx == TIM3)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	}
	else if(TIMx == TIM4)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	}
	else if(TIMx == TIM5)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	}
	else if(TIMx == TIM6)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
	}
	else if(TIMx == TIM7)		{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	}
	else if(TIMx == TIM8)		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
	}
	else if(TIMx == TIM9)		{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	}
	else if(TIMx == TIM10)	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
	}
	else if(TIMx == TIM11)	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
	}
	else if(TIMx == TIM12)	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	}
	else if(TIMx == TIM13)	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	}
	else if(TIMx == TIM14)	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
	}
}
void setModeTimeBase(uint8_t NVIC_IRQChannel, TIM_TypeDef* TIMx){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	uint16_t PrescalerValue = 0;
	NVIC_InitTypeDef NVIC_InitStructure;	
	
	RCC_APB_TIM(TIMx);
	/* Enable the TIM3 gloabal Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 500 KHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /500 KHz) - 1

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */  

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock / 2) / 500000) - 1;

  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

  /* Prescaler configuration */
  TIM_PrescalerConfig(TIMx, PrescalerValue, TIM_PSCReloadMode_Immediate);
}

void setModeTimeChannels(TIM_TypeDef* TIMx, char channels, uint16_t frequency){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	char a = 0;
	if(TIMx == TIM1)				{
		a = 1;
	}
	else if(TIMx == TIM2)		{
		a = 2;
	}
	else if(TIMx == TIM3)		{
		a = 3;
	}
	else if(TIMx == TIM4)		{
		a = 4;
	}
	else if(TIMx == TIM5)		{
		a = 5;
	}
	else if(TIMx == TIM6)		{
		a = 6;
	}
	else if(TIMx == TIM7)		{
		a = 7;
	}
	else if(TIMx == TIM8)		{
		a = 8;
	}
	else if(TIMx == TIM9)		{
		a = 9;
	}
	else if(TIMx == TIM10)	{
		a = 10;
	}
	else if(TIMx == TIM11)	{
		a = 11;
	}
	else if(TIMx == TIM12)	{
		a = 12;
	}
	else if(TIMx == TIM13)	{
		a = 13;
	}
	else if(TIMx == TIM14)	{
		a = 14;
	}
		/* -----------------------------------------------------------------------
    TIM3 Configuration: Output Compare Timing Mode:
                                              
    CC1 update rate = TIM3 counter clock / CCR1_Val = frequency(Hz)

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.    
  ----------------------------------------------------------------------- */
	
	CCR_Val[a][channels] = TIMCLK/frequency; 
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR_Val[a][channels];
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	if(channels == 1){
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC1, ENABLE);
	}else if(channels == 2){
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC2, ENABLE);
	}else if(channels == 3){
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC3, ENABLE);
	}else if(channels == 4){
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC4, ENABLE);
	}

  /* TIMx enable counter */
  TIM_Cmd(TIMx, ENABLE);
}

void clearModeTimeChannels(TIM_TypeDef* TIMx, char channels){
	if(channels == 1){
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE);
	}else if(channels == 2){
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE);
	}else if(channels == 3){
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC3, DISABLE);
	}else if(channels == 4){
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);
		/* TIM Interrupts enable */
		TIM_ITConfig(TIMx, TIM_IT_CC4, DISABLE);
	}
}

void clearTimeAllChannels(TIM_TypeDef* TIMx){
	/* TIM3 disable counter */
	TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Disable);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIMx, TIM_IT_CC1, DISABLE);
	
	TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Disable);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIMx, TIM_IT_CC2, DISABLE);
	
	TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Disable);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIMx, TIM_IT_CC3, DISABLE);
	
	TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Disable);
	/* TIM Interrupts enable */
	TIM_ITConfig(TIMx, TIM_IT_CC4, DISABLE);
	
  TIM_Cmd(TIMx, DISABLE);	
}

void setModePwm(TIM_TypeDef* TIMx, uint32_t TIM_Period, uint32_t TIM3CLK){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	/* TIMx clock enable */
	RCC_APB_TIM(TIMx);
/* -----------------------------------------------------------------------
    TIM3 Configuration: generate 4 PWM signals with 4 different duty cycles.
    
    In this example TIM3 input clock (TIM3CLK) is set to 2 * APB1 clock (PCLK1), 
    since APB1 prescaler is different from 1.   
      TIM3CLK = 2 * PCLK1  
      PCLK1 = HCLK / 4 
      => TIM3CLK = HCLK / 2 = SystemCoreClock /2
          
    To get TIM3 counter clock at 28 MHz, the prescaler is computed as follows:
       Prescaler = (TIM3CLK / TIM3 counter clock) - 1
       Prescaler = ((SystemCoreClock /2) /28 MHz) - 1
                                              
    To get TIM3 output clock at 30 KHz, the period (ARR)) is computed as follows:
       ARR = (TIM3 counter clock / TIM3 output clock) - 1
           = 665  
  ----------------------------------------------------------------------- */  

  /* Compute the prescaler value */
  PrescalerValue = (uint16_t) ((SystemCoreClock /2) / TIM3CLK) - 1;
	
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = TIM_Period;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); 
}

void setModePwmChannels(TIM_TypeDef* TIMx, char channels){
	TIM_OCInitTypeDef  TIM_OCInitStructure;
/* -----------------------------------------------------------------------	
	  TIM3 Channel1 duty cycle = (TIM3_CCR1/ TIM3_ARR)* 100 = 50%
    TIM3 Channel2 duty cycle = (TIM3_CCR2/ TIM3_ARR)* 100 = 37.5%
    TIM3 Channel3 duty cycle = (TIM3_CCR3/ TIM3_ARR)* 100 = 25%
    TIM3 Channel4 duty cycle = (TIM3_CCR4/ TIM3_ARR)* 100 = 12.5%

    Note: 
     SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
     Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
     function to update SystemCoreClock variable value. Otherwise, any configuration
     based on this variable will be incorrect.
  ----------------------------------------------------------------------- */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	if(channels == 1){
		/* PWM1 Mode configuration: Channel1 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC1Init(TIMx, &TIM_OCInitStructure);
		TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}else if(channels == 2){
		/* PWM1 Mode configuration: Channel2 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC2Init(TIMx, &TIM_OCInitStructure);
		TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}else if(channels == 3){
		/* PWM1 Mode configuration: Channel3 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC3Init(TIMx, &TIM_OCInitStructure);
		TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}else if(channels == 4){
		/* PWM1 Mode configuration: Channel4 */
		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
		TIM_OC4Init(TIMx, &TIM_OCInitStructure);
		TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable);
	}

  TIM_ARRPreloadConfig(TIMx, ENABLE);

  /* TIM3 enable counter */
  TIM_Cmd(TIMx, ENABLE);
	
	/* TIM3 Main Output Enable */
  TIM_CtrlPWMOutputs(TIMx, ENABLE);
}

void setModeEncoder(void){
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStruct;
	
	GPIO_InitTypeDef  GPIO_InitStructure;

	//Configure peripheral clocks
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  

	//Configure pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);  //C6 -TIM8_CH1 
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM8);  //C7  TIM8_CH2

	//Configure Timer
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Period = 60000;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure);
	TIM_ARRPreloadConfig(TIM8, ENABLE);

	//Debounce filter
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_1;
	TIM_ICInitStruct.TIM_ICFilter=3;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);
	TIM_ICInitStruct.TIM_Channel=TIM_Channel_2;
	TIM_ICInitStruct.TIM_ICFilter=3;
	TIM_ICInit(TIM8, &TIM_ICInitStruct);

	//Setup quadrature encoder and enable timer
	TIM_EncoderInterfaceConfig(TIM8,TIM_EncoderMode_TI12,TIM_ICPolarity_Falling,TIM_ICPolarity_Falling);
	TIM_Cmd(TIM8, ENABLE);   
}

void setModeCapture(void){
	unsigned short PrescalerValue;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure;
  
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

  /* GPIOA clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
  
  /* TIM1 channel 2 pin (PE.11) configuration */
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOE, &GPIO_InitStructure);

  /* Connect TIM pins to AF2 */
  GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
  
  /* Enable the TIM1 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* TIM1 configuration: Input Capture mode ---------------------
     The external signal is connected to TIM1 CH2 pin (PE.11)  
     The Rising edge is used as active edge,
     The TIM1 CCR2 is used to compute the frequency value
  ------------------------------------------------------------ */

 	PrescalerValue = (unsigned short) (SystemCoreClock / 200000) - 1;

	TIM_TimeBaseStructure.TIM_Period       			= 65534; //65535;
  TIM_TimeBaseStructure.TIM_Prescaler     		= (200000 - 1); //0;
  TIM_TimeBaseStructure.TIM_ClockDivision   	= 0;
  TIM_TimeBaseStructure.TIM_CounterMode     	= TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM1, PrescalerValue, TIM_PSCReloadMode_Immediate);
 
  TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  TIM_ICInitStructure.TIM_ICFilter = 0x0;

  TIM_ICInit(TIM1, &TIM_ICInitStructure);
  
  /* TIM enable counter */
  TIM_Cmd(TIM1, ENABLE);
	
  /* Enable the CC2 Interrupt Request */
  TIM_ITConfig(TIM1, TIM_IT_CC2, ENABLE);
}
int readTimeCCR(char tim,char channel){
	return CCR_Val[tim][channel];
}

