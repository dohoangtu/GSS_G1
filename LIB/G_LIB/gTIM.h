/* Public include -----------------------------------------------------------*/
#include "stm32f4xx_tim.h"
#include "stm32f4_discovery.h"
#include "gGPIO.h"
#include "gADC.h"
#include "stdio.h"
#include "gUSART.h"

/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define TIMCLK 500000
#define Hz			1
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void TimInitMode(void);
void setModeTimeBase(uint8_t NVIC_IRQChannel, TIM_TypeDef* TIMx);
void setModeTimeChannels(TIM_TypeDef* TIMx, char channels, uint16_t frequency);
void clearModeTimeChannels(TIM_TypeDef* TIMx, char channels);
void clearTimeAllChannels(TIM_TypeDef* TIMx);
void setModePwm(TIM_TypeDef* TIMx, uint32_t TIM_Period, uint32_t TIM3CLK);
void setModePwmChannels(TIM_TypeDef* TIMx, char channels);
int readTimeCCR(char tim,char channel);
void setModeEncoder(void);
void setModeCapture(void);

