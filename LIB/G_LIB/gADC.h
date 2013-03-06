/* Public include -----------------------------------------------------------*/
#include "stm32f4xx_adc.h"
#include "stm32f4_discovery.h"
#include "gGPIO.h"
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
#define ADC_CCR_ADDRESS    ((uint32_t)0x40012308)
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
void DMA2_Config(void);
void adcInitMode(void);
void ADC123_CH15_CH11_CH12_Config(void);
int readAdc(char channels);
