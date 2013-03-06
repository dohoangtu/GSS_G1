
#include "gADC.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t ADCDualConvertedValue[20], ADCVal[3];
/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  ADC1 regular channels 10 and 11 configuration
  * @param  None
  * @retval None
  */
void adcInitMode(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	/* Enable peripheral clocks *************************************************/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1|RCC_APB2Periph_ADC2|RCC_APB2Periph_ADC3, ENABLE);

  /* DMA2 Stream0 channel0 configuration **************************************/
  DMA2_Config();
  
  /* ADCs configuration ------------------------------------------------------*/
  /* Configure ADC Channel15, 11, 12 pin as analog input */
	gpioSetMode(GPIO_Pin_5 | GPIO_Pin_1 | GPIO_Pin_2,
							GPIO_Mode_AN, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOC);

  /* ADC Common Init */
  ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_Interl;
  ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
  ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
  ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
  ADC_CommonInit(&ADC_CommonInitStructure);

  /* ADC2 regular channels 11, 12 configuration */
  ADC123_CH15_CH11_CH12_Config();

  /* Enable DMA request after last transfer (Multi-ADC mode)  */
  ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC2 */
  ADC_Cmd(ADC2, ENABLE);
	
	/* Enable ADC3 */
  ADC_Cmd(ADC3, ENABLE);
  /* Start ADC1 Software Conversion */
  ADC_SoftwareStartConv(ADC1);
///////////////////////////////////	
}

/**
  * @brief  ADC1 ADC2 ADC3 regular channels 15, 11, 12 configuration
  * @param  None
  * @retval None
  */
void ADC123_CH15_CH11_CH12_Config(void)
{
	
	ADC_InitTypeDef ADC_InitStructure;
	
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;		//cho phem muntilchannels
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfConversion = 1;		//bang so kenh sua dung
  ADC_Init(ADC1, &ADC_InitStructure);
	ADC_Init(ADC2, &ADC_InitStructure);
	ADC_Init(ADC3, &ADC_InitStructure);
  /* ADC1 regular channels 15 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 1, ADC_SampleTime_3Cycles);  

  /* ADC2 regular channels 11 configuration */ 
  ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, ADC_SampleTime_3Cycles);  

  /* ADC3 regular channels 12 configuration */
  ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
}

/**
  * @brief  DMA Configuration
  * @param  None
  * @retval None
  */
void DMA2_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
  DMA_InitStructure.DMA_Channel = DMA_Channel_0; 
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCDualConvertedValue;
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC_CCR_ADDRESS;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_BufferSize = 36;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;         
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA2_Stream0, &DMA_InitStructure);
  /* DMA2_Stream0 enable */
  DMA_Cmd(DMA2_Stream0, ENABLE);
}

int readAdc(char channels){
//	ADCVal[channels] = ADCDualConvertedValue[channels];
 	ADCVal[channels] = (ADCDualConvertedValue[channels] + ADCDualConvertedValue[channels +  4] + ADCDualConvertedValue[channels + 8] +
											ADCDualConvertedValue[channels + 12] + ADCDualConvertedValue[channels + 16])/5;
	return ADCVal[channels];
}

