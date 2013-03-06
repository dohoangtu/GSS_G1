#include "gEXTI.h"
#define 	RCC_(A) RCC_AHB1Periph_(A)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*
	function khi ngat se thuc hien
*/
/*
	function cai dat ngat cho chuong trinh
*/
void extiInitMode(void){
}

/*
	function set mode exti
*/
void extiSetMode(uint32_t EXTI_Line, EXTITrigger_TypeDef EXTI_Trigger, uint8_t EXTI_PortSourceGPIOx){
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	uint8_t EXTI_PinSourcex = 0;
	uint8_t NVIC_IRQChannel = 0;
	if(EXTI_Line == EXTI_Line0){
		EXTI_PinSourcex = EXTI_PinSource0;
		NVIC_IRQChannel = EXTI0_IRQn;
	}else if(EXTI_Line == EXTI_Line1){
		EXTI_PinSourcex = EXTI_PinSource1;
		NVIC_IRQChannel = EXTI1_IRQn;
	}else if(EXTI_Line == EXTI_Line2){
		EXTI_PinSourcex = EXTI_PinSource2;
		NVIC_IRQChannel = EXTI2_IRQn;
	}else if(EXTI_Line == EXTI_Line3){
		EXTI_PinSourcex = EXTI_PinSource3;
		NVIC_IRQChannel = EXTI3_IRQn;
	}else if(EXTI_Line == EXTI_Line4){
		EXTI_PinSourcex = EXTI_PinSource4;
		NVIC_IRQChannel = EXTI4_IRQn;
	}else if(EXTI_Line == EXTI_Line5){
		EXTI_PinSourcex = EXTI_PinSource5;
		NVIC_IRQChannel = EXTI9_5_IRQn ;
	}else if(EXTI_Line == EXTI_Line6){
		EXTI_PinSourcex = EXTI_PinSource6;
		NVIC_IRQChannel = EXTI9_5_IRQn ;
	}else if(EXTI_Line == EXTI_Line7){
		EXTI_PinSourcex = EXTI_PinSource7;
		NVIC_IRQChannel = EXTI9_5_IRQn ;
	}else if(EXTI_Line == EXTI_Line8){
		EXTI_PinSourcex = EXTI_PinSource8;
		NVIC_IRQChannel = EXTI9_5_IRQn;
	}else if(EXTI_Line == EXTI_Line9){
		EXTI_PinSourcex = EXTI_PinSource9;
		NVIC_IRQChannel = EXTI9_5_IRQn ;
	}else if(EXTI_Line == EXTI_Line10){
		EXTI_PinSourcex = EXTI_PinSource10;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}else if(EXTI_Line == EXTI_Line11){
		EXTI_PinSourcex = EXTI_PinSource11;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}else if(EXTI_Line == EXTI_Line12){
		EXTI_PinSourcex = EXTI_PinSource12;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}else if(EXTI_Line == EXTI_Line13){
		EXTI_PinSourcex = EXTI_PinSource13;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}else if(EXTI_Line == EXTI_Line14){
		EXTI_PinSourcex = EXTI_PinSource14;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}else if(EXTI_Line == EXTI_Line15){
		EXTI_PinSourcex = EXTI_PinSource15;
		NVIC_IRQChannel = EXTI15_10_IRQn;
	}
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Configure EXTI Line0 */
  EXTI_InitStructure.EXTI_Line = EXTI_Line;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  /* Enable and set EXTI Line0 Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
	
	/* Connect EXTI Line0 to PA0 pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOx, EXTI_PinSourcex);
}
