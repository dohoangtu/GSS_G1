#include "gUSART.h"
#define MasterAddressed 0x00
#define Slave1Addressed 0x01
#define Slave2Addressed 0x02

/*
	init mode usart
	- khai bao chan
	- set mode
*/
void usartInitMode(void){
	/* khai bao io cho usart2 */
	gpioSetMode(GPIO_Pin_2, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOA);
	gpioSetMode(GPIO_Pin_3, GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOA);
	
	// Map USART2 to A.02
 	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	/* set mode */
	usartSetMode(9600,USART_Mode_Rx | USART_Mode_Tx,USART2);
	
	/* khai bao io cho usart3 */
	gpioSetMode(GPIO_Pin_10,GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOC);
	gpioSetMode(GPIO_Pin_11,GPIO_Mode_AF, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, GPIOC);
	
	// Map USART2 to A.02
 	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	
	/* set mode */
	usartSetModeCommunication(9600,USART_Mode_Rx | USART_Mode_Tx,USART3,MasterAddressed);
}

void usartSetMode(uint32_t USART_BaudRate, uint16_t USART_Mode, USART_TypeDef* USARTx){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure_UART;
	uint8_t NVIC_IRQChannelCmd = 0;
	
	if(USARTx == USART1){
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		NVIC_IRQChannelCmd = USART1_IRQn;
	}else if(USARTx == USART2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		NVIC_IRQChannelCmd = USART2_IRQn;
	}else if(USARTx == USART3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		NVIC_IRQChannelCmd = USART3_IRQn;
	}else if(USARTx == UART4){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		NVIC_IRQChannelCmd = UART4_IRQn;
	}else if(USARTx == UART5){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		NVIC_IRQChannelCmd = UART5_IRQn;
	}else if(USARTx == USART6){
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		NVIC_IRQChannelCmd = USART6_IRQn;
	}
	// Initialize USART
	USART_InitStructure.USART_BaudRate = USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode;
	/* Configure USART */
	USART_Init(USARTx, &USART_InitStructure);
	
  /* Enable the USART2 Receive interrupt: this interrupt is generated when the
  USART2 receive data register is not empty */
  USART_ITConfig(USARTx, USART_IT_RXNE,  ENABLE);              // Here Is have disabled the interrupts from USART3 - for testing in polling mode- u can change to ENABLE
  
    /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                             // Not sure about this one yet.
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure_UART.NVIC_IRQChannel = NVIC_IRQChannelCmd;
  NVIC_InitStructure_UART.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure_UART.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_UART.NVIC_IRQChannelCmd =  ENABLE;       // Here Is have disabled the interrupts from USART3 - for testing in polling mode- u can change to ENABLE
  NVIC_Init(&NVIC_InitStructure_UART);

	/* Enable the USART */
	USART_Cmd(USARTx, ENABLE);
}

void usartSetModeCommunication(uint32_t USART_BaudRate, uint16_t USART_Mode, USART_TypeDef* USARTx, uint8_t USART_Address){
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure_UART;
	uint8_t NVIC_IRQChannelCmd = 0;
	
	if(USARTx == USART1){
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		NVIC_IRQChannelCmd = USART1_IRQn;
	}else if(USARTx == USART2){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		NVIC_IRQChannelCmd = USART2_IRQn;
	}else if(USARTx == USART3){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
		NVIC_IRQChannelCmd = USART3_IRQn;
	}else if(USARTx == UART4){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
		NVIC_IRQChannelCmd = UART4_IRQn;
	}else if(USARTx == UART5){
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
		NVIC_IRQChannelCmd = UART5_IRQn;
	}else if(USARTx == USART6){
		RCC_APB1PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
		NVIC_IRQChannelCmd = USART6_IRQn;
	}
	// Initialize USART
	USART_InitStructure.USART_BaudRate = USART_BaudRate;
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode;
	/* Configure USART */
	USART_Init(USARTx, &USART_InitStructure);
	/* dia chi */
	if(USART_Address != 0){
		USART_SetAddress(USARTx, USART_Address);
		USART_WakeUpConfig(USARTx,USART_WakeUp_AddressMark);
	}
  /* Enable the USART2 Receive interrupt: this interrupt is generated when the
  USART2 receive data register is not empty */
  USART_ITConfig(USARTx, USART_IT_RXNE,  ENABLE);              // Here Is have disabled the interrupts from USART3 - for testing in polling mode- u can change to ENABLE
  
    /* NVIC configuration */
  /* Configure the Priority Group to 2 bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);                             // Not sure about this one yet.
  
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure_UART.NVIC_IRQChannel = NVIC_IRQChannelCmd;
  NVIC_InitStructure_UART.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure_UART.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_UART.NVIC_IRQChannelCmd =  ENABLE;       // Here Is have disabled the interrupts from USART3 - for testing in polling mode- u can change to ENABLE
  NVIC_Init(&NVIC_InitStructure_UART);

	/* Enable the USART */
	USART_Cmd(USARTx, ENABLE);
	if(USART_Address != 0)	USART_ReceiverWakeUpCmd(USARTx, ENABLE);
}
/**
* @brief Function that printf uses to push characters to serial port
* @param ch: ascii character
* @retval character
*/
void putCharUsart(uint16_t c, USART_TypeDef* USARTx){
	while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET);
	USART_SendData(USARTx, (uint16_t)c);
}

void putStringUsart(char *s, USART_TypeDef* USARTx){
	/* check to make sure we have a good pointer */
	if (!s) return;
	/* print data */
	while(*s){
		putCharUsart(*s++, USARTx);
	}
}
