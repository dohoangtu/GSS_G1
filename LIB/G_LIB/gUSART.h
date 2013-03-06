/* Public include -----------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "gGPIO.h"
#include "stm32f4xx_usart.h"

#define CMT_PRODUCT "HW Ver. :STM32F4 Discovery\n\
										FW Built : 3/2013\n\
										GREMSY Co.,LTD\n"
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void usartInitMode(void);
void usartSetMode(uint32_t USART_BaudRate, uint16_t USART_Mode, USART_TypeDef* USARTx);
void usartSetModeCommunication(uint32_t USART_BaudRate, uint16_t USART_Mode, USART_TypeDef* USARTx, uint8_t USART_Address);
void putCharUsart(uint16_t c, USART_TypeDef* USARTx);
void putStringUsart(char *s, USART_TypeDef* USARTx);

