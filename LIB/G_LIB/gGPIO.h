/* Public include -----------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_gpio.h"
/* Public typedef -----------------------------------------------------------*/
/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void gpioInitMode(void);
void gpioSetMode(uint32_t GPIO_PIN, GPIOMode_TypeDef GPIO_Mode,  GPIOOType_TypeDef GPIO_OType,
									GPIOSpeed_TypeDef GPIO_Speed, GPIOPuPd_TypeDef GPIO_PuPd, GPIO_TypeDef* GPIOx);
