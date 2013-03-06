/* Public include -----------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "stm32f4xx_exti.h"
#include "gGPIO.h"
#include "gDelay.h"
#include "gTIM.h"
/* Public typedef -----------------------------------------------------------*/
#define NULL 0
/* Public define ------------------------------------------------------------*/
/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void extiInitMode(void);
void extiSetMode(uint32_t EXTI_Line, EXTITrigger_TypeDef EXTI_Trigger, uint8_t EXTI_PortSourceGPIOx);
