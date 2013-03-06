/**
  ******************************************************************************
  * @file    DMA/FLASH_RAM/main.h 
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stdio.h"
#include "main.h"
#include "gGPIO.h"
#include "gEXTI.h"
#include "gDelay.h"
#include "gUSART.h"
#include "gTIM.h"
#include "gADC.h"
#include "gControlMotor.h"

/* Exported types ------------------------------------------------------------*/
/*enum counter*/
enum {
	STR = 0,
	STL = 1,
	DT = 2,
	MAIN = 3,
	TIME_4 = 4,
	TESTING = 5
};
/*enum flag*/
enum {
	RX_COMPLET = 0,
	START_MAIN = 1,
	STOP_MAIN = 2,
	TEST = 3,
	TX_EN = 4,
	TOGGLE_PWM = 5,
	RX_LASER = 6,
	LASER_LOST = 7
	
};
/*enum ADC*/
enum{
	ADC_SPEED 	= 0,			
	ADC_ANGLE_L 	= 1,			
	ADC_ANGLE_R 	= 2				
};
/*enum dia chi cua cac slave*/
enum{
	ADDRESS_SLAVE_LASER_F = 258,		 	//slave co dia chi la 0x01
	ADDRESS_SLAVE_LASER_B = 257				//ta truyen 9bit bit dau bat len mot la truyen address
																		//-> dia chi can truyen la BIN: 0b1000000001 -> HEX: 0x101 -> DEC: 257
};

typedef enum{FALSE = 0, TRUE = 1} Bit;
typedef enum{LASER_FORNT = 0, LASER_BACK = 1} SLAVE;
/* Exported define ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

