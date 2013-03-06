/**
  ******************************************************************************
  * @file    IO_Toggle/main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************  
  */ 
/* Private define ------------------------------------------------------------------*/
//define in main
#define test 0

#define SCAN_MAIN_RUN		1
#define SCAN_MAIN_TEST	1
#define NUM_BUFF_PID	5
#define MDRIVER				0
#define MSTEERING_L		1
#define MSTEERING_R		2
#define ROBOT_GS			3
#define SPEED_MOTOR		3000
#define PULSE_RATIO		1900					// la don vi xung/m
 
#define NUM_BUF_TX_USART	32
/*define laser ---------------------------------------------------------------*/
#define NUM_BUF_RX_LASER 	2
/*define ADC -----------------------------------------------------------------*/
#define NUM_BUF_ADC 			3
/*define Motor ---------------------------------------------------------------*/
#define GAP_PWM_MOTOR		500
#define NUM_BUFF_MOTOR	3
/*define SW begib & end ------------------------------------------------------*/
#define PORT_SW 			GPIOD
#define PIN_SW_END		GPIO_Pin_0
#define PIN_SW_BEGIN	GPIO_Pin_1
/* Private Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gPID.h"
#include <math.h>
/* Private typedef -------------------------------------------------------------------*/

typedef struct{
	int D;
	int V;
	int P;
	char checked;
} GARDEN;

typedef struct{ 
	char point;
	char flag;
	char length;
	char rxData[200];
}BUFF;
/* Private macro -----------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------*/
/* allway user */
Bit flag[10];
__IO uint16_t count[10];

/* garden parameter-----------*/
GARDEN garden;

/* motor parameter*/
int aset;
MOTOR Motor[3];
char tempID;
/* ADC bufffer */
__IO uint16_t ADCValue[NUM_BUF_ADC];

/*laser sensor */
static signed char rxBufLaser[NUM_BUF_RX_LASER];
SLAVE slectedLaser;
int a,b;

/* usart */
char txBufUsart[NUM_BUF_TX_USART];
BUFF rxUI;
char pointTX;

/* PID ---- */
tPID velocityPID;
tPID posPID;
tPID aPID;
tPID stL_PID;
tPID stR_PID;
double Kcoeff[5][3] ={{0.01,0,0},
											{0,0,0},
											{0,0,0},
											{20,0,0},
											{20,0,0}};
double ABCcoeff[5][3];
double historyControl[5][3];

int32_t countEncoder,countEncoderOld;
double vCurrent, vOld;
double aCurrent;
double pwmPID[3];
double velocitySet;

//capture variable.
__IO uint32_t CaptureEncoder = 0;

/* Private function prototypes -----------------------------------------------*/
/* usart -----------------------------------------------*/
void delRxUI(void);
void txComputer(char c);
Bit CheckDataRxUI(void);
double ABS(double a);
/* support ---------------------------------------------*/
void initPID(void);
void configMotor(void);
void controlMD(void);
void controlSTL(void);
void controlSTR(void);
void laserLost(void);

/*laser sensor ----------------------------------------*/
void findSetpointST(int ratioF,int ratioB);
void mainTest(void);
void mainRun(void);

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
	gpioInitMode();
	usartInitMode();
	adcInitMode(); 
	TimInitMode();
	configMotor();
	initPID();
	delayMs(2000);
	putStringUsart(CMT_PRODUCT,USART2);
	GPIO_ResetBits(GPIOA, GPIO_Pin_6);
	GPIO_ResetBits(GPIOA, GPIO_Pin_7);
	
  while (1){
		count[MAIN]++;
		if(test){
			mainTest();
		}
		else{
			mainRun();
		}
  }
}

/*function main ----------------------------------------*/
void mainTest(void){
	if(flag[0] == TRUE){
			if(CheckDataRxUI() == TRUE) 	GPIO_SetBits(GPIOD, GPIO_Pin_14);
			else 													GPIO_ResetBits(GPIOD, GPIO_Pin_14);
	
			flag[0] = FALSE;
	}
	
	if(flag[START_MAIN] == TRUE){
		/* read data slave --------------------------------------------------------------------*/
		if((count[MAIN] % 5) == 0){
			if(slectedLaser == LASER_BACK){
				/*truyen cho slaver laser truoc*/
				putCharUsart(ADDRESS_SLAVE_LASER_F, USART3);	/*truyen dia chi cho slave laser truoc*/
				slectedLaser = LASER_FORNT;
			}else if(slectedLaser == LASER_FORNT){
				/*truyen cho slave laser sau*/
				putCharUsart(ADDRESS_SLAVE_LASER_B, USART3);	/*truyen dia chi cho slave laser sau*/
				slectedLaser = LASER_BACK;
			}
		}
	}
	else{
		Motor[0].pwm = 0;
		Motor[1].pwm = 0;
		Motor[2].pwm = 0;
		controlMotor(Motor[0]);
		controlMotor(Motor[1]);
		controlMotor(Motor[2]);			
	}
	delayMs(SCAN_MAIN_TEST);
}

void mainRun(void){
		if(flag[0] == TRUE){
  			if(CheckDataRxUI() == TRUE) 	GPIO_SetBits(GPIOD, GPIO_Pin_14);
  			else 													GPIO_ResetBits(GPIOD, GPIO_Pin_14);
		
				flag[0] = FALSE;
		}
		
		if(flag[START_MAIN] == TRUE){
			flag[TEST] = FALSE;
			if(count[MAIN] % 300 == 0) GPIO_ToggleBits(GPIOA, GPIO_Pin_6);		
			/*read SW Begin & End -----------------------------------------------------------------
				PORTD -> pin0 -> END
							-> pin1 -> BEGIN
			*/
// 			if(count[MAIN]%10 == 0) findSetpointST(rxBufLaser[LASER_FORNT],rxBufLaser[LASER_BACK]);
// 			controlMD();
// 			controlSTL();
// 			controlSTR();
			
			if(GPIO_ReadInputDataBit(PORT_SW,PIN_SW_END) == 0){
			}
			else if(GPIO_ReadInputDataBit(PORT_SW,PIN_SW_BEGIN) == 0){
					GPIO_ResetBits(GPIOD, GPIO_Pin_2);
					flag[8] = TRUE;
			}
			else if(GPIO_ReadInputDataBit(PORT_SW,PIN_SW_BEGIN) == 1 && flag[8] == TRUE){
					TIM_Cmd(TIM4, ENABLE);
					Motor[MDRIVER].dir = CW;
					flag[8] = FALSE;
			}
			/* read data slave --------------------------------------------------------------------*/
			if((count[MAIN] % 5) == 0){
				if(slectedLaser == LASER_BACK){
					/*truyen cho slaver laser truoc*/
					putCharUsart(ADDRESS_SLAVE_LASER_F, USART3);	/*truyen dia chi cho slave laser truoc*/
					slectedLaser = LASER_FORNT;
				}else if(slectedLaser == LASER_FORNT){
					/*truyen cho slave laser sau*/
					putCharUsart(ADDRESS_SLAVE_LASER_B, USART3);	/*truyen dia chi cho slave laser sau*/
					slectedLaser = LASER_BACK;
				}
			}
		}
		else{
			Motor[0].pwm = 0;
			Motor[1].pwm = 0;
			Motor[2].pwm = 0;
			controlMotor(Motor[0]);
			controlMotor(Motor[1]);
			controlMotor(Motor[2]);			
		}
		
		if(count[MAIN]%50 == 0){
			flag[TOGGLE_PWM]^=1;
			if(flag[TOGGLE_PWM] == TRUE) 	TIM11->CCR1 = 4300;
			else													TIM11->CCR1 = 0;
		}
		
 		if(count[MAIN]%300 == 0){
			if(flag[TX_EN] == TRUE)	txComputer(pointTX);
 		}
		delayMs(SCAN_MAIN_RUN);
}

/* function support ------------------------------------*/
void initPID(void){
	PIDInit(&velocityPID);
	velocityPID.ABCcoeff = &ABCcoeff[0][0];
	velocityPID.controlHistory = &historyControl[0][0];
	PIDCoeffCalc(&Kcoeff[0][0],&velocityPID);	
	
	PIDInit(&posPID);
	posPID.ABCcoeff = &ABCcoeff[1][0];
	posPID.controlHistory = &historyControl[1][0];
	PIDCoeffCalc(&Kcoeff[1][0],&posPID);
	
	PIDInit(&stL_PID);
	stL_PID.ABCcoeff = &ABCcoeff[3][0];
	stL_PID.controlHistory = &historyControl[3][0];
	PIDCoeffCalc(&Kcoeff[3][0],&stL_PID);
	
	PIDInit(&stR_PID);
	stR_PID.ABCcoeff = &ABCcoeff[4][0];
	stR_PID.controlHistory = &historyControl[4][0];
	PIDCoeffCalc(&Kcoeff[4][0],&stR_PID);
	
	stR_PID.controlReference = MID_STR;
	stL_PID.controlReference = MID_STL;
//	flag[LASER_LOST] = TRUE;

}

void configMotor(void){
	MotorConfig();
	Motor[MDRIVER].id = md;
	Motor[MSTEERING_L].id = msl;
	Motor[MSTEERING_R].id = msr;
}

void controlMD(void){
	count[9]++;
	
	countEncoder += TIM_GetCounter(TIM8) - 30000;
	vCurrent = 200000.0/CaptureEncoder;
	aCurrent = (double)(vCurrent - vOld);
	vOld  = vCurrent;
	
	/* control theo van toc --------------------------------------*/
	velocityPID.measureOutput =(double)vCurrent;	//nhap gia tri hien tai
	pwmPID[MDRIVER] += PID(&velocityPID);	
	if(pwmPID[MDRIVER] > 4095) pwmPID[MDRIVER] = 4095;
	else if(pwmPID[MDRIVER] < 0) pwmPID[MDRIVER] = 0;
	Motor[MDRIVER].pwm = (int)pwmPID[MDRIVER]; 
	controlMotor(Motor[MDRIVER]);
	
	if(count[9]%10 == 0){
		if(flag[LASER_LOST] == FALSE){
			/* control theo vi tri --------------------------------------*/
			posPID.measureOutput = countEncoder;
			velocitySet = PID(&posPID);
		
			if(velocitySet < 0){
				velocitySet = -velocitySet;
				Motor[MDRIVER].dir = CCW;
			}
			else{
				Motor[MDRIVER].dir =CW;
			}

			if(velocitySet > garden.V) velocitySet = garden.V;
			velocityPID.controlReference = (double)velocitySet;
		}
		else{
			/* mat tin hieu laser */
			laserLost();
		}
	}
}
void controlSTL(void){
	stL_PID.measureOutput = readAdc(ADC_ANGLE_L);

	pwmPID[1] = PID(&stL_PID);
	
	if(pwmPID[1] < 0 && stL_PID.measureOutput > MIN_STL){
		pwmPID[1] = -pwmPID[1];
		Motor[MSTEERING_L].dir = CCW;
	}
	else if(pwmPID[1] > 0 && stL_PID.measureOutput < MAX_STL){
		Motor[MSTEERING_L].dir = CW;
	}
	else{
		pwmPID[1] = 0;
	}
	
	if(pwmPID[1] > 4095) pwmPID[1] = 4095;
	Motor[MSTEERING_L].pwm = (int)pwmPID[1]; 
	controlMotor(Motor[MSTEERING_L]);	
}

void controlSTR(void){
	stR_PID.measureOutput = readAdc(ADC_ANGLE_R);
	pwmPID[2] = PID(&stR_PID);
	
	if(pwmPID[2] < 0 && stL_PID.measureOutput > MIN_STR){
		pwmPID[2] = -pwmPID[2];
		Motor[MSTEERING_R].dir = CCW;
	}
	else if(pwmPID[2] > 0 && stL_PID.measureOutput < MAX_STR){
		Motor[MSTEERING_R].dir = CW;
	}
	else{
		pwmPID[2] = 0;
	}
	
	if(pwmPID[2] > 4095) pwmPID[2] = 4095;
	Motor[MSTEERING_R].pwm = (int)pwmPID[2]; 
	controlMotor(Motor[MSTEERING_R]);	
}

/*
	@void: laserLost.
	@para: none.
	@return var: none.
	fuction: ham xu li khi mat tin hieu laser.
*/
void laserLost(void){
	if(vCurrent == 0){
		GPIO_SetBits(GPIOA, GPIO_Pin_7);
	}
	else{
		velocityPID.controlReference = 0;
	}
}
/*math fuction------------------------------------------*/
double ABS(double a){
	if(a<0) return -a;
	else		return a;
}

void findSetpointST(int ratioF,int ratioB){
	int value_angle_steering;
	int setpointSTR,setpointSTL;
	if(ratioF != -51) a = ratioF;
	if(ratioB != -51) b = ratioB;
	
	if(Motor[MDRIVER].dir == CCW){
		value_angle_steering = a*5 + (a - b)*2;
		
		if(ratioF != -51){
			setpointSTR = MID_STR - value_angle_steering;
			if(setpointSTR > MAX_STR)	setpointSTR = MAX_STR;
			else if(setpointSTR < MIN_STR)	setpointSTR = MIN_STR;
			stR_PID.controlReference = (double)setpointSTR;
			
			setpointSTL = MID_STL - value_angle_steering;
			if(setpointSTL > MAX_STL)	setpointSTR = MAX_STL;
			else if(setpointSTL < MIN_STL)	setpointSTR = MIN_STL;
			stL_PID.controlReference = (double)setpointSTL;
			
			flag[LASER_LOST] = FALSE;
//			posPID.controlReference = (double)(rxUI.rxData[2]*PULSE_RATIO);
//			countEncoderOld =countEncoder;
		}
		else{
			flag[LASER_LOST] = TRUE;
//		posPID.controlReference = countEncoderOld + 1200;
		}
	}else{
		value_angle_steering = b*5 + (b - a)*2;
		if(ratioB != -51){
			setpointSTR = MID_STR - value_angle_steering;
			if(setpointSTR > MAX_STR)	setpointSTR = MAX_STR;
			else if(setpointSTR < MIN_STR)	setpointSTR = MIN_STR;
			stR_PID.controlReference = setpointSTR;
			
			setpointSTL = MID_STL - value_angle_steering;
			if(setpointSTL > MAX_STL)	setpointSTR = MAX_STL;
			else if(setpointSTL < MIN_STL)	setpointSTR = MIN_STL;
			stL_PID.controlReference = setpointSTL;
			
			flag[LASER_LOST] = FALSE;
//			posPID.controlReference = (double)(rxUI.rxData[2]*PULSE_RATIO);
//			countEncoderOld =countEncoder;
		}
		else{
//			posPID.controlReference = countEncoderOld + 1200;
			flag[LASER_LOST] = TRUE;
		}
	}
}

/* USART--------------*/
void delRxUI(void){
	while(rxUI.point != 0){
		rxUI.rxData[rxUI.point--] = '\0';
	}
}

Bit CheckDataRxUI(void){
		Bit temp = FALSE;
		char tempValue = 0;
		char i;
	
		if(rxUI.rxData[1] == 1){
			if(rxUI.rxData[2] == 'r'){
				TIM_Cmd(TIM4, ENABLE);
				flag[START_MAIN] = TRUE;
				flag[STOP_MAIN] = FALSE;
			}
			else if(rxUI.rxData[2] == 's'){
				TIM_Cmd(TIM4, DISABLE);
				flag[STOP_MAIN] = TRUE;
				flag[START_MAIN] = FALSE;
			}
			else if(rxUI.rxData[2] == 't'){
				pointTX++;
				if(pointTX > 2) pointTX=0;
				if(pointTX == 0) flag[TX_EN] = FALSE;
				else flag[TX_EN] = TRUE;
			}
		}
		else if(rxUI.rxData[1] == 2){
			for(i = 2; i < rxUI.length; i++){
				tempValue += rxUI.rxData[i];
			}
			
			if(tempValue == rxUI.rxData[rxUI.length]){
				posPID.controlReference = (double)(rxUI.rxData[2]*PULSE_RATIO);garden.V = rxUI.rxData[3]*100; garden.P = rxUI.rxData[4];

// 				pid[MSTEERING_L].Kp = rxUI.rxData[8];
// 				pid[MSTEERING_L].Ki = rxUI.rxData[9];
// 				pid[MSTEERING_L].Kd = rxUI.rxData[10];
				
				Kcoeff[1][0] = (double)rxUI.rxData[5]/10.0;
				Kcoeff[1][1] = (double)rxUI.rxData[6]/1000.0;
				Kcoeff[1][2] = (double)rxUI.rxData[7]/10.0;
				
// 				Kcoeff[0][0] = rxUI.rxData[14];
// 				Kcoeff[0][1] = rxUI.rxData[15]/100.0;
// 				Kcoeff[0][2] = rxUI.rxData[16];
				
				PIDCoeffCalc(&Kcoeff[1][0],&posPID);
			}
			else temp = TRUE;
		}
		else if(rxUI.rxData[1] == 3){
			if( rxUI.rxData[4] == (char)(rxUI.rxData[2]+rxUI.rxData[3])){
				tempID = rxUI.rxData[2];
				Motor[tempID].dir = rxUI.rxData[3];
				Motor[tempID].pwm = SPEED_MOTOR;
				flag[TEST] = TRUE;
			}
			else{
				temp = TRUE;
			}
		}		
		return temp;
}

void txComputer(char c){
	if(c == 1){
 		sprintf(txBufUsart,"ABCcoeff: %d %d\r\n",readAdc(ADC_ANGLE_L),rxBufLaser[LASER_BACK]);
 		putStringUsart(txBufUsart,USART2);
// 		sprintf(txBufUsart,"LF: %d   LB: %d\r\n",rxBufLaser[LASER_FORNT],rxBufLaser[LASER_BACK]);
// 		putStringUsart(txBufUsart,USART2);
// 		sprintf(txBufUsart,"SPEED: %d \r\n",ADCValue[ADC_SPEED]);
// 		putStringUsart(txBufUsart,USART2);
		
	}
	else if(c == 2){
		sprintf(txBufUsart,"GARDEN-> D: %3.4f  \t V: %d \t P: %d\n",posPID.controlReference,garden.V,garden.P);
		putStringUsart(txBufUsart,USART2);
		sprintf(txBufUsart,"VPID-> Kp: %3.4f  \t Ki: %3.4f \t Kd: %3.4f\n",Kcoeff[0][0],Kcoeff[0][1],Kcoeff[0][2]);
		putStringUsart(txBufUsart,USART2);
		sprintf(txBufUsart,"PPID-> Kp: %3.4f  \t Ki: %3.4f \t Kd: %3.4f\n",Kcoeff[1][0],Kcoeff[1][1],Kcoeff[1][2]);
		putStringUsart(txBufUsart,USART2);
		flag[TX_EN] = FALSE;
	}
}


/* function interrupt*/
/*
	interrupt interface Usart
		USART2
		USART3
*/
/*
	usart2 	interface slave 	-> computer
														-> touch pad
					connect portA 		-> pin2-RX
														-> pin3-TX
*/
void USART2_IRQHandler(void)       
{
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    if(USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
    {
			/* nhan du lieu tu may tinh */
      char data = USART_ReceiveData(USART2);

			if(data == 254){
				rxUI.flag = 1;				
				delRxUI();
				rxUI.point = 0;
			}
			else{
				if(rxUI.flag == 1){
						if(rxUI.point == 0){
							rxUI.length = data;
							rxUI.point = 1;
						}
						else if(rxUI.point < rxUI.length){
							rxUI.rxData[rxUI.point++] = data;
						}
						else if(rxUI.point == rxUI.length){
							rxUI.rxData[rxUI.length] = data;
							rxUI.flag = 0;
							flag[0] = TRUE;
						}
				}
			}
		}
}
/*
	usart3	interface slave 	-> modul laser fornt
														-> modul laser back
					connect portC 		-> pin10-RX
														-> pin11-TX
*/
void USART3_IRQHandler(void)       
{
    if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == SET)
    {
			/* nhan du lieu tu slave */
      char data = USART_ReceiveData(USART3);
			
			/*luu gia tri slave trong mang 
				gia tri tra ve tu 10:90
				gia tri mong mun cu ta la tu -40:40
				-> gia tri tra ve phai  -50
				gia tri khi ko nhan duoc gi la -51 */
			rxBufLaser[slectedLaser] = data - 50;
			/* co ngan khi nhan duoc gia tri */
			flag[RX_LASER] = FALSE;
			/* chop tat led5 khi nhan dc gia tri tra ve */
			STM_EVAL_LEDToggle(LED3);
		}
}


/**
  * @brief  This function handles TIM4 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
	int capture;
/* PROCESS TIME -------------------------------------------------------------------------------------*/
  if (TIM_GetITStatus(TIM4, TIM_IT_CC1) != RESET)
  {
		STM_EVAL_LEDToggle(LED6);
	/* CONFIG TIME -----------------------------------------------------------------------------------*/
		/*CCR_Val[TIM4->4][channels]. quy dinh thoi gian quay lai.*/		
		capture = TIM_GetCapture1(TIM4);
    TIM_SetCompare1(TIM4, capture + readTimeCCR(TIME_4,1));	
		
		findSetpointST(rxBufLaser[LASER_FORNT],rxBufLaser[LASER_BACK]);
 		controlMD();
 		controlSTL();
 		controlSTR();
		
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
		TIM_SetCounter(TIM8,30000);
	}
}

void TIM1_CC_IRQHandler(void)
{ 
  if(TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) 
  {
    /* Clear TIM1 Capture compare interrupt pending bit */
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
		CaptureEncoder = (uint32_t)TIM_GetCapture2(TIM1);
		TIM_SetCounter(TIM1,0);
	}
}
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
 	GPIO_SetBits(GPIOA, GPIO_Pin_7);
	sprintf(txBufUsart,"Wrong parameters value: file %s on line %d\r\n", file, line);
	putStringUsart(txBufUsart,USART2);
	
  while (1){
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
