/* Public include -----------------------------------------------------------*/
#include "stm32f4_discovery.h"
#include "gTIM.h"
#include "gGPIO.h"

/* Public typedef -----------------------------------------------------------*/
/* enum channels mmotor*/
enum{md , msr, msl};
/* typedef enum of dir motor*/
enum{CW, CCW};
/* typedef struct of MOTOR*/
typedef struct{
	char id;
	int pwm;
	char dir;
} MOTOR;

/* Public define ------------------------------------------------------------*/
#define OFFSET_MOTOR 10

#define MAX_STR 3712
#define MIN_STR 2452
#define MID_STR 3082

#define MAX_STL 3573
#define MIN_STL 2313
#define MID_STL 2943

/* Public macro -------------------------------------------------------------*/
/* Public variables ---------------------------------------------------------*/
/* Public function prototypes -----------------------------------------------*/
/* Public functions ---------------------------------------------------------*/
void MotorConfig(void);
void controlMotor(MOTOR M);
