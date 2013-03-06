#include "gPID.h"

/* private typedef ---------------------------------------------------------------------*/
/* private define ----------------------------------------------------------------------*/
/* Private macro -----------------------------------------------------------------------*/
/* Private variables -------------------------------------------------------------------*/
/* Private function prototypes ---------------------------------------------------------*/
void PIDInit(tPID *controller){
	*controller->ABCcoeff 		=	0;
	*(controller->ABCcoeff+1) =	0;
	*(controller->ABCcoeff+2) = 0;
	
	*controller->controlHistory = 0;
	*(controller->controlHistory + 1) = 0;
	*(controller->controlHistory + 2) = 0;
	
	controller->measureOutput 		= 0;
	controller->controlReference 	= 0;
	controller->controlOutput 		= 0;
}

void PIDCoeffCalc(double *kCoeff,tPID *controller){
	double kp = *kCoeff;
	double ki = *(kCoeff + 1);
	double kd = *(kCoeff + 2);
	*controller->ABCcoeff 		=	kp + ki + kd;
	*(controller->ABCcoeff+1) =	-(kp +2*kd);
	*(controller->ABCcoeff+2) = kd;
}

double PID(tPID *controller){
	*controller->controlHistory = controller->controlReference - controller->measureOutput;
	
	controller->controlOutput +=(*controller->controlHistory)		 	 *(*controller->ABCcoeff) 		  +
															(*(controller->controlHistory + 1))*(*(controller->ABCcoeff + 1)) +
															(*(controller->controlHistory + 2))*(*(controller->ABCcoeff + 2));
	*(controller->controlHistory + 2) = *(controller->controlHistory + 1);
	*(controller->controlHistory + 1) = *controller->controlHistory;
	return controller->controlOutput;
}
