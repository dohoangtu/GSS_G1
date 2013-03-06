
/* External headers. */
#include				<stdint.h>
#include        <stdlib.h>              /* malloc, NULL */
#include        <math.h>                /* fabs, sin, cos, atan, sqrt */

/* Pulic define --------------------------------------------------------------------*/

/* Pulic typedef -------------------------------------------------------------------*/
typedef struct{
	double *ABCcoeff;
	double *controlHistory;
	double controlOutput;
	double measureOutput;
	double controlReference;
}tPID;

void PIDInit(tPID *controller);
void PIDCoeffCalc(double *kCoeff,tPID *controller);
double PID(tPID *controller);

