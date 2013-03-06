#include "gDelay.h"

void oneCycle(unsigned int nCount){
  while(nCount--);
}

void delayUs(int microseconds){
  while(microseconds--){
		oneCycle(56);
	}
}

void delayMs(int milliseconds){
	while(milliseconds--){
		delayUs(1000);
	}
}

void delayS(int seconds){
	while(seconds--){
		delayMs(1000);
	}
}
