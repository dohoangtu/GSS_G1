/*
	Remote IR
	Hardware: IR_Remote_10f_V1
	
	History:
	22/11/09:
	        - Change back to 38KHz 
	        change revision to r2
	          Modyfied code to generate IR carrier freq = 37KHz
	          
	05/11/09: release 091105R1, used with Dimmer release 091105D1
 * 4/5/2012: Dieu chinh de dung cho phat Laser trong du an GSS
	
    IR signal:
    Carrier Freq: 38KHz
    + button code: 2(pulse+dark) = 48ms   + 150ms delay
    - button code: 4(pulse+dark) = 96ms   + 100ms delay
    ON/OFF button: 6(pulse+dark) = 144ms  + 50ms delay
    pulse length: ~12ms
    dark length:  ~12ms
    Total time frame for 1 command: ~200ms

	3/11/09: updated from previous version.
	         - Change the methos of tranmisting data according to the TSOP1828 datasheet, see Readme.txt
	         
	
*/
#include <htc.h>
#include <pic10f222.h>


#ifndef _XTAL_FREQ
 // Unless already defined assume 4MHz system frequency
 // This definition is required to calibrate __delay_us() and __delay_ms()
 #define _XTAL_FREQ 8000000
#endif



#define downkey     GP0         // down,  
#define upkey       GP1         // up,
#define IR_LED      GP2         // generate pulse for IR LED
#define ON_OFF_key  GP3
#define TRIS_val    0b00001011  // GP2 as output, GP0, GP1, GP3 as input


void upkey_pressed(void);
void downkey_pressed(void);
void pulse(const unsigned int counter);
void dark(void);
void pulse_gen(void);

unsigned char cnt;

__CONFIG(IOSCFS_8MHZ & MCPU_ON & WDTE_OFF & CP_OFF & MCLRE_OFF);

void config(void)
{
    asm("movwf OSCCAL ; update register with factory cal value");
    TRIS = TRIS_val;      
    OPTION = 0b00010110;    // GPWP = 0: Enable Wake-up On Pin Change bit (GP0, GP1, GP3)
                            // GPPU = 0: enable weak pull-up ressistor 
                            // T0CS = 0: disable overiding TRIS on GP2
                            // *****************
                            //    // Setup WDT
                            //    OPTION.PSA = 0;                // Prescaler assigned to the timer0
                            //    // Prescaler = 1:128, cycle = 64us
                            //    OPTION.PS2 = 1;
                            //    OPTION.PS1 = 1;
                            //    OPTION.PS0 = 0;
    ANS0 = 0;               // GP0 as Digital IO
    ANS1 = 0;               // GP1 as Digital IO
    FOSC4 = 0;              // GP2 as Digital IO
    IR_LED = 0;             // turn off IR LED
}
void pulse_gen(void)
{
    for(cnt=0;cnt<6;cnt++){   // send 6 pulses
        pulse(465);    // 12ms = 26us x 
        dark();
    }
    dark();
    __delay_ms(50);
    
}
/* use PICKit 2 logic tool to measure the frequency and adjust 
// to get the desired frequency
// this rountine will generate f ~ 37.04KHz/38.46KHz
// Test condition:
    - Config GP0 as output.
    - use PICKit2 measure frequency at CH0
*/ 
void pulse(const unsigned int counter)
{
    unsigned int i;
    for(i=0;i<counter;i++)
    {
        // <<< New code to create a fo = 38KHz--> To = 26.3uS --> 26uS
        // 1 cycle = 0.5uS (Fosc = 8MHz/4)
        //  IR_LED hi for 13uS, Lo for 13us
        IR_LED = 1;     // 0.5us
        #asm 
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
         //   goto $+1;   2 cycle, 1us    , 9us
            NOP
            NOP
         #endasm
         IR_LED = 0;    //  0.5us
         #asm 
            goto $+1;   2 cycle, 1us
            goto $+1;   2 cycle, 1us
            //goto $+1;   2 cycle, 1us
            NOP
         #endasm         
         // +11us from for loop
    }    
}
// will delay about 12ms
void dark(void)
{
    unsigned int i;
    IR_LED = 0;
    for(i=0;i<1200;i++){
         asm("goto $+1;   2 cycle, 1us");
         asm("goto $+1;   2 cycle, 1us    , get 11.6ms--12.8ms");
    }     
}
void main(void)
{
    config();
    while(1)
    {
        pulse_gen();
    }
}
        
