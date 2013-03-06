#include "LCD.h"

/*---------------------------------------------------------------------------------------------------------
	Function lcd
*/
void initLCD(void){
	gpioSetMode(LCD_PIN_USER,GPIO_Mode_OUT, GPIO_OType_PP, GPIO_Speed_100MHz, GPIO_PuPd_NOPULL, LCD_PORT);
	
  GPIO_ResetBits(LCD_PORT, LCD_PIN_E);
  GPIO_ResetBits(LCD_PORT, LCD_PIN_RW);
  GPIO_ResetBits(LCD_PORT, LCD_PIN_RS);
  
  delayMs(30);
  LCD_NYB(0x03,0);
  delayMs(5);
  LCD_NYB(0x03,0);
  delayMs(1);
  LCD_CONTROL(0x02);
  delayMs(1);
  LCD_CONTROL(0x28);
  delayMs(1);
  LCD_CONTROL(0x0C);
  delayMs(1);
  LCD_CONTROL(0x01);
  delayMs(1);
  LCD_CONTROL(0x06);
  delayMs(1);
}

void LCD_DATA(unsigned char data){
  delayMs(10);
	switch(data){
		case '\f':
			LCD_CONTROL(LCD_CLEAR_DISPLAY);
			break;
		case '\n':
			LCD_LINE(0, 1);
			break;
		default:
			LCD_NYB(data>>4, 1);
			delayMs(1);
			LCD_NYB(data , 1);
			delayMs(1);
			break;
	}
}

void LCD_CONTROL(unsigned char data){
		delayMs(10);
		LCD_NYB(data>>4, 0);
		delayMs(1);
		LCD_NYB(data , 0);
		delayMs(1);
}

void LCD_NYB(unsigned char nyb, char type){
  uint16_t temp;
  temp = nyb & 0xF;
  GPIO_Write(LCD_PORT,temp);
  
  if(type == 0) GPIO_ResetBits(LCD_PORT, LCD_PIN_RS); 	//COMMAND MODE
  else			GPIO_SetBits(LCD_PORT, LCD_PIN_RS); 	//character/data MODE
  
  GPIO_SetBits(LCD_PORT, LCD_PIN_E);	//ENABLE LCD DATA LINE
  delayMs(1);
  GPIO_ResetBits(LCD_PORT, LCD_PIN_E);		//DISABLE LCD DATA LINE
}

void LCD_LINE(unsigned char col, unsigned char row){
  unsigned char address;
	
	if(row!=0)
		address=0x40;
	else
		address=0;
	
	address += col;

	// set data address
	LCD_CONTROL(0x80|address);
	delayMs(1);
}

void LCD_STR(const unsigned char *text){
  while(*text){
	LCD_DATA(*text++);
  }
}

void displayTemplate(void){
  //garden name
  LCD_STR("\fVn:");
  //display status
  LCD_LINE(7,0);
  LCD_STR("COMMANDS");
 //garden dimension
  LCD_LINE(0,1);
  LCD_STR("D:");
	displayNumber(10);
  //robot velocity  
  LCD_LINE(5,1);
  LCD_STR("V:");
  //ap suat phun  
  LCD_LINE(11,1);
  LCD_STR("P:");
  //display parameter
  displayParameter();
  LCD_LINE(4,0);
}
void displayParameter(void){  
}

void displayWellcome(void){
}

void displayNumber(char number){
  char donvi, chuc;
  chuc = number/10;
  donvi = number - chuc*10;
  LCD_DATA('0'+chuc);
  LCD_DATA('0'+donvi);
}
