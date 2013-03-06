#include "GPIO_by_DTU.h"
#include "Delay_by_DTU.h"
/* define lcd control ---------------------------------------*/
/* LCD memory map */
#define LCD_LINE0_ADDR 				0x00 		// Start of line 0 in the DD-Ram
#define LCD_LINE1_ADDR 				0x40 		// Start of line 1 in the DD-Ram

/* LCD Commands */
#define LCD_DD_RAM_PTR 				0x80 		// Address Display Data RAM pointer
#define LCD_CG_RAM_PTR 				0x40 		// Address Character Generator RAM pointer
#define LCD_CLEAR_DISPLAY 			0x01 		// Clear entire display and set Display Data Address to 0
#define LCD_RETRN_HOME 				0x02 		// sets DDRAM address 0 and returns display from being shifted to original position.
#define LCD_DISP_INIT 				0x28 		// function set is 4 bit data length and 2 lines
#define LCD_INC_MODE 				0x06 		// Entry mode is display Data RAM pointer incremented after write
#define LCD_DISP_ON					0x0C		// Sets entire display on, cursor on and blinking of cursor position character
#define LCD_DISP_OFF				0x08   	// Sets entire display off, cursor off
#define LCD_CURSOR_ON				0x04		// turn on cursor
#define LCD_CURSOR_OFF				0x00    // turn off cursor
#define LCD_CUR_MOV_LEFT			0x10		// Cursor move and shift to left
#define LCD_CUR_MOV_RIGHT			0x14		// Cursor move and shift to right
#define LCD_BUSY            		0x80    // LCD is busy

/* LCD Config PIN -------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
#define LCD_PORT 			GPIOE
#define LCD_PIN_RS		GPIO_Pin_4
#define LCD_PIN_RW		GPIO_Pin_5
#define LCD_PIN_E 		GPIO_Pin_6
#define LCD_PIN_DATA 	GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3
#define LCD_PIN_USER	GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6
/* khai bao bien --------------------------------------------*/

/* khai bao truong trinh con --------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void initLCD(void);
void LCD_DATA(unsigned char data);
void LCD_CONTROL(unsigned char data);
void LCD_NYB(unsigned char nyb, char type);
void LCD_LINE(unsigned char col, unsigned char row);
void LCD_STR(const unsigned char *text);

void displayLCD(void);
void displayNumber(char number);
void displayTemplate(void);
void displayWellcome(void);
void displayParameter(void);
void nuberToAscii(char number);
