#include "qaq_lcd.h"

#include "lcd.h"



 // 1表示第一次组数, 2表示"+", 3表示第二组数
uint8_t LCD_DATE1[7] = {"000"};  	//LCD显示数组
uint8_t LCD_DATE2[7] = {"+"}; 
uint8_t LCD_DATE3[7] = {"000"}; 



void QR_Show(uint8_t* receive_DATE) 
{ 
	switch (receive_DATE[4])
	{ 
		case 1:	
		LCD_DATE1[0] = '1';
		LCD_DATE1[1] = '2';
		LCD_DATE1[2] = '3';
			break;
		case 2:	
		LCD_DATE1[0] = '1';
		LCD_DATE1[1] = '3';
		LCD_DATE1[2] = '2';
			break;
		case 3:	
		LCD_DATE1[0] = '2';
		LCD_DATE1[1] = '1';
		LCD_DATE1[2] = '3';
			break;
		case 4:	
		LCD_DATE1[0] = '2';
		LCD_DATE1[1] = '3';
		LCD_DATE1[2] = '1';
			break;
		case 5:	
		LCD_DATE1[0] = '3';
		LCD_DATE1[1] = '1';
		LCD_DATE1[2] = '2';
			break;
		case 6:	
		LCD_DATE1[0] = '3';
		LCD_DATE1[1] = '2';
		LCD_DATE1[2] = '1';
			break;
	}  ///FE 00 01 00 01 FF
	switch (receive_DATE[6])
	{ 
		case 1:	
		LCD_DATE3[0] = '1'; 
		LCD_DATE3[1] = '2'; 
		LCD_DATE3[2] = '3'; 
			break;
		case 2:	
		LCD_DATE3[0] = '1';
		LCD_DATE3[1] = '3';
		LCD_DATE3[2] = '2';
			break;
		case 3:	
		LCD_DATE3[0] = '2';
		LCD_DATE3[1] = '1';
		LCD_DATE3[2] = '3';
			break;
		case 4:	
		LCD_DATE3[0] = '2';
		LCD_DATE3[1] = '3';
		LCD_DATE3[2] = '1';
			break;
		case 5:	
		LCD_DATE3[0] = '3';
		LCD_DATE3[1] = '1';
		LCD_DATE3[2] = '2';
			break;
		case 6:	
		LCD_DATE3[0] = '3';
		LCD_DATE3[1] = '2';
		LCD_DATE3[2] = '1';
			break;
	}
	
	LCD_ShowString(5,5,LCD_DATE1,GREEN,BLACK,64,0);	
	LCD_ShowString(120,5,LCD_DATE2,GREEN,BLACK,64,0);	
	LCD_ShowString(5,70,LCD_DATE3,GREEN,BLACK,64,0);
	
} 


























