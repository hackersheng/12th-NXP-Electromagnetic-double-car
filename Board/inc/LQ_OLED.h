//#include "common.h"
//#include "includes.h"

#ifndef _Oled_H
#define _Oled_H

#include "include.h"
#include "common.h"
#include "MK60_gpio.h"


#define byte uint8
#define word uint16

//#define GPIO_PIN_MASK      0x1Fu    //0x1f=31,限制位数为0--31有效
//#define GPIO_PIN(x)        (((1)<<(x & GPIO_PIN_MASK)))  //把当前位置1

//#define LCD_Port_Init()            GPIO_INIT(PORTC,16, GPO,0);   \
 //                                  GPIO_INIT(PORTC,17, GPO,0);   \
   //                                GPIO_INIT(PORTC,18, GPO,0);   \
     //                              GPIO_INIT(PORTC,19, GPO,0);   

#define SCL_PIN PTC16
#define SDA_PIN PTC17
#define RST_PIN PTC18
#define DC_PIN  PTC19

//#define LCD_Port_Init()		OLED_GPIO_Init()

#define LCD_DC_HIGH     gpio_set(DC_PIN,1);
#define LCD_DC_LOW      gpio_set(DC_PIN,0);

#define LCD_SCL_HIGH   gpio_set(SCL_PIN,1);
#define LCD_SCL_LOW     gpio_set(SCL_PIN,0);

#define LCD_SDA_HIGH    gpio_set(SDA_PIN,1);
#define LCD_SDA_LOW     gpio_set(SDA_PIN,0);

#define LCD_RST_HIGH     gpio_set(RST_PIN,1);
#define LCD_RST_LOW      gpio_set(RST_PIN,0);


 extern byte longqiu96x64[768];
 void LCD_Port_Init(void);
 void LCD_Init(void);
 void LCD_CLS(void);
 void LCD_Set_Pos(byte x, byte y);
 void LCD_WrDat(byte data);
 void LCD_P6x8Str(byte x,byte y,byte ch[]);
 void LCD_P8x16Str(byte x,byte y,byte ch[]);
 void LCD_P14x16Str(byte x,byte y,byte ch[]);
 void LCD_Print(byte x, byte y, byte ch[]);
 void LCD_PutPixel(byte x,byte y);
 void LCD_Rectangle(byte x1,byte y1,byte x2,byte y2,byte gif);
 void Draw_LQLogo(void);
 void Draw_LibLogo(void);
 void Draw_BMP(byte x0,byte y0,byte x1,byte y1,byte bmp[]); 
 void LCD_Fill(byte dat);
 void LCD_P8x16SingleNum(uint8 x, uint8 y, int8 num);
 int LCD_P8x16Num(uint8 x, uint8 y, int16 num);
 
 
 void user_lcd_p6x8str_single(uint8 x,uint8 y,char c);
 void user_lcd_cls();
 void user_lcd_fill();
 void user_lcd_p6x8str(uint8 x,uint8 y,char ch[]);
 void user_lcd_p6x8clear(uint8 x,uint8 y,uint8 n);
 void user_lcd_p6x8num_single(uint8 x, uint8 y, int8 num);
 int user_lcd_p6x8num3(uint8 x, uint8 y, int16 num);
 int user_lcd_p6x8num4(uint8 x, uint8 y, int16 num);
 void lcd_refresh();
 
#endif