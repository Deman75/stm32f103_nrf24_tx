#ifndef SSD1306_H_
#define SSD1306_H_

#include "stm32f1xx_hal.h"
#include <string.h>

#define OLED_DEFAULT_ADDRESS 0x78
#define OLED_SETCONTRAST 0x81
#define OLED_DISPLAYALLON_RESUME 0xA4
#define OLED_DISPLAYALLON 0xA5
#define OLED_NORMALDISPLAY 0xA6
#define OLED_INVERTDISPLAY 0xA7
#define OLED_DISPLAYOFF 0xAE
#define OLED_DISPLAYON 0xAF
#define OLED_SETDISPLAYOFFSET 0xD3
#define OLED_SETCOMPINS 0xDA
#define OLED_SETVCOMDETECT 0xDB
#define OLED_SETDISPLAYCLOCKDIV 0xD5
#define OLED_SETPRECHARGE 0xD9
#define OLED_SETMULTIPLEX 0xA8
#define OLED_SETLOWCOLUMN 0x00
#define OLED_SETHIGHCOLUMN 0x10
#define OLED_SETSTARTLINE 0x40
#define OLED_MEMORYMODE 0x20
#define OLED_COLUMNADDR 0x21
#define OLED_PAGEADDR   0x22
#define OLED_COMSCANINC 0xC0
#define OLED_COMSCANDEC 0xC8
#define OLED_SEGREMAP 0xA0
#define OLED_CHARGEPUMP 0x8D
#define OLED_SWITCHCAPVCC 0x2
#define OLED_NOP 0xE3

#define OLED_WIDTH 128
#define OLED_HEIGHT 32
#define OLED_BUFFERSIZE (OLED_WIDTH*OLED_HEIGHT)/8
#define OLED_DEFAULT_SPACE 5

#define COMAND 0x00
#define DATA 0x40

uint8_t LCD_init(void);
void LCD_Clear(void);
void SSD1306_sendCmd(uint8_t cmd_s);
void SSD1306_sendData(uint8_t data_s);
void LCD_Goto(unsigned char x, unsigned char y);
void LCD_DrawPixel(uint16_t x, uint16_t y, uint8_t color);
void PrintBuf(void);
void LCD_BarHorizont(uint8_t xStart, uint8_t yStart, uint8_t height, uint8_t lenght, uint8_t percent, uint8_t color, uint8_t BkgColor);
void LCD_Line  (uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void LCD_Circle (uint16_t x, uint16_t y, uint16_t radius, uint16_t color);
void LCD_Rect (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_RectFill (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);






#endif //SSD1306_H_