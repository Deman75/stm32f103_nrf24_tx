#include "ssd1306.h"
#include "New_CalibriLight14.h"

extern I2C_HandleTypeDef hi2c1;

static uint16_t gg; // для счетчика в while
static uint8_t temp[2] = {0,0};
static uint8_t SSD1306_Buffer[OLED_WIDTH * OLED_HEIGHT / 8] = {0x00};
//unsigned char LCD_X, LCD_Y;

#define abs(x)   ((x) > 0 ? (x) : -(x))


void SSD1306_sendCmd(uint8_t cmd_s)
{
	temp[0] = COMAND;
	temp[1] = cmd_s;
	HAL_I2C_Master_Transmit(&hi2c1,OLED_DEFAULT_ADDRESS, temp,2,100);
}

void SSD1306_sendData(uint8_t data_s)
{
	temp[0] = DATA;
	temp[1] = data_s;
	HAL_I2C_Master_Transmit(&hi2c1,OLED_DEFAULT_ADDRESS, temp,2,100);
}

uint8_t LCD_init(void)
{

					/* Check if LCD connected to I2C */
			if (HAL_I2C_IsDeviceReady(&hi2c1, OLED_DEFAULT_ADDRESS, 1, 20000) != HAL_OK) {
				/* Return false */
				return 0;
			}
	
      // Turn display off
      SSD1306_sendCmd(OLED_DISPLAYOFF);

      SSD1306_sendCmd(OLED_SETDISPLAYCLOCKDIV);
      SSD1306_sendCmd(0x80);

      SSD1306_sendCmd(OLED_SETMULTIPLEX);
      SSD1306_sendCmd(0x1F);//128x32
      //SSD1306_sendCmd(0x3F);//128x64
      
      SSD1306_sendCmd(OLED_SETDISPLAYOFFSET);
      SSD1306_sendCmd(0x00);
      
      SSD1306_sendCmd(OLED_SETSTARTLINE | 0x00);//0
      
      // We use internal charge pump
      SSD1306_sendCmd(OLED_CHARGEPUMP);
      SSD1306_sendCmd(0x14);
      
      // Horizontal memory mode
      SSD1306_sendCmd(OLED_MEMORYMODE);
      SSD1306_sendCmd(0x00);
      
      SSD1306_sendCmd(OLED_SEGREMAP | 0x1);

      SSD1306_sendCmd(OLED_COMSCANDEC);

      SSD1306_sendCmd(OLED_SETCOMPINS);
      SSD1306_sendCmd(0x02);//128x32
      //SSD1306_sendCmd(0x12);//128x64
      
      // Max contrast
      SSD1306_sendCmd(OLED_SETCONTRAST);
      SSD1306_sendCmd(0xCF);//0xCF

      SSD1306_sendCmd(OLED_SETPRECHARGE);
      SSD1306_sendCmd(0xF1);

      SSD1306_sendCmd(OLED_SETVCOMDETECT);
      SSD1306_sendCmd(0x40);//0x40

      SSD1306_sendCmd(OLED_DISPLAYALLON_RESUME);

      // Non-inverted display
      SSD1306_sendCmd(OLED_NORMALDISPLAY);

      // Turn display back on
      //SSD1306_sendCmd(OLED_DISPLAYON);
			
			LCD_Clear();
			PrintBuf();
			SSD1306_sendCmd(OLED_DISPLAYON);
			
			return 1;
}

void PrintBuf(void){
	uint16_t i;
	LCD_Goto(0,0);
	for (i=0;i<512;i++){
		SSD1306_sendData(SSD1306_Buffer[i]);
	}
}

void LCD_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
	x-=1;
	y-=1;
	
	if (x >= OLED_HEIGHT || y >= OLED_WIDTH) 
	{
		/* Error */
		return;
	}
	
	/* Set color */
	if (color == 1) {
		SSD1306_Buffer[y + (x / 8) * OLED_WIDTH] |= 1 << (x % 8);
	} else {
		SSD1306_Buffer[y + (x / 8) * OLED_WIDTH] &= ~(1 << (x % 8));
	}
}

void LCD_Clear(void)
{
	uint16_t i;
	for (i=0; i<512; i++) {
	 SSD1306_Buffer[i]=0x00;
	} 
}

void LCD_Goto(unsigned char x, unsigned char y)
{
//LCD_X = x;
//LCD_Y = y;
SSD1306_sendCmd(0xB0 + y);
SSD1306_sendCmd(x & 0xf);
SSD1306_sendCmd(0x10 | (x >> 4));
}

void LCD_BarHorizont(uint8_t xStart, uint8_t yStart, uint8_t height, uint8_t lenght, uint8_t percent, uint8_t color, uint8_t BkgColor)
{
	char str[20];
	float fillLenght;
	LCD_Rect(xStart, yStart, xStart+height, yStart+lenght, color);
	LCD_RectFill(xStart+1, yStart+1, xStart+height-1, yStart+lenght-1, BkgColor);
	
	
	if (percent<0) percent = 0;
	if (percent>100) percent = 100;

	fillLenght = ((lenght-2)/100.0)*percent;
	
	LCD_RectFill(xStart+2, yStart+2, xStart+height-2, yStart+fillLenght, color);
	
	
}

void LCD_Line  (uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
	int16_t A, B, sign;
	A=y1-y0;
	B=x0-x1;
	if (abs(A) > abs(B))
			sign=1;
		else
			sign=-1;
	int16_t signa,signb;
	if (A<0)
			signa = -1;
		else
			signa = 1;
	if (B<0)
			signb = -1;
		else
			signb = 1;
	int16_t f=0;
	LCD_DrawPixel (x0,y0,color);
	int16_t x = x0, y=y0;
	if (sign == -1)
		{
			do
			{
				f += A*signa;
				if (f>0)
				{
					f -= B*signb;
					y += signa;
				}
				x -=signb;
				LCD_DrawPixel(x,y,color);
			} while (x != x1 || y !=y1);
		}
	else
		{
			do
			{
				f += B*signb;
				if (f>0)
				{
					f -= A*signa;
					x -= signb;
				}
				y += signa;
				LCD_DrawPixel(x,y,color);
			} while (x != x1 || y != y1);
		}
}


void LCD_Circle (uint16_t x, uint16_t y, uint16_t radius, uint16_t color)
{
  int dx=0, dy, d;
  dy=radius; 
  d=3-((int16_t)radius<<1); 
  while (dx<=dy) 
  { 
    LCD_DrawPixel(x+dx,y+dy,color); 
    LCD_DrawPixel(x-dx,y+dy,color); 
    LCD_DrawPixel(x+dx,y-dy,color); 
    LCD_DrawPixel(x-dx,y-dy,color); 
    LCD_DrawPixel(x+dy,y+dx,color); 
    LCD_DrawPixel(x-dy,y+dx,color); 
    LCD_DrawPixel(x+dy,y-dx,color); 
    LCD_DrawPixel(x-dy,y-dx,color); 
    if (d < 0) 
      d += ((dx<<2) + 6); 
    else
    { 
      d += (((dx - dy)<<2) + 10); 
      dy--; 
    } 
    dx++; 
  }
}

void LCD_Rect (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t i;
	for (i=x1;i<=x2;i++)
	{
		LCD_DrawPixel(i,y1,color);
		LCD_DrawPixel(i,y2,color);
	}
	for (i=y1;i<=y2;i++)
	{
		LCD_DrawPixel(x1,i,color);
		LCD_DrawPixel(x2,i,color);
	}
}

void LCD_RectFill (uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	uint16_t x,y;
	for (x=x1;x<=x2;x++)
		for (y=y1;y<=y2;y++)
			LCD_DrawPixel(x,y,color);
}