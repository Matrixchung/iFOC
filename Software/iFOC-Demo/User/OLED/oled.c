/*
 * oled.c
 *
 *  Created on: Nov 2, 2022
 *      Author: Matrix
 */
#include "oled.h"
#include "oledfont.h"

uint8_t OLED_GRAM[OLED_WIDTH][OLED_PAGE];
uint8_t OLED_HardFault = 0;

static int inHandlerMode(void)
{
	return __get_IPSR();
}

void OLED_WriteByte(uint8_t data, uint8_t cmd)
{
	if(!OLED_HardFault)
	{
		if(inHandlerMode() != 0) vPortEnterCritical();
		// I2C_LL_Write(I2C1, OLED_ADDR, cmd==OLED_CMD?0x00:0x40, data);
		OLED_HardFault = I2C_Soft_Write(OLED_ADDR, cmd==OLED_CMD?0x00:0x40, data);
		if(inHandlerMode() != 0) vPortExitCritical();
	}
}

void OLED_Display_On(void)
{
	OLED_WriteByte(0x8D, OLED_CMD); // SET DCDC
	OLED_WriteByte(0x14, OLED_CMD); // DCDC ON
	OLED_WriteByte(0xAF, OLED_CMD); // DISPLAY ON
}

void OLED_Display_Off(void)
{
	OLED_WriteByte(0x8D, OLED_CMD); // SET DCDC
	OLED_WriteByte(0x10, OLED_CMD); // DCDC OFF
	OLED_WriteByte(0xAE, OLED_CMD); // DISPLAY OFF
}
void OLED_RefreshGram(void)
{
	if(!OLED_HardFault)
	{
		for(uint8_t i=0;i<OLED_PAGE;i++){
			OLED_WriteByte(0xB0+i, OLED_CMD); // 设置页地址
			OLED_WriteByte(0x00, OLED_CMD); // 设置列低地址
			OLED_WriteByte(0x10, OLED_CMD); // 设置列高地址
			for(uint8_t j=0;j<OLED_WIDTH;j++) OLED_WriteByte(OLED_GRAM[j][i], OLED_DATA);
		}
	}
}
void OLED_Clear(uint8_t refresh)
{
	memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
	if(refresh==OLED_REFRESH) OLED_RefreshGram();
}
// state: 1: 填充 0: 清空
void OLED_DrawPixel(uint8_t x, uint8_t y, uint8_t state)
{
	if(x>OLED_WIDTH-1||y>OLED_HEIGHT-1) return; // 超出范围
	if(state) OLED_GRAM[x][7-y/8] |= (1<<(7-y%8));
	else OLED_GRAM[x][7-y/8] &= ~(1<<(7-y%8));
}
// size: 16: 1608字体 12: 1206字体
// 1206: 12x6; 1608: 16x8
// mode: 0: 反白 1: 正常
void OLED_DrawChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t size, uint8_t mode)
{
	chr-=' '; // 字库内' '对应0
	uint8_t y0 = y;
	uint8_t temp = 0;
	if(size!=BIG_FONT&&size!=SMALL_FONT) size = SMALL_FONT;
	for(uint8_t i=0;i<size;i++){
		// temp = size==BIG_FONT?oled_asc2_1608[chr][i]:oled_asc2_1206[chr][i];
		temp = oled_asc2_1206[chr][i];
		for(uint8_t j=0;j<8;j++){
			OLED_DrawPixel(x,y,temp&0x80?mode:!mode);
			temp<<=1;
			y++;
			if((y-y0)==size){
				y=y0;
				x++;
				break;
			}
		}
	}
}
void OLED_DrawNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t size)
{
	uint8_t temp = 0, letter = 0;
	while(num>0){
		letter = num % 10;
		num /= 10;
		OLED_DrawChar(x+(size/2)*temp, y, letter+'0', size, 1);
		temp++;
	}
}
void OLED_DrawString(uint8_t x,uint8_t y, uint8_t size, const char *p)
{
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=size;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear(OLED_REFRESH);}
        OLED_DrawChar(x,y,*p,size,1);
        x+=size/2;
        p++;
    }
}
void OLED_printf(uint8_t x, uint8_t y, uint8_t size, char *fmt, ...)
{
	char buffer[100];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, 100, fmt, args);
	OLED_DrawString(x, y, size, buffer);
	va_end(args);
}
void OLED_Init(void)
{
	OLED_WriteByte(0xAE, OLED_CMD); // 关闭显示
	OLED_WriteByte(0xD5, OLED_CMD); // 设置时钟分频因子, 震荡频率
	OLED_WriteByte(0x80, OLED_CMD); // [3:0],分频因子;[7:4],震荡频率
	OLED_WriteByte(0xA8, OLED_CMD); // 设置驱动路数
	OLED_WriteByte(0x3F, OLED_CMD); // 默认0x3F(1/64)
	OLED_WriteByte(0xD3, OLED_CMD); // 设置显示偏移
	OLED_WriteByte(0x00, OLED_CMD); // 默认0

	OLED_WriteByte(0x40, OLED_CMD); // 设置显示开始行 [5:0], 行数

	OLED_WriteByte(0x8D, OLED_CMD); // 电荷泵设置
	OLED_WriteByte(0x14, OLED_CMD); // Bit2, 开启/关闭
	OLED_WriteByte(0x20, OLED_CMD); // 设置内存地址模式
	OLED_WriteByte(0x02, OLED_CMD); // [1:0], 00,列地址模式; 01,行地址模式; 10,页地址模式; 默认10;
	OLED_WriteByte(0xA1, OLED_CMD); // 段重定义设置, bit0:0,0->0;1,0->127;
	OLED_WriteByte(0xC0, OLED_CMD); // 设置 COM 扫描方向, bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
	OLED_WriteByte(0xDA, OLED_CMD); // 设置 COM 硬件引脚配置
	OLED_WriteByte(0x12, OLED_CMD); // [5:4] 配置

	OLED_WriteByte(0x81, OLED_CMD); // 对比度设置
	OLED_WriteByte(0xEF, OLED_CMD); // 1~255; 默认 0x7F (亮度设置,越大越亮)
	OLED_WriteByte(0xD9, OLED_CMD); // 设置预充电周期
	OLED_WriteByte(0xF1, OLED_CMD); // [3:0],PHASE 1;[7:4],PHASE 2; 分频可设置为 0xF0
	OLED_WriteByte(0xDB, OLED_CMD); // 设置 VCOMH 电压倍率
	OLED_WriteByte(0x30, OLED_CMD); // [6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WriteByte(0xA4, OLED_CMD); // 全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
	OLED_WriteByte(0xA6, OLED_CMD); // 设置显示方式;bit0:1,反相显示;0,正常显示
	OLED_WriteByte(0xAF, OLED_CMD); // 开启显示
	OLED_Clear(OLED_REFRESH);
}
