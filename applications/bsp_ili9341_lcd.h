#ifndef __BSP_ILI9341_LCD_H
#define	__BSP_ILI9341_LCD_H

#include "rtdef.h"
#include "drv_gpio.h"
/***************************************************************************************
2^26 =0X0400 0000 = 64MB,每个 BANK 有4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF

选择BANK1-BORSRAM1 连接 TFT，地址范围为0X6000 0000 ~ 0X63FF FFFF
FSMC_A16 接LCD的DC(寄存器/数据选择)脚
寄存器基地址 = 0X60000000
RAM基地址 = 0X60020000 = 0X60000000+2^16*2 = 0X60000000 + 0X20000 = 0X60020000
当选择不同的地址线时，地址要重新计算  
****************************************************************************************/



#define	LCD_BL_GPIO_Port		GPIOB
#define LCD_BL_Pin					GPIO_PIN_1

#define LCD_RESET_GPIO_Port	GPIOE
#define LCD_RESET_Pin				GPIO_PIN_1

#define KEY1_GPIO_Port 			GPIOA
#define KEY1_Pin						GPIO_PIN_0

#define KEY2_GPIO_Port 			GPIOC
#define KEY2_Pin						GPIO_PIN_13

void MX_GPIO_Init(void);
void MX_FSMC_Init(void);



#define LCD_RST    GET_PIN(E, 1)

#define Bank1_LCD_C    ((rt_uint32_t)0x60000000)	   //Disp Reg ADDR
#define Bank1_LCD_D    ((rt_uint32_t)0x60020000)     //Disp Data ADDR       // A16 PD11

/*选定LCD指定寄存器*/
#define LCD_WR_REG(index)    ((*( rt_uint16_t *) (Bank1_LCD_C)) = ((rt_uint16_t)index))
/*往LCD GRAM写入数据*/
#define LCD_WR_Data(val)       ((*( rt_uint16_t *) (Bank1_LCD_D)) = ((rt_uint16_t)(val)))

#define LCD_ILI9341_CMD(index)       LCD_WR_REG(index)
#define LCD_ILI9341_Parameter(val)	 LCD_WR_Data(val)

#define COLUMN		240
#define PAGE		320	

// SRT 是string的缩写
#define STR_WIDTH		6		/* 字符宽度 */
#define STR_HEIGHT		12		/* 字符高度 */

#define BACKGROUND		BLACK

#define WHITE		 		 0xFFFF	/* 白色 */
#define BLACK        0x0000	/* 黑色 */
#define GREY         0xF7DE	/* 灰色 */
#define BLUE         0x001F	/* 蓝色 */
#define BLUE2        0x051F	/* 浅蓝色 */
#define RED          0xF800	/* 红色 */
#define MAGENTA      0xF81F	/* 红紫色，洋红色 */
#define GREEN        0x07E0	/* 绿色 */
#define CYAN         0x7FFF	/* 蓝绿色，青色 */
#define YELLOW       0xFFE0	/* 黄色 */

void LCD_Init(void);
void Lcd_GramScan( rt_uint16_t option );
void LCD_Clear(rt_uint16_t x, rt_uint16_t y, rt_uint16_t width, rt_uint16_t height, uint16_t color);
void LCD_SetCursor(rt_uint16_t x, rt_uint16_t y);
void LCD_OpenWindow(rt_uint16_t x, rt_uint16_t y, rt_uint16_t width, rt_uint16_t height);
void LCD_SetPoint(rt_uint16_t x , rt_uint16_t y , rt_uint16_t color);
rt_uint16_t LCD_GetPoint(uint16_t x , rt_uint16_t y);
void LCD_DispChar(rt_uint16_t x, rt_uint16_t y, rt_uint8_t ascii, rt_uint16_t color);
void LCD_DispStr(rt_uint16_t x, rt_uint16_t y, rt_uint8_t *pstr, rt_uint16_t color);
void LCD_DisNum(rt_uint16_t x, rt_uint16_t y, rt_uint32_t num, rt_uint16_t color);


#endif /* __BSP_ILI9341_LCD_H */
