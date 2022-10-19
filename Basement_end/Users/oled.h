#ifndef __OLED_H
#define __OLED_H 

#include "main.h"
#include "stdlib.h"	

//-----------------OLED端口定义---------------- 

#define OLED_SCLK_Clr() HAL_GPIO_WritePin(oled_scl_GPIO_Port,oled_scl_Pin, GPIO_PIN_RESET)    ///< 拉低
#define OLED_SCLK_Set() HAL_GPIO_WritePin(oled_scl_GPIO_Port,oled_scl_Pin, GPIO_PIN_SET)      ///< 拉高

#define OLED_SDIN_Clr() HAL_GPIO_WritePin(oled_sda_GPIO_Port,oled_sda_Pin, GPIO_PIN_RESET)    ///< 拉低
#define OLED_SDIN_Set() HAL_GPIO_WritePin(oled_sda_GPIO_Port,oled_sda_Pin, GPIO_PIN_SET)      ///< 拉高


#define OLED_CMD  0	//写命令
#define OLED_DATA 1	//写数据

void OLED_ClearPoint(uint8_t x,uint8_t y);
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_WaitAck(void);
void Send_Byte(uint8_t dat);
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);

void OLED_Refresh(void);
void OLED_Clear(void);

void OLED_DrawPoint(uint8_t x,uint8_t y);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1);
void OLED_ScrollDisplay(uint8_t num,uint8_t space);
void OLED_WR_BP(uint8_t x,uint8_t y);
void OLED_ShowPicture(uint8_t x0,uint8_t y0,uint8_t x1,uint8_t y1,uint8_t BMP[]);
void OLED_Init(void);

#endif
