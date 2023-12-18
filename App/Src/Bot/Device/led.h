//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//�о�԰����
//���̵�ַ��http://shop73023976.taobao.com/?spm=2013.1.0.0.M4PqC2
//
//  �� �� ��   : main.c
//  �� �� ��   : v2.0
//  ��    ��   : HuangKai
//  ��������   : 2018-03-29
//  ����޸�   : 
//  ��������   : OLED 4�ӿ���ʾ����(STM32F0ϵ��)
//              ˵��: 
//              ----------------------------------------------------------------
//              GND    ��Դ��
//              VCC  ��3.3v��Դ
//              D0   ��PA9��CLK��
//              D1   ��PA7��MOSI��
//              RES  ��PA6
//              DC   ��PA5
//              CS   ��PA4               
//              ----------------------------------------------------------------
// �޸���ʷ   :
// ��    ��   : 
// ��    ��   : HuangKai
// �޸�����   : �����ļ�
//��Ȩ���У�����ؾ���
//Copyright(C) �о�԰����2014/3/16
//All rights reserved
//******************************************************************************/
//The x-direction limit value is 121
//The y-direction limit value is 6
//The minimum width of the character x direction is 7(recommand 8
//The minimum width of the character y direction is 2

#ifndef __LED_H
#define __LED_H

#include "main.h"
#define LED_GPIO_CLKA   RCC_AHBPeriph_GPIOA 
#define LED_GPIO_CLKB   RCC_AHBPeriph_GPIOB
#define LED_PORT   	   GPIOB
#define LED_PIN        GPIO_Pin_1
#define Max_Column	128
#define Max_Row		64
#define SIZE 16
void LED_Init(void);
void LED_ON(void);
void LED_OFF(void);

#define OLED_RST_Clr() OLED_RES_GPIO_Port->BRR=OLED_RES_Pin//RES
#define OLED_RST_Set() OLED_RES_GPIO_Port->BSRR=OLED_RES_Pin

#define OLED_DC_Clr() OLED_DC_GPIO_Port->BRR=OLED_DC_Pin//DC
#define OLED_DC_Set() OLED_DC_GPIO_Port->BSRR=OLED_DC_Pin
 		     
#define OLED_CMD  0	//д����
#define OLED_DATA 1	//д����


//OLED�����ú���
void OLED_WR_Byte(uint8_t dat,uint8_t cmd);	    
void OLED_Display_On(void);
void OLED_Display_Off(void);	   							   		    
void OLED_Init(void);
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_Fill(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t dot);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowString(uint8_t x,uint8_t y, uint8_t *p);	 
void OLED_Set_Pos(unsigned char x, unsigned char y);
void OLED_ShowCHinese(uint8_t x,uint8_t y,uint8_t no);
void OLED_DrawBMP(unsigned char x0, unsigned char y0,unsigned char x1, unsigned char y1,unsigned char BMP[]);
#endif  
