/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		�����ļ�
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Target core		STC32G12K128
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2018-05-24
 ********************************************************************************************************************/


#ifndef _SEEKFREE_FONT_h
#define _SEEKFREE_FONT_h

#include "common.h"




extern const uint8 code tft_ascii[95][16];
extern const uint8 code oled_8x16[];
extern const uint8 code oled_6x8[][6];

//-------������ɫ----------
#define RED          	    0xF800	//��ɫ
#define BLUE         	    0x001F  //��ɫ
#define YELLOW       	    0xFFE0	//��ɫ
#define GREEN        	    0x07E0	//��ɫ
#define WHITE        	    0xFFFF	//��ɫ
#define BLACK        	    0x0000	//��ɫ 
#define GRAY  				0X8430 	//��ɫ
#define BROWN 				0XBC40 	//��ɫ
#define PURPLE    			0XF81F	//��ɫ
#define PINK    		    0XFE19	//��ɫ


#endif
