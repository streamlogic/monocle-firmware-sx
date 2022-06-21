/******************************************************************************
  Copyright (2022), Brilliant Labs Limited (Hong Kong)
  Licensed under the MIT License
*******************************************************************************/

// only change: added comments in lines 26~28

#ifndef _OV5640AF_H
#define _OV5640AF_H
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
//ALIENTEK STM32������
//OV5640 ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/12/30
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 
 

 // Auto Focus Initialization
 // used by ov5640_focus_init() in ov5640.c
 // MCU firmware, written starting at register 0x8000
 // See OV5640 Product Specification, Section 6.3
const unsigned char OV5640_AF_Config[] =
{
	0x02, 0x0f, 0xd6, 0x02, 0x0a, 0x39, 0xc2, 0x01, 0x22, 0x22, 0x00, 0x02, 0x0f, 0xb2, 0xe5, 0x1f, //0x8000,
	0x70, 0x72, 0xf5, 0x1e, 0xd2, 0x35, 0xff, 0xef, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xe4, 0xf6, 0x08, //0x8010,
	0xf6, 0x0f, 0xbf, 0x34, 0xf2, 0x90, 0x0e, 0x93, 0xe4, 0x93, 0xff, 0xe5, 0x4b, 0xc3, 0x9f, 0x50, //0x8020,
	0x04, 0x7f, 0x05, 0x80, 0x02, 0x7f, 0xfb, 0x78, 0xbd, 0xa6, 0x07, 0x12, 0x0f, 0x04, 0x40, 0x04, //0x8030,
	0x7f, 0x03, 0x80, 0x02, 0x7f, 0x30, 0x78, 0xbc, 0xa6, 0x07, 0xe6, 0x18, 0xf6, 0x08, 0xe6, 0x78, //0x8040,
	0xb9, 0xf6, 0x78, 0xbc, 0xe6, 0x78, 0xba, 0xf6, 0x78, 0xbf, 0x76, 0x33, 0xe4, 0x08, 0xf6, 0x78, //0x8050,
	0xb8, 0x76, 0x01, 0x75, 0x4a, 0x02, 0x78, 0xb6, 0xf6, 0x08, 0xf6, 0x74, 0xff, 0x78, 0xc1, 0xf6, //0x8060,
	0x08, 0xf6, 0x75, 0x1f, 0x01, 0x78, 0xbc, 0xe6, 0x75, 0xf0, 0x05, 0xa4, 0xf5, 0x4b, 0x12, 0x0a, //0x8070,
	0xff, 0xc2, 0x37, 0x22, 0x78, 0xb8, 0xe6, 0xd3, 0x94, 0x00, 0x40, 0x02, 0x16, 0x22, 0xe5, 0x1f, //0x8080,
	0xb4, 0x05, 0x23, 0xe4, 0xf5, 0x1f, 0xc2, 0x01, 0x78, 0xb6, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x78, //0x8090,
	0x4e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xa2, 0x37, 0xe4, 0x33, 0xf5, 0x3c, 0x90, 0x30, 0x28, 0xf0, //0x80a0,
	0x75, 0x1e, 0x10, 0xd2, 0x35, 0x22, 0xe5, 0x4b, 0x75, 0xf0, 0x05, 0x84, 0x78, 0xbc, 0xf6, 0x90, //0x80b0,
	0x0e, 0x8c, 0xe4, 0x93, 0xff, 0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x78, //0x80c0,
	0xbc, 0xe6, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0xef, 0x12, 0x0f, 0x0b, //0x80d0,
	0xd3, 0x78, 0xb7, 0x96, 0xee, 0x18, 0x96, 0x40, 0x0d, 0x78, 0xbc, 0xe6, 0x78, 0xb9, 0xf6, 0x78, //0x80e0,
	0xb6, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x8c, 0xe4, 0x93, 0x12, 0x0f, 0x0b, 0xc3, 0x78, //0x80f0,
	0xc2, 0x96, 0xee, 0x18, 0x96, 0x50, 0x0d, 0x78, 0xbc, 0xe6, 0x78, 0xba, 0xf6, 0x78, 0xc1, 0xa6, //0x8100,
	0x06, 0x08, 0xa6, 0x07, 0x78, 0xb6, 0xe6, 0xfe, 0x08, 0xe6, 0xc3, 0x78, 0xc2, 0x96, 0xff, 0xee, //0x8110,
	0x18, 0x96, 0x78, 0xc3, 0xf6, 0x08, 0xa6, 0x07, 0x90, 0x0e, 0x95, 0xe4, 0x18, 0x12, 0x0e, 0xe9, //0x8120,
	0x40, 0x02, 0xd2, 0x37, 0x78, 0xbc, 0xe6, 0x08, 0x26, 0x08, 0xf6, 0xe5, 0x1f, 0x64, 0x01, 0x70, //0x8130,
	0x4a, 0xe6, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xdf, 0x40, 0x05, 0x12, 0x0e, 0xda, 0x40, 0x39, 0x12, //0x8140,
	0x0f, 0x02, 0x40, 0x04, 0x7f, 0xfe, 0x80, 0x02, 0x7f, 0x02, 0x78, 0xbd, 0xa6, 0x07, 0x78, 0xb9, //0x8150,
	0xe6, 0x24, 0x03, 0x78, 0xbf, 0xf6, 0x78, 0xb9, 0xe6, 0x24, 0xfd, 0x78, 0xc0, 0xf6, 0x12, 0x0f, //0x8160,
	0x02, 0x40, 0x06, 0x78, 0xc0, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbf, 0xe6, 0xff, 0x78, 0xbe, 0xa6, //0x8170,
	0x07, 0x75, 0x1f, 0x02, 0x78, 0xb8, 0x76, 0x01, 0x02, 0x02, 0x4a, 0xe5, 0x1f, 0x64, 0x02, 0x60, //0x8180,
	0x03, 0x02, 0x02, 0x2a, 0x78, 0xbe, 0xe6, 0xff, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xe0, 0x40, 0x08, //0x8190,
	0x12, 0x0e, 0xda, 0x50, 0x03, 0x02, 0x02, 0x28, 0x12, 0x0f, 0x02, 0x40, 0x04, 0x7f, 0xff, 0x80, //0x81a0,
	0x02, 0x7f, 0x01, 0x78, 0xbd, 0xa6, 0x07, 0x78, 0xb9, 0xe6, 0x04, 0x78, 0xbf, 0xf6, 0x78, 0xb9, //0x81b0,
	0xe6, 0x14, 0x78, 0xc0, 0xf6, 0x18, 0x12, 0x0f, 0x04, 0x40, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, //0x81c0,
	0x00, 0x78, 0xbf, 0xa6, 0x07, 0xd3, 0x08, 0xe6, 0x64, 0x80, 0x94, 0x80, 0x40, 0x04, 0xe6, 0xff, //0x81d0,
	0x80, 0x02, 0x7f, 0x00, 0x78, 0xc0, 0xa6, 0x07, 0xc3, 0x18, 0xe6, 0x64, 0x80, 0x94, 0xb3, 0x50, //0x81e0,
	0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xbf, 0xa6, 0x07, 0xc3, 0x08, 0xe6, 0x64, 0x80, //0x81f0,
	0x94, 0xb3, 0x50, 0x04, 0xe6, 0xff, 0x80, 0x02, 0x7f, 0x33, 0x78, 0xc0, 0xa6, 0x07, 0x12, 0x0f, //0x8200,
	0x02, 0x40, 0x06, 0x78, 0xc0, 0xe6, 0xff, 0x80, 0x04, 0x78, 0xbf, 0xe6, 0xff, 0x78, 0xbe, 0xa6, //0x8210,
	0x07, 0x75, 0x1f, 0x03, 0x78, 0xb8, 0x76, 0x01, 0x80, 0x20, 0xe5, 0x1f, 0x64, 0x03, 0x70, 0x26, //0x8220,
	0x78, 0xbe, 0xe6, 0xff, 0xc3, 0x78, 0xc0, 0x12, 0x0e, 0xe0, 0x40, 0x05, 0x12, 0x0e, 0xda, 0x40, //0x8230,
	0x09, 0x78, 0xb9, 0xe6, 0x78, 0xbe, 0xf6, 0x75, 0x1f, 0x04, 0x78, 0xbe, 0xe6, 0x75, 0xf0, 0x05, //0x8240,
	0xa4, 0xf5, 0x4b, 0x02, 0x0a, 0xff, 0xe5, 0x1f, 0xb4, 0x04, 0x10, 0x90, 0x0e, 0x94, 0xe4, 0x78, //0x8250,
	0xc3, 0x12, 0x0e, 0xe9, 0x40, 0x02, 0xd2, 0x37, 0x75, 0x1f, 0x05, 0x22, 0x30, 0x01, 0x03, 0x02, //0x8260,
	0x04, 0xc0, 0x30, 0x02, 0x03, 0x02, 0x04, 0xc0, 0x90, 0x51, 0xa5, 0xe0, 0x78, 0x93, 0xf6, 0xa3, //0x8270,
	0xe0, 0x08, 0xf6, 0xa3, 0xe0, 0x08, 0xf6, 0xe5, 0x1f, 0x70, 0x3c, 0x75, 0x1e, 0x20, 0xd2, 0x35, //0x8280,
	0x12, 0x0c, 0x7a, 0x78, 0x7e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8b, 0xa6, 0x09, 0x18, 0x76, //0x8290,
	0x01, 0x12, 0x0c, 0x5b, 0x78, 0x4e, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8b, 0xe6, 0x78, 0x6e, //0x82a0,
	0xf6, 0x75, 0x1f, 0x01, 0x78, 0x93, 0xe6, 0x78, 0x90, 0xf6, 0x78, 0x94, 0xe6, 0x78, 0x91, 0xf6, //0x82b0,
	0x78, 0x95, 0xe6, 0x78, 0x92, 0xf6, 0x22, 0x79, 0x90, 0xe7, 0xd3, 0x78, 0x93, 0x96, 0x40, 0x05, //0x82c0,
	0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0x93, 0xe7, 0x78, 0x90, 0x96, 0xff, 0x78, 0x88, 0x76, //0x82d0,
	0x00, 0x08, 0xa6, 0x07, 0x79, 0x91, 0xe7, 0xd3, 0x78, 0x94, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, //0x82e0,
	0x80, 0x08, 0xc3, 0x79, 0x94, 0xe7, 0x78, 0x91, 0x96, 0xff, 0x12, 0x0c, 0x8e, 0x79, 0x92, 0xe7, //0x82f0,
	0xd3, 0x78, 0x95, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, 0x08, 0xc3, 0x79, 0x95, 0xe7, 0x78, //0x8300,
	0x92, 0x96, 0xff, 0x12, 0x0c, 0x8e, 0x12, 0x0c, 0x5b, 0x78, 0x8a, 0xe6, 0x25, 0xe0, 0x24, 0x4e, //0x8310,
	0xf8, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x78, 0x8a, 0xe6, 0x24, 0x6e, 0xf8, 0xa6, 0x09, 0x78, 0x8a, //0x8320,
	0xe6, 0x24, 0x01, 0xff, 0xe4, 0x33, 0xfe, 0xd3, 0xef, 0x94, 0x0f, 0xee, 0x64, 0x80, 0x94, 0x80, //0x8330,
	0x40, 0x04, 0x7f, 0x00, 0x80, 0x05, 0x78, 0x8a, 0xe6, 0x04, 0xff, 0x78, 0x8a, 0xa6, 0x07, 0xe5, //0x8340,
	0x1f, 0xb4, 0x01, 0x0a, 0xe6, 0x60, 0x03, 0x02, 0x04, 0xc0, 0x75, 0x1f, 0x02, 0x22, 0x12, 0x0c, //0x8350,
	0x7a, 0x78, 0x80, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0x12, 0x0c, 0x7a, 0x78, 0x82, 0xa6, 0x06, 0x08, //0x8360,
	0xa6, 0x07, 0x78, 0x6e, 0xe6, 0x78, 0x8c, 0xf6, 0x78, 0x6e, 0xe6, 0x78, 0x8d, 0xf6, 0x7f, 0x01, //0x8370,
	0xef, 0x25, 0xe0, 0x24, 0x4f, 0xf9, 0xc3, 0x78, 0x81, 0xe6, 0x97, 0x18, 0xe6, 0x19, 0x97, 0x50, //0x8380,
	0x0a, 0x12, 0x0c, 0x82, 0x78, 0x80, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 0x6e, 0x2f, 0xf9, 0x78, //0x8390,
	0x8c, 0xe6, 0xc3, 0x97, 0x50, 0x08, 0x74, 0x6e, 0x2f, 0xf8, 0xe6, 0x78, 0x8c, 0xf6, 0xef, 0x25, //0x83a0,
	0xe0, 0x24, 0x4f, 0xf9, 0xd3, 0x78, 0x83, 0xe6, 0x97, 0x18, 0xe6, 0x19, 0x97, 0x40, 0x0a, 0x12, //0x83b0,
	0x0c, 0x82, 0x78, 0x82, 0xa6, 0x04, 0x08, 0xa6, 0x05, 0x74, 0x6e, 0x2f, 0xf9, 0x78, 0x8d, 0xe6, //0x83c0,
	0xd3, 0x97, 0x40, 0x08, 0x74, 0x6e, 0x2f, 0xf8, 0xe6, 0x78, 0x8d, 0xf6, 0x0f, 0xef, 0x64, 0x10, //0x83d0,
	0x70, 0x9e, 0xc3, 0x79, 0x81, 0xe7, 0x78, 0x83, 0x96, 0xff, 0x19, 0xe7, 0x18, 0x96, 0x78, 0x84, //0x83e0,
	0xf6, 0x08, 0xa6, 0x07, 0xc3, 0x79, 0x8c, 0xe7, 0x78, 0x8d, 0x96, 0x08, 0xf6, 0xd3, 0x79, 0x81, //0x83f0,
	0xe7, 0x78, 0x7f, 0x96, 0x19, 0xe7, 0x18, 0x96, 0x40, 0x05, 0x09, 0xe7, 0x08, 0x80, 0x06, 0xc3, //0x8400,
	0x79, 0x7f, 0xe7, 0x78, 0x81, 0x96, 0xff, 0x19, 0xe7, 0x18, 0x96, 0xfe, 0x78, 0x86, 0xa6, 0x06, //0x8410,
	0x08, 0xa6, 0x07, 0x79, 0x8c, 0xe7, 0xd3, 0x78, 0x8b, 0x96, 0x40, 0x05, 0xe7, 0x96, 0xff, 0x80, //0x8420,
	0x08, 0xc3, 0x79, 0x8b, 0xe7, 0x78, 0x8c, 0x96, 0xff, 0x78, 0x8f, 0xa6, 0x07, 0xe5, 0x1f, 0x64, //0x8430,
	0x02, 0x70, 0x69, 0x90, 0x0e, 0x91, 0x93, 0xff, 0x18, 0xe6, 0xc3, 0x9f, 0x50, 0x72, 0x12, 0x0c, //0x8440,
	0x4a, 0x12, 0x0c, 0x2f, 0x90, 0x0e, 0x8e, 0x12, 0x0c, 0x38, 0x78, 0x80, 0x12, 0x0c, 0x6b, 0x7b, //0x8450,
	0x04, 0x12, 0x0c, 0x1d, 0xc3, 0x12, 0x06, 0x45, 0x50, 0x56, 0x90, 0x0e, 0x92, 0xe4, 0x93, 0xff, //0x8460,
	0x78, 0x8f, 0xe6, 0x9f, 0x40, 0x02, 0x80, 0x11, 0x90, 0x0e, 0x90, 0xe4, 0x93, 0xff, 0xd3, 0x78, //0x8470,
	0x89, 0xe6, 0x9f, 0x18, 0xe6, 0x94, 0x00, 0x40, 0x03, 0x75, 0x1f, 0x05, 0x12, 0x0c, 0x4a, 0x12, //0x8480,
	0x0c, 0x2f, 0x90, 0x0e, 0x8f, 0x12, 0x0c, 0x38, 0x78, 0x7e, 0x12, 0x0c, 0x6b, 0x7b, 0x40, 0x12, //0x8490,
	0x0c, 0x1d, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x18, 0x75, 0x1f, 0x05, 0x22, 0xe5, 0x1f, 0xb4, 0x05, //0x84a0,
	0x0f, 0xd2, 0x01, 0xc2, 0x02, 0xe4, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, 0x33, 0xd2, 0x36, //0x84b0,
	0x22, 0xef, 0x8d, 0xf0, 0xa4, 0xa8, 0xf0, 0xcf, 0x8c, 0xf0, 0xa4, 0x28, 0xce, 0x8d, 0xf0, 0xa4, //0x84c0,
	0x2e, 0xfe, 0x22, 0xbc, 0x00, 0x0b, 0xbe, 0x00, 0x29, 0xef, 0x8d, 0xf0, 0x84, 0xff, 0xad, 0xf0, //0x84d0,
	0x22, 0xe4, 0xcc, 0xf8, 0x75, 0xf0, 0x08, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xec, 0x33, 0xfc, //0x84e0,
	0xee, 0x9d, 0xec, 0x98, 0x40, 0x05, 0xfc, 0xee, 0x9d, 0xfe, 0x0f, 0xd5, 0xf0, 0xe9, 0xe4, 0xce, //0x84f0,
	0xfd, 0x22, 0xed, 0xf8, 0xf5, 0xf0, 0xee, 0x84, 0x20, 0xd2, 0x1c, 0xfe, 0xad, 0xf0, 0x75, 0xf0, //0x8500,
	0x08, 0xef, 0x2f, 0xff, 0xed, 0x33, 0xfd, 0x40, 0x07, 0x98, 0x50, 0x06, 0xd5, 0xf0, 0xf2, 0x22, //0x8510,
	0xc3, 0x98, 0xfd, 0x0f, 0xd5, 0xf0, 0xea, 0x22, 0xe8, 0x8f, 0xf0, 0xa4, 0xcc, 0x8b, 0xf0, 0xa4, //0x8520,
	0x2c, 0xfc, 0xe9, 0x8e, 0xf0, 0xa4, 0x2c, 0xfc, 0x8a, 0xf0, 0xed, 0xa4, 0x2c, 0xfc, 0xea, 0x8e, //0x8530,
	0xf0, 0xa4, 0xcd, 0xa8, 0xf0, 0x8b, 0xf0, 0xa4, 0x2d, 0xcc, 0x38, 0x25, 0xf0, 0xfd, 0xe9, 0x8f, //0x8540,
	0xf0, 0xa4, 0x2c, 0xcd, 0x35, 0xf0, 0xfc, 0xeb, 0x8e, 0xf0, 0xa4, 0xfe, 0xa9, 0xf0, 0xeb, 0x8f, //0x8550,
	0xf0, 0xa4, 0xcf, 0xc5, 0xf0, 0x2e, 0xcd, 0x39, 0xfe, 0xe4, 0x3c, 0xfc, 0xea, 0xa4, 0x2d, 0xce, //0x8560,
	0x35, 0xf0, 0xfd, 0xe4, 0x3c, 0xfc, 0x22, 0x75, 0xf0, 0x08, 0x75, 0x82, 0x00, 0xef, 0x2f, 0xff, //0x8570,
	0xee, 0x33, 0xfe, 0xcd, 0x33, 0xcd, 0xcc, 0x33, 0xcc, 0xc5, 0x82, 0x33, 0xc5, 0x82, 0x9b, 0xed, //0x8580,
	0x9a, 0xec, 0x99, 0xe5, 0x82, 0x98, 0x40, 0x0c, 0xf5, 0x82, 0xee, 0x9b, 0xfe, 0xed, 0x9a, 0xfd, //0x8590,
	0xec, 0x99, 0xfc, 0x0f, 0xd5, 0xf0, 0xd6, 0xe4, 0xce, 0xfb, 0xe4, 0xcd, 0xfa, 0xe4, 0xcc, 0xf9, //0x85a0,
	0xa8, 0x82, 0x22, 0xb8, 0x00, 0xc1, 0xb9, 0x00, 0x59, 0xba, 0x00, 0x2d, 0xec, 0x8b, 0xf0, 0x84, //0x85b0,
	0xcf, 0xce, 0xcd, 0xfc, 0xe5, 0xf0, 0xcb, 0xf9, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, //0x85c0,
	0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xeb, 0x33, 0xfb, 0x10, 0xd7, 0x03, 0x99, 0x40, 0x04, 0xeb, //0x85d0,
	0x99, 0xfb, 0x0f, 0xd8, 0xe5, 0xe4, 0xf9, 0xfa, 0x22, 0x78, 0x18, 0xef, 0x2f, 0xff, 0xee, 0x33, //0x85e0,
	0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xc9, 0x33, 0xc9, 0x10, 0xd7, 0x05, 0x9b, 0xe9, 0x9a, //0x85f0,
	0x40, 0x07, 0xec, 0x9b, 0xfc, 0xe9, 0x9a, 0xf9, 0x0f, 0xd8, 0xe0, 0xe4, 0xc9, 0xfa, 0xe4, 0xcc, //0x8600,
	0xfb, 0x22, 0x75, 0xf0, 0x10, 0xef, 0x2f, 0xff, 0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xcc, 0x33, //0x8610,
	0xcc, 0xc8, 0x33, 0xc8, 0x10, 0xd7, 0x07, 0x9b, 0xec, 0x9a, 0xe8, 0x99, 0x40, 0x0a, 0xed, 0x9b, //0x8620,
	0xfd, 0xec, 0x9a, 0xfc, 0xe8, 0x99, 0xf8, 0x0f, 0xd5, 0xf0, 0xda, 0xe4, 0xcd, 0xfb, 0xe4, 0xcc, //0x8630,
	0xfa, 0xe4, 0xc8, 0xf9, 0x22, 0xeb, 0x9f, 0xf5, 0xf0, 0xea, 0x9e, 0x42, 0xf0, 0xe9, 0x9d, 0x42, //0x8640,
	0xf0, 0xe8, 0x9c, 0x45, 0xf0, 0x22, 0xe8, 0x60, 0x0f, 0xec, 0xc3, 0x13, 0xfc, 0xed, 0x13, 0xfd, //0x8650,
	0xee, 0x13, 0xfe, 0xef, 0x13, 0xff, 0xd8, 0xf1, 0x22, 0xe8, 0x60, 0x0f, 0xef, 0xc3, 0x33, 0xff, //0x8660,
	0xee, 0x33, 0xfe, 0xed, 0x33, 0xfd, 0xec, 0x33, 0xfc, 0xd8, 0xf1, 0x22, 0xe4, 0x93, 0xfc, 0x74, //0x8670,
	0x01, 0x93, 0xfd, 0x74, 0x02, 0x93, 0xfe, 0x74, 0x03, 0x93, 0xff, 0x22, 0xe6, 0xfb, 0x08, 0xe6, //0x8680,
	0xf9, 0x08, 0xe6, 0xfa, 0x08, 0xe6, 0xcb, 0xf8, 0x22, 0xec, 0xf6, 0x08, 0xed, 0xf6, 0x08, 0xee, //0x8690,
	0xf6, 0x08, 0xef, 0xf6, 0x22, 0xa4, 0x25, 0x82, 0xf5, 0x82, 0xe5, 0xf0, 0x35, 0x83, 0xf5, 0x83, //0x86a0,
	0x22, 0xd0, 0x83, 0xd0, 0x82, 0xf8, 0xe4, 0x93, 0x70, 0x12, 0x74, 0x01, 0x93, 0x70, 0x0d, 0xa3, //0x86b0,
	0xa3, 0x93, 0xf8, 0x74, 0x01, 0x93, 0xf5, 0x82, 0x88, 0x83, 0xe4, 0x73, 0x74, 0x02, 0x93, 0x68, //0x86c0,
	0x60, 0xef, 0xa3, 0xa3, 0xa3, 0x80, 0xdf, 0x90, 0x38, 0x04, 0x78, 0x52, 0x12, 0x0b, 0xfd, 0x90, //0x86d0,
	0x38, 0x00, 0xe0, 0xfe, 0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 0x12, 0x0b, 0x9e, 0x90, 0x38, 0x10, //0x86e0,
	0x12, 0x0b, 0x92, 0x90, 0x38, 0x06, 0x78, 0x54, 0x12, 0x0b, 0xfd, 0x90, 0x38, 0x02, 0xe0, 0xfe, //0x86f0,
	0xa3, 0xe0, 0xfd, 0xed, 0xff, 0xc3, 0x12, 0x0b, 0x9e, 0x90, 0x38, 0x12, 0x12, 0x0b, 0x92, 0xa3, //0x8700,
	0xe0, 0xb4, 0x31, 0x07, 0x78, 0x52, 0x79, 0x52, 0x12, 0x0c, 0x13, 0x90, 0x38, 0x14, 0xe0, 0xb4, //0x8710,
	0x71, 0x15, 0x78, 0x52, 0xe6, 0xfe, 0x08, 0xe6, 0x78, 0x02, 0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, //0x8720,
	0xf9, 0x79, 0x53, 0xf7, 0xee, 0x19, 0xf7, 0x90, 0x38, 0x15, 0xe0, 0xb4, 0x31, 0x07, 0x78, 0x54, //0x8730,
	0x79, 0x54, 0x12, 0x0c, 0x13, 0x90, 0x38, 0x15, 0xe0, 0xb4, 0x71, 0x15, 0x78, 0x54, 0xe6, 0xfe, //0x8740,
	0x08, 0xe6, 0x78, 0x02, 0xce, 0xc3, 0x13, 0xce, 0x13, 0xd8, 0xf9, 0x79, 0x55, 0xf7, 0xee, 0x19, //0x8750,
	0xf7, 0x79, 0x52, 0x12, 0x0b, 0xd9, 0x09, 0x12, 0x0b, 0xd9, 0xaf, 0x47, 0x12, 0x0b, 0xb2, 0xe5, //0x8760,
	0x44, 0xfb, 0x7a, 0x00, 0xfd, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x5a, 0xa6, 0x06, 0x08, 0xa6, //0x8770,
	0x07, 0xaf, 0x45, 0x12, 0x0b, 0xb2, 0xad, 0x03, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x56, 0xa6, //0x8780,
	0x06, 0x08, 0xa6, 0x07, 0xaf, 0x48, 0x78, 0x54, 0x12, 0x0b, 0xb4, 0xe5, 0x43, 0xfb, 0xfd, 0x7c, //0x8790,
	0x00, 0x12, 0x04, 0xd3, 0x78, 0x5c, 0xa6, 0x06, 0x08, 0xa6, 0x07, 0xaf, 0x46, 0x7e, 0x00, 0x78, //0x87a0,
	0x54, 0x12, 0x0b, 0xb6, 0xad, 0x03, 0x7c, 0x00, 0x12, 0x04, 0xd3, 0x78, 0x58, 0xa6, 0x06, 0x08, //0x87b0,
	0xa6, 0x07, 0xc3, 0x78, 0x5b, 0xe6, 0x94, 0x08, 0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, 0x00, //0x87c0,
	0x08, 0x76, 0x08, 0xc3, 0x78, 0x5d, 0xe6, 0x94, 0x08, 0x18, 0xe6, 0x94, 0x00, 0x50, 0x05, 0x76, //0x87d0,
	0x00, 0x08, 0x76, 0x08, 0x78, 0x5a, 0x12, 0x0b, 0xc6, 0xff, 0xd3, 0x78, 0x57, 0xe6, 0x9f, 0x18, //0x87e0,
	0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x5a, 0xe6, 0x13, 0xfe, 0x08, 0xe6, 0x78, 0x57, 0x12, 0x0c, 0x08, //0x87f0,
	0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 0x78, 0x5e, 0x12, 0x0b, 0xbe, 0xff, 0xd3, 0x78, 0x59, 0xe6, //0x8800,
	0x9f, 0x18, 0xe6, 0x9e, 0x40, 0x0e, 0x78, 0x5c, 0xe6, 0x13, 0xfe, 0x08, 0xe6, 0x78, 0x59, 0x12, //0x8810,
	0x0c, 0x08, 0x80, 0x04, 0x7e, 0x00, 0x7f, 0x00, 0xe4, 0xfc, 0xfd, 0x78, 0x62, 0x12, 0x06, 0x99, //0x8820,
	0x78, 0x5a, 0x12, 0x0b, 0xc6, 0x78, 0x57, 0x26, 0xff, 0xee, 0x18, 0x36, 0xfe, 0x78, 0x66, 0x12, //0x8830,
	0x0b, 0xbe, 0x78, 0x59, 0x26, 0xff, 0xee, 0x18, 0x36, 0xfe, 0xe4, 0xfc, 0xfd, 0x78, 0x6a, 0x12, //0x8840,
	0x06, 0x99, 0x12, 0x0b, 0xce, 0x78, 0x66, 0x12, 0x06, 0x8c, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x08, //0x8850,
	0x12, 0x0b, 0xce, 0x78, 0x66, 0x12, 0x06, 0x99, 0x78, 0x54, 0x12, 0x0b, 0xd0, 0x78, 0x6a, 0x12, //0x8860,
	0x06, 0x8c, 0xd3, 0x12, 0x06, 0x45, 0x40, 0x0a, 0x78, 0x54, 0x12, 0x0b, 0xd0, 0x78, 0x6a, 0x12, //0x8870,
	0x06, 0x99, 0x78, 0x61, 0xe6, 0x90, 0x60, 0x01, 0xf0, 0x78, 0x65, 0xe6, 0xa3, 0xf0, 0x78, 0x69, //0x8880,
	0xe6, 0xa3, 0xf0, 0x78, 0x55, 0xe6, 0xa3, 0xf0, 0x7d, 0x01, 0x78, 0x61, 0x12, 0x0b, 0xe9, 0x24, //0x8890,
	0x01, 0x12, 0x0b, 0xa6, 0x78, 0x65, 0x12, 0x0b, 0xe9, 0x24, 0x02, 0x12, 0x0b, 0xa6, 0x78, 0x69, //0x88a0,
	0x12, 0x0b, 0xe9, 0x24, 0x03, 0x12, 0x0b, 0xa6, 0x78, 0x6d, 0x12, 0x0b, 0xe9, 0x24, 0x04, 0x12, //0x88b0,
	0x0b, 0xa6, 0x0d, 0xbd, 0x05, 0xd4, 0xc2, 0x0e, 0xc2, 0x06, 0x22, 0x85, 0x08, 0x41, 0x90, 0x30, //0x88c0,
	0x24, 0xe0, 0xf5, 0x3d, 0xa3, 0xe0, 0xf5, 0x3e, 0xa3, 0xe0, 0xf5, 0x3f, 0xa3, 0xe0, 0xf5, 0x40, //0x88d0,
	0xa3, 0xe0, 0xf5, 0x3c, 0xd2, 0x34, 0xe5, 0x41, 0x12, 0x06, 0xb1, 0x09, 0x31, 0x03, 0x09, 0x35, //0x88e0,
	0x04, 0x09, 0x3b, 0x05, 0x09, 0x3e, 0x06, 0x09, 0x41, 0x07, 0x09, 0x4a, 0x08, 0x09, 0x5b, 0x12, //0x88f0,
	0x09, 0x73, 0x18, 0x09, 0x89, 0x19, 0x09, 0x5e, 0x1a, 0x09, 0x6a, 0x1b, 0x09, 0xad, 0x80, 0x09, //0x8900,
	0xb2, 0x81, 0x0a, 0x1d, 0x8f, 0x0a, 0x09, 0x90, 0x0a, 0x1d, 0x91, 0x0a, 0x1d, 0x92, 0x0a, 0x1d, //0x8910,
	0x93, 0x0a, 0x1d, 0x94, 0x0a, 0x1d, 0x98, 0x0a, 0x17, 0x9f, 0x0a, 0x1a, 0xec, 0x00, 0x00, 0x0a, //0x8920,
	0x38, 0x12, 0x0f, 0x74, 0x22, 0x12, 0x0f, 0x74, 0xd2, 0x03, 0x22, 0xd2, 0x03, 0x22, 0xc2, 0x03, //0x8930,
	0x22, 0xa2, 0x37, 0xe4, 0x33, 0xf5, 0x3c, 0x02, 0x0a, 0x1d, 0xc2, 0x01, 0xc2, 0x02, 0xc2, 0x03, //0x8940,
	0x12, 0x0d, 0x0d, 0x75, 0x1e, 0x70, 0xd2, 0x35, 0x02, 0x0a, 0x1d, 0x02, 0x0a, 0x04, 0x85, 0x40, //0x8950,
	0x4a, 0x85, 0x3c, 0x4b, 0x12, 0x0a, 0xff, 0x02, 0x0a, 0x1d, 0x85, 0x4a, 0x40, 0x85, 0x4b, 0x3c, //0x8960,
	0x02, 0x0a, 0x1d, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x85, 0x40, 0x31, 0x85, 0x3f, 0x30, 0x85, 0x3e, //0x8970,
	0x2f, 0x85, 0x3d, 0x2e, 0x12, 0x0f, 0x46, 0x80, 0x1f, 0x75, 0x22, 0x00, 0x75, 0x23, 0x01, 0x74, //0x8980,
	0xff, 0xf5, 0x2d, 0xf5, 0x2c, 0xf5, 0x2b, 0xf5, 0x2a, 0x12, 0x0f, 0x46, 0x85, 0x2d, 0x40, 0x85, //0x8990,
	0x2c, 0x3f, 0x85, 0x2b, 0x3e, 0x85, 0x2a, 0x3d, 0xe4, 0xf5, 0x3c, 0x80, 0x70, 0x12, 0x0f, 0x16, //0x89a0,
	0x80, 0x6b, 0x85, 0x3d, 0x45, 0x85, 0x3e, 0x46, 0xe5, 0x47, 0xc3, 0x13, 0xff, 0xe5, 0x45, 0xc3, //0x89b0,
	0x9f, 0x50, 0x02, 0x8f, 0x45, 0xe5, 0x48, 0xc3, 0x13, 0xff, 0xe5, 0x46, 0xc3, 0x9f, 0x50, 0x02, //0x89c0,
	0x8f, 0x46, 0xe5, 0x47, 0xc3, 0x13, 0xff, 0xfd, 0xe5, 0x45, 0x2d, 0xfd, 0xe4, 0x33, 0xfc, 0xe5, //0x89d0,
	0x44, 0x12, 0x0f, 0x90, 0x40, 0x05, 0xe5, 0x44, 0x9f, 0xf5, 0x45, 0xe5, 0x48, 0xc3, 0x13, 0xff, //0x89e0,
	0xfd, 0xe5, 0x46, 0x2d, 0xfd, 0xe4, 0x33, 0xfc, 0xe5, 0x43, 0x12, 0x0f, 0x90, 0x40, 0x05, 0xe5, //0x89f0,
	0x43, 0x9f, 0xf5, 0x46, 0x12, 0x06, 0xd7, 0x80, 0x14, 0x85, 0x40, 0x48, 0x85, 0x3f, 0x47, 0x85, //0x8a00,
	0x3e, 0x46, 0x85, 0x3d, 0x45, 0x80, 0x06, 0x02, 0x06, 0xd7, 0x12, 0x0d, 0x7e, 0x90, 0x30, 0x24, //0x8a10,
	0xe5, 0x3d, 0xf0, 0xa3, 0xe5, 0x3e, 0xf0, 0xa3, 0xe5, 0x3f, 0xf0, 0xa3, 0xe5, 0x40, 0xf0, 0xa3, //0x8a20,
	0xe5, 0x3c, 0xf0, 0x90, 0x30, 0x23, 0xe4, 0xf0, 0x22, 0xc0, 0xe0, 0xc0, 0x83, 0xc0, 0x82, 0xc0, //0x8a30,
	0xd0, 0x90, 0x3f, 0x0c, 0xe0, 0xf5, 0x32, 0xe5, 0x32, 0x30, 0xe3, 0x74, 0x30, 0x36, 0x66, 0x90, //0x8a40,
	0x60, 0x19, 0xe0, 0xf5, 0x0a, 0xa3, 0xe0, 0xf5, 0x0b, 0x90, 0x60, 0x1d, 0xe0, 0xf5, 0x14, 0xa3, //0x8a50,
	0xe0, 0xf5, 0x15, 0x90, 0x60, 0x21, 0xe0, 0xf5, 0x0c, 0xa3, 0xe0, 0xf5, 0x0d, 0x90, 0x60, 0x29, //0x8a60,
	0xe0, 0xf5, 0x0e, 0xa3, 0xe0, 0xf5, 0x0f, 0x90, 0x60, 0x31, 0xe0, 0xf5, 0x10, 0xa3, 0xe0, 0xf5, //0x8a70,
	0x11, 0x90, 0x60, 0x39, 0xe0, 0xf5, 0x12, 0xa3, 0xe0, 0xf5, 0x13, 0x30, 0x01, 0x06, 0x30, 0x33, //0x8a80,
	0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x09, 0x30, 0x02, 0x06, 0x30, 0x33, 0x03, 0xd3, 0x80, 0x01, //0x8a90,
	0xc3, 0x92, 0x0a, 0x30, 0x33, 0x0c, 0x30, 0x03, 0x09, 0x20, 0x02, 0x06, 0x20, 0x01, 0x03, 0xd3, //0x8aa0,
	0x80, 0x01, 0xc3, 0x92, 0x0b, 0x90, 0x30, 0x01, 0xe0, 0x44, 0x40, 0xf0, 0xe0, 0x54, 0xbf, 0xf0, //0x8ab0,
	0xe5, 0x32, 0x30, 0xe1, 0x14, 0x30, 0x34, 0x11, 0x90, 0x30, 0x22, 0xe0, 0xf5, 0x08, 0xe4, 0xf0, //0x8ac0,
	0x30, 0x00, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x08, 0xe5, 0x32, 0x30, 0xe5, 0x12, 0x90, 0x56, //0x8ad0,
	0xa1, 0xe0, 0xf5, 0x09, 0x30, 0x31, 0x09, 0x30, 0x05, 0x03, 0xd3, 0x80, 0x01, 0xc3, 0x92, 0x0d, //0x8ae0,
	0x90, 0x3f, 0x0c, 0xe5, 0x32, 0xf0, 0xd0, 0xd0, 0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0x90, //0x8af0,
	0x0e, 0x7e, 0xe4, 0x93, 0xfe, 0x74, 0x01, 0x93, 0xff, 0xc3, 0x90, 0x0e, 0x7c, 0x74, 0x01, 0x93, //0x8b00,
	0x9f, 0xff, 0xe4, 0x93, 0x9e, 0xfe, 0xe4, 0x8f, 0x3b, 0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0xab, //0x8b10,
	0x3b, 0xaa, 0x3a, 0xa9, 0x39, 0xa8, 0x38, 0xaf, 0x4b, 0xfc, 0xfd, 0xfe, 0x12, 0x05, 0x28, 0x12, //0x8b20,
	0x0d, 0xe1, 0xe4, 0x7b, 0xff, 0xfa, 0xf9, 0xf8, 0x12, 0x05, 0xb3, 0x12, 0x0d, 0xe1, 0x90, 0x0e, //0x8b30,
	0x69, 0xe4, 0x12, 0x0d, 0xf6, 0x12, 0x0d, 0xe1, 0xe4, 0x85, 0x4a, 0x37, 0xf5, 0x36, 0xf5, 0x35, //0x8b40,
	0xf5, 0x34, 0xaf, 0x37, 0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0xa3, 0x12, 0x0d, 0xf6, 0x8f, 0x37, //0x8b50,
	0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0xe5, 0x3b, 0x45, 0x37, 0xf5, 0x3b, 0xe5, 0x3a, 0x45, 0x36, //0x8b60,
	0xf5, 0x3a, 0xe5, 0x39, 0x45, 0x35, 0xf5, 0x39, 0xe5, 0x38, 0x45, 0x34, 0xf5, 0x38, 0xe4, 0xf5, //0x8b70,
	0x22, 0xf5, 0x23, 0x85, 0x3b, 0x31, 0x85, 0x3a, 0x30, 0x85, 0x39, 0x2f, 0x85, 0x38, 0x2e, 0x02, //0x8b80,
	0x0f, 0x46, 0xe0, 0xa3, 0xe0, 0x75, 0xf0, 0x02, 0xa4, 0xff, 0xae, 0xf0, 0xc3, 0x08, 0xe6, 0x9f, //0x8b90,
	0xf6, 0x18, 0xe6, 0x9e, 0xf6, 0x22, 0xff, 0xe5, 0xf0, 0x34, 0x60, 0x8f, 0x82, 0xf5, 0x83, 0xec, //0x8ba0,
	0xf0, 0x22, 0x78, 0x52, 0x7e, 0x00, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x02, 0x04, 0xc1, 0xe4, 0xfc, //0x8bb0,
	0xfd, 0x12, 0x06, 0x99, 0x78, 0x5c, 0xe6, 0xc3, 0x13, 0xfe, 0x08, 0xe6, 0x13, 0x22, 0x78, 0x52, //0x8bc0,
	0xe6, 0xfe, 0x08, 0xe6, 0xff, 0xe4, 0xfc, 0xfd, 0x22, 0xe7, 0xc4, 0xf8, 0x54, 0xf0, 0xc8, 0x68, //0x8bd0,
	0xf7, 0x09, 0xe7, 0xc4, 0x54, 0x0f, 0x48, 0xf7, 0x22, 0xe6, 0xfc, 0xed, 0x75, 0xf0, 0x04, 0xa4, //0x8be0,
	0x22, 0x12, 0x06, 0x7c, 0x8f, 0x48, 0x8e, 0x47, 0x8d, 0x46, 0x8c, 0x45, 0x22, 0xe0, 0xfe, 0xa3, //0x8bf0,
	0xe0, 0xfd, 0xee, 0xf6, 0xed, 0x08, 0xf6, 0x22, 0x13, 0xff, 0xc3, 0xe6, 0x9f, 0xff, 0x18, 0xe6, //0x8c00,
	0x9e, 0xfe, 0x22, 0xe6, 0xc3, 0x13, 0xf7, 0x08, 0xe6, 0x13, 0x09, 0xf7, 0x22, 0xad, 0x39, 0xac, //0x8c10,
	0x38, 0xfa, 0xf9, 0xf8, 0x12, 0x05, 0x28, 0x8f, 0x3b, 0x8e, 0x3a, 0x8d, 0x39, 0x8c, 0x38, 0xab, //0x8c20,
	0x37, 0xaa, 0x36, 0xa9, 0x35, 0xa8, 0x34, 0x22, 0x93, 0xff, 0xe4, 0xfc, 0xfd, 0xfe, 0x12, 0x05, //0x8c30,
	0x28, 0x8f, 0x37, 0x8e, 0x36, 0x8d, 0x35, 0x8c, 0x34, 0x22, 0x78, 0x84, 0xe6, 0xfe, 0x08, 0xe6, //0x8c40,
	0xff, 0xe4, 0x8f, 0x37, 0x8e, 0x36, 0xf5, 0x35, 0xf5, 0x34, 0x22, 0x90, 0x0e, 0x8c, 0xe4, 0x93, //0x8c50,
	0x25, 0xe0, 0x24, 0x0a, 0xf8, 0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x22, 0xe6, 0xfe, 0x08, 0xe6, 0xff, //0x8c60,
	0xe4, 0x8f, 0x3b, 0x8e, 0x3a, 0xf5, 0x39, 0xf5, 0x38, 0x22, 0x78, 0x4e, 0xe6, 0xfe, 0x08, 0xe6, //0x8c70,
	0xff, 0x22, 0xef, 0x25, 0xe0, 0x24, 0x4e, 0xf8, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0x22, 0x78, 0x89, //0x8c80,
	0xef, 0x26, 0xf6, 0x18, 0xe4, 0x36, 0xf6, 0x22, 0x75, 0x89, 0x03, 0x75, 0xa8, 0x01, 0x75, 0xb8, //0x8c90,
	0x04, 0x75, 0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x15, 0x75, 0x37, 0x0d, 0x12, 0x0e, 0x9a, //0x8ca0,
	0x12, 0x00, 0x09, 0x12, 0x0f, 0x16, 0x12, 0x00, 0x06, 0xd2, 0x00, 0xd2, 0x34, 0xd2, 0xaf, 0x75, //0x8cb0,
	0x34, 0xff, 0x75, 0x35, 0x0e, 0x75, 0x36, 0x49, 0x75, 0x37, 0x03, 0x12, 0x0e, 0x9a, 0x30, 0x08, //0x8cc0,
	0x09, 0xc2, 0x34, 0x12, 0x08, 0xcb, 0xc2, 0x08, 0xd2, 0x34, 0x30, 0x0b, 0x09, 0xc2, 0x36, 0x12, //0x8cd0,
	0x02, 0x6c, 0xc2, 0x0b, 0xd2, 0x36, 0x30, 0x09, 0x09, 0xc2, 0x36, 0x12, 0x00, 0x0e, 0xc2, 0x09, //0x8ce0,
	0xd2, 0x36, 0x30, 0x0e, 0x03, 0x12, 0x06, 0xd7, 0x30, 0x35, 0xd3, 0x90, 0x30, 0x29, 0xe5, 0x1e, //0x8cf0,
	0xf0, 0xb4, 0x10, 0x05, 0x90, 0x30, 0x23, 0xe4, 0xf0, 0xc2, 0x35, 0x80, 0xc1, 0xe4, 0xf5, 0x4b, //0x8d00,
	0x90, 0x0e, 0x7a, 0x93, 0xff, 0xe4, 0x8f, 0x37, 0xf5, 0x36, 0xf5, 0x35, 0xf5, 0x34, 0xaf, 0x37, //0x8d10,
	0xae, 0x36, 0xad, 0x35, 0xac, 0x34, 0x90, 0x0e, 0x6a, 0x12, 0x0d, 0xf6, 0x8f, 0x37, 0x8e, 0x36, //0x8d20,
	0x8d, 0x35, 0x8c, 0x34, 0x90, 0x0e, 0x72, 0x12, 0x06, 0x7c, 0xef, 0x45, 0x37, 0xf5, 0x37, 0xee, //0x8d30,
	0x45, 0x36, 0xf5, 0x36, 0xed, 0x45, 0x35, 0xf5, 0x35, 0xec, 0x45, 0x34, 0xf5, 0x34, 0xe4, 0xf5, //0x8d40,
	0x22, 0xf5, 0x23, 0x85, 0x37, 0x31, 0x85, 0x36, 0x30, 0x85, 0x35, 0x2f, 0x85, 0x34, 0x2e, 0x12, //0x8d50,
	0x0f, 0x46, 0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 0x72, 0x12, 0x0d, 0xea, 0x12, 0x0f, 0x46, //0x8d60,
	0xe4, 0xf5, 0x22, 0xf5, 0x23, 0x90, 0x0e, 0x6e, 0x12, 0x0d, 0xea, 0x02, 0x0f, 0x46, 0xe5, 0x40, //0x8d70,
	0x24, 0xf2, 0xf5, 0x37, 0xe5, 0x3f, 0x34, 0x43, 0xf5, 0x36, 0xe5, 0x3e, 0x34, 0xa2, 0xf5, 0x35, //0x8d80,
	0xe5, 0x3d, 0x34, 0x28, 0xf5, 0x34, 0xe5, 0x37, 0xff, 0xe4, 0xfe, 0xfd, 0xfc, 0x78, 0x18, 0x12, //0x8d90,
	0x06, 0x69, 0x8f, 0x40, 0x8e, 0x3f, 0x8d, 0x3e, 0x8c, 0x3d, 0xe5, 0x37, 0x54, 0xa0, 0xff, 0xe5, //0x8da0,
	0x36, 0xfe, 0xe4, 0xfd, 0xfc, 0x78, 0x07, 0x12, 0x06, 0x56, 0x78, 0x10, 0x12, 0x0f, 0x9a, 0xe4, //0x8db0,
	0xff, 0xfe, 0xe5, 0x35, 0xfd, 0xe4, 0xfc, 0x78, 0x0e, 0x12, 0x06, 0x56, 0x12, 0x0f, 0x9d, 0xe4, //0x8dc0,
	0xff, 0xfe, 0xfd, 0xe5, 0x34, 0xfc, 0x78, 0x18, 0x12, 0x06, 0x56, 0x78, 0x08, 0x12, 0x0f, 0x9a, //0x8dd0,
	0x22, 0x8f, 0x3b, 0x8e, 0x3a, 0x8d, 0x39, 0x8c, 0x38, 0x22, 0x12, 0x06, 0x7c, 0x8f, 0x31, 0x8e, //0x8de0,
	0x30, 0x8d, 0x2f, 0x8c, 0x2e, 0x22, 0x93, 0xf9, 0xf8, 0x02, 0x06, 0x69, 0x00, 0x00, 0x00, 0x00, //0x8df0,
	0x12, 0x01, 0x17, 0x08, 0x31, 0x15, 0x53, 0x54, 0x44, 0x20, 0x20, 0x20, 0x20, 0x20, 0x13, 0x01, //0x8e00,
	0x10, 0x01, 0x56, 0x40, 0x1a, 0x30, 0x29, 0x7e, 0x00, 0x30, 0x04, 0x20, 0xdf, 0x30, 0x05, 0x40, //0x8e10,
	0xbf, 0x50, 0x03, 0x00, 0xfd, 0x50, 0x27, 0x01, 0xfe, 0x60, 0x00, 0x11, 0x00, 0x3f, 0x05, 0x30, //0x8e20,
	0x00, 0x3f, 0x06, 0x22, 0x00, 0x3f, 0x01, 0x2a, 0x00, 0x3f, 0x02, 0x00, 0x00, 0x36, 0x06, 0x07, //0x8e30,
	0x00, 0x3f, 0x0b, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x30, 0x01, 0x40, 0xbf, 0x30, 0x01, 0x00, //0x8e40,
	0xbf, 0x30, 0x29, 0x70, 0x00, 0x3a, 0x00, 0x00, 0xff, 0x3a, 0x00, 0x00, 0xff, 0x36, 0x03, 0x36, //0x8e50,
	0x02, 0x41, 0x44, 0x58, 0x20, 0x18, 0x10, 0x0a, 0x04, 0x04, 0x00, 0x03, 0xff, 0x64, 0x00, 0x00, //0x8e60,
	0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x06, 0x06, 0x00, 0x03, 0x51, 0x00, 0x7a, //0x8e70,
	0x50, 0x3c, 0x28, 0x1e, 0x10, 0x10, 0x50, 0x2d, 0x28, 0x16, 0x10, 0x10, 0x02, 0x00, 0x10, 0x0c, //0x8e80,
	0x10, 0x04, 0x0c, 0x6e, 0x06, 0x05, 0x00, 0xa5, 0x5a, 0x00, 0xae, 0x35, 0xaf, 0x36, 0xe4, 0xfd, //0x8e90,
	0xed, 0xc3, 0x95, 0x37, 0x50, 0x33, 0x12, 0x0f, 0xe2, 0xe4, 0x93, 0xf5, 0x38, 0x74, 0x01, 0x93, //0x8ea0,
	0xf5, 0x39, 0x45, 0x38, 0x60, 0x23, 0x85, 0x39, 0x82, 0x85, 0x38, 0x83, 0xe0, 0xfc, 0x12, 0x0f, //0x8eb0,
	0xe2, 0x74, 0x03, 0x93, 0x52, 0x04, 0x12, 0x0f, 0xe2, 0x74, 0x02, 0x93, 0x42, 0x04, 0x85, 0x39, //0x8ec0,
	0x82, 0x85, 0x38, 0x83, 0xec, 0xf0, 0x0d, 0x80, 0xc7, 0x22, 0x78, 0xbe, 0xe6, 0xd3, 0x08, 0xff, //0x8ed0,
	0xe6, 0x64, 0x80, 0xf8, 0xef, 0x64, 0x80, 0x98, 0x22, 0x93, 0xff, 0x7e, 0x00, 0xe6, 0xfc, 0x08, //0x8ee0,
	0xe6, 0xfd, 0x12, 0x04, 0xc1, 0x78, 0xc1, 0xe6, 0xfc, 0x08, 0xe6, 0xfd, 0xd3, 0xef, 0x9d, 0xee, //0x8ef0,
	0x9c, 0x22, 0x78, 0xbd, 0xd3, 0xe6, 0x64, 0x80, 0x94, 0x80, 0x22, 0x25, 0xe0, 0x24, 0x0a, 0xf8, //0x8f00,
	0xe6, 0xfe, 0x08, 0xe6, 0xff, 0x22, 0xe5, 0x3c, 0xd3, 0x94, 0x00, 0x40, 0x0b, 0x90, 0x0e, 0x88, //0x8f10,
	0x12, 0x0b, 0xf1, 0x90, 0x0e, 0x86, 0x80, 0x09, 0x90, 0x0e, 0x82, 0x12, 0x0b, 0xf1, 0x90, 0x0e, //0x8f20,
	0x80, 0xe4, 0x93, 0xf5, 0x44, 0xa3, 0xe4, 0x93, 0xf5, 0x43, 0xd2, 0x06, 0x30, 0x06, 0x03, 0xd3, //0x8f30,
	0x80, 0x01, 0xc3, 0x92, 0x0e, 0x22, 0xa2, 0xaf, 0x92, 0x32, 0xc2, 0xaf, 0xe5, 0x23, 0x45, 0x22, //0x8f40,
	0x90, 0x0e, 0x5d, 0x60, 0x0e, 0x12, 0x0f, 0xcb, 0xe0, 0xf5, 0x2c, 0x12, 0x0f, 0xc8, 0xe0, 0xf5, //0x8f50,
	0x2d, 0x80, 0x0c, 0x12, 0x0f, 0xcb, 0xe5, 0x30, 0xf0, 0x12, 0x0f, 0xc8, 0xe5, 0x31, 0xf0, 0xa2, //0x8f60,
	0x32, 0x92, 0xaf, 0x22, 0xd2, 0x01, 0xc2, 0x02, 0xe4, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, //0x8f70,
	0x33, 0xd2, 0x36, 0xd2, 0x01, 0xc2, 0x02, 0xf5, 0x1f, 0xf5, 0x1e, 0xd2, 0x35, 0xd2, 0x33, 0x22, //0x8f80,
	0xfb, 0xd3, 0xed, 0x9b, 0x74, 0x80, 0xf8, 0x6c, 0x98, 0x22, 0x12, 0x06, 0x69, 0xe5, 0x40, 0x2f, //0x8f90,
	0xf5, 0x40, 0xe5, 0x3f, 0x3e, 0xf5, 0x3f, 0xe5, 0x3e, 0x3d, 0xf5, 0x3e, 0xe5, 0x3d, 0x3c, 0xf5, //0x8fa0,
	0x3d, 0x22, 0xc0, 0xe0, 0xc0, 0x83, 0xc0, 0x82, 0x90, 0x3f, 0x0d, 0xe0, 0xf5, 0x33, 0xe5, 0x33, //0x8fb0,
	0xf0, 0xd0, 0x82, 0xd0, 0x83, 0xd0, 0xe0, 0x32, 0x90, 0x0e, 0x5f, 0xe4, 0x93, 0xfe, 0x74, 0x01, //0x8fc0,
	0x93, 0xf5, 0x82, 0x8e, 0x83, 0x22, 0x78, 0x7f, 0xe4, 0xf6, 0xd8, 0xfd, 0x75, 0x81, 0xcd, 0x02, //0x8fd0,
	0x0c, 0x98, 0x8f, 0x82, 0x8e, 0x83, 0x75, 0xf0, 0x04, 0xed, 0x02, 0x06, 0xa5,                   //0x8fe0
}; 
 #endif













