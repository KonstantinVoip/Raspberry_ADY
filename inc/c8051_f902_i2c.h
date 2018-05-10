/**********************************************************************************************************************
*                                        (c) COPYRIGHT by ZAO RCZI Fort.                                          *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
* Module      : C8051_F902_I2C.h
* Description : Работа с шиной I2C КН-019
* Author      : Konstantin Shiluaev
******************************************************************************
******************************************************************************
* Module's Description Record:
* ============================
* $State: Debug$
* $Revision: 0.0.0 $
* $Date: 2015/03/02 10:40:51 $
******************************************************************************/
#ifndef __C8051_F902_I2C_H
#define __C8051_F902_I2C_H



void SMBus_Init ();
void SMBus_ISR  ();

void SMBus_ByteWrite(unsigned char addr, unsigned char dat);
unsigned char SMBus_ByteRead(unsigned char addr);


//Тестовые Функции
//Чтение из I2C EEPROM девайса КН-019
void test_kn19_single_byte_read_array();

#endif //__C8051_F902_I2C_H






