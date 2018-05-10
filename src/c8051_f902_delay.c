 /**********************************************************************************************************************
*                                        (c) COPYRIGHT by ZAO RCZI FORT.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
* Module        :�8051_f902_delay.c
* Description : ������� ������ � ���������� ���������������� 8051_f902
* Author      : Konstantin Shiluaev
******************************************************************************
******************************************************************************
* Module's Description Record:
* ============================
* $State: Debug$
* $Revision: 0.0.1 $
* $Description: ������ ������� ����� ������ � ���������� ��� 24.5 [���]
* $Date: 2016/08/31 10:40:51 $
* $Revision: 0.0.2 $
******************************************************************************/

//------------------------------------------------------------------------------------------------------------------  
#include <compiler_defs.h>    
#include <C8051F912_defs.h>



#include <c8051_f902_delay.h>


/*****************************************************************************
Syntax:  void Timer1_Init(void)  
Remarks: 8-bit counter/timer with autoreload Mode  SYSCLK 24500000  (24.5Mhz)  
         for I2C ����   
*******************************************************************************/
void Timer1_Init()
 {
	 // Make sure the Timer can produce the appropriate frequency in 8-bit mode
	 // Supported SMBus Frequencies range from 10kHz to 100kHz. The CKCON register
	 // settings may need to change for frequencies outside this range.
	 #if ((SYSCLK/SMB_FREQUENCY/3) < 255)
	 	 #define SCALE 1
		 CKCON |= 0x08; // Timer1 clock source = SYSCLK	 
	 #elif ((SYSCLK/SMB_FREQUENCY/4/3) < 255)
	 	 #define SCALE 4
	  	 CKCON |= 0x01;
	 	 CKCON &= ~0x0A; // Timer1 clock source = SYSCLK / 4
	 #endif


	 TMOD = 0x20; // Timer1 in 8-bit auto-reload mode
	 // Timer1 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
	 TH1 = -(SYSCLK/SMB_FREQUENCY/SCALE/3);
	 TL1 = TH1; // Init Timer1
	 TR1 = 1; 	// Timer1 enabled
 }












/////////////////////////////////////������� ���������� �������� ���������� ������_2///////////////////////////////

/*******************************************************************************
* Function Name  : void Wait_MS_timer2(unsigned int ms)  
* Description    : 
*    1) unsigned int ms - number of milliseconds of delay range is full range of integer: 0 to 65335
*    This routine inserts a delay of <ms> milliseconds.
*    [16-bit auto-reload mode] _ timer2  - >> [T2SPLIT_bit=0] TIMER_2_CN p.289  use SYSCLK 24500 
*******************************************************************************/
void Wait_MS_timer2(unsigned int ms)     
{
     CKCON = 0x10;                       //Timer use System CLK 24.5[Mhz]


    //TMR2RL_ =   Timer_2 Reload  16 bit value  
    //TMR2    =   Timer_2 Counter 16 bit value
                                                         //60000 - [1 ��  !!Succesful!!]
     TMR2RL=60000;//61554;//20518; //41036;              //20518 - [8 ��] //61554 - [700 ���] //64000 - 280[���] //65000 - 110[���] //65400 - 36[���]
     TMR2 = TMR2RL;                                      //65500 - [16 ���] -- ��������� ������ GPIO pin

     ET2 = 0;                            				 // Disable Timer 2 interrupts
     TR2 = 1;                            				 // Start Timer 2

	 while(ms)
	 {
	      TF2H = 0;                         			// Clear flag to initialize
	      while(!TF2H);                     			// Wait until timer overflows
	      ms--;                             			// Decrement ms
	 }

	 TR2 = 0;                               			// Stop Timer 2
 
}
/*******************************************************************************
* Function Name  : void Wait_Sec_timer2(unsigned int second)
* Description    : �������� �������
*******************************************************************************/
void Wait_Sec_timer2(unsigned int second)
{

	 CKCON = 0x10;                       			//Timer use System CLK 24.5[Mhz]
    //TMR2RL_ =   Timer_2 Reload  16 bit value  
    //TMR2    =   Timer_2 Counter 16 bit value
                                                    //60000 - [1 ��  !!Succesful!!]
     TMR2RL=10;//1000;//20000;//60000;              //10000 = 10 [��]        //20000 - [8 ��  !!Succesful!!]
     TMR2 = TMR2RL;                                  //1000 - 12 [��]    

     ET2 = 0;                             			// Disable Timer 2 interrupts
     TR2 = 1;                             			// Start Timer 2

	 while(second)
	 {
	      TF2H = 0;                         // Clear flag to initialize
	      while(!TF2H);                     // Wait until timer overflows
	      second--;                             // Decrement ms
	 }

	 TR2 = 0;                               // Stop Timer 2

}




/**************************************************************************************************
Parameters:  void while_delay(uint32_t  counter);       
*������ ������
*������ ������ ����� GPIO ��� ������ T = 80 [���] ������������� ��������� = 40 [���]
*while_delay(2);=40; while_delay(2); = 40[���]   = 80 [���] ������ ������ T 
*while_delay(4);=70; while_delay(4); = 70[���]   = 140 [���]  
*while_delay(6);=100; while_delay(4); = 100[���] = 200 [���] 6=100 [���]

***************************************************************************************************/
void  while_delay(unsigned long counter)
{
	
	    while(counter!=0)
        {  	       
			counter--; 
		}
	
}	


/*******************************************************************************
* Function Name  : void test_delay()
* Description    : �������� �������
*******************************************************************************/
void test_delay()
{
    

}


