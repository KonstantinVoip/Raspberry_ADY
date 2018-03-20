 /**********************************************************************************************************************
*                                        (c) COPYRIGHT by PAO Inteltech.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
* Module      : bky_arm_delay.c
* Description : ������� ������ � ���������� � �������
* Author      : Konstantin Shiluaev
******************************************************************************
******************************************************************************
* Module's Description Record:
* ============================
* $State: Debug$
* $Revision: 0.0.1 $
* $Description: ������ ������� ����� ������ � ���������� ��� 96 [���]
* $Date: 2016/08/31 10:40:51 $
* $Revision: 0.0.2 $
*
*
******************************************************************************/

//------------------------------------------------------------------------------------------------------------------  
#include <compiler_defs.h>    
#include <C8051F912_defs.h>



#include <c8051_f902_delay.h>





/*****************************************************************************/
/*  PRIVATE FUNCTIONS                                                        */
/*****************************************************************************/




//-------------------------------------------------------------------------------------------------------------------





///////////////////������ � ��������� ��������////////////////////
/**************************************************************************************************
Parameters:         void sys_timer_start_en_irq(uint8_t second)
Remarks:            �������� ��������� ������ � ���������� ����������
***************************************************************************************************/


/**************************************************************************************************
Parameters:         void sys_timer_start_dis_irq(uint16_t  millisecond)
Remarks:            �������� ��������� ������ ��� ����������
***************************************************************************************************/


/**************************************************************************************************
Parameters:         void sys_timer_stop ()
Remarks:            ������������� ��������� ������
***************************************************************************************************/


/**************************************************************************************************
Parameters:         void sys_timer_second_delay ()
Remarks:            ���������� �������� �������� ���� �� ��������
Description:        � ������ ������� ������ ������� �� 1 �� 60 ������
***************************************************************************************************/



/**************************************************************************************************
Parameters:         void sys_timer_millisecond_delay ()
Remarks:            ���������� �������� �������� ���� �� ��������
Description:        � ������ ������� ������ ������� �� 1 �� 1000 �����������
***************************************************************************************************/


/**************************************************************************************************
Parameters:         void sys_timer_microsecond_delay ()
Remarks:            ���������� �������� �������� ���� �� ��������
* 									���� ������� �������� ����������� � ����������� 2 ���  ��� ������� ��� = 96 [���] 
* 									�������������  in 1 *2    = 2[���]
                    �������������  in 2 *2    = 4[���]
                    �������������  in 10 *2   =20[���]
                    �������������  in 12 *2   =24[���]
                    �������������  in 25 *2   =50[���]
                    �������������  in 98 *2   = 196[���]
                    �������������  in 231 *2  = 462[���]
                    �������������  in 342 *2  = 684[���]
                    �������������  in 500 *2  = 1000[���]
***************************************************************************************************/

/*******************************************************************************
* Function Name  : SysTick_GetCounter
* Description    : Gets SysTick counter value.
* Input          : None
* Output         : None
* Return         : SysTick current value
*******************************************************************************/


/////////////////////////////////////������� ���������� ��������///////////////////////////////
//-----------------------------------------------------------------------------
// Wait_MS
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters:
//   1) unsigned int ms - number of milliseconds of delay
//                        range is full range of integer: 0 to 65335
// This routine inserts a delay of <ms> milliseconds.
// [16-bit auto-reload mode] _ timer2  - >> [T2SPLIT_bit=0] TIMER_2_CN p.289
// use SYSCLK 24500 
//
//
//
//-----------------------------------------------------------------------------
void Wait_MS_timer2(unsigned int ms)     //for 0 = 8 [���]
{
     CKCON = 0x10;                       //Timer use System CLK 24.5[Mhz]


    //TMR2RL_ =   Timer_2 Reload  16 bit value  
    //TMR2    =   Timer_2 Counter 16 bit value
                                                         //60000 - [1 ��  !!Succesful!!]
     TMR2RL=60000;//61554;//20518; //41036;              //20518 - [8 ��] //61554 - [700 ���] //64000 - 280[���] //65000 - 110[���] //65400 - 36[���]
     TMR2 = TMR2RL;                                      //65500 - [16 ���] -- ��������� ������ GPIO pin

     ET2 = 0;                             // Disable Timer 2 interrupts
     TR2 = 1;                             // Start Timer 2

	 while(ms)
	 {
	      TF2H = 0;                         // Clear flag to initialize
	      while(!TF2H);                     // Wait until timer overflows
	      ms--;                             // Decrement ms
	 }

	 TR2 = 0;                               // Stop Timer 2
 
}
/*******************************************************************************
* Function Name  : SysTick_GetCounter
* Description    : Gets SysTick counter value.
* Input          : None
* Output         : None
* Return         : SysTick current value
*******************************************************************************/
void Wait_Sec_timer2(unsigned int second)
{

	 CKCON = 0x10;                       			//Timer use System CLK 24.5[Mhz]
    //TMR2RL_ =   Timer_2 Reload  16 bit value  
    //TMR2    =   Timer_2 Counter 16 bit value
                                                    //60000 - [1 ��  !!Succesful!!]
     TMR2RL=10;//1000;//20000;//60000;                  //10000 = 10 [��]        //20000 - [8 ��  !!Succesful!!]
     TMR2 = TMR2RL;                                  //1000 - 12 [��]    

     ET2 = 0;                             // Disable Timer 2 interrupts
     TR2 = 1;                             // Start Timer 2

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
Remarks:    //�������� � ������� ������� ��� ������ ��������� (�����)  c������ ����� 1 while                                         
																				    //while						 [24 ���]                	��������� *7	[168 ���]
 																					//1    =    	   ~1 ���      -          		   ~ 160  [��]
																					//10   =    	   ~3 ���      -
																					//100  =    	   ~20 ���     -          			
																					//500  =  	       ~80 ���     -                  
																					//800  =  	     ~140 ���    -
																					//1000 =  	     ~170 ���    -                  ~   24 [���]

																					//���� ����� ���� ������� �� ���������� ���� ������ �� while
																					// -  ��������� *4     [96 ���] 
																					//1 ��� = 10 [while]
																					//2 ��� = 40 [while] 

***************************************************************************************************/
void  while_delay(unsigned long counter)
{
	
	    while(counter!=0)
        {  	       
			counter--; 
		}
	
}	


//-----------------------------------------------------------------------------
// void test_delay()
//-----------------------------------------------------------------------------
void test_delay()
{
    

}
//-----------------------------------------------------------------------------
// T0_Waitms
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) U8 ms - number of milliseconds to wait range is full range of character: 0 to 255
// Configure Timer0 to wait for <ms> milliseconds using SYSCLK as its time
// base.
//
//-----------------------------------------------------------------------------
/*
void T0_Waitms (unsigned int ms)
{
   TCON &= ~0x30;                      // Stop Timer0; Clear TF0
   TMOD &= ~0x0f;                      // 16-bit free run mode
   TMOD |=  0x01;

   CKCON |= 0x04;                      // Timer0 counts SYSCLKs

   while (ms)
   {
      TR0 = 0;                         // Stop Timer0
      TH0 = ((-SYSCLK/1000) >> 8);     // Overflow in 1ms
      TL0 = ((-SYSCLK/1000) & 0xFF);
      TF0 = 0;                         // Clear overflow indicator
      TR0 = 1;                         // Start Timer0
      while (!TF0);                    // Wait for overflow
      ms--;                            // Update ms counter
   }

   TR0 = 0;                            // Stop Timer0
}
*/


