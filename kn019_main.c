/********************************************************************************************************
*
* File                : AT24Cxx.c
* Hardware Environment:	DVK501 && F020+ EX
* Build Environment   : Silicon LABs 3.42.00 20100913
* Version             : 
* By                  : Su Wei Feng
*
*                                  (c) Copyright 2005-2010, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/
#include <inc/compiler_defs.h>    
#include <inc/C8051F912_defs.h>




//Local Include Files 
#include <inc/c8051_f902_delay.h> //–еализаци€ временных задержек.
	   
		
		#define   SYSCLK          24500000L        //System clock frequency in 24500000 (24.5 Mhz)		
	    /**
 		 * KSZ8765 and MAX24287+ Reset
 		 */
		sbit RST_0 =  P0^6; 
        #define RST_0_PORT_MASK (0x40)  //  port  6   Reset KSZ8765 and MAX24287+
		/**
 		 * The NH_LED
 		 */
		 sbit    NH_led = P0^5; 
		#define  LED_NH_PORT_MASK (0x20)   		  // port 5 ->NH_LED_SVETODIOD
		
		
		sbit SDA = P0^3;   						  //P0.3  ->SDA  (24_from_scematic)  Smbus
		sbit SCL = P0^4;   						  //P0.4  ->SCL  (23_from_schematic) Smbus







//Function Prototype
  void select_oscilator();
  void port_init();
  void Wait_MS_timer2(unsigned int ms);
  void T0_Waitms(unsigned int ms);
/*****************************************************************************
Syntax: !!!!!!!!!!!!!!!!!!!! void main(void)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
Remarks:MAIN Functions			    
*******************************************************************************/
void main(void)
{
 	                             
     //Watch DOG TImer  Debug_kit
     



     PCA0MD &= ~0x40;          // WDTE = 0 (clear watchdog timer enable)
     select_oscilator();
     port_init();

 
   	//Proverka Portov
	//SCL = 0;  //CLK GET to NULL  //rozoviu 
    ////////////////////////////////////////////////////
	//SCL = 1;  //CLK GET to NULL 

     //ћеньше нельз€
     //ѕредел –аботы ножек GPIO это ѕериод T = 80 [м с] соответсвенно половинка = 40 [м с]
     //while_delay(2);=40; while_delay(2); = 40[м с]   = 80 [м с]
	 //while_delay(4);=70; while_delay(4); = 70[мкс]   = 140 [м с]  
	 //while_delay(6);=100; while_delay(4); = 100[мкс] = 200 [м с]  
	 //“ест дл€ while_delay_pin
	 /*
	 for(;;)
	 {
	    SCL = 0;
		while_delay(6);  
        SCL = 1;	 
        while_delay(6);
	 } 
     */


     
	 for(;;)
	 {
	    SCL = 0;
		Wait_MS_timer2(2);  
        SCL = 1;	 
        Wait_MS_timer2(2);
	 }





	  ///////////	
	  while(1);
 
	  /////////


}

/*****************************************************************************
Syntax:  select_oscilator()	    
Remarks: Vybor Oscilators to SYSCLK	    
*******************************************************************************/
void select_oscilator()
{
  
  volatile unsigned char  oscilator_rdy=0;
  volatile unsigned char  sysclk_rdy=0;

			
  //Ustanovka Internal Oscilator SYSCLK = 24.5 Mhz 
  OSCICL = 0x01; 
  OSCICN = 0x80;   								   			//Set internal oscillator to highest setting of 24500000 (Mhz) 
		   													//Zdes nugna proverka cto oscilator startoval u nas     
  do
  {
  oscilator_rdy=(OSCICN>>6); // + nalogit bit maska 
  oscilator_rdy = (oscilator_rdy & 1) ; 				 
  }while(!oscilator_rdy); 

  ///////////////////////////////////////////////////////////////////////////////////
  CLKSEL = 0x00;                    				//Ustanovka SYSCLK from oscilator   
  do
  {	 
  sysclk_rdy = (CLKSEL >> 7) ;	     	
  }while(!sysclk_rdy);                  			 //Zdes Proverks clock Ready   

}


/*****************************************************************************
Syntax:  void port_init()	    
Remarks:			    
*******************************************************************************/
void port_init()
{	    
		//P0SKIP = 0x07;                                  //SKIP SDA SCL Pins
	    XBR0 = 0x00; 									//Enable SMBus pins all other digital pin disable
	    XBR1 = 0x00; 									//pins all other digital  pin disable 
		XBR2 = 0x40; 									//Enable crossbar and weak pull-ups 
	    P0 = 0xFF;

}




//-----------------------------------------------------------------------------
// Wait_MS
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters:
//   1) unsigned int ms - number of milliseconds of delay
//                        range is full range of integer: 0 to 65335
//
// This routine inserts a delay of <ms> milliseconds.
//
//-----------------------------------------------------------------------------
void Wait_MS_timer2(unsigned int ms)
{

   CKCON &= ~0x20;                     // use SYSCLK/12 as timebase

   TMR2RL = -(SYSCLK/1000/12);         // Timer 2 overflows at 1 kHz
   TMR2 = TMR2RL;

   ET2 = 0;                            // Disable Timer 2 interrupts

   TR2 = 1;                            // Start Timer 2

   while(ms)
   {
      TF2H = 0;                         // Clear flag to initialize
      while(!TF2H);                     // Wait until timer overflows
      ms--;                            // Decrement ms
   }

   TR2 = 0;                            // Stop Timer 2

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




