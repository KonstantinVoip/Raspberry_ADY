/********************************************************************************************************
*
* File                : kn019_main.c
* Hardware Environment:	
* Build Environment   : 
* Version             : 
* By                  : 
*********************************************************************************************************/
#include <compiler_defs.h>    
#include <C8051F912_defs.h>


//Local Include Files 
#include <c8051_f902_delay.h>     //–еализаци€ временных задержек.
#include <c8051_f902_max24287.h>  //MAX 24287 +
#include <c8051_f902_i2c.h>       //I2C шина 

        //#define   SYSCLK          24500000L        //System clock frequency in 24500000 (24.5 Mhz)		
                                                     //go to delay.h
		sbit RST_0 =  P0^6; 
        #define RST_0_PORT_MASK (0x40)             //  port  6   Reset KSZ8765 and MAX24287+
		/**
 		 * The NH_LED
 		 */
		 sbit    NH_led = P0^5; 
		#define  LED_NH_PORT_MASK (0x20)   		   // port 5 ->NH_LED_SVETODIOD
		
		//rab_variant
		//sbit SDA = P0^3;   						  //P0.3  ->SDA  (24_from_scematic)  Smbus
		//sbit SCL = P0^4;   						  //P0.4  ->SCL  (23_from_schematic) Smbus

        //sbit mdio_=P1^4;                          //P1.4  ->MDIO 
		//sbit mdc_ =P1^5;                          //P1.5  ->MDC
		#define MDIO_PORT_MASK (0x10) //MDIO port mask.
		#define MDC_PORT_MASK (0x20) // MDC port mask.


		//Function Prototype
		  void select_oscilator();
		  void port_init();
  
   




/*****************************************************************************
Syntax: !!!!!!!!!!!!!!!!!!!! void main(void)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
Remarks:MAIN Functions			    
*******************************************************************************/
void main(void)
{
 	 // U16 val=0;        
	 // U16 error=0;		                     
       
	
	 //Watch DOG TImer  Debug_kit
     PCA0MD &= ~0x40;          // WDTE = 0 (clear watchdog timer enable)
     select_oscilator();
     port_init();
     //NH_led=0;                //√асим —ветодиод H

     RST_0 =0 ;                 //Sbrasivaem reset from ksz and switch. to null 
     
     Timer1_Init();             //Timer1 for I2c bus CLK
     SMBus_Init();              //Enable I2C Smbus 
	                            

     init_max_24287();          //»нициализаци€ MAX24287+



     EA = 1;                     //Global interrupt enable  

     Wait_MS_timer2(1000);       //∆дЄм 1 секунду
     //NH_led=1;                 //«ажигаем —ветодиод H
     NH_led=0; 
  
    
	 //!!!!!“есты!!!!!
     //test_kn19_single_byte_read_array();


   	  //Proverka Portov
  	//  SCL = 0;  //CLK GET to NULL  //rozoviu 
      ////////////////////////////////////////////////////
 	//  SCL = 1;  //CLK GET to NULL 


     //ћеньше нельз€
     //ѕредел –аботы ножек GPIO это ѕериод T = 80 [м с] соответсвенно половинка = 40 [м с]
     //while_delay(2);=40; while_delay(2); = 40[м с]   = 80 [м с] полный период T 
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


     /*
	 for(;;)
	 {
	    //SCL = 0;
		//Wait_MS_timer2(1);  

		//SCL = 1;	 

		//Wait_MS_timer2(1);
	 }*/


	  ///////////	
	  while(1);
	  {
	  
	  }

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
	  }while(!sysclk_rdy);                  			//Zdes Proverks clock Ready   


}


/*****************************************************************************
Syntax:  void port_init()	    
Remarks:			    
*******************************************************************************/
void port_init()
{	    
		
		//MDIO PORT INIT
	   /* Set MDIO port is open-drain */
		  P1MDOUT &= ~MDIO_PORT_MASK;
	   /* Set MDC port is push-pull */  
          P1MDOUT |= MDC_PORT_MASK;		

      /* Set P1.4 and P1.5 are skipped by the Crossbar and used for GPIO */ 

	     P1SKIP |= MDIO_PORT_MASK | MDC_PORT_MASK;

		 P0SKIP = 0x07;                                 //SKIP SDA SCL Pins
	     XBR0 = 0x04; 									//Enable SMBus pins all other digital pin disable
	     XBR1 = 0x00; 									//pins all other digital  pin disable 
		 XBR2 = 0x40; 									//Enable crossbar and weak pull-ups 
	     P0 = 0xFF;
         P1 = 0xFF;
}
















