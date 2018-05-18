/********************************************************************************************************
*
* File                : kn019_main.c
* Hardware Environment:	
* Build Environment   : 
* Version             : ������� 0.3 �� 18.05.2018 ���� ����� ����� ��������� ���� ���� �������
* By                  : 
*********************************************************************************************************/
#include <compiler_defs.h>    
#include <C8051F912_defs.h>


//Local Include Files 
#include <c8051_f902_delay.h>     //���������� ��������� ��������.
#include <c8051_f902_max24287.h>  //MAX 24287 +
#include <c8051_f902_i2c.h>       //I2C ���� 
#include <c8051_f902_spi.h>       //���� SPI


		sbit RST_0 =  P0^6; 
        #define RST_0_PORT_MASK (0x40)             //  port  6   Reset KSZ8765 and MAX24287+
		/**
 		 * The NH_LED
 		 */
		 sbit    NH_led = P0^5; 
		#define  LED_NH_PORT_MASK (0x20)   		   // port 5 ->NH_LED_SVETODIOD
		

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


	//���� SPI  
	static unsigned char reg_addr        = 0x0;     
    static unsigned char out_reg_val     = 0x0;

    //���� I2C
    static unsigned char i2c_reg_addr    = 0x0;     
    static unsigned char i2c_out_reg_val = 0x0;

	 //Watch DOG TImer  Debug_kit
     PCA0MD &= ~0x40;          // WDTE = 0 (clear watchdog timer enable)
     select_oscilator();
     port_init();

     RST_0 =0 ;                 //Sbrasivaem reset from ksz and switch. to null 
     
     Timer1_Init();             //Timer1 for I2c bus CLK
     SMBus_Init();              //Enable I2C  Smbus 
	 SPI1_Init ();              //Enable SPI1 Bus
	                            

     init_max_24287();          //������������� MAX24287+
     EA = 1;                    //Global interrupt enable  
     
	 Wait_MS_timer2(4000);

     //�������� � I2C SFP ������
     i2c_readreg (&i2c_reg_addr,&i2c_out_reg_val);
     if(i2c_out_reg_val==0x3){}  //��������� ��� �������� SFP ������ ���� �� OK
	 else
	 {
               for(;;)
			   {		 
				 NH_led=0;      //����� ��������� H 
                 Wait_MS_timer2(100);
                 /////////////////////////////////////////////////
                 NH_led=1;      //�������� ��������� H 
			  	 Wait_MS_timer2(100);			  
			   } 

	 }


     //�������� � SPI KSZ8765
	 spi_ksz8765_readreg (&reg_addr,&out_reg_val);    //��������� ��� SPI � KSZ8765 �������� ��� ����. 
	 if(out_reg_val==0x78) {}   //�� � �������
     else                      //�� ����� KSZ � SPI �� ��������
	 {
               for(;;)
			   {		 
				 NH_led=0;      //����� ��������� H 
                 Wait_MS_timer2(500);
                 /////////////////////////////////////////////////
                 NH_led=1;      //�������� ��������� H 
			  	 Wait_MS_timer2(500);			  
			   }            
	 }



     Wait_MS_timer2(1000);       //��� 1 �������
     NH_led=0;                   //����� ���������
  
     //����� 5 ������ ��� �� �� ������
	 //!!!!!�����!!!!!
     //test_kn19_single_byte_read_array();

   	  //Proverka Portov
  	//  SCL = 0;  //CLK GET to NULL  //rozoviu 
      ////////////////////////////////////////////////////
 	//  SCL = 1;  //CLK GET to NULL 

   

     //������ ������
     //������ ������ ����� GPIO ��� ������ T = 80 [���] ������������� ��������� = 40 [���]
     //while_delay(2);=40; while_delay(2); = 40[���]   = 80 [���] ������ ������ T 
	 //while_delay(4);=70; while_delay(4); = 70[���]   = 140 [���]  
	 //while_delay(6);=100; while_delay(4); = 100[���] = 200 [���]  
	 //���� ��� while_delay_pin
	 
   


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
		
        //SPI1 Port Init initialization
        // P1.0  -  SCK  (SPI0), Push-Pull,  Digital
		// P1.1  -  MISO (SPI0), Open-Drain, Digital
		// P1.2  -  MOSI (SPI0), Push-Pull,  Digital
		// P1.3  -  NSS  (SPI0), Push-Pull,  Digital

          
        // P1.4  - MDIO   /* Set MDIO port is open-drain */  
		// P1.5  - MDC    /* Set MDC port  is push-pull */ 


          P1MDOUT = 0x2D; //MDIO/MDC  + SPI 

		//MDIO PORT INIT
	   /* Set MDIO port is open-drain */
	    //P1MDOUT &= ~MDIO_PORT_MASK;
	   /* Set MDC port is push-pull */  
        //P1MDOUT |= MDC_PORT_MASK;		
      


       ///Skipping Ports

	  
	  /* Set P1.4 and P1.5 are skipped by the Crossbar and used for GPIO */ 
	     P1SKIP |= MDIO_PORT_MASK | MDC_PORT_MASK;

		 P0SKIP = 0x07;                                 //SKIP SDA SCL Pins
	     XBR0 = 0x04; 									//Enable SMBus pins all other digital pin disable
	     XBR1 = 0x40; 									//SPI1_enable
		                                                // 	    SCK (for SPI1) routed to P1.0.
														// 		MISO (for SPI1) routed to P1.1.
														// 		MOSI (for SPI1) routed to P1.2.
														//		NSS (for SPI1) routed to P1.3 only if SPI1 is configured to 4-wire mode.
		 
		 XBR2 = 0x40; 									//Enable crossbar and weak pull-ups 
	    // P0 = 0xFF;                                     //set output logic latch logik to h 
         P1 = 0xFF;
}
















