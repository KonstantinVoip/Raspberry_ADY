/********************************************************************************************************
*
* File                : kn019_main.c
* Hardware Environment:	
* Build Environment   : 
* Version             : 
* By                  : 
*********************************************************************************************************/
//#include <inc/compiler_defs.h>    
//#include <inc/C8051F912_defs.h>

#include <compiler_defs.h>    
#include <C8051F912_defs.h>


//Local Include Files 
#include <c8051_f902_delay.h> //���������� ��������� ��������.


       //MAX 
	   /**
	 * The device reset port mask.
	 */
//	#define RESET_PORT_MASK (0x04)

	/**
	 * The device physical bus address.
	 */
	#define PHY_ADDR (0x04)

	/**
	 * PAGESEL (RW)
	 */
	#define REG_MAX_PAGESEL (0x1f)

	/**C
	 * Mask of register address.
	 */
	#define REG_ADDR_MASK (0x001f)

	/**
	 * Mask of register page.
	 */
	#define REG_PAGE_MASK (0x0003)

	/**
	 * Mask of register address.
	 */
	#define REG_PAGE_SHIFT (5)


    /**
 * The device register addresses.
 */
	enum RegMax
	{
		  /**
		   * BMCR (RW)
		   */
		  REG_MAX_BMCR = 0x00,  
  
		  /**
		   * BMSR (RO)
		   */
		  REG_MAX_BMSR = 0x01,
  
		  /**
		   * ID1 (RO)
		   */
		  REG_MAX_ID1 = 0x02,
  
		  /**
		   * ID2 (RO)
		   */
		  REG_MAX_ID2 = 0x03,
  
		  /**
		   * AN_ADV (RW)
		   */
		  REG_MAX_AN_ADV = 0x04,
  
		  /**
		   * AN_RX (RO)
		   */
		  REG_MAX_AN_RX = 0x05,
  
		  /**
		   * AN_EXP (RO)
		   */
		  REG_MAX_AN_EXP = 0x06,
  
		  /**
		   * EXT_STAT (RO)
		   */
		  REG_MAX_EXT_STAT = 0x0f,
  
		  /**
		   * JIT_DIAG (RW)
		   */
		  REG_MAX_JIT_DIAG = 0x10,
  
		  /**
		   * PCSCR (RW)
		   */
		  REG_MAX_PCSCR = 0x11,
  
		  /**
		   * GMIICR (RW)
		   */
		  REG_MAX_GMIICR = 0x12,
  
		  /**
		   * CR (RW)
		   */
		  REG_MAX_CR = 0x13,
  
		  /**
		   * IR (RW)
		   */
		  REG_MAX_IR = 0x14,
  
		  /**
		   * ID (RO)
		   */
		  REG_MAX_ID = 0x30,
  
		  /**
		   * GPIOCR1 (RW)
		   */
		  REG_MAX_GPIOCR1 = 0x31,
  
		  /**
		   * GPIOCR2 (RW)
		   */
		  REG_MAX_GPIOCR2 = 0x32,
  
		  /**
		   * GPIOSR (RO)
		   */
		  REG_MAX_GPIOSR = 0x33,
  
		  /**
		   * PTPCR1 (RW)
		   */
		  REG_MAX_PTPCR1 = 0x70
  
	};


		#define   SYSCLK          24500000L        //System clock frequency in 24500000 (24.5 Mhz)		
	    /**
 		 * KSZ8765 and MAX24287+ Reset
 		 */
		sbit RST_0 =  P0^6; 
        #define RST_0_PORT_MASK (0x40)             //  port  6   Reset KSZ8765 and MAX24287+
		/**
 		 * The NH_LED
 		 */
		 sbit    NH_led = P0^5; 
		#define  LED_NH_PORT_MASK (0x20)   		   // port 5 ->NH_LED_SVETODIOD
		
		
		//rab_variant
		sbit SDA = P0^3;   						  //P0.3  ->SDA  (24_from_scematic)  Smbus
		sbit SCL = P0^4;   						  //P0.4  ->SCL  (23_from_schematic) Smbus

        sbit mdio_=P1^4;                          //P1.4  ->MDIO 
		sbit mdc_ =P1^5;                          //P1.5  ->MDC

        /**
 		 *MDIO port mask.
 		 */
		#define MDIO_PORT_MASK (0x10)
		/**
 		 * MDC port mask.
 		 */
		#define MDC_PORT_MASK (0x20)


 static U8 page_;


//Function Prototype
  void select_oscilator();
  void port_init();
  

  

  //MDIO LINE FUNCTIONS
  U16 mdioRead(U8 phyAddr, U8 regAddr);
  void mdioWrite(U8 phyAddr, U8 regAddr, U16 value);
  //
  U16  input();
  void output(U32 val, U8 num);
  
  //MAX Functions
  U16 maxRead(enum RegMax regAddr);
  void maxWrite(enum RegMax regAddr, U16 value);

  


/*****************************************************************************
Syntax: !!!!!!!!!!!!!!!!!!!! void main(void)  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
Remarks:MAIN Functions			    
*******************************************************************************/
void main(void)
{
 	  U16 val=0;        
	  U16 error=0;		                     
     //Watch DOG TImer  Debug_kit
     PCA0MD &= ~0x40;          // WDTE = 0 (clear watchdog timer enable)
     select_oscilator();
     port_init();


     RST_0 =0 ;  //Sbrasivaem reset from ksz and switch. to null 
     
    
	 /* Read-write test */
     for(val=3; val>=0; val--)
  	 {
   	    mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, val);
	    if(mdioRead(PHY_ADDR, REG_MAX_PAGESEL) != val)
	    {
	      error = 1;//BOOS_ERROR;
	      break;
	    }      
 	
	 }



	 /* Make the startup procedure */
     /* Selects MDIO page 2 */
     mdioWrite(PHY_ADDR, REG_MAX_PAGESEL , 0x0012);      
     /* Write 0x4004 to PTPCR1 to power-down the receive CDR */
     mdioWrite(PHY_ADDR, REG_MAX_PTPCR1 & REG_ADDR_MASK, 0x4004);            
     /* Wait 1ms */
     //threadSleep(1);
	 Wait_MS_timer2(1);
     /* Write 0x4000 to PTPCR1 to power-up the receive CDR */
     mdioWrite(PHY_ADDR, REG_MAX_PTPCR1 & REG_ADDR_MASK, 0x4000);
     /* Set the BMCR.DP_RST bit to reset the datapath */
     mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, 0x0010);
     mdioWrite(PHY_ADDR, REG_MAX_BMCR & REG_ADDR_MASK, 0x8000u);



      //val = maxRead(REG_MAX_BMCR);  //0x0000
      //val = maxRead(REG_MAX_BMSR);  //0x7949
        val = maxRead(REG_MAX_ID);    //0x1ee0;
 
  





    //    mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, 0x0011);    
    //val = mdioRead(PHY_ADDR, REG_MAX_ID & REG_ADDR_MASK);    


   	  //Proverka Portov
  	//  SCL = 0;  //CLK GET to NULL  //rozoviu 
      ////////////////////////////////////////////////////
 	//  SCL = 1;  //CLK GET to NULL 



     //������ ������
     //������ ������ ����� GPIO ��� ������ T = 80 [���] ������������� ��������� = 40 [���]
     //while_delay(2);=40; while_delay(2); = 40[���]   = 80 [���]
	 //while_delay(4);=70; while_delay(4); = 70[���]   = 140 [���]  
	 //while_delay(6);=100; while_delay(4); = 100[���] = 200 [���]  
	 //���� ��� while_delay_pin
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
	    //SCL = 0;
		//Wait_MS_timer2(1);  

		//SCL = 1;	 

		//Wait_MS_timer2(1);
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

		 //P0SKIP = 0x07;                                  //SKIP SDA SCL Pins
	     XBR0 = 0x00; 									//Enable SMBus pins all other digital pin disable
	     XBR1 = 0x00; 									//pins all other digital  pin disable 
		 XBR2 = 0x40; 									//Enable crossbar and weak pull-ups 
	     P0 = 0xFF;
         P1 = 0xFF;
}





/*****************************************************************************
Syntax:  void port_init()	    
Remarks: * Read from a PHY device.
  		 * @param phyAddr an device address. 
  		 * @param regAddr an register address.
   		 * @return the read resualt.			    
*******************************************************************************/
U16 mdioRead(U8 phyAddr, U8 regAddr)
{
 	    U16 val;  
   
	    output(0xffffffffu, 32);
	    output(0x6, 4);
	    output(phyAddr, 5);
	    output(regAddr, 5);
	    output(1, 1);  
	    val = input();
	    output(1, 1);
	  return val;  
}
  
  /**
 * Writes to a PHY device.
 * @param phyAddr an device address. 
 * @param regAddr an register address.
 * @param value   a writing value.
 */
void mdioWrite(U8 phyAddr, U8 regAddr, U16 value)
{
 
	    output(0xffffffffu, 32);
	    output(0x5, 4);
	    output(phyAddr, 5);
	    output(regAddr, 5);
	    output(0x2, 2);
	    output(value, 16);
	    output(1, 1);
 
}
 
  /**
   * Inputs a value from the bus.
   * @param val an outputing value.
   * @param num an number of outputing bits.
  */
  U16  input()
  {
    U8 i;
  	U16 val = 0;
    for(i=0; i<16; i++)
    {
      mdc_ = 1;
      //delay();    
      mdc_ = 0;    
      //delay();    
      val <<= 1;    
      val |= mdio_;  
    }
   return val;
  }
 
  /**
   * Outputs a value to the bus.
   * @param val an outputing value.
   * @param num an number of outputing bits.
  */
 void output(U32 val, U8 num)
  {
	  for(val <<= (32 - num); num; val <<= 1, num--)
	  {
	    // Set the value bit to MDO
	    if(val & 0x80000000u)
	    {
	      mdio_ = 1;
	    }
	    else
	    {
	      mdio_ = 0;      
	    }
	    // Set a clock to MDC    
	    mdc_ = 1;
	    test_delay();     //esli_chto  
	   // while_delay(1);
	    mdc_ = 0;    
	    test_delay();    
	//	while_delay(1);     //esli_chto
	  }
  
 }
  


  //MAX Functions
  /**
   * Read from the device.
   *
   * @param regAddr an register address.
   * @return the read resualt.
   */
 U16 maxRead(enum RegMax regAddr)
 {
      U16 value;
      U16 addr, page;

      addr = regAddr & REG_ADDR_MASK;
      page = regAddr >> REG_PAGE_SHIFT & REG_PAGE_MASK;
      if(page != page_)
      {
      mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, page | 0x0010);
      page_ = page;
      }
      value = mdioRead(PHY_ADDR, addr);  

  return value;
 }

   
  /**
  * Writes to the device.
  *
  * @param regAddr an register address.
  * @param value   a writing value.
  */
void maxWrite(enum RegMax regAddr, U16 value)
{
    U16 addr, page; 

    addr = regAddr & REG_ADDR_MASK;    
    page = regAddr >> REG_PAGE_SHIFT & REG_PAGE_MASK;    
    if(page != page_)
    {
      mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, page | 0x0010);
      page_ = page;
    }
    mdioWrite(PHY_ADDR, addr, value);

}











