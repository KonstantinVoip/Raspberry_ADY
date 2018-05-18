 /**********************************************************************************************************************
*                                        (c) COPYRIGHT by ZAO RCZI FORT.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
* Module        :с8051_f902_max24287.c
* Description : ђабота с MAX24287.c
* Author      : Konstantin Shiluaev
******************************************************************************
******************************************************************************
* Module's Description Record:
* ============================
* $State: Debug$
* $Revision: 0.0.1 $
* $Description: Первая ревизия блока работы с Задержками для 24.5 [мгЦ]
* $Date: 2016/08/31 10:40:51 $
* $Revision: 0.0.2 $
******************************************************************************/

//------------------------------------------------------------------------------------------------------------------  
#include <compiler_defs.h>    
#include <C8051F912_defs.h>

#include <c8051_f902_delay.h>    //Реализация временных задержек.

#include <c8051_f902_max24287.h>


    //Порты для Работы
    sbit mdio_=P1^4;                          //P1.4  ->MDIO 
    sbit mdc_ =P1^5;                          //P1.5  ->MDC



	#define PHY_ADDR 		(0x04)             //The device physical MAX24287+  MDIO/MDC bus address.
	#define REG_MAX_PAGESEL (0x1f)             //PAGESEL (RW)
	#define REG_ADDR_MASK   (0x001f)           //Mask of register address.
	#define REG_PAGE_MASK   (0x0003)           //Mask of register page.
	#define REG_PAGE_SHIFT  (5)                //Mask of register address.

 	// The device register addresses.
	enum RegMax
	{
		  REG_MAX_BMCR = 0x00,   	 // BMCR (RW)
		  REG_MAX_BMSR = 0x01,   	 // BMSR (RO)
		  REG_MAX_ID1 = 0x02,  	   	 //  ID1 (RO)
		  REG_MAX_ID2 = 0x03,  		 // ID2 (RO)
		  REG_MAX_AN_ADV = 0x04, 	//AN_ADV (RW)
		  REG_MAX_AN_RX = 0x05,  	//AN_RX (RO)
		  REG_MAX_AN_EXP = 0x06, 	//AN_EXP (RO)
		  REG_MAX_EXT_STAT = 0x0f,  // EXT_STAT (RO)
		  REG_MAX_JIT_DIAG = 0x10,  // JIT_DIAG (RW)
		  REG_MAX_PCSCR = 0x11,     // PCSCR (RW)
		  REG_MAX_GMIICR = 0x12,    //  GMIICR (RW)
		  REG_MAX_CR = 0x13,        //  CR (RW)
		  REG_MAX_IR = 0x14,         //   IR (RW)
		  REG_MAX_ID = 0x30,        //   ID (RO)
		  REG_MAX_GPIOCR1 = 0x31,   // GPIOCR1 (RW)
		  REG_MAX_GPIOCR2 = 0x32,   // GPIOCR2 (RW)
		  REG_MAX_GPIOSR = 0x33,    // GPIOSR (RO)
		  REG_MAX_PTPCR1 = 0x70     // PTPCR1 (RW)
  
	};

  
static U8 page_;


      //Глобальные Функции

      U16 mdioRead(U8 phyAddr, U8 regAddr);
      void mdioWrite(U8 phyAddr, U8 regAddr, U16 value);

      U16  input();
      void output(U32 val, U8 num);

      U16 maxRead(enum RegMax regAddr);
      void maxWrite(enum RegMax regAddr, U16 value); 




/*******************************************************************************
* Function Name  : void init_max_24287()
* Description    : Initialization MAX24287
*******************************************************************************/
void init_max_24287()
{



	 /* Make the startup procedure */
     /* Selects MDIO page 2 */
     //mdioWrite(PHY_ADDR, REG_MAX_PAGESEL , 0x0012); 
	   maxWrite(REG_MAX_PAGESEL, 0x0012) ;     
     /* Write 0x4004 to PTPCR1 to power-down the receive CDR */
      // mdioWrite(PHY_ADDR, REG_MAX_PTPCR1 & REG_ADDR_MASK, 0x4004);            
       maxWrite(REG_MAX_PTPCR1, 0x4004) ;  
	 
	  /* Wait 2ms */
	   Wait_MS_timer2(2);
     /* Write 0x4000 to PTPCR1 to power-up the receive CDR */
    // mdioWrite(PHY_ADDR, REG_MAX_PTPCR1 & REG_ADDR_MASK, 0x4000);
       maxWrite(REG_MAX_PTPCR1, 0x4000); 
	   maxWrite(REG_MAX_BMCR, 0x8000); 
	 /* Set the BMCR.DP_RST bit to reset the datapath */
    // mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, 0x0010);
    // mdioWrite(PHY_ADDR, REG_MAX_BMCR & REG_ADDR_MASK, 0x8000u);



       //GPIO3 Light_on
       // maxWrite(REG_MAX_GPIOCR1, 0x0001) ; //Light_OFF_MAX_DEVICE
  	   // maxWrite(REG_MAX_GPIOCR1, 0x0002) ; //Light_ON_MAX_DEVICE_GREEN     drive_ligic_1
        maxWrite(REG_MAX_GPIOCR1, 0x0006) ; //Output Real_TIME_LINK_STATUS    Link /UP/or/down
 
       //maxWrite(REG_MAX_CR, 0x0220);
       
	   maxWrite(REG_MAX_CR     , 0x0320);
	   maxWrite(REG_MAX_GPIOCR1, 0x000E) ; //Output Real_TIME_LINK_STATUS[Link /UP/or/down] + TX_DISABLE_0
	   maxWrite(REG_MAX_BMCR   , 0x1200) ; 



       
      //Тесты для MAX24287

             //maxWrite(enum RegMax regAddr, U16 value) 
 	//       val = maxRead(REG_MAX_BMCR);
    //		val = maxRead(REG_MAX_BMSR);      //0x7949  
    //       val = maxRead(REG_MAX_GMIICR);    
    //       val = maxRead(REG_MAX_CR);
    //		val = maxRead(REG_MAX_PCSCR);
   //	    val = maxRead(REG_MAX_AN_ADV);
   //		val = maxRead(REG_MAX_ID);        //0x1ee0; //revision B      
   		 //mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, 0x0011);    
    	 //val = mdioRead(PHY_ADDR, REG_MAX_ID & REG_ADDR_MASK);   


      /* Read-write test */
     /*
	 for(val=3; val>=0; val--)
  	 {
   	    mdioWrite(PHY_ADDR, REG_MAX_PAGESEL, val);
	    if(mdioRead(PHY_ADDR, REG_MAX_PAGESEL) != val)
	    {
	      error = 1;//BOOS_ERROR;
	      break;
	    }      
 	
	 }
     */



}

/*****************************************************************************
Syntax:   U16 mdioRead(U8 phyAddr, U8 regAddr)	    
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
  

/*****************************************************************************
Syntax:  * void mdioWrite(U8 phyAddr, U8 regAddr, U16 value)	    
Remarks: * Read from a PHY device.
  		 * @param phyAddr an device address. 
  		 * @param regAddr an register address.
   		 * @param value   a writing value.			    
*******************************************************************************/
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
 
/*****************************************************************************
Syntax:  * U16  input()	    
Remarks: * 	    
*******************************************************************************/
U16  input()
{
    U8 i;
  	U16 val = 0;
    for(i=0; i<16; i++)
    {
      mdc_ = 1;
      test_delay();     //esli_chto так было (сделано в примере задержка)
 
      mdc_ = 0;    
      test_delay();     //esli_chto так было (сделано в примере задержка)
      val <<= 1;    
      val |= mdio_;  
    }
   return val;
}
/*****************************************************************************
Syntax:  * void output(U32 val, U8 num)   
Remarks: * 	    
*******************************************************************************/ 
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
	    test_delay();     //esli_chto так было (сделано в примере задержка)
	    mdc_ = 0;    
	    test_delay();      //esli_chto так было (сделано в примере задержка)
 
	  }
  
}
  


//MAX Functions
/*****************************************************************************
Syntax:  * U16 maxRead(enum RegMax regAddr) 
Remarks: * Read from the device.	    
*******************************************************************************/ 
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

   

/*****************************************************************************
Syntax:  * void maxWrite(enum RegMax regAddr, U16 value)
Remarks: * Writes to the device. param regAddr an register address.	    
*******************************************************************************/ 
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




