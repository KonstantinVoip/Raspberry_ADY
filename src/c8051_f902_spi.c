 /**********************************************************************************************************************
*                                        (c) COPYRIGHT by ZAO RCZI FORT.                                             *
*                                               All rights reserved.                                                  *
***********************************************************************************************************************
* Module        :с8051_f902_spi.c
* Description : ђабота с SPI C8051_f902.c
* Author      : Konstantin Shiluaev
******************************************************************************
******************************************************************************
* Module's Description Record:
* ============================
* $State: Debug$
* $Revision: 0.0.1 $
* $Description: 
* $Date: 2016/08/31 10:40:51 $
* $Revision: 0.0.2 $
******************************************************************************/

//------------------------------------------------------------------------------------------------------------------  
#include <compiler_defs.h>    
#include <C8051F912_defs.h>

#include <c8051_f902_delay.h> //Задержки

#include <c8051_f902_spi.h>   //Шина SPI

   
  
    // Instruction Set
	#define  MAX_BUFFER_SIZE    2           // Maximum buffer Master will send
	#define  SPI_WRITE         0x04        // Send a byte from the Master to the Slave
	#define  SPI_READ          0x08        // Send a byte from the Slave to the  Master
	#define  SPI_WRITE_BUFFER  0x10        // Send a series of bytes from the    Master to the Slave
	#define  SPI_READ_BUFFER   0x20        // Send a series of bytes from the Slave to the Master
	#define  ERROR_OCCURRED    0x40        // Indicator for the Slave to tell the Master an error occurred



    U8 SPI_Data = 0x00;      //Глобальные Данные для записи в шину SPI
    U8 SPI_Data_Array[MAX_BUFFER_SIZE] = {0x00,0x00};
	bit Error_Flag = 0;
	static unsigned char Command = 0x00;


/*****************************************************************************
Syntax:  void SPI1_Init ()	    
Remarks: 			   
	    * Configures SPI1 to use 4-wire Single Master mode. The SPI timing is
		* configured for Mode 1,1 (data centered on first edge of clock phase and
		* SCK line low in idle state).
*******************************************************************************/
void SPI1_Init ()
{
   
         //НАстройка Фронтов Сигнала CLK для KSZ 8765 должна быть такая p.267  RM C8051F902
		 //SCK (CKPOL=1, CKPHA=1) //SPInCFG p.270  
		 SPI1CFG    = 0x70;                   	   // Enable the SPI as a Master MODE + CKPHA = '1', CKPOL = '1'   
   		 SPI1CN     = 0x0D;                   	   // 4-wire Single Master, SPI enabled 
  	     //SPI clock frequency equation from the datasheet
   		 SPI1CKR   = (SYSCLK/(2*SPI_CLOCK))-1;     //шина SPI у нас работает на 500 [КгЦ]
	     EIE2 |= 0x08;  //Enable ESPI1 Interrupt 
        
}


/*****************************************************************************
Syntax:  void spi_ksz8765_readreg (const unsigned char *reg_addr,unsigned char *out_reg_val)  	    
Remarks:*Пока Запихнём Функцию чтения из SPI KSZ сюда 			   
	    *!!!Внимание Инвертор!!! на входных данных полученных ис KSZ8765 
		*!!!Ещё есть баг с адресами работает 0x00 0x02 ,0x04, 0x06 ,0x08 итд ???
		*!!!В будущих ревизиях нужно Поправить!!!! ??двойная буферизация ??
*******************************************************************************/
void spi_ksz8765_readreg (const unsigned char *reg_addr,unsigned char *out_reg_val)
{

		
   while (!NSS1MD0);                    // Wait until the SPI is free, in case it's already busy
   EIE2 = 0x00;  						// Disable SPI1 interrupts 
   NSS1MD0 = 0;

   //Начало Информационной Посылки 12 байт на Чтение KSZ8765
   //1-е 8 байт KSZ8765 
   SPI1DAT = 0x60;					    // 00000011 for “READ DATA,” KSZ READ COMMAND p.26 RM
   while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
   {                                }
   //2-е 8 байт KSZ8765
   SPI1DAT = *reg_addr;                 //Здесь будет адрес Регистра KSZ
   while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
   {                                }
   //3-е 8 байт KSZ8765
   SPI1DAT = 0x00;
   while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
   {                                }
   //Конец Информационной Поылки в KSZ8765


   SPIF1 = 0;
   while (SPIF1 != 1)                   // Wait until the last byte of the data reaches the Slave
   {                                  
   }
   SPIF1 = 0;
   NSS1MD0 = 1;                         // Diable the Slave
   EIE2 = 0x8 ;                         // Re-enable SPI interrupts

   *out_reg_val=SPI1DAT;


}

/*****************************************************************************
Syntax:  void spi_ksz8765_writereg(const unsigned char *reg_addr,const unsigned char *in_reg_addr) 	    
Remarks:*Пока Запихнём Функцию чтения из SPI KSZ сюда 			   
	    *Пишем Значение в Регистры KSZ Сделать в !!!Будущем!!! видимо пока комментарю.
*******************************************************************************/
/*void spi_ksz8765_writereg(const unsigned char *reg_addr,const unsigned char *in_reg_addr)
{



}
*/

/*****************************************************************************
Syntax:  void SPI1_ISR(void) interrupt 18	    

Remarks: Handles all error checks and single-byte writes.			   
	    *Note: SPI_WRITE_ARRAY is not handled by this ISR in order to take 
		*advantage of double-buffering (checking the TXBMT0 flag) using polling. 
		* 
		*Typical Write:
		*| 1st sent | 2nd sent | 3rd sent |   ...    | last sent |
		*---------------------------------------------------------
		*Master NSSv | Command  |   Data1  |   Data2  |   ...    |   DataN   |  NSS^
		*Slave       |   N/A    |    N/A   |    N/A   |   ...    |    N/A    |
		*
		*Typical Read:
		*| 1st sent | 2nd sent | 3rd sent |   ...    | last sent |
		*---------------------------------------------------------
		*Master NSSv | Command  |   dummy  |   dummy  |   ...    |   dummy   |  NSS^
		*Slave       |   N/A    |   Data1  |   Data2  |   ...    |   DataN   |

*******************************************************************************/
void SPI1_ISR(void) interrupt 18
{
       static unsigned char array_index = 0;
       static unsigned char state = 0;

       
       if (WCOL1 == 1)
       {
         // Write collision occurred
         WCOL1 = 0;                        // Clear the write collision flag
         Error_Flag = 1;
       }
       else
	   {
	      
		  if (SPI1DAT == ERROR_OCCURRED)
	      {
	         // This example recognizes when an error occurs, but does not include
	         // any error handling.  The transfer can be aborted or rescheduled,
	         // if desired.
	         Error_Flag = 1;
	      } //end if

          // When the Master enters the ISR, the SPIF0 flag should be set from
     	  // sending the Command byte.  This ISR handles the remaining steps of the SPI transfer process.
      	  // <state> == 0: writing or reading 1 byte of data
          // <state> == 1: for READ commands (first time, only a dummy byte is
          //               sent but the second time, the data must be read from SPI0DAT)         
          // <state> == 2: NSS = 1 to end the transfer, final byte read
          // Note: SPI_WRITE_BUFFER is not handled here because it's done in polled mode
          if (state == 0)
          {
	         switch (Command)
	         {
	            //case SLAVE_LED_ON:
	            //case SLAVE_LED_OFF:
	            //     NSS1MD0 = 1;             // Release the slave (not expecting data back)
	            //break;
                //
	            
				case SPI_WRITE:
	                 SPI1DAT = SPI_Data;
	                 state = 2;                 // Advance to the final state (only writing one byte)
	            break;
                //
	            case SPI_READ:
	                 SPI1DAT = 0xFF;           // Send a dummy byte so the Slave can send the data send the data
	                 state = 2;                // Advance to the final state (only reading one byte)
	            break;
                //
	            case SPI_READ_BUFFER:
	                 array_index = 0;          // Clear the data counter

	                 SPI1DAT = 0xFF;           // Send a dummy byte so the Slave can start sending the data
	                 state = 1;                // Advance to the next state where the data can be received
	                                           // The data from the slave is not available until after the second
	                                           // transfer is completed.
	                                           // The dummy byte allows the slave to send data, since the Master controls SCK.
	            break;
                //
	            default:
	                 state = 2;              // Any errors in the Command parsing should go to state 2 where NSS0MD0
	                                         // should go to state 2 where NSS0MD0 is de-asserted

	         }//end switch
         
		 }//end if state=0
        
		 else if (state == 1)             // This state is for READ_ARRAY commands where the data must be read
         {                                // after the first dummy byte is sent
                                       
	         switch (Command)
	         {
	            case SPI_READ_BUFFER:
		               SPI_Data_Array[array_index] = SPI1DAT;
		               SPI1DAT = 0xFF;

		               array_index++;

		               if (array_index == (MAX_BUFFER_SIZE-1))
		               {
		                  state = 2;
		               }

	            break;
	            //
				default:
	              	  state = 2;       // Any errors in the Command parsing should go to state 2 where NSS1MD0
	                                   // is de-asserted
	         }//end switch
      
	   }//end else if state_1
       
	   else if (state == 2)
       {
         switch (Command)
         {
            case SPI_READ:
               		SPI_Data = SPI1DAT;     // Read the data 8 bit from the slave

            break;
            //
            case SPI_READ_BUFFER:
                   SPI_Data_Array[array_index] = SPI1DAT; // Read the last data without sending a dummy byte
                   state = 0;                                                                              
            break;
            //
		    //default:
			// What a Fuck
			//break; 
		 
		 } // end switch 
        NSS1MD0 = 1;                   // De-select the Slave
        state = 0;                     // Reset the state
       }  //end else state = 2


            SPIF1 = 0;                             // Clear the SPIF1 flag
           //SBIT (SPIF0, SFR_SPI0CN, 7);           // SPI0 Interrupt Flag

	 }//end global else






}//end if



#if 0   //Пока комментарим


/*****************************************************************************
Syntax:  SPI1_Byte_Write (unsigned char dat) 	    
Remarks: 			   
	    * Запись Байта
		* 
		* 
*******************************************************************************/
void SPI1_Byte_Write (unsigned char dat)   //Запись Байта
{

	while (!NSS1MD0);                     // Wait until the SPI is free, in case it's already busy
    NSS1MD0 = 0;
    
	Command = SPI_WRITE;  				  //пишем в SPI команда для машины состояний
    SPI1DAT = dat;                        //Заполняем регистр данными

   // The rest of this command will be handled by the SPI ISR, which will
   // trigger when SPIF0 is set from sending the Command

}

/*****************************************************************************
Syntax:  unsigned char SPI1_Byte_Read ()   	    
Remarks: 			   
	    * Чтение Байта
*******************************************************************************/
unsigned char SPI1_Byte_Read ()           
{

     while (!NSS1MD0);                    // Wait until the SPI is free, in case it's already busy
  	 NSS1MD0 = 0;
   	 Command = SPI_READ;				  //пишем в SPI команда для машины состояний

   	 //SPI0DAT = Command;

   // The rest of this command will be handled by the SPI ISR, which will
   // trigger when SPIF0 is set from sending the Command
 
     return   SPI_Data;
    
}



//#if 0   //Пока Комментарю SPI 

/*****************************************************************************
Syntax:  void SPI_Array_Write (void)	    
Remarks: Запись Массива 			   
	    * 
		* 
		* 
*******************************************************************************/
void SPI_Array_Write (void)
{
   unsigned char array_index;
   static unsigned short low_index =0;
   static unsigned short high_index=0;

   while (!NSS1MD0);                    // Wait until the SPI is free, in case it's already busy


   EIE2 = 0x00;  						// Disable SPI1 interrupts 
   //ESPI0 = 0;                         // Disable SPI interrupts

   NSS1MD0 = 0;



    SPI1DAT = 0x60;//SPI_WRITE_BUFFER;    // Load the XMIT register
    while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
    {                                
    }


    
 
    //пробегаемся 
    SPI1DAT = low_index++;//SPI_WRITE_BUFFER;    // Load the XMIT register
    while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
    {                                
    }

	//пробегаемся 
    SPI1DAT = 0x21;//SPI_WRITE_BUFFER;    // Load the XMIT register
    while (TXBMT1 != 1)                  // Wait until the command is moved into the XMIT buffer
    {                                
    }






/*
   for (array_index = 0; array_index < MAX_BUFFER_SIZE; array_index++)
   {
	  //нужно модифицировать массив.
	  SPI1DAT = SPI_Data_Array[array_index]; // Load the data into the buffer
      while (TXBMT1 != 1)                    // Wait until the data is moved into the XMIT buffer
      {                             
      }
   } //end for
*/

   SPIF1 = 0;
   while (SPIF1 != 1)                   // Wait until the last byte of the data reaches the Slave
   {                                  
   }
   SPIF1 = 0;

   NSS1MD0 = 1;                         // Diable the Slave

   //ESPI0 = 1;                          // Re-enable SPI interrupts
   EIE2 = 0x8 ;                           // Re-enable SPI interrupts

}

#endif















