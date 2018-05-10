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



#include <c8051_f902_i2c.h>


#define  MY_ADDR        0x02       // Address of this SMBus device
		                           // (dummy value since this device does
		                           // not have any defined slave states)


// Device addresses (7 bits, lsb is a don't care) //optical module
//#define  EEPROM_ADDR    0xA0       // Device address for slave target Note: This address is specified in the Microchip 24LC02B
		                            // datasheet.

		       		
// Device addresses (7 bits, lsb is a don't care)   //EEPROM in KN-019 device
#define  EEPROM_ADDR    0xA6          				// Device address for slave target  Note: This address is specified in the Microchip 24LC02B
		                                       		// datasheet.


    		//rab_variant
   sbit SDA = P0^3;   						  //P0.3  ->SDA  (24_from_scematic)  Smbus
   sbit SCL = P0^4;   						  //P0.4  ->SCL  (23_from_schematic) Smbus
		





///////////////////////////I2C DEFENITION////////////////////////////////////////
// Status vector - top 4 bits only     //SMB0CN register for state Machine
// (MASTER, TXMODE, STA, and STO) form a status vector
	#define  SMB_MTSTA      0xE0           // (MT) start transmitted
	#define  SMB_MTDB       0xC0           // (MT) data byte transmitted
	#define  SMB_MRDB       0x80           // (MR) data byte received
// End status vector definition
	#define  WRITE          0x00           // WRITE direction bit
	#define  READ           0x01           // READ direction bit
// SMBus Buffer Size
	#define  SMB_BUFF_SIZE  0x08           // Defines the maximum number of bytes that can be sent or received in a  single transfer
	#define  SMB_BUS_ERROR  0x00           // (all modes) BUS ERROR
	#define  SMB_START      0x08           // (MT & MR) START transmitted
	#define  SMB_RP_START   0x10           // (MT & MR) repeated START
	#define  SMB_MTADDACK   0x18           // (MT) Slave address + W transmitted;ACK received
	#define  SMB_MTADDNACK  0x20           // (MT) Slave address + W transmitted;NACK received
	#define  SMB_MTDBACK    0x28           // (MT) data byte transmitted; ACK rec'vd
	#define  SMB_MTDBNACK   0x30           // (MT) data byte transmitted; NACK rec'vd
	#define  SMB_MTARBLOST  0x38           // (MT) arbitration lost
	#define  SMB_MRADDACK   0x40           // (MR) Slave address + R transmitted;ACK received
	#define  SMB_MRADDNACK  0x48           // (MR) Slave address + R transmitted; NACK received
	#define  SMB_MRDBACK    0x50           // (MR) data byte rec'vd;ACK transmitted
	#define  SMB_MRDBNACK   0x58           // (MR) data byte rec'vd;NACK transmitted
//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------
	unsigned char* pSMB_DATA_IN=0x00;        // Global pointer for SMBus data All receive data is written here
	unsigned char SMB_SINGLEBYTE_OUT=0x00;   // Global holder for single byte writes.
	unsigned char* pSMB_DATA_OUT=0x00;       // Global pointer for SMBus data.All transmit data is read from here
	unsigned char SMB_DATA_LEN=0x00;        // Global holder for number of bytes  to send or receive in the current SMBus transfer.
	unsigned char WORD_ADDR=0x00;            // Global holder for the EEPROM word address that will be accessed in the next transfer
	unsigned char TARGET=0x00;              // Target SMBus slave address
	bit SMB_BUSY = 0;                       // Software flag to indicate when the
	                                        // EEPROM_ByteRead() or EEPROM_ByteWrite()
	                                        // EEPROM_ByteWrite() functions have claimed the SMBus
	bit SMB_RW;                             // Software flag to indicate the direction of the current transfer
	bit SMB_SENDWORDADDR;                  // When set, this flag causes the ISR to send the 8-bit <WORD_ADDR> after sending the slave address.
	bit SMB_RANDOMREAD;                    // When set, this flag causes the ISR
	                                       // to send a START signal after sending
	                                       // the word address.
	                                       // For the 24LC02B EEPROM, a random read
	                                       // (a read from a particular address in
	                                       // memory) starts as a write then
	                                       // changes to a read after the repeated
	                                       // start is sent. The ISR handles this
	                                       // switchover if the <SMB_RANDOMREAD>
	                                       // bit is set.
	bit SMB_ACKPOLL;                       // When set, this flag causes the ISR to send a repeated START until the slave has acknowledged 
										   // its address




/*****************************************************************************
Syntax:  void SMBus_Init () 	    
Remarks:			    
*******************************************************************************/
void SMBus_Init ()
{
    	
                 SMB0CF =  0x5D;       // Use Timer1 overflows as SMBus clock source;
                          			   // Disable slave mode;
                          			   // Enable setup & hold time extensions;
                          			   // Enable SMBus Free timeout detect;
                          			   // Enable SCL low timeout detect;
	 			 SMB0ADR = MY_ADDR;    // Set own slave address 
	 		 	 SMB0CF |= 0x80;   	   // Enable SMBus;       
                 EIE1 |= 0x01;         // Enable the SMBus interrupt 

}



/*****************************************************************************
Syntax:  unsigned char SMBus_ByteRead(unsigned char addr)	    
Remarks:			   
* Return Value :
*  1) unsigned char data - data read from address <addr> in the EEPROM
*                       range is full range of character: 0 to 255
*
* Parameters   :
*   1) unsigned char addr - address to read data from the EEPROM
*                        range is full range of character: 0 to 255
*
* This function returns a single byte from location <addr> in the EEPROM then
* polls the <SMB_BUSY> flag until the read is complete.
*******************************************************************************/
unsigned char SMBus_ByteRead(unsigned char addr)
{
   unsigned char retval=0x00;               // Holds the return value

   while (SMB_BUSY);                        // Wait for SMBus to be free.
   SMB_BUSY = 1;                            // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;                   // Set target slave address
   SMB_RW = WRITE;                         // A random read starts as a write
                                       	   // then changes to a read after
                                           // the repeated start is sent. The
                                           // ISR handles this switchover if
                                           // the <SMB_RANDOMREAD> bit is set.
   SMB_SENDWORDADDR = 1;                   // Send Word Address after Slave Address
   SMB_RANDOMREAD = 1;                     // Send a START after the word address
   SMB_ACKPOLL = 1;                        // Enable Acknowledge Polling


   // Specify the Incoming Data
   WORD_ADDR = addr;                      // Set the target address in the
                                          // EEPROM's internal memory space

   pSMB_DATA_IN = &retval;                // The incoming data pointer points to
                                          // the <retval> variable.

   SMB_DATA_LEN = 1;                      // Specify to ISR that the next transfer
                                          // will contain one data byte

   // Initiate SMBus Transfer
   STA = 1;
   while(SMB_BUSY);                      // Wait until data is read

   return retval;

}


/*****************************************************************************
Syntax:  void SMBus_ByteWrite(unsigned char addr, unsigned char dat)	    
Remarks:			   
* Return Value :
*  1)unsigned char addr - address to write in the EEPROM
*                        range is full range of character: 0 to 255
*
* Parameters   :
*   1) unsigned char dat - data to write to the address <addr> in the EEPROM
*                       range is full range of character: 0 to 255
*
* This function writes the value in <dat> to location <addr> in the EEPROM
* then polls the EEPROM until the write is complete.
*******************************************************************************/
void SMBus_ByteWrite(unsigned char addr, unsigned char dat)
{
   while (SMB_BUSY);                   // Wait for SMBus to be free.
   SMB_BUSY = 1;                       // Claim SMBus (set to busy)

   // Set SMBus ISR parameters
   TARGET = EEPROM_ADDR;               // Set target slave address
   SMB_RW = WRITE;                     // Mark next transfer as a write
   SMB_SENDWORDADDR = 1;               // Send Word Address after Slave Address
   SMB_RANDOMREAD = 0;                 // Do not send a START signal after
                                       // the word address
   SMB_ACKPOLL = 1;                    // Enable Acknowledge Polling (The ISR
                                       // will automatically restart the
                                       // transfer if the slave does not
                                       // acknowledge its address.

   // Specify the Outgoing Data
   WORD_ADDR = addr;                   // Set the target address in the
                                       // EEPROM's internal memory space

   SMB_SINGLEBYTE_OUT = dat;           // Store <dat> (local variable) in a
                                       // global variable so the ISR can read
                                       // it after this function exits

   // The outgoing data pointer points to the <dat> variable
   pSMB_DATA_OUT = &SMB_SINGLEBYTE_OUT;

   SMB_DATA_LEN = 1;                   // Specify to ISR that the next transfer
                                       // will contain one data byte

   // Initiate SMBus Transfer
   STA = 1;
}


/*****************************************************************************
Syntax:  void SMBUS_ISR (void) interrupt 7 	    
Remarks: SMBus ISR state machine
*		 - Master only implementation - no slave states defined	
*        - All incoming data is written starting at the global pointer <pSMB_DATA_IN>
*        - All outgoing data is read from the global pointer <pSMB_DATA_OUT>			    
*******************************************************************************/
void SMBUS_ISR (void) interrupt 7
{
   bit FAIL = 0;                       // Used by the ISR to flag failed
                                       // transfers
   static char i;                      // Used by the ISR to count the
                                       // number of data bytes sent or
                                       // received
   static bit SEND_START = 0;          // Send a start

  
		 switch (SMB0CN & 0xF0)
	     {
              
            // Master Transmitter/Receiver: START condition transmitted.
            case SMB_MTSTA:
							  SMB0DAT  = TARGET;  				// Load address of the target slave
						      SMB0DAT &= 0xFE;   				// Clear the LSB of the address for the R/W bit
						      SMB0DAT |= SMB_RW; 				// Load R/W bit
                  			  STA=0;							// REG_SMB0CN_BIT_BUS_START = 0;   	    // Manually clear START bit   
                  			  i=0;						        // Reset data byte counter
			break;
		    // Master Transmitter: Data byte (or Slave Address) transmitted
            case SMB_MTDB:
						     
                  if(ACK/*REG_SMB0CN_BIT_BUS_Acknowelege*/)  						
				  {
										
					     if(SEND_START)
						 {
                        	STA=1;				//REG_SMB0CN_BIT_BUS_START	= 1;  //STA =1;
                       		SEND_START = 0;
                        	break;											 
                     	 }
                     //
						if(SMB_SENDWORDADDR)               // Are we sending the word address? 
						{
                        	SMB_SENDWORDADDR = 0;   	    // Clear flag
							SMB0DAT = WORD_ADDR;    		// Send word address 
                        
							if(SMB_RANDOMREAD) 
							{ 	
								 SEND_START = 1;      				// Send a START after the next ACK cycle
								 SMB_RW = READ;
						    }													
                        break;				  
                        }
										 //
						if(SMB_RW==WRITE)
						{
						   if(i < SMB_DATA_LEN)
						   {
							SMB0DAT = *pSMB_DATA_OUT;  // send data byte
							pSMB_DATA_OUT++;           		// increment data out pointer  
							i++;  													// increment number of bytes sent
                        }													
						else
                        {
                           STO=0;///REG_SMB0CN_BIT_BUS_STOP        =0;     //STO
                           SMB_BUSY = 0;         									// Clear software busy flag
						}													
                      
                     } 											 
					 else
					 {
                       //no_action
                     }											 
										 //
										
									}
                  else //if SMB no slave ACK Poll
                  {
											
					   if(SMB_ACKPOLL)
					   {
						  STA=1;//REG_SMB0CN_BIT_BUS_START	= 1;  // Restart transfer STA Start=1;
                       }	
                      else
                      {
                         FAIL = 1;                     // Indicate failed transfer and handle at end of ISR
				       }												
										
                 }										
							
            break;		
						// Master Receiver: byte received
            case SMB_MRDB:
					if(i < SMB_DATA_LEN)     // Is there any data remaining?
				    {
										
						*pSMB_DATA_IN = SMB0DAT;   				 // Store received byte 
						 pSMB_DATA_IN++;            						 // Increment data in pointer
						 i++;  
 						 // Increment number of bytes received
						 //REG_SMB0CN_BIT_BUS_Acknowelege =1;     //ACK 
                         ACK=1;
					}

                    if(i == SMB_DATA_LEN)    								 // This is the last byte
                    {
                        SMB_BUSY = 0;              			 // Free SMBus interface
						ACK=0;
						STO=1;
						//REG_SMB0CN_BIT_BUS_Acknowelege =0;    //ACK 
                        //REG_SMB0CN_BIT_BUS_STOP        =1;    //STO Stop Transfer on I2C
                    }											
						
											
						break;
						
			//Default Fail 
			default:
			
			 FAIL = 1;                     // Indicate failed transfer and handle at end of ISR
			
			break;
						
          }
 
				// If the transfer failed,
		  if(FAIL) 
		  {
          	 SMB0CF &= ~0x80;                   // Reset communication
			 SMB0CF  |= 0x80;  
					
			
			 STO=0;
             STA=0;
			 ACK=0;  
			//REG_SMB0CN_BIT_BUS_STOP        =0;     //STO
			//REG_SMB0CN_BIT_BUS_START       =0;     //STA
			//REG_SMB0CN_BIT_BUS_Acknowelege =0;     //ACK 
            FAIL = 0;
         }  					

    //Clear interrupt flag SI on register 
    //REG_SMB0CN_BIT_BUS_INT=0;
      SI=0;   

}



////Тестовые Функции
/*****************************************************************************
Syntax:  void test_kn19_single_byte_read_array()	    
Remarks: 			    
*******************************************************************************/
void test_kn19_single_byte_read_array()
{
     	unsigned char temp_char=0x00;            // Temporary variable
		bit error_flag = 0;                 	 // Flag for checking EEPROM contents
	
		
	   //Test EEPROM for KN_019

	
		   // Write the value 0xAA to location 0x25 in the EEPROM
		   // Read the value at location 0x25 in the EEPROM
		   temp_char = SMBus_ByteRead(0x00);
           temp_char = SMBus_ByteRead(0x01);
		   temp_char = SMBus_ByteRead(0x02);
		   temp_char = SMBus_ByteRead(0x03);
		   temp_char = SMBus_ByteRead(0x04);
		   temp_char = SMBus_ByteRead(0x05);
		   temp_char = SMBus_ByteRead(0x06);
		   temp_char = SMBus_ByteRead(0x07);
		   temp_char = SMBus_ByteRead(0x08);
		   temp_char = SMBus_ByteRead(0x10);
           temp_char = SMBus_ByteRead(0x20);
		   temp_char = SMBus_ByteRead(0x02);
		   temp_char = SMBus_ByteRead(0x60);
		   temp_char = SMBus_ByteRead(0x61);
		   temp_char = SMBus_ByteRead(0x62);
		   temp_char = SMBus_ByteRead(0x63);
		   temp_char = SMBus_ByteRead(0x64);
		   temp_char = SMBus_ByteRead(0x65); 
           


		   // Check that the data was read properly
		   //if (temp_char != 0x55)
		   //{
		    //  error_flag = 1;
		   //}

		   // Write the value 0xBB to location 0x25 in the EEPROM
		   //EEPROM_ByteWrite(0x25, 0xBA);

		   // Write the value 0xCC to location 0x38 in the EEPROM
		   //EEPROM_ByteWrite(0x38, 0xCA);

		   // Read the value at location 0x25 in the EEPROM
		   //temp_char = EEPROM_ByteRead(0x01);

		   // Check that the data was read properly
		   //if (temp_char != 0x99)
		   //{
		    //  error_flag = 1;
		  // }

		   // Read the value at location 0x38 in the EEPROM
		   //temp_char = EEPROM_ByteRead(0x10);

		   // Check that the data was read properly
		   //if (temp_char != 0x0E)
		  // {
		    //  error_flag = 1;
		  // }


}








