 		//--- Intersection Traffic Controller Programme ---
/*-----------------------------------------------------------------------------------
	File Name:Traffic control System
	Author:	Jhanvi Prajapati
	Date:	12/10/2022	
	Modified:	None
	Description: Build a Circuit for a Intersection Traffic Controller using LCD and display the 
		output on LCD screen.
-----------------------------------------------------------------------------------*/

// Preprocessor ------------------------------------------------------------------------------

// Libraries ---------------------------------------------------------------------------------
#include "pragmas.h"
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <usart.h>
#include <delays.h>
#include "myXlcd.h"
//#include <xlcd.h>

// Constants ---------------------------------------------------------------------------------
#define TRUE		1
#define	FALSE		0	
#define STOP 		0
#define WALK 		1
#define ON 		1
#define OFF		0
#define TMR0FLAG 	INTCONbits.TMR0IF //T0flag bit 

#define HEADERCHAR	0x59
#define DISTL 		2
#define DISTH 		3
#define RC2FLAG 	PIR3bits.RC2IF
#define BUFFERSIZE 	9
#define RS 		PORTBbits.RB1
#define EN 		PORTBbits.RB2

#define LDATA 		PORTA 
#define LCDPORT 	TRISA



// Global Variables --------------------------------------------------------------------------

char triggerLiDAR[] 		= {0x5A,0x04,0x04,0x62,0x00};
char resetLiDAR[] 		= {0x5A, 0x04, 0x02, 0x60, 0x00};
char firmwareVersLiDAR[] 	= {0x5A, 0x04, 0x01, 0x5F, 0x00};
unsigned char buf[BUFFERSIZE];
unsigned char hello[] = {'H','E','L','L','O'};


// Functions ---------------------------------------------------------------------------------
/*--- portConfig: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		12/10/2022
Modified:	None
Desc:		This function configures port registeries to their specific criteria.
Input: 		None
Returns:	None
------------------------------------------------------------------------------*/
void portConfig()
{
	ANSELA = 0x00; //All Digital pins
	LATA   = 0x00; // All pins off
	TRISA  = 0x00; // All input
	
	ANSELB = 0x00; // All Digital pins
	LATB   = 0x00; // All pins off
	TRISB  = 0xF9; // All input, RB3-0 set as output

	ANSELC = 0x00; // All Digital pins
	LATC   = 0x00; // All pins off
	TRISC  = 0xFF; // All input	
	 
	ANSELD = 0x00; // All Digital pins
	LATD   = 0x00; // All pins off
	TRISD  = 0xFF; // All output

	ANSELE = 0x00; // All Digital pins
	LATE   = 0x00; // All pins off
	TRISE  = 0xFF; // All input
}//eo portConfig::
/*--- configSerialPort: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the serial port to the given values. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void configSerialPort()
{
	SPBRG1 = 34; // 115200 baud
	SPBRGH1 = 0x00, // no value
	TXSTA1 = 0x26; //TREN-1,8-bit,no parity,non-inverted
	RCSTA1 = 0x90; //SPEN on,RC enabled
	BAUDCON1 = 0x48; //all default
}// eo configSerialPort::

/*--- configSerialPort2: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the serial port to the given values. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void configSerialPort2()
{
	SPBRG2 = 34; //  115200  baud
	SPBRGH2 = 0x00, // 115200 no value
	TXSTA2 = 0x26; //TREN-1,8-bit,no parity,non-inverted
	RCSTA2 = 0x90; //SPEN on,RC enabled
	BAUDCON2 = 0x48; //all default
}// eo configSerialPort2::
/*--- resetTimer0: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		12/10/2022
Modified:	None
Desc:		Resets the timer module to use it again in main function. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void resetTimer0()
{
	//Pre-scaler and pre-set count for 1 second rollover
	TMR0FLAG = FALSE;
	TMR0H = 0x0B;
	TMR0L = 0xDC;
}// eo resetTimer0::
/*--- configTimer0: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the timer function. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void configTimer0()
{
	T0CON = 0x93;//Regarding to the PSV T0CON
	resetTimer0();
}// eo configTimer0::
/*---delay: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		12/10/2022
Modified:	None
Desc:		This function configures port registeries to their specific criteria.
Input: 		None
Returns:	None
------------------------------------------------------------------------------*/
void delay()
{
	int index = 0;
	for(index = 0; index<10000; index++)
	{
		Nop();
	}

}// eo delay::
/*>>> setOSC: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		CTalbot
Date:		12/10/2022
Modified:	None
Desc:		This function configures FOSC for 4MHz
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void setOSC()
{
	OSCCON = 0x52;
	while( !OSCCONbits.HFIOFS );	// Wait for freq. to be stable.
}//eo setOSC::
/*>>> configADC: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Jhanvi Prajapati
Date:		12/10/2022
Modified:	None
Desc:		This function configures the ADC module 
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void configADC (void)
{
	ADCON0 = 0x01;
	ADCON1 = 0x00;
	ADCON2 = 0xA9;
}//eo configADC::

/*--- LCD_Command: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the serial port to the given values. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void LCD_Command(unsigned char command)
{
	RS = 0;  /*Command Register is selected i.e.RS=0*/ 
	delay();
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	delay();
	LDATA = command;  /*Send higher nibble of command first to PORT*/  
	delay();
	EN = 0; /*High-to-low pulse on Disable pin to unlatch data*/
	delay();
}// eo LCD_Command::
/*--- sendData: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the serial port to the given values. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void sendData(unsigned int data)
{
	RS = 1;  /*Command Register is selected i.e.RS=0*/ 
	delay();
	EN = 1;  /*High-to-low pulse on Enable pin to latch data*/
	delay();
	LDATA = data;  /*Send higher nibble of command first to PORT*/  
	delay();
	EN = 0; /*High-to-low pulse on Disable pin to unlatch data*/
	delay();
}//eo sendData::
/*--- initializeLCD: ------------------------------------------------------------
Author:		Jhanvi Prajapati
Date:		28/09/2022
Modified:	None
Desc:		Configures the serial port to the given values. 
Input: 		None 
Returns:	None 
------------------------------------------------------------------------------*/
void initializeLCD(void)
{
	LCD_Command(0x38);    /*send for initialization of LCD with nibble method */
    	LCD_Command(0x0F);    /*use 2 line and initialize 5*7 matrix in (8-bit mode)*/
    	LCD_Command(0x01);    /*clear display screen*/
    	LCD_Command(0x80);    /*display on cursor off*/
}//eo initializeLCD::

/*>>> initializeSystem: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:		Jhanvi Prajapati
Date:		12/10/2022
Modified:	None
Desc:		Calls all the config functions for this platform.
Input: 		None
Returns:	None
>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>*/
void initializeSystem(void)
{
	setOSC();	// configs the Fosc to 4MHz
	portConfig();	// configs all the port pins for this code.
	configSerialPort();
	configSerialPort2();
	initializeLCD();	
} // eo initializeSystem::


/*--- MAIN: FUNCTION ------------------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
void main( void )
{
	char hold = 0;
	int index = 0;
	int distance = 0;
	int currentDistance = 0;
	int index1 = 0;
	int disthold = 0;
	int newdist = 0;
	initializeSystem();
	
	while(1)
	{	
		puts2USART( triggerLiDAR );	// Trigger the LiDAR to measure and send package.
		while( !RC2FLAG );			// Do nothing until a byte is received.
		if( RC2FLAG )				// If a byte is received, collect the remaining bytes from the LiDAR.
		{
			hold = RCREG2;
			if( hold == HEADERCHAR )
			{
				buf[ 0 ] = hold;	// save byte into the buf array in location 0.
				index = 1;			// set this for inserting the remaining bytes into the buf array.
				
				// Collect remaining bytes, only collect 8 more past the first header char.
				while( index < BUFFERSIZE )
				{
					if(RC2FLAG)	// If a byte is ready...
					{
						buf[ index ] = RCREG2;	// Get the byte from the receiver.
						index++;				// Increment the insert point to the next element of the array.
					}//eo if RC2FLAG
				} 
				currentDistance = ( buf[ DISTH ]<<8 ) + buf[ DISTL ];
				printf("The distance is %d cm\n\r", currentDistance);
			}//eo if
		}// eo first header byte if
			LCD_Command(0x80);
			sendData (currentDistance);
	}//eo while 1
}//eo main
