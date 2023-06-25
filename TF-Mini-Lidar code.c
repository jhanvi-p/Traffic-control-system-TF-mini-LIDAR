	//--- TF-mini LIDAR sensor code configuration ---
/*-----------------------------------------------------------------------------------
	Author:	 Jhanvi Prajapati
	Date:	12/10/2022	
	Modified:	None
	Description: Configured TF-Mini LIDAR using the datasheet to measure the distance from the ground to its surface. 	 
-----------------------------------------------------------------------------------*/

// Preprocessor ------------------------------------------------------------------------------

// Libraries ---------------------------------------------------------------------------------
#include "pragmas.h"
#include <p18f45k22.h>
#include <stdio.h>
#include <stdlib.h>
#include <usart.h>

// Constants ---------------------------------------------------------------------------------
#define TRUE		1
#define	FALSE		0	
#define STOP 		0
#define WALK 		1
#define ON 		1
#define OFF		0
#define MODESIZE	6
#define TMR0FLAG 	INTCONbits.TMR0IF //T0flag bit 

#define HEADERCHAR	0x59
#define DISTL 		2
#define DISTH 		3
#define RC2FLAG 	PIR3bits.RC2IF
#define BUFFERSIZE 	9


// Global Variables --------------------------------------------------------------------------

char triggerLiDAR[] 		= {0x5A,0x04,0x04,0x62,0x00};
char resetLiDAR[] 		= {0x5A, 0x04, 0x02, 0x60, 0x00};
char firmwareVersLiDAR[] 	= {0x5A, 0x04, 0x01, 0x5F, 0x00};
unsigned char buf[BUFFERSIZE];

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
	//LEDs of walk Indicators. 
	ANSELA 	= 0x00;	// Set all to digital
	LATA 	= 0x00; // No output voltage 
	TRISA	= 0x0F;	// Higher Nibble, Output (RA4-RA7)

	//LEDs RB0-RB5 LEDs
	ANSELB 	= 0x00;	// Set all to digital
	LATB 	= 0x00; // No output voltage   
	TRISB	= 0x00;	//Higher Nibble & Lower Nibble, Output (RB0-7)
			
	ANSELC 	= 0x00;	// Set all to digital
	LATC	= 0x00; // No output voltage
	TRISC	= 0xFF;	// Set all to input direction
	
	// PBs RD0-RD1 Pushbuttons
	ANSELD 	= 0x00;	// Set all to digital
	LATD 	= 0x00; // No output voltage
	TRISD	= 0xFF;	// Set all to input direction,(RD9-10)
	
	ANSELE 	= 0x00;	// Set all to digital
	LATE 	= 0x00; // No output voltage
	TRISE	= 0xFF;	// Set all to input direction

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
	OSCCON = 0x72;
	while( !OSCCONbits.HFIOFS );	// Wait for freq. to be stable.
}//eo setOSC::
/*>>> configADC: >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
Author:	Jhanvi Prajapati
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
} // eo initializeSystem::

/*--- MAIN: FUNCTION ------------------------------------------------------------------------
-------------------------------------------------------------------------------------------*/
void main( void )
{
	char hold = 0;
	int index = 0;
	int distance = 0;
	int currentDistance = 0;
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
		
				/*for(index = 1; index<BUFFERSIZE; index++)
				{
					while(!RC2FLAG);
					buf[index] = RCREG2;
				}*/
			 
				currentDistance = ( buf[ DISTH ]<<8 ) + buf[ DISTL ];
				//distance = ORIGINALDISTANCE - currentDistance; 
				printf("The distance is %d cm\n\r", currentDistance);
				//printf("System Ready...\n\r");
			}//eo if
		}// eo first header byte if
	}//eo while 1
}//eo main


