/*************************************************************************
 * Simple joystick with 2 axis and 12 buttons with V-USB stack, tested on
 *ATMega328p (Arduino NANO board) @16Mhz.
 *************************************************************************/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <stdio.h>
#include <stdint.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"

void hardwareInit(void);
void read_joy(void);
void intitADC();
int16_t readADC( uint8_t ADCchannel );
int8_t mapTo8Bit( int16_t val );

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

// (X/Y) and (Rx/Ry) Gamepad w/ 8-bit readings (-127 to +127), 16 buttons

PROGMEM const char usbHidReportDescriptor [62] =
{
		0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
		0x09, 0x05,     // USAGE (Game Pad)
		0xa1, 0x01,     // COLLECTION (Application)

		0xa1, 0x00,     //   COLLECTION (Physical)

		0x05, 0x01,     //	   USAGE_PAGE (Generic Desktop)
		0x09, 0x30,     //     USAGE (X)
		0x09, 0x31,     //     USAGE (Y)
		0x09, 0x33,		//     USAGE (Rx)
		0x09, 0x34,		//     USAGE (Ry)
		0x15, 0x81,     //     LOGICAL_MINIMUM (-127)
		0x25, 0x7f,     //     LOGICAL_MAXIMUM (127)
		0x75, 0x08,     //     REPORT_SIZE (8)
		0x95, 0x04,     //     REPORT_COUNT (4)
		0x81, 0x02,     //     INPUT (Data,Var,Abs)

		0x05, 0x09,     // 	   USAGE_PAGE (Button)
		0x19, 0x01,     //     USAGE_MINIMUM (Button 1)
		0x29, 0x08,     //     USAGE_MAXIMUM (Button 8)
		0x15, 0x00,     //     LOGICAL_MINIMUM (0)
		0x25, 0x01,     //     LOGICAL_MAXIMUM (1)
		0x75, 0x01,     //     REPORT_SIZE (1)
		0x95, 0x08,     //     REPORT_COUNT (8)
		0x81, 0x02,     //     INPUT (Data,Var,Abs)

		0x05, 0x09,     // 	   USAGE_PAGE (Button)
		0x19, 0x01,     //     USAGE_MINIMUM (Button 1)
		0x29, 0x08,     //     USAGE_MAXIMUM (Button 8)
		0x15, 0x00,     //     LOGICAL_MINIMUM (0)
		0x25, 0x01,     //     LOGICAL_MAXIMUM (1)
		0x75, 0x01,     //     REPORT_SIZE (1)
		0x95, 0x08,     //     REPORT_COUNT (8)
		0x81, 0x02,     //     INPUT (Data,Var,Abs)

		0xc0,           // 	 END_COLLECTION
		0xc0,           // END_COLLECTION

};

uint8_t reportCurr[6];
uint8_t reportLast[6];
uint8_t    idleRate;

/* ------------------------------------------------------------------------- */

int main(void)
{
	hardwareInit();

    usbInit();

    sei();

    for(;;)
    {                /* main event loop */
    	usbPoll();
        if(usbInterruptIsReady())
        {
			/* called after every poll of the interrupt endpoint */
			read_joy();
			if ( memcmp( reportLast, reportCurr, sizeof reportCurr ) )
			{
				memcpy( reportLast, reportCurr, sizeof reportCurr );
				usbSetInterrupt( reportLast, sizeof reportLast );
			}
        }
    }
}
/* ------------------------------------------------------------------------- */

void hardwareInit(void)
{
 DDRB	= 	0b00000000;
 PORTB	=	0b00011111;

 DDRD	= 	0b00000000;
 PORTD	=	0b11110011;

 DDRC 	= 	0b00000000;
 PORTC	= 	0b00011100;

 intitADC();
}
/* ------------------------------------------------------------------------- */

void intitADC()

{	//Select Vref = AVcc
	ADMUX |= ( 1 << REFS0 );
	//set Prescaller to 128 and enable ADC
	ADCSRA |= ( 1 << ADPS2 ) | ( 1 << ADPS1 ) | ( 1 << ADPS0 ) | ( 1 << ADEN );
}
/* ------------------------------------------------------------------------- */

int16_t readADC(uint8_t ADCchannel)
{
    //select ADC channel with safety mask
    ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
    //single conversion mode
    ADCSRA |= (1<<ADSC);
    // wait until ADC conversion is complete
    while( ADCSRA & (1<<ADSC) );
    return ( ADC );
}
/* ------------------------------------------------------------------------- */

int8_t mapTo8Bit( int16_t val )						/*Note: The nano has 10 bits ADC precision.*/
{											 		/*This needs to be scalled down to 8 bits*/
	return (( val )*( 254L ))/( 1023 )  - 127 ;
}
/* ------------------------------------------------------------------------- */

void read_joy( void )
{	//Sending 0 when no event is registered.
	reportCurr [0] = 0;
	reportCurr [1] = 0;
	reportCurr [2] = 0;
	reportCurr [3] = 0;
	reportCurr [4] = 0;
	reportCurr [5] = 0;


	// X AXIS
	reportCurr [0] = -mapTo8Bit( readADC( 1 ) ); 	/*	The negative sign is for correcting 	*/
	// Y AXIS									 	/*	the way the hardware (analog joystick) 	*/
	reportCurr [1] = -mapTo8Bit( readADC( 0 ) ); 	/*	is wired. Left and right are flipped.	*/
	// Rx AXIS (RIGHT ANALOG X AXIS)			 	/*	Up and down are flipped.				*/
	reportCurr [2] = mapTo8Bit( readADC( 7 ) ); 	//ANALOG ONLY PIN A7
	// Ry AXIS (RIGHT ANALOG Y AXIS)
	reportCurr [3] = -mapTo8Bit( readADC( 6 ) );	//ANALOG ONLY PIN A6


	// Buttons
	if ( !( PIND & 0x10 ) ) reportCurr[4] |= 0x01;		    //PRESSED  A                DIGITAL PIN 4
	if ( !( PIND & 0x20 ) ) reportCurr[4] |= 0x02;		    //PRESSED  B                DIGITAL PIN 5
	if ( !( PIND & 0x40 ) ) reportCurr[4] |= 0x04;		    //PRESSED  X                DIGITAL PIN 6
	if ( !( PIND & 0x80 ) ) reportCurr[4] |= 0x08;		    //PRESSED  Y                DIGITAL PIN 7
	if ( !( PINB & 0x01 ) ) reportCurr[4] |= 0x10;		    //PRESSED  LB 				DIGITAL PIN 8
	if ( !( PINB & 0x02 ) ) reportCurr[4] |= 0x20;		    //PRESSED  RB 				DIGITAL PIN 9
	if ( !( PINB & 0x04 ) ) reportCurr[4] |= 0x40;	   		 //PRESSED  UP 				DIGITAL PIN 10
	if ( !( PINB & 0x08 ) ) reportCurr[4] |= 0x80;			//PRESSED  DOWN 			DIGITAL PIN 11
	if ( !( PINB & 0x10 ) ) reportCurr[5] |= 0x01;			//PRESSED  LEFT 			DIGITAL PIN 12
	if ( !( PINC & 0x04 ) ) reportCurr[5] |= 0x02;			//PRESSED  RIGHT 			ANALOG PIN A2
	if ( !( PINC & 0x08 ) ) reportCurr[5] |= 0x04;			//PRESSED  BACK / SELECT	ANALOG PIN A3
	if ( !( PINC & 0x10 ) ) reportCurr[5] |= 0x08;			//PRESSED  START			ANALOG PIN A4
}
/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */

        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one reportCurr type, so don't look at wValue */
            usbMsgPtr = (void *)&reportCurr;
            return sizeof(reportCurr);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;   /* default for not implemented requests: return no data back to host */
}

/* ------------------------------------------------------------------------- */
