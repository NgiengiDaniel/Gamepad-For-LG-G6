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
#include <string.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv/usbdrv.h"

void hardwareInit(void);
void read_joy(void);
void intitADC();
int16_t readADC( uint8_t ADCchannel );
int8_t mapTo8Bit( int16_t val );

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

// X/Y joystick w/ 16-bit readings (-127 to +127), 16 buttons
PROGMEM const char usbHidReportDescriptor [42] =
{
		0x05, 0x01,     // USAGE_PAGE (Generic Desktop)
		0x09, 0x05,     // USAGE (Game Pad)
		0xa1, 0x01,     // COLLECTION (Application)
		0x09, 0x01,     //   USAGE (Pointer)
		0xa1, 0x00,     //   COLLECTION (Physical)
		0x09, 0x30,     //     USAGE (X)
		0x09, 0x31,     //     USAGE (Y)
		0x15, 0x81,     //   LOGICAL_MINIMUM (-127)
		0x25, 0x7f,     //   LOGICAL_MAXIMUM (127)
		0x75, 0x10,     //   REPORT_SIZE (16)
		0x95, 0x02,     //   REPORT_COUNT (2)
		0x81, 0x02,     //   INPUT (Data,Var,Abs)
		0xc0,           // END_COLLECTION
		0x05, 0x09,     // USAGE_PAGE (Button)
		0x19, 0x01,     //   USAGE_MINIMUM (Button 1)
		0x29, 0x10,     //   USAGE_MAXIMUM (Button 16)
		0x15, 0x00,     //   LOGICAL_MINIMUM (0)
		0x25, 0x01,     //   LOGICAL_MAXIMUM (1)
		0x75, 0x01,     // REPORT_SIZE (1)
		0x95, 0x10,     // REPORT_COUNT (16)
		0x81, 0x02,     // INPUT (Data,Var,Abs)
		0xc0            // END_COLLECTION
};

uint16_t reportCurr[3];
uint16_t reportLast[3];
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

int8_t mapTo8Bit( int16_t val )
{
	return (( val )*( 254L ))/( 1023 )  - 127 ;
}
/* ------------------------------------------------------------------------- */

void read_joy( void )
{
	reportCurr [0] = 0;
	reportCurr [1] = 0;
	reportCurr [2] = 0;

	// X AXIS
	reportCurr [0] = -mapTo8Bit( readADC( 1 ) );		//	The negative sign is for correcting the way the hardware (analog joystick) is wired. Left and right are flipped.
	// Y AXIS
	reportCurr [1] = -mapTo8Bit( readADC( 0 ) );		//	The negative sign is for correcting the way the hardware (analog joystick) is wired. Up and down are flipped.


	// Buttons
	if ( !( PIND & 0x10 ) ) reportCurr[2] |= 0x01;		    //PRESSED  A
	if ( !( PIND & 0x20 ) ) reportCurr[2] |= 0x02;		    //PRESSED  B
	if ( !( PIND & 0x40 ) ) reportCurr[2] |= 0x04;		    //PRESSED  X
	if ( !( PIND & 0x80 ) ) reportCurr[2] |= 0x08;		    //PRESSED  Y
	if ( !( PINB & 0x01 ) ) reportCurr[2] |= 0x10;		    //PRESSED  LB
	if ( !( PINB & 0x02 ) ) reportCurr[2] |= 0x20;		    //PRESSED  RB
	if ( !( PINB & 0x04 ) ) reportCurr[2] |= 0x1000;	    //PRESSED  UP
	if ( !( PINB & 0x08 ) ) reportCurr[2] |= 0x2000;		//PRESSED  DOWN
	if ( !( PINB & 0x10 ) ) reportCurr[2] |= 0x4000;		//PRESSED  LEFT
	if ( !( PINC & 0x04 ) ) reportCurr[2] |= 0x8000;		//PRESSED  RIGHT
	if ( !( PINC & 0x08 ) ) reportCurr[2] |= 0x100;			//PRESSED  BACK / SELECT
	if ( !( PINC & 0x10 ) ) reportCurr[2] |= 0x200;			//PRESSED  START
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
