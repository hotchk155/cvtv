
//////////////////////////////////////////////////////////////
//
// MAIN MODULE
//
//////////////////////////////////////////////////////////////

//
// HEADER FILES
//
#include <system.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

/*
VDD        VSS
TRIG_IN		RA5        RA0	SCAN1
SCAN3		RA4        RA1	SCAN2
			RA3/MCLR#  RA2	IR_RX
CLOCK_IN	RC5        RC0	SCL	
READ1		RC4        RC1	SDA
READ2		RC3        RC2	GATE_OUT
*/


#define P_TRIG_IN		porta.5
#define P_SCAN3_OUT		lata.4
#define P_CLOCK_IN		portc.5
#define P_SW_READ1		portc.4
#define P_SW_READ2		portc.3

#define P_SCAN1_OUT		lata.0
#define P_SCAN2_OUT		lata.1
#define P_IR_RX			porta.2
#define P_GATE_OUT		latc.2

					    //76543210
#define TRIS_A			0b11111111
//#define TRIS_A			0b11101100
#define TRIS_C			0b11111011
//#define TRIS_C			0b11111011

#define DAC_ADDR    0b1100000

typedef unsigned char byte;


//
// TYPES
//


// Timer related stuff
#define TIMER_0_INIT_SCALAR		5		// Timer 0 initialiser to overlow at 1ms intervals
volatile byte ms_tick = 0;				// once per millisecond tick flag used to synchronise stuff
//volatile int millis = 0;				// millisecond counter

//
// GLOBAL DATA
//
#define I2C_TX_BUF_SZ 20
volatile byte g_cv_dac_pending;				// flag to say whether dac data is pending
volatile byte g_i2c_tx_buf[I2C_TX_BUF_SZ];	// transmit buffer for i2c
volatile byte g_i2c_tx_buf_index = 0;		// index of next byte to send over i2c
volatile byte g_i2c_tx_buf_len = 0;			// total number of bytes in buffer


////////////////////////////////////////////////////////////
// INTERRUPT SERVICE ROUTINE
void interrupt( void )
{
	/////////////////////////////////////////////////////
	// TIMER0 OVERFLOW
	// once per millisecond
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		ms_tick = 1;
//		++millis;
		intcon.2 = 0;		
	}		

/*	
	/////////////////////////////////////////////////////
	// UART RECEIVE
	if(pir1.5)
	{	
		byte b = rcreg;
		byte next_head = (rx_head + 1)&SZ_RXBUFFER_MASK;
		if(next_head != rx_tail) {
			rx_buffer[rx_head] = b;
			rx_head = next_head;
		}
		LED_1_PULSE(LED_PULSE_MIDI_IN);
		pir1.5 = 0;
	}
*/
	/////////////////////////////////////////////////////
	// I2C INTERRUPT
/*	
	if(pir1.3) 
	{
		pir1.3 = 0;
		if(g_i2c_tx_buf_index < g_i2c_tx_buf_len) {
			// send next data byte
			ssp1buf = g_i2c_tx_buf[g_i2c_tx_buf_index++];
		}
		else if(g_i2c_tx_buf_index == g_i2c_tx_buf_len) {
			++g_i2c_tx_buf_index;			
			ssp1con2.2 = 1; // send stop condition
		}
		else {			
			// check if there is any synchronised gate data (This mechanism is designed to trigger
			// a gate associated with a note only after the CV has been output to the DAC, so the 
			// gate does not open before the note CV sweeps to the new value)
			//if(g_sync_sr_data_pending) {
				//g_sr_data |= g_sync_sr_data;	// set the new gates
				//g_sync_sr_data = 0;				// no syncronised data pending now..
				//g_sync_sr_data_pending = 0;		
				//g_sr_data_pending = 1;			// but we do need to load the new info to shift regs
			//}			
			pie1.3 = 0; // we're done - disable the I2C interrupt
		}
	}
	*/
}

////////////////////////////////////////////////////////////
// I2C MASTER INIT
static void i2c_init() {
	// disable output drivers on i2c pins
	trisc.0 = 1;
	trisc.1 = 1;
	
	//ssp1con1.7 = 
	//ssp1con1.6 = 
	ssp1con1.5 = 1; // Enable synchronous serial port
	ssp1con1.4 = 1; // Enable SCL
	ssp1con1.3 = 1; // }
	ssp1con1.2 = 0; // }
	ssp1con1.1 = 0; // }
	ssp1con1.0 = 0; // } I2C Master with clock = Fosc/(4(SSPxADD+1))
	
	ssp1stat.7 = 1;	// slew rate disabled	
	ssp1add = 19;	// 100kHz baud rate
}

////////////////////////////////////////////////////////////
// I2C WRITE BYTE TO BUS
void i2c_send(byte data) {
	ssp1buf = data;
	while((ssp1con2 & 0b00011111) || // SEN, RSEN, PEN, RCEN or ACKEN
		(ssp1stat.2)); // data transmit in progress	
}

////////////////////////////////////////////////////////////
// I2C START WRITE MESSAGE TO A SLAVE
void i2c_begin_write(byte address) {
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.0 = 1; // signal start condition
	while(!pir1.3); // wait for it to complete
	i2c_send(address<<1); // address + WRITE(0) bit
}

////////////////////////////////////////////////////////////
// I2C FINISH MESSAGE
void i2c_end() {
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.2 = 1; // signal stop condition
	while(!pir1.3); // wait for it to complete
}

////////////////////////////////////////////////////////////
// I2C ASYNC SEND
void i2c_send_async() {
	pir1.3 = 0; // clear interrupt flag
	pie1.3 = 1; // enable the interrupt
	ssp1con2.0 = 1; // signal start condition					
}

void dac_init() {
	i2c_begin_write(DAC_ADDR);
	i2c_send(0b10011001); // buffered Vref, powered up, 2x
	i2c_end();	
}

void dac_set(int dac) {
    i2c_begin_write(DAC_ADDR); 
    i2c_send((dac>>8)&0xF); 
    i2c_send((byte)dac); 
	i2c_end();	
}
    
////////////////////////////////////////////////////////////
// INITIALISE TIMER
void timer_init() {
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 4MHz
	// 	prescaled 1/16 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
}

/*
////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void uart_init()
{
	pir1.1 = 0;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 0;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte (31250)	
	
}
*/

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 		
	// osc control / 16MHz / internal
	osccon = 0b01111010;

	trisa 	= TRIS_A;              	
    trisc 	= TRIS_C;   	
	ansela 	= 0b00000000;
	anselc 	= 0b00000000;
	porta 	= 0b00000000;
	portc 	= 0b00000000;

	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	g_cv_dac_pending = 0;
	// initialise the various modules
	//uart_init();
	i2c_init();	
	timer_init();	

	dac_init();
	//dac_set(2000);
	int i=0;
	while(1) {
		dac_set(i);
		i+=1; if(i>4095) i=0;
		P_GATE_OUT = 0;
		delay_ms(100);
		P_GATE_OUT = 1;
		delay_ms(100);
	}
	
	
	for(;;)
	{	
		// once per millisecond tick event
		if(ms_tick) {
			ms_tick = 0;
		}
		// check if there is any CV data to send out and no i2c transmit in progress
		if(!pie1.3 && g_cv_dac_pending) {
			//cv_dac_prepare(); 
			//i2c_send_async();
			g_cv_dac_pending = 0; 
		}				
	}
}

//
// END
//