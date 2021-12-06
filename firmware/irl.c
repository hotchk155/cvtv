
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
#define P_SCAN3		lata.4
#define P_CLOCK_IN		portc.5
#define P_SW_READ1		portc.4
#define P_SW_READ2		portc.3

#define M_WPUC			0b00011000

#define P_SCAN1		lata.0
#define P_SCAN2		lata.1
#define P_IR_RX			porta.2
#define P_GATE_OUT		latc.2

					    //76543210
#define TRIS_A			0b11101100
//#define TRIS_A			0b11101100
#define TRIS_C			0b11111011
//#define TRIS_C			0b11111011

#define DAC_ADDR    0b1100000

typedef unsigned char byte;

enum {
	ST_WAITING,
	ST_CAPTURING,
	ST_PENDING
};

#define MAX_EDGES 	100
volatile unsigned short edge[MAX_EDGES];
volatile int edge_count = 0;
volatile byte capture_state = 0;

//
// TYPES
//

// we want a timer 1 tick every 64us


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
	// indicates a timeout waiting for an IR transition
	if(intcon.2)
	{
		switch(capture_state) {
			case ST_WAITING:
			case ST_PENDING:			
				// the timer is always running, so we'll periodically
				// get this interrupt while waiting for IR info or for
				// a message to be processed
				break;
			case ST_CAPTURING:
P_GATE_OUT = 0;		
				capture_state = ST_PENDING;
				intcon.4 = 0;		// disable the INT pin
				option_reg.6 = 0; 	// next INT pin interrupt will be falling edge
				break;
				
		}
		intcon.2 = 0;				
	}
	
	// external interrupt condition
	if(intcon.1) {
		byte d = tmr0;
		tmr0 = 0;
P_SCAN3 = !P_SCAN3;		
		switch(capture_state) {
			case ST_WAITING:
				edge_count = 0;
				capture_state = ST_CAPTURING;
P_GATE_OUT = 1;		
				break;
			case ST_CAPTURING:				
				if(edge_count < MAX_EDGES-1) {
					edge[edge_count++] = d;
				}
				break;
			case ST_PENDING:			
				// should not happen... the external interrupt
				// should be disabled in the pending state
				break;
		}
		option_reg.6 = !option_reg.6;			
		intcon.1 = 0;
	}		

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
// INITIALISE TIMER0
void timer_init() {
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 4MHz (
	//  each tick 0.25us
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // } timer 0 is prescaled /64
	option_reg.2 = 1; // }
	option_reg.1 = 0; // } 
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
}

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
	txsta.5 = 1;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 0;	// SREN 	enable receiver
	rcsta.4 = 0;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte 
	
}

void uart_send(byte ch) 
{
	txreg = ch;
	while(!txsta.1);
}

void uart_send_string(byte *ch) 
{
	while(*ch++) {
		uart_send(*ch);
	}
}

void uart_send_number(byte ch) {
	uart_send('0' + ch/100);
	ch %= 100;
	uart_send('0' + ch/10);
	ch %= 10;
	uart_send('0' + ch);
}


enum {
	POS_MIDDLE,
	POS_UP,
	POS_DOWN
};
byte switch_pos[3] = {0};

void read_switches() {

	switch_pos[0] = POS_MIDDLE;
	switch_pos[1] = POS_MIDDLE;
	switch_pos[2] = POS_MIDDLE;

	P_SCAN1 = 0;
	P_SCAN2 = 1;
	P_SCAN3 = 1;
	delay_ms(1);
	if(!P_SW_READ2) {
		switch_pos[2] = POS_UP;
	}
	if(!P_SW_READ1) {
		switch_pos[2] = POS_DOWN;
	}
	
	P_SCAN1 = 1;
	P_SCAN2 = 0;
	P_SCAN3 = 1;
	delay_ms(1);
	if(!P_SW_READ2) {
		switch_pos[1] = POS_UP;
	}
	if(!P_SW_READ1) {
		switch_pos[1] = POS_DOWN;
	}

	P_SCAN1 = 1;
	P_SCAN2 = 1;
	P_SCAN3 = 0;
	delay_ms(1);
	if(!P_SW_READ2) {
		switch_pos[0] = POS_UP;
	}
	if(!P_SW_READ1) {
		switch_pos[0] = POS_DOWN;
	}

	P_SCAN1 = 1;
	P_SCAN2 = 1;
	P_SCAN3 = 1;
	
}

void blink(int count) {
	if(!count) {
		P_GATE_OUT = 1;
		delay_ms(255);
		delay_ms(255);
		P_GATE_OUT = 0;
	}
	else while(count-->0) {
		P_GATE_OUT = 1;
		delay_ms(20);
		P_GATE_OUT = 0;
		delay_ms(200);
	}
	delay_ms(255);
	delay_ms(255);
}



// each timer tick is 8us
#define TIMER0_TICK_US	8

unsigned int parse_NEC_Samsung() {
	const int MIN_HIGH_TICKS = (500/TIMER0_TICK_US);
	const int MAX_HIGH_TICKS = (700/TIMER0_TICK_US);
	const int MIN_LOW_TICKS = (500/TIMER0_TICK_US);
	const int MARK_THRESHOLD_TICKS = (700/TIMER0_TICK_US);
	unsigned int data = 0;
	for(int i=0; i<edge_count; i+=2) {
		if( edge[i]<MIN_HIGH_TICKS || 
			edge[i]>MAX_HIGH_TICKS ||
			edge[i+1]<MIN_LOW_TICKS ) {
			// pulse length out of bounds
			return 0;
		}
		data<<=1;
		if(edge[i+1]>=MARK_THRESHOLD_TICKS) {
			// mark (digit 1) defined by long LOW time
			data |= 1;
		}
	}
	return data;
}

unsigned int parse_SONY() {
	const int MIN_HIGH_TICKS = (450/TIMER0_TICK_US);
	const int MAX_HIGH_TICKS = (1500/TIMER0_TICK_US);
	const int MIN_LOW_TICKS = (450/TIMER0_TICK_US);
	const int MAX_LOW_TICKS = (450/TIMER0_TICK_US);
	const int MARK_THRESHOLD_TICKS = (1000/TIMER0_TICK_US);
	unsigned int data = 0;
	for(int i=0; i<edge_count; i+=2) {
		if( edge[i]<MIN_HIGH_TICKS || 
			edge[i]>MAX_HIGH_TICKS ||
			edge[i+1]<MIN_LOW_TICKS ||
			edge[i+1]>MAX_LOW_TICKS ) {
			// pulse length out of bounds
			return 0;
		}
		data<<=1;
		if(edge[i]>=MARK_THRESHOLD_TICKS) {
			// mark (digit 1) defined by long HIGH time
			data |= 1;
		}
	}
	return data;	
}

unsigned int parse_RC5() {
	const int MIN_SGL_TICKS = (700/TIMER0_TICK_US);
	const int MAX_SGL_TICKS = (1100/TIMER0_TICK_US);
	const int MIN_DBL_TICKS = (1600/TIMER0_TICK_US);
	const int MAX_DBL_TICKS = (2000/TIMER0_TICK_US);
	enum {
		LEFTOVER_NONE,
		LEFTOVER_MARK,
		LEFTOVER_SPACE
	};
	
	unsigned int data = 0; // first bit is always a 1
	int leftover = LEFTOVER_SPACE;
	
	
	// eat through the data a pair of edges at a time
	// so ON time followed by OFF time
	for(int i=0; i<edge_count; i+=2) {
	
		if(1) { // MARK
			if(leftover == LEFTOVER_SPACE) {
				//_X this is a 1
				data<<=1; data|=1;
				// we're up to date
				leftover = LEFTOVER_NONE;
			}
			else {
				// will deal with it when we see next space
				leftover = LEFTOVER_MARK;
			}
		}
		else if(1) { // DOUBLE MARK
			if(leftover == LEFTOVER_SPACE) {
				//_X(X) this is a 1
				data<<=1; data|=1;
				// half a mark to go
				leftover = LEFTOVER_MARK;
			}
			else {
				// not valid!
				return 0;
			}
		}
		else {
			// pulse width out of whack
		}


		if(1) { // SPACE
			if(leftover == LEFTOVER_MARK) {
				// X_ this is a 0
				data<<=1; 
				// we're up to date
				leftover = LEFTOVER_NONE;
			}
			else {
				// will deal with it when we see next mark
				leftover = LEFTOVER_SPACE;
			}
		}
		else if(1) { // DOUBLE SPACE
			if(leftover == LEFTOVER_MARK) {
				// X_(_) this is a 0
				data<<=1; 
				// half a mark to go
				leftover = LEFTOVER_SPACE;
			}
			else {
				// not valid!
				return 0;
			}
		}
	}
	
	// the receiving routine will time out after the last pulse, so
	// if we have a leftover mark we need to add another zero bit
	// to the end of the data
	if(leftover == LEFTOVER_MARK) {
		data<<=1;
	}
	return data;	
}


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

	// weak pullups on the switches
	wpuc 	= M_WPUC; 
	option_reg.7 = 0;
	
	option_reg.6 = 0; 	// INT pin interrupt on FALLING edge (active low output)
	intcon.4 = 1;		// enable INT pint interrupt
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	g_cv_dac_pending = 0;
	// initialise the various modules
	uart_init();
	//i2c_init();	
	timer_init();	

	capture_state = ST_WAITING;
	/*
	while(1) {	
		uart_send_string("Hello");
		delay_ms(100);
	}
	*/
	
	while(1) {	
		if(ST_PENDING == capture_state) {
			for(int i=0; i<edge_count; i+=2) {
				uart_send_string("MARK ");
				uart_send_number(edge[i]);
				uart_send_string(" SPACE ");
				uart_send_number(edge[i+1]);
				uart_send_string("\r\n");
			}
			delay_s(1);
			capture_state = ST_WAITING;
			intcon.4 = 1;
		}
	}

	while(1) {
		read_switches();
		blink(switch_pos[0]);
		blink(switch_pos[1]);
		blink(switch_pos[2]);
		delay_ms(200);
		delay_ms(200);
		delay_ms(200);
	}

	dac_init();
	//dac_set(2000);
	int i=0;
	while(1) {
		dac_set(i);
		i+=4; if(i>4095) i=0;
		P_GATE_OUT = !P_TRIG_IN;
		//delay_ms(1);
		//P_GATE_OUT = 1;
		//delay_ms(1);
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



