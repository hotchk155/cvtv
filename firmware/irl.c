
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
//#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _BORV_19 & _LVP_OFF
//#pragma CLOCK_FREQ 32000000

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

typedef struct {
	unsigned long raw;
	unsigned int address;
	unsigned int command;
} RC_MESSAGE;

enum {
	ST_WAITING,
	ST_LEAD_MARK,
	ST_LEAD_SPACE,
	ST_CAPTURING,
	ST_PENDING
};

#define MAX_EDGES 	100
volatile byte edge[MAX_EDGES];
volatile int edge_count = 0;
volatile byte capture_state = 0;
volatile int lead_mark_count = 0;
volatile int lead_space_count = 0;

//
// TYPES
//

// we want a timer 1 tick every 64us


// Timer related stuff
#define TIMER_0_INIT_SCALAR		5		// Timer 0 initialiser to overlow at 1ms intervals
volatile byte ms_tick = 0;				// once per millisecond tick flag used to synchronise stuff



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
	// Timer 1 overflows 
	if(pir1.0)
	{
		switch(capture_state) {	
		case ST_LEAD_MARK:				
			// timer overflow during the mark period of message leader
			// means we can add a count of 256 to the period
			lead_mark_count += 256;
			break;
		case ST_LEAD_SPACE:				
			// timer overflow during the space period of message leader
			// means we can add a count of 256 to the period
			lead_space_count += 256;
			break;
		default:
			// timer overflow during capture means that we've reached the
			// end of the message
			t1con.0 = 0;				// stop the timer
			intcon.4 = 0;				// disable the INT pin
			option_reg.6 = 0; 			// next INT pin interrupt will be falling edge
			capture_state = ST_PENDING;	// ready for the application to pick up 
			break;
		}
		pir1.0 = 0;				
	}
	
	// Edge detected from the infra-red receiver
	if(intcon.1) {
		// capture the timer value and reset the timer
		byte d = tmr1h;
		tmr1l = 0;
		tmr1h = 0;
		switch(capture_state) {
			// the first edge of a new message
			case ST_WAITING:
				lead_mark_count = 0;
				lead_space_count = 0;
				edge_count = 0;
				capture_state = ST_LEAD_MARK;
				t1con.0 = 1;	// start the timer
				break;
			// edge at the end of the mark period of the message leader
			case ST_LEAD_MARK:				
				lead_mark_count += d;
				capture_state = ST_LEAD_SPACE;
				break;
			// edge at the end of the space period of the message leader
			case ST_LEAD_SPACE:				
				lead_space_count += d;
				capture_state = ST_CAPTURING;
				break;
			// edge during capturing message content
			case ST_CAPTURING:				
				if(edge_count < MAX_EDGES-1) {
					edge[edge_count++] = d;
				}
				break;
		}
		// toggle edge bit - next edge will be the opposite direction
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
	baudcon.3 = 0;	// BRG16	enable 16 bit brg
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
	//spbrg = 51;		// brg low byte 
	spbrg = 25;		// brg low byte 
	
}

void uart_send(byte ch) 
{
	txreg = ch;
	while(!txsta.1);
}

void uart_send_string(byte *ch) 
{
	while(*ch) {
		uart_send(*ch);
		++ch;
	}
}

void uart_send_number(int ch) {
	uart_send('0' + ch/10000);
	ch %= 10000;
	uart_send('0' + ch/1000);
	ch %= 1000;
	uart_send('0' + ch/100);
	ch %= 100;
	uart_send('0' + ch/10);
	ch %= 10;
	uart_send('0' + ch);
}

void uart_send_hex(unsigned int ch) {
	const char lat[] = "0123456789abcdef";
	uart_send(lat[ch/0x1000]);
	ch &= 0x0FFF;
	uart_send(lat[ch/0x100]);
	ch &= 0x00FF;
	uart_send(lat[ch/0x10]);
	ch &= 0x000F;
	uart_send(lat[ch]);
}

void uart_send_binary(unsigned long data) {
	unsigned long  mask = 0x80000000;
	for(int i=0; i<32; ++i) {
		if(!!(data&mask)) {
			uart_send('1');
		}
		else {
			uart_send('0');
		}
		mask>>=1;
	}
}

void uart_dump_timings() {
	uart_send_string("=======\r\n");			
	uart_send_string("MARK ");			
	uart_send_number(lead_mark_count);
	uart_send_string(" SPACE ");			
	uart_send_number(lead_space_count);
	uart_send_string(" (leader)\r\n");
	for(int i=0; i<edge_count; i+=2) {
		uart_send_string("MARK ");
		uart_send_number(edge[i]);
		uart_send_string(" SPACE ");
		uart_send_number(edge[i+1]);
		uart_send_string("\r\n");
	}
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

//////////////////////////////////////////////////////////////////////////
// PARSE NEC
//
// Time tick base in the edge[] array is 16us (16MHz / 256)
//
// NEC protocol:
// Leader pulse 9000us  MARK, 4500us   SPACE (562, 281 ticks)  
// zero bit		562.5us MARK, 562.5us  SPACE (35, 35 ticks)
// one bit		562.5us MARK, 1687.5us SPACE (35, 105 ticks)
// Timeout is 4096us (256) after last edge
//
// SAMSUNG: as above with
// Leader pulse 5000us  MARK, 5000us   SPACE (312, 312 ticks)  
//
// data is 32 bits long (16 bit address, 16 bit command)
//
int parse_NEC(RC_MESSAGE *msg) {
	enum {
		LEAD_MARK_MIN = 250/2,
		LEAD_MARK_MAX = 620/2,
		LEAD_SPACE_MIN = 250/2,
		LEAD_SPACE_MAX = 310/2,
		DATA_MARK_MIN = 20/2,
		DATA_MARK_MAX = 50/2,
		DATA_LONG_SPACE = 60/2
	};
	unsigned long data = 0;
	
	// checks on the message leader
	if(	lead_mark_count  < LEAD_MARK_MIN || 
		lead_mark_count  > LEAD_MARK_MAX ||		
		lead_space_count < LEAD_SPACE_MIN || 
		lead_space_count > LEAD_SPACE_MAX ) {
		return 0;
	}
	--edge_count; // remove the last pulse terminator
	for(int i=0; i<edge_count; i+=2) {
		if( edge[i]<DATA_MARK_MIN || 
			edge[i]>DATA_MARK_MAX ) {
			// pulse length out of bounds
			return 0;
		}
		data<<=1;
		if(edge[i+1]>=DATA_LONG_SPACE) {
			// mark (digit 1) defined by long LOW time
			data |= 1;
		}
	}
	msg->raw = data;
	msg->address = ((data>>16) & 0x00FF);
	msg->command = ((data) & 0x00FF);
	return 1;
}

//////////////////////////////////////////////////////////////////////////
// PARSE SONY
//
// Time tick base in the edge[] array is 32us (8MHz / 256)
//
// Leader pulse 2400us  MARK, 600us   SPACE (75, 18 ticks)  
// zero bit		600us MARK, 600us  SPACE (18, 18 ticks)
// one bit		600us MARK, 1200us SPACE (18, 37 ticks)
// Timeout is 8096us (256) after last edge
//
// data is 7 bit command + 5, 8 or 13 address bits
//
int parse_SONY(RC_MESSAGE *msg) {
	enum {
		_LEAD_MARK_MIN  = 65,
		_LEAD_MARK_MAX  = 85,
		_DATA_SPACE_MIN = 12,
		_DATA_SPACE_MAX = 24,
		_DATA_MARK_MIN  = 12,
		_DATA_MARK_MAX  = 45,
		_DATA_LONG_MARK = 30
	};
	unsigned long data = 0;
	// checks on the message leader
	if(	lead_mark_count  < _LEAD_MARK_MIN || 
		lead_mark_count  > _LEAD_MARK_MAX ||		
		lead_space_count < _DATA_SPACE_MIN || 
		lead_space_count > _DATA_SPACE_MAX ) {
		return 0;
	}
	int bits = 0;
	for(int i=0; i<edge_count; i+=2) {
		if( edge[i] < _DATA_MARK_MIN || 
			edge[i] > _DATA_MARK_MAX ) {
			// mark length out of bounds
			return 0;
		}
		if( edge[i+1] < _DATA_SPACE_MIN ) {
			//edge[i+1] > _DATA_SPACE_MAX ) {
			// space length out of bounds
			return 0;
		}
		++bits;
		data<<=1;
		if(edge[i] >= _DATA_LONG_MARK) {
			data |= 1;
		}
	}
	if(bits<12) {
		return 0;
	}
	msg->raw = data;
	int address_len = (bits - 7);
	msg->address = data & ~(((unsigned int)0x7F)<<address_len);
	msg->command = ((data>>address_len) & 0x7F);
	return 1;
}
/*
	Timer 1 speed is 16MHz
	one tick of timer1 high byte is is 16us	
	
	leader pulse
	2666us MARK 889us SPACE (166, 55)
	
	normal bit
	444us MARK 444us SPACE (27)
	
	trailer bits
	889us MARK 889us SPACE (55)
	
	timeout is 4096us (256)
	
*/


//////////////////////////////////////////////////////////////////////////
// PARSE RC6
//
// Time tick base in the edge[] array is 16us (16MHz / 256)
//
// RC6 protocol:
// Leader pulse 2666us MARK, 889us SPACE (166, 55 ticks)
// Normal bit	444us  MARK, 444us SPACE (27, 27 ticks)
// trailer bit	889us  MARK, 889us SPACE (55, 55 ticks)
// Timeout is 4096us (256) after last edge
//
// Logically, the process is to expand the edge timings data
// into mark/space periods of t, 2t or 3t where t=~444us.
// (The leader is compressed to total of 5t)
//
// Encoded bit values can now be read from slots at t, 3t, 5t etc..
//
// The implementation combines the expansion and decoding into a 
// single pass
//
// 0         1         2         3         4         
// 012345678901234567890123456789012345678901234
// 11100........................................... AGC leader (fixed)
//      10......................................... start bit
//        XXXXXX .................................. field (usually 010101)
//              XXXX .............................. toggle (1100/0011)
//                  XXXXXXXXXXXXXX ................ address
//                                XXXXXXXXXXXXXX .. command
//
//
int parse_RC6(RC_MESSAGE *msg) {
	const int DBL_WIDTH_TICKS = 45; 	// threshold between t and 2t pulse in 16us ticks
	const int TRB_WIDTH_TICKS = 70; 	// threshold between 2t and 3t pulse in 16us ticks
	const int MAX_WIDTH_TICKS = 180;	// maximum pulse length (6t for AGC leader) in 16us ticks
	int input_level = 1;		
	unsigned long data = 0;
	int t = 0;
	for(int i=0; i<edge_count; ++i) {			
		if(edge[i] > MAX_WIDTH_TICKS) {
			return 0;
		}
		if(edge[i] > TRB_WIDTH_TICKS) {
			if(t++ & 1) {
				data<<=1;
				data|=input_level;
			}
		}
		if(edge[i] > DBL_WIDTH_TICKS) {
			if(t++ & 1) {
				data<<=1;
				data|=input_level;
			}
		}
		if(t++ & 1) {
			data<<=1;
			data|=input_level;
		}
		input_level	= !input_level;
	}
	// check for AGC leader and start bit
	if((data & 0xE00000) != 0xa00000) {
		//return 0;
	}
	// break up the message
	msg->raw = data;
	msg->address = ((data>>8) & 0xFF);
	msg->command = (data & 0xFF);
		
	return 1;
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 		
	// osc control / 16MHz / internal
	osccon = 0b01111010;
	//osccon = 0b11110000; //32MHz

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
	
	t1con.7 = 0;	// Fosc source for timer 1 
	t1con.6 = 1;	//
	t1con.5 = 0;	// Prescaler = 1:2
	t1con.4 = 1;
	pie1.0 = 1;		// enable timer 1 interrupt

	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	g_cv_dac_pending = 0;
	// initialise the various modules
	uart_init();
	//i2c_init();	
	//timer_init();	

	

	capture_state = ST_WAITING;
	
	//while(1) {	
		//uart_send_string("Hello\r\n");
		//delay_ms(100);
	//}
	
	
	while(1) {	
		if(ST_PENDING == capture_state) {
			uart_dump_timings();
			RC_MESSAGE msg;
			msg.raw=0;
			msg.address=0;
			msg.command=0;
			parse_SONY(&msg);
			uart_send_binary(msg.raw);
			uart_send_string(" ");
			uart_send_hex(msg.raw);
			uart_send_string(" addr:");
			uart_send_hex(msg.address);
			uart_send_string(" cmd:");
			uart_send_hex(msg.command);
			uart_send_string("\r\n");			
			delay_s(1);
			capture_state = ST_WAITING;
			intcon.1 = 0; // clear INT fired status
			intcon.4 = 1; // enable the INT pin
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



