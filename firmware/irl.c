
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
	unsigned int full_message;
	unsigned int address;
	unsigned int command;
} RC_MESSAGE;

enum {
	ST_WAITING,
	ST_CAPTURING,
	ST_PENDING
};

#define MAX_EDGES 	100
volatile byte edge[MAX_EDGES];
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
	// Timer 1 overflows when we time out at end of a message
	if(pir1.0)
	{
		t1con.0 = 0;	// stop the timer
		capture_state = ST_PENDING;
		intcon.4 = 0;		// disable the INT pin
		option_reg.6 = 0; 	// next INT pin interrupt will be falling edge
		pir1.0 = 0;				
	}
	
	// external interrupt condition
	if(intcon.1) {
		// capture the timer value
		byte d = tmr1h;
		
		// reset the timer
		tmr1l = 0;
		tmr1h = 0;
		switch(capture_state) {
			case ST_WAITING:
				edge_count = 0;
				capture_state = ST_CAPTURING;
				t1con.0 = 1;	// start the timer
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

void uart_send_number(byte ch) {
	uart_send('0' + ch/100);
	ch %= 100;
	uart_send('0' + ch/10);
	ch %= 10;
	uart_send('0' + ch);
}

void uart_send_binary(unsigned int data) {
	unsigned int mask = 1<<32;
	uart_send((data&mask)? '1':'0' + ch/100);
	while(mask) {
		mask>>=1;
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


////////////////////////////////////////////////
// 0         1         2         3         4         
// 012345678901234567890123456789012345678901234
// HHHLLHL ........................................ start
//        010101 .................................. field
//              XXXX .............................. toggle
//                  XXXXXXXXXXXXXX ................ address
//                                XXXXXXXXXXXXXX .. command

int parse_RC6(RC_MESSAGE *msg) {
	const int DBL_WIDTH_TICKS = 45;
	const int TRB_WIDTH_TICKS = 70;
	const int MAX_WIDTH_TICKS = 180;
	int input_level = 1;		
	unsigned int full_message = 0;
	int t = 0;
	for(int i=0; i<edge_count; ++i) {			
		if(edge[i] > MAX_WIDTH_TICKS) {
			return 0;
		}
		if(edge[i] > TRB_WIDTH_TICKS) {
			if(t++ & 1) {
				full_message<<=1;
				full_message|=input_level;
			}
		}
		if(edge[i] > DBL_WIDTH_TICKS) {
			if(t++ & 1) {
				full_message<<=1;
				full_message|=input_level;
			}
		}
		if(t++ & 1) {
			full_message<<=1;
			full_message|=input_level;
		}
		input_level	= !input_level;
	}
	if((full_message & 0xE00000) != 0xa00000) {
		return 0;
	}
	msg->full_message = full_message;
	msg->address = ((full_message>>8) & 0xFF);
	msg->command = (full_message & 0xFF);
	return 1;
}


////////////////////////////////////////////////
void xxparse_RC6() {
	const int DBL_WIDTH_TICKS = 45;
	const int TRB_WIDTH_TICKS = 70;
	const int MAX_WIDTH_TICKS = 180;
	int input_level = 1;
	int half_bit_remainder = 0;
		
	char sample[100];  // has to be signed char or compiler messes up
	byte samp = 0;
	for(int i=0; i<edge_count; ++i) {			
		if(edge[i] > MAX_WIDTH_TICKS) {
			//return 0;
		}
		if(edge[i] > TRB_WIDTH_TICKS) {
			sample[samp++] = input_level;
		}
		if(edge[i] > DBL_WIDTH_TICKS) {
			sample[samp++] = input_level;
		}
		sample[samp++] = input_level;
		input_level	= !input_level;
	}

	uart_send_number(samp);								
	uart_send_string("samples \r\n");								

	// 0         1         2         3         4         
	// 012345678901234567890123456789012345678901234
	// HHHLLHL ........................................ start
	//        010101 .................................. field
	//              XXXX .............................. toggle
	//                  XXXXXXXXXXXXXX ................ address
	//                                XXXXXXXXXXXXXX .. command
	
	
	for(int i=0; i<samp; ++i) {			
		uart_send_string(sample[i]? "X":"_");								
	}
	uart_send_string("\r\n");							
	for(int i=0; i<samp; ++i) {			
		if(i>5 && !!(i&1)) {
			uart_send_string(sample[i]? "1":"0");								
		}
		else {
			uart_send_string(" ");								
		}
	}
}


////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
////////////////////////////////////////////////
#if 0
////////////////////////////////////////////////
void parse_RC6() {
	const int DBL_WIDTH_TICKS = 45;
	const int LEADER_MARK_MIN_TICKS = 150;
	const int LEADER_MARK_MAX_TICKS = 180;
	const int LEADER_SPACE_MIN_TICKS = 40;
	const int LEADER_SPACE_MAX_TICKS = 60;
	int input_level = 1;
	int half_bit_remainder = 0;
	
	
	if( edge[0] < LEADER_MARK_MIN_TICKS ||
		edge[0] > LEADER_MARK_MAX_TICKS ||
		edge[1] < LEADER_SPACE_MIN_TICKS ||
		edge[1] > LEADER_SPACE_MAX_TICKS) {
	//	return 0;
	}

	
	for(int i=2; i<edge_count; ++i) {			
		if(edge[i] >= DBL_WIDTH_TICKS) {
			if(half_bit_remainder) { // half a LOW bit from previous period
				uart_send_string(input_level? "0":"1");				
			}
			else {
				// must be a long half-bit (double HIGH period not valid here)
				half_bit_remainder = 1;
			}
		}
		else { 
			if(half_bit_remainder) {
				// mark a rising edge and clear half bit remainder
				uart_send_string(input_level? "0":"1");				
				half_bit_remainder = 0;
			}
			else {
				half_bit_remainder = 1;
			}
		}
		input_level	= !input_level;
	}
	if(half_bit_remainder) {
		uart_send_string(input_level? "0.":"1.");				
	}
}
void parse_RC5() {
	const int DBL_WIDTH_TICKS = 45;
	char qq[100];
	int q=0;
	int input_level = 1;
	int half_bit_remainder = 0;
	

	byte t[MAX_EDGES];	
	int min = 0;
	for(int i=0; i<edge_count; i+=2) {
		if(!min || edge[i]<min) {
			min = edge[i];
		}
	}
	if(!min) {
		return;
	}
	
	
	
	
	for(int i=0; i<edge_count; i+=2) {
		t[i] = (min/2+edge[i])/min;
		t[i+1] = (min/2+edge[i+1])/min;
		uart_send_string("mark ");						
		uart_send_number(t[i]);
		uart_send_string("t space ");						
		uart_send_number(t[i+1]);
		uart_send_string("t\r\n");						
	}
	
	for(int i=0; i<edge_count; ++i) {
	
		
		if(edge[i] >= DBL_WIDTH_TICKS) {
			if(input_level) { // input is HIGH for this period
				if(half_bit_remainder) { // half a LOW bit from previous period
					// mark a rising edge and leave half bit remainder
					qq[q++] = '1';
					qq[q++] = ' ';
					uart_send_string("XX");				
				}
				else {
					// must be a long half-bit (double HIGH period not valid here)
					half_bit_remainder = 1;
					uart_send_string("XX");				
					qq[q++] = ' ';
					qq[q++] = ' ';
				}
				
			}
			else { // input is LOW for this period
				if(half_bit_remainder) { // half a HIGH bit from previous period
					// mark a falling edge and leave half bit remainder
					qq[q++] = '0';
					qq[q++] = ' ';
					uart_send_string("__");				
				}
				else {
					// must be a long half-bit (double LOW period not valid here)
					half_bit_remainder = 1;
					uart_send_string("__");				
					qq[q++] = ' ';
					qq[q++] = ' ';
				}
			}		
		}
		else if(input_level) { // input is HIGH
			uart_send_string("X");				
			if(half_bit_remainder) {
				// mark a rising edge and clear half bit remainder
				qq[q++] = '1';
				half_bit_remainder = 0;
			}
			else {
				qq[q++] = ' ';
				half_bit_remainder = 1;
			}
		}
		else { // input is LOW
			uart_send_string("_");				
			if(half_bit_remainder) {
				// mark a falling edge and clear half bit remainder
				qq[q++] = '0';
				half_bit_remainder = 0;
			}
			else {
				qq[q++] = ' ';
				half_bit_remainder = 1;
			}
		}
		input_level	= !input_level;
	}
	if(half_bit_remainder) {
		if(input_level) {
			qq[q++] = '1';	
			uart_send_string("X.");				
		}
		else {
			qq[q++] = '0';	
			uart_send_string("_.");				
		}
	}
	qq[q++] = 0;	
	uart_send_string("\r\n");				
	uart_send_string(qq);				
	
}
#endif 


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
	t1con.5 = 0;	// Prescaler = 1:1
	t1con.4 = 0;
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
#if 0		
			uart_send_string("=======\r\n");			
			for(int i=0; i<edge_count; i+=2) {
				uart_send_string("MARK ");
				uart_send_number(edge[i]);
				uart_send_string(" SPACE ");
				uart_send_number(edge[i+1]);
				uart_send_string("\r\n");
			}
#endif			
			parse_RC6();
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



