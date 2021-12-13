
//////////////////////////////////////////////////////////////
//
// MAIN MODULE
//
//////////////////////////////////////////////////////////////

//
// HEADER FILES
//
#include <system.h>
#include <memory.h>
#include "irl.h"

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

enum {
	ST_WAITING,
	ST_LEAD_MARK,
	ST_LEAD_SPACE,
	ST_CAPTURING,
	ST_PENDING
};


volatile byte g_capture_state = 0;
volatile byte g_edge[MAX_EDGES];
volatile int g_edge_count = 0;
volatile int g_lead_mark_count = 0;
volatile int g_lead_space_count = 0;

int g_gate_timeout = 0;
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
	// timer 0 rollover ISR. Maintains the count of 
	// "system ticks" that we use for key debounce etc
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		ms_tick = 1;
		intcon.2 = 0;
	}
	
	/////////////////////////////////////////////////////
	// Timer 1 overflows 
	if(pir1.0)
	{
		switch(g_capture_state) {	
		case ST_LEAD_MARK:				
			// timer overflow during the mark period of message leader
			// means we can add a count of 256 to the period
			g_lead_mark_count += 256;
			break;
		case ST_LEAD_SPACE:				
			// timer overflow during the space period of message leader
			// means we can add a count of 256 to the period
			g_lead_space_count += 256;
			break;
		default:
			// timer overflow during capture means that we've reached the
			// end of the message
			t1con.0 = 0;				// stop the timer
			intcon.4 = 0;				// disable the INT pin
			option_reg.6 = 0; 			// next INT pin interrupt will be falling edge
			g_capture_state = ST_PENDING;	// ready for the application to pick up 
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
		switch(g_capture_state) {
			// the first edge of a new message
			case ST_WAITING:
				g_lead_mark_count = 0;
				g_lead_space_count = 0;
				g_edge_count = 0;
				g_capture_state = ST_LEAD_MARK;
				t1con.0 = 1;	// start the timer
				break;
			// edge at the end of the mark period of the message leader
			case ST_LEAD_MARK:				
				g_lead_mark_count += d;
				g_capture_state = ST_LEAD_SPACE;
				break;
			// edge at the end of the space period of the message leader
			case ST_LEAD_SPACE:				
				g_lead_space_count += d;
				g_capture_state = ST_CAPTURING;
				break;
			// edge during capturing message content
			case ST_CAPTURING:				
				if(g_edge_count < MAX_EDGES-1) {
					g_edge[g_edge_count++] = d;
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

#define GATE_DURATION_MS 100
void on_ir_msg(RC_MESSAGE *msg) {
	if(msg->format) { // successfully parsed?
		P_GATE_OUT = 0;
		dac_set((1000 * (unsigned long)msg->command)/256);
		P_GATE_OUT = 1;
		g_gate_timeout = 100;
	}
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

	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	//g_cv_dac_pending = 0;
	i2c_init();	
	dac_init();
	// initialise the various modules
	//uart_init();
	//timer_init();	

/*
	byte q=0;
	for(;;) {
		if(ms_tick) {
			q++;
			ms_tick  = 0;
			P_GATE_OUT = !!(q&0x80);
		}
	}
*/

	g_capture_state = ST_WAITING;
	
	for(;;)
	{	
		if(ST_PENDING == g_capture_state) {
			P_GATE_OUT = 1;
			g_gate_timeout=100;
			//uart_dump_timings();
			RC_MESSAGE msg;
			memset(&msg,0,sizeof(msg));
			if(!parse_NEC(&msg)) {
				if(!parse_SONY(&msg)) {
					parse_RC6(&msg);
				}
			}
			on_ir_msg(&msg);
			g_capture_state = ST_WAITING;
			intcon.1 = 0; // clear INT fired status
			intcon.4 = 1; // enable the INT pin
		}
	
		// once per millisecond tick event
		if(ms_tick) {
			if(g_gate_timeout) {
				if(!--g_gate_timeout) {
					P_GATE_OUT = 0;
				}
			}
			ms_tick = 0;
		}
		
		/*
		// check if there is any CV data to send out and no i2c transmit in progress
		if(!pie1.3 && g_cv_dac_pending) {
			cv_dac_prepare(); 
			i2c_send_async();
			g_cv_dac_pending = 0; 
		}
		*/				
	}
}

//
// END
//



