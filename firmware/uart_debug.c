#include <system.h>

#include "irl.h"

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

void uart_send_hex(unsigned long ch) {
	const char lat[] = "0123456789abcdef";
	uart_send(lat[ch/0x100000]);
	ch &= 0x0FFFFF;
	uart_send(lat[ch/0x10000]);
	ch &= 0x0FFFF;
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
	uart_send_number(g_lead_mark_count);
	uart_send_string(" SPACE ");			
	uart_send_number(g_lead_space_count);
	uart_send_string(" (leader)\r\n");
	for(int i=0; i<g_edge_count; i+=2) {
		uart_send_string("MARK ");
		uart_send_number(g_edge[i]);
		uart_send_string(" SPACE ");
		uart_send_number(g_edge[i+1]);
		uart_send_string("\r\n");
	}
}

void uart_dump_msg(RC_MESSAGE *msg) {
	switch(msg->format) {
		case IR_UNKNOWN: uart_send_string("UNK "); break;
		case IR_NEC: uart_send_string("NEC "); break;
		case IR_SONY: uart_send_string("SONY "); break;
		case IR_RC6: uart_send_string("RC6 "); break;
	}
	uart_send_binary(msg->raw);
	uart_send_string(" ");
	uart_send_hex(msg->raw);
	uart_send_string(" addr:");
	uart_send_hex(msg->address);
	uart_send_string(" cmd:");
	uart_send_hex(msg->command);
	uart_send_string("\r\n");			
}