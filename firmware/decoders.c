#include "irl.h"

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
	if(	g_lead_mark_count  < LEAD_MARK_MIN || 
		g_lead_mark_count  > LEAD_MARK_MAX ||		
		g_lead_space_count < LEAD_SPACE_MIN || 
		g_lead_space_count > LEAD_SPACE_MAX ) {
		return 0;
	}
	for(int i=0; i<g_edge_count-1; i+=2) {
		if( g_edge[i]<DATA_MARK_MIN || 
			g_edge[i]>DATA_MARK_MAX ) {
			// pulse length out of bounds
			return 0;
		}
		data<<=1;
		if(g_edge[i+1]>=DATA_LONG_SPACE) {
			// mark (digit 1) defined by long LOW time
			data |= 1;
		}
	}
	msg->format = IR_NEC;
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
	if(	g_lead_mark_count  < _LEAD_MARK_MIN || 
		g_lead_mark_count  > _LEAD_MARK_MAX ||		
		g_lead_space_count < _DATA_SPACE_MIN || 
		g_lead_space_count > _DATA_SPACE_MAX ) {
		return 0;
	}
	int bits = 0;
	for(int i=0; i<g_edge_count; i+=2) {
		if( g_edge[i] < _DATA_MARK_MIN || 
			g_edge[i] > _DATA_MARK_MAX ) {
			// mark length out of bounds
			return 0;
		}
		if( ( g_edge[i+1] < _DATA_SPACE_MIN ||
			  g_edge[i+1] > _DATA_SPACE_MAX ) &&
				i<g_edge_count-2) {
			// space length out of bounds (we can ignore the
			// very last space)
			return 0;
		}
		++bits;
		data<<=1;
		if(g_edge[i] >= _DATA_LONG_MARK) {
			data |= 1;
		}
	}
	if(bits<12) {
		return 0;
	}
	msg->format = IR_SONY;
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
// Time tick base in the edge[] array is 32us (8MHz / 256)
//
// RC6 protocol:
// Leader pulse 2666us MARK, 889us SPACE (83, 27 ticks)
// Normal bit	444us  MARK, 444us SPACE (13, 13 ticks)
// trailer bit	889us  MARK, 889us SPACE (27, 27 ticks)
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

int parse_RC6( RC_MESSAGE *msg ) 
{
	enum {
		RC6_LEAD_MARK_MIN = 75,
		RC6_LEAD_MARK_MAX = 90,
		RC6_LEAD_SPACE_MIN = 23,
		RC6_LEAD_SPACE_MAX = 32,
		RC6_DBL_WIDTH_TICKS = 23, 	// threshold between t and 2t pulse in 16us ticks
		RC6_TRB_WIDTH_TICKS = 35, 	// threshold between 2t and 3t pulse in 16us ticks
		RC6_MAX_WIDTH_TICKS = 50	// maximum pulse length 
	};

	int input_level = 1;		
	unsigned long data = 0;
	int t = 0;	
	if(	g_lead_mark_count  < RC6_LEAD_MARK_MIN || 
		g_lead_mark_count  > RC6_LEAD_MARK_MAX ||		
		g_lead_space_count < RC6_LEAD_SPACE_MIN || 
		g_lead_space_count > RC6_LEAD_SPACE_MAX ) {
		return 0;
	}
	
	for(int i=2; i<g_edge_count; ++i) {			
		if(g_edge[i] > RC6_MAX_WIDTH_TICKS && (i<g_edge_count-1)) {
			return 0;
		}
		if(g_edge[i] > RC6_TRB_WIDTH_TICKS) {
			if(t++ & 1) {
				data<<=1;
				data|=input_level;
			}
		}
		if(g_edge[i] > RC6_DBL_WIDTH_TICKS) {
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
	// break up the message
	msg->format = IR_RC6;
	msg->raw = data;
	msg->address = ((data>>8) & 0xFF);
	msg->command = (data & 0xFF);		
	return 1;
}
