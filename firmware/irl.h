typedef unsigned char byte;

enum {
	IR_UNKNOWN,
	IR_NEC,
	IR_SONY,
	IR_RC5,
	IR_RC6,
};
typedef struct {
	byte format;
	unsigned long raw;
	unsigned int address;
	unsigned int command;
} RC_MESSAGE;

enum {
	MAX_EDGES = 100
};
extern volatile byte g_capture_state;
extern volatile byte g_edge[MAX_EDGES];
extern volatile int g_edge_count;
extern volatile int g_lead_mark_count;
extern volatile int g_lead_space_count;

extern int parse_NEC(RC_MESSAGE *msg);
extern int parse_SONY(RC_MESSAGE *msg);
extern int parse_RC6(RC_MESSAGE *msg);



void uart_init();
void uart_send(byte ch);
void uart_send_string(byte *ch);
void uart_send_number(int ch);
void uart_send_hex(unsigned long ch);
void uart_send_binary(unsigned long data);
void uart_dump_timings();
