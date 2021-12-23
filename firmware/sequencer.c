#include "irl.h"
#include "rand.h"


enum {
	SEQ_MAX = 32
};





enum {
	PLAY_ONESHOT,		// new code plays immediately and can be retrigged. Will use internal clock if ext clock not present
	PLAY_ONESHOT_TRIG,  // new code waits for trig and can be retrigged
	PLAY_LOOP			// new code plays to the external clock
};

enum {
	SRC_SINGLE,			// a single note from each message (from command field)
	SRC_SHORT,			// a short sequence of single notes (based on address field)
	SRC_LONG			// full bar sequence derived from seeded random number generator
};

enum {
	SCALE_CHROMATIC,	// pitch scale chromatically
	SCALE_FREE,			// no pitch scaling
	SCALE_DIATONIC		// pitch scale diatonically
};


int seq_step = 0;
int seq_seed = 0;

void seq_start() {
	srand(seq_seed);
	seq_step = 8+(rand()&7);
}
void seq_on_ir(RC_MESSAGE *msg) {
	seq_seed = msg->command ^ msg->address;
	seq_start();
}
void seq_on_trig(int state) {
	seq_start();
}
void seq_on_clock(int state) {
	if(seq_step) {
		int dac = rand()&0x1FFF;
		output_cv(dac, 100);
		--seq_step;
	}
}
void seq_run() {
}



 
