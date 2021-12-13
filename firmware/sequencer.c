#include "irl.h"


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

void seq_build(RC_MESSAGE *msg) {
}


void 
