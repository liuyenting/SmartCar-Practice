#include "state_machine.hpp"

void FiniteStateMachine::run() {
	// The state machine will start from the idle state.
	while(true) {
		next_state();
		System::DelayMs(loop_delay);
	}
}

/*
 * Press 'a' to enter auto mode, and 'm' to enter manual mode.
 * After release the key, press 'q' to return back to the idle mode.
 *
 * In manual mode, use 'w', 's' to move forward and backward...
 * ... use 'a', 'd' to turn left and right.
 */
