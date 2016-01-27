#include "state_machine.hpp"

void FiniteStateMachine::run() {
	// The state machine will start from the idle state.
	while(true)
		next_state(NULL);
}
