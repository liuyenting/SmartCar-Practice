#ifndef __STATE_MACHINE_HPP__
#define __STATE_MACHINE_HPP__

#include "states.hpp"

class FiniteStateMachine
{
public:
	FiniteStateMachine()
		: first_call(false), curr_state(new Idle()) {
	};
	void run();

private:
	const int loop_delay = 1; // Control how fast the loop is.

	bool first_call;
	State *curr_state;

	void next_state(void *condition) {
		curr_state = curr_state->change_state();
	}
};

#endif
