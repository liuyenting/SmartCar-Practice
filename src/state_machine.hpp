#ifndef __STATE_MACHINE__
#define __STATE_MACHINE__

// Primitive for the states.
class State
{
public:
	State(int _state_id)
		: state_id(_state_id) {
	};

	// The function to move on to next state.
	virtual State* change_state(int next_state) = 0;

private:
	int state_id;
};

class FiniteStateMachine
{
public:
	// Enter the "idle" state at first.
	FiniteStateMachine()
		: curr_state(new IdleState()) {
	};

	void next_state(int next_state_id) {
		curr_state = curr_state->change_state(next_state_id);
	}

	void run();

private:
	State* curr_state;
};

#endif
