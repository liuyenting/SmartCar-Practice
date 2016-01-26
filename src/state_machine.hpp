#ifndef __STATE_MACHINE_HPP__
#define __STATE_MACHINE_HPP__

class State
{
public:
	State(int new_state)
		: this_state(new_state) {
	}
	virtual State * change_state(int next_state) = 0;

private:
	int this_state;
};

#endif
