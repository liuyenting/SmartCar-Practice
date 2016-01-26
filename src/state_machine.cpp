#include "state_machine.hpp"

class WaitInstruction : public State
{
public:
	WaitInstruction() : state(5), ring_count(0) {
	};
	state * change_state(int choice);

private:
	int ring_count;
};
