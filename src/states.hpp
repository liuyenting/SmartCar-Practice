#ifndef __STATES_HPP__
#define __STATES_HPP__

extern uint8_t new_cmd;
extern bool unread_cmd;

class State
{
public:
	State();
	virtual State * change_state() = 0;
};

#endif
