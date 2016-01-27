#include "states.hpp"

class Idle : public State
{
public:
	Idle();
	State * change_state(void *condition) {
		if(unread_cmd) {
			switch(new_cmd) {
			case 'a':
				reinterpret_cast<AutoMode *>(this)->AutoMode::AutoMode();
			case 'm':
				reinterpret_cast<ManualMode *>(this)->ManualMode::ManualMode();
			}
			unread_cmd = false;
		}
		return this;
	}
};

class AutoMode : public State
{
public:
	AutoMode();
	State * change_state(void *condition) {
		// Make sure no more message.
		if(unread_cmd && (new_cmd == 'a'))
			unread_cmd = false;
		else
			reinterpret_cast<AutoModeEngaged *>(this)->AutoModeEngaged::AutoModeEngaged();
		return this;
	}
};

class AutoModeEngaged : public State
{
public:
	AutoModeEngaged();
	State * change_state(void *condition) {
		if(unread_cmd && (new_cmd == 'q')) {
			unread_cmd = false;
			reinterpret_cast<Idle *>(this)->Idle::Idle();
		} else
			run();
		return this;
	}

private:
	void run() {
		/* Detail about the PID movement here. */
	}
};

class ManualMode : public State
{
public:
	AutoMode();
	State * change_state(void *condition) {
		// Make sure no more message.
		if(unread_cmd && (new_cmd == 'm'))
			unread_cmd = false;
		else
			reinterpret_cast<AutoModeEngaged *>(this)->AutoModeEngaged::AutoModeEngaged();
		return this;
	}
};
