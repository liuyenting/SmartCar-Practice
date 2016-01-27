#include "states.hpp"

class Idle : public State
{
public:
	Idle();
	State * change_state() {
		if(unread_cmd) {
			unread_cmd = false;
			switch(new_cmd) {
			case 'a':
				reinterpret_cast<AutoMode *>(this)->AutoMode::AutoMode();
			case 'm':
				reinterpret_cast<ManualMode *>(this)->ManualMode::ManualMode();
			}
		}
		return this;
	}
};

class AutoMode : public State
{
public:
	AutoMode();
	State * change_state() {
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
	State * change_state() {
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
	State * change_state() {
		// Make sure no more message.
		if(unread_cmd && (new_cmd == 'm'))
			unread_cmd = false;
		else
			reinterpret_cast<ManualModeEngaged *>(this)->ManualModeEngaged::ManualModeEngaged();
		return this;
	}
};

class ManualModeEngaged : public State
{
public:
	ManualModeEngaged();
	State * change_state() {
		if(unread_cmd) {
			unread_cmd = false;
			switch(new_cmd) {
			case 'w':
				reinterpret_cast<MoveForward *>(this)->MoveForward::MoveForward();
				break;
			case 'a':
				/* Jump to turn left state. */
				break
			case 's':
				reinterpret_cast<MoveBackward *>(this)->MoveBackward::MoveBackward();
				break;
			case 'd':
				/* Jump to turn right state. */
				break;
			case 'q':
				reinterpret_cast<Idle *>(this)->Idle::Idle();
				break;
			}
		}
		return this;
	}
};

class MoveForward : public State
{
public:
	MoveForward();
	State * change_state() {
		if(unread_cmd) {
			unread_cmd = false;
			switch(new_cmd) {
			case 's':
				reinterpret_cast<MoveBackward *>(this)->MoveBackward::MoveBackward();
				break;
			case 'w':
				break;
			default:
				reinterpret_cast<StopDriving *>(this)->StopDriving::StopDriving();
				break;
			}
		}
		return this;
	}
};

class MoveBackward : public State
{
public:
	MoveBackward();
	State * change_state() {
		if(unread_cmd) {
			unread_cmd = false;
			switch(new_cmd) {
			case 'w':
				reinterpret_cast<MoveForward *>(this)->MoveForward::MoveForward();
				break;
			case 's':
				break;
			default:
				reinterpret_cast<StopDriving *>(this)->StopDriving::StopDriving();
				break;
			}
		}
		return this;
	}
};

class StopDriving : public State
{
public:
	StopDriving();
	State * change_state() {
		run();
		reinterpret_cast<ManualModeEngaged *>(this)->ManualModeEngaged::ManualModeEngaged();
		return this;
	}

private:
	void run() {
		/* Set the motor power to 0. */
	}
};
