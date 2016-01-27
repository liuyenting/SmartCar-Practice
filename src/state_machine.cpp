#include "state_machine.hpp"

class Idle : public State
{
public:
	Idle()
		: State(1) {
	}

	State* change_state() {

		reinterpret_cast<state_2 *>(this)->state_2::state_2();
		return this;
	}
};

/*
 * AUTO
 */
class AutoMode : public State
{

};

class AutoModeEngaged : public State
{

};

class DecideAction : public State
{
	// Line follower main logic.
};

/*
 * MANUAL
 */

class ManualMode : public State
{

};

class ManualEngaged : public State
{

};

class ManualIdle : public State
{

};

class MoveForward : public State
{

};

class MoveBackward : public State
{

};

class StopDrivingMotor : public State
{

};

class SteerLeft : public State
{

};

class SteerRight : public State
{

};

class StopSteeringServo : public State
{

};

/*
 * The body of the FSM.
 */
void FiniteStateMachine::run() {
	while(true) {
		if(unread_new_data)
			next_state();

		System::DelayMs(REFRESH_INT);
	}
}
