#ifndef __PID_HPP__
#define __PID_HPP__

#include <cmath>

class PidImpl;
class Pid
{
public:
	Pid(float min, float max,    // Response boundary.
	    float kp, float ki, float kd);
	~Pid();

	// Set the target value to maintain.
	void set_target(float _target_val);

	// Return the calculation result.
	float calculate(float dt, float curr_val);

	void reset_time();

private:
	PidImpl *pid_impl;
	float target_val;
};

#endif
