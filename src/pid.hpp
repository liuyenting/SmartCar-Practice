#ifndef __PID_HPP__
#define __PID_HPP__

#include <cmath>

class PidImpl;
class Pid
{
public:
	Pid(double dt, // Interval between each calculation.
	    double min, double max,    // Response boundary.
	    double kp, double ki, double kd);
	~Pid();

	// Set the target value to maintain.
	void set_target(double _target_val);

	// Return the calculation result.
	double calculate(double curr_val);

private:
	PidImpl *pid_impl;
	double target_val;
};

#endif
