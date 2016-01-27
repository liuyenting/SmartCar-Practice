#include "pid.hpp"

class PidImpl
{
public:
	PidImpl(double dt,    // Interval between each calculation.
	        double min, double max, // Response boundary.
	        double kp, double ki, double kd);
	double calculate(double target, double curr_val);

private:
	double _dt;
	double _min, _max;
	double _kp, _kd, _ki;

	double _pre_error;
	double _integral;
};

Pid::Pid(double dt, double min, double max, double kp, double ki, double kd) {
	pid_impl = new PidImpl(dt, min, max, kp, kd, ki);
}

void Pid::set_target(double _target_val) {
	target_val = _target_val;
}

double Pid::calculate(double curr_val) {
	return pid_impl->calculate(target_val, curr_val);
}

Pid::~Pid() {
	delete pid_impl;
}

PidImpl::PidImpl(double dt, double min, double max, double kp, double ki, double kd)
	: _dt(dt), _min(min), _max(max), _kp(kp), _ki(ki), _kd(kd),
	_pre_error(0), _integral(0) {
}

double PidImpl::calculate(double target_val, double curr_val) {

	// Calculate error
	double error = target_val - curr_val;

	// Proportional term
	double Pout = _kp * error;

	// Integral term
	_integral += error * _dt;
	double Iout = _ki * _integral;

	// Derivative term
	double derivative = (error - _pre_error) / _dt;
	double Dout = _kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	// Restrict to max/min
	if(output > _max)
		output = _max;
	else if(output < _min)
		output = _min;

	// Save error to previous error
	_pre_error = error;

	return output;
}
