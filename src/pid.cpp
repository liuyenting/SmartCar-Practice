#include "pid.hpp"

class PidImpl
{
public:
	PidImpl(double _dt,    // Interval between each calculation.
	        double _min, double _max, // Response boundary.
	        double _kp, double _ki, double _kd);
	double calculate(double target, double curr_val);

private:
	double dt;
	double min, max;
	double kp, ki, kd;

	double prev_err_val;
	double integral_val;
};

Pid::Pid(double dt,
         double min, double max,
         double kp, double ki, double kd) {
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

PidImpl::PidImpl(double _dt,
                 double _min, double _max,
                 double _kp, double _ki, double _kd)
	: dt(_dt),
	min(_min), max(_max),
	kp(_kp), ki(_ki), kd(_kd),
	prev_err_val(0), integral_val(0) {
}

double PidImpl::calculate(double target_val, double curr_val) {
	// Calculate error.
	double curr_err_val = target_val - curr_val;

	// Proportional.
	double p_out = kp * curr_err_val;

	// Integral.
	integral_val += curr_err_val * dt;
	double i_out = ki * integral_val;

	// Derivative.
	double derivative = (curr_err_val - prev_err_val) / dt;
	double d_out = kd * derivative;

	// Calculate total output.
	double output = p_out + i_out + d_out;

	// Restrict to max/min.
	if(output > max)
		output = max;
	else if(output < min)
		output = min;

	// Save error to previous error.
	prev_err_val = curr_err_val;

	return output;
}
