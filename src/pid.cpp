#include "pid.hpp"

class PidImpl
{
public:
	PidImpl(float _min, float _max, // Response boundary.
	        float _kp, float _ki, float _kd);
	float calculate(float dt, float target, float curr_val);

private:
	float min, max;
	float kp, ki, kd;

	float prev_err_val;
	float integral_val;
};

Pid::Pid(float min, float max,
         float kp, float ki, float kd) {
	set_target(0.0);
	pid_impl = new PidImpl(min, max, kp, kd, ki);
}

void Pid::set_target(float _target_val) {
	target_val = _target_val;
}

float Pid::calculate(float dt, float curr_val) {
	return pid_impl->calculate(dt, target_val, curr_val);
}

Pid::~Pid() {
	delete pid_impl;
}

PidImpl::PidImpl(float _min, float _max,
                 float _kp, float _ki, float _kd)
	: min(_min), max(_max),
	kp(_kp), ki(_ki), kd(_kd),
	prev_err_val(0), integral_val(0) {
}

float PidImpl::calculate(float dt, float target_val, float curr_val) {
	// Calculate error.
	float curr_err_val = target_val - curr_val;

	// Proportional.
	float p_out = kp * curr_err_val;

	// Integral.
	integral_val += curr_err_val * dt;
	float i_out = ki * integral_val;

	// Derivative.
	float derivative = (curr_err_val - prev_err_val) / dt;
	float d_out = kd * derivative;

	// Calculate total output.
	float output = p_out + i_out + d_out;

	// Restrict to max/min.
	if(output > max)
		output = max;
	else if(output < min)
		output = min;

	// Save error to previous error.
	prev_err_val = curr_err_val;

	return output;
}
