#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <cstdio>
#include <cassert>
#include <cstdint> // uint16_t
#include <cstring>
#include <array> // std::array

#include <libbase/k60/mcg.h>
#include <libsc/system.h>

namespace libbase
{
namespace k60
{
Mcg::Config Mcg::GetMcgConfig() {
	Mcg::Config config;
	config.external_oscillator_khz = 50000;
	config.core_clock_khz = 150000;
	return config;
}
}
}

#include <libsc/st7735r.h>
#include <libsc/tsl1401cl.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>
#include <libsc/lcd_typewriter.h>

#include "pid.hpp"
#include "pVarManager.h"

#define REFRESH_INTERVAL 20 // 50ms interval, 20FPS
#define AVERAGE_COUNTS  5 // Average interval, unit: sample.

#define STEERING_CENTER 1000

typedef std::array<uint16_t, libsc::Tsl1401cl::kSensorW> ccd_buffer_t;

/* Type definitions */
typedef struct struct_peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;

	libsc::FutabaS3010 *steering;
	libsc::AlternateMotor *driving;

	libsc::LcdTypewriter *typewriter;
} peripherals_t;

typedef struct struct_range_t {
	int min;
	int max;

	struct_range_t(int _min, int _max)
		: min(_min), max(_max) {
	}
} range_t;

typedef struct struct_pid_var_t {
	float kp;
	float ki;
	float kd;

	struct_pid_var_t(float _kp, float _ki, float _kd)
		: kp(_kp), ki(_ki), kd(_kd) {
	}
} pid_var_t;

/* Prototypes */
void init(peripherals_t &peripherals);
float calculate_center_pos(ccd_buffer_t &ccd_data);
void print_scan_result(peripherals_t &peripherals, ccd_buffer_t& ccd_data);

#endif
