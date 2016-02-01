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

#include "pid.hpp"

#define REFRESH_INTERVAL 5 // 50ms interval, 20FPS
#define AVERAGE_COUNTS  5 // Average interva, unit: sample.

#define STEERING_CENTER  900
#define STEERING_RANGE  400

#define DRIVING_POWER 500

#define KD 1
#define KI 1
#define KP 1

typedef std::array<uint16_t, libsc::Tsl1401cl::kSensorW> ccd_buffer_t;

/* Type definitions */
struct peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;

	libsc::FutabaS3010 *steering;
	libsc::AlternateMotor *driving;

	libsc::LcdTypeWriter *typewriter;
};

/* Prototypes */
void init(struct peripherals_t &peripherals);
double calculate_error(ccd_buffer_t &ccd_data);
void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t& ccd_data);
void print_error_pos(struct peripherals_t &peripherals, double error);

#endif
