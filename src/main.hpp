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

#define REFRESH_INTERVAL 50 // 50ms interval, 20FPS
#define AVERAGE_COUNTS  5 // Average interval, unit: sample.

#define STEERING_CENTER 1000

#define INI_DRIVING_POWER 500

// K for servo
#define KD 0
#define KI 0
#define KP 18

// k for motor
#define kd 0
#define ki 0
#define kp 10

typedef std::array<uint16_t, libsc::Tsl1401cl::kSensorW> ccd_buffer_t;

/* Type definitions */
typedef struct {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;

	libsc::FutabaS3010 *steering;
	libsc::AlternateMotor *driving;

	libsc::LcdTypewriter *typewriter;
} peripherals_t;

typedef struct {
	int min;
	int max;
} range_t;

typedef struct {
	double kp;
	double ki;
	double kd;
} pid_var_t;

/* Prototypes */
void init(peripherals_t &peripherals);
double calculate_center_pos(ccd_buffer_t &ccd_data);
void print_scan_result(peripherals_t &peripherals, ccd_buffer_t& ccd_data);

#endif
