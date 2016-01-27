#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <cstdio>
#include <cassert>
#include <cstdint> // uint16_t
#include <cstring>
#include <array> // std::array
#include <libbase/k60/mcg.h>
#include <libsc/system.h>

#include <libsc/st7735r.h>
#include <libsc/tsl1401cl.h>
#include <libsc/alternate_motor.h>
#include <libsc/futaba_s3010.h>

#define REFRESH_INT 10 // 50ms interval, 20FPS

typedef std::array<uint16_t, libsc::Tsl1401cl::kSensorW> ccd_buffer_t;

/* Type definitions */
struct peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;

	libsc::FutabaS3010 *steering;
	libsc::AlternateMotor *driving;
};

/* Prototypes */
void init(struct peripherals_t &peripherals);

bool bluetooth_listener(const Byte *data, const size_t data_size);

void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t& ccd_data);

#endif
