#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <cstdio>
#include <cassert>
#include <cstdint> // uint16_t
#include <cstring>
#include <array> // std::array
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libsc/st7735r.h>  // LCD
#include <libsc/tsl1401cl.h> // CCD

#define UPDATE_INT 30 // ~30FPS

/* Type definitions */
struct peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;
};

/* Prototypes */
void init(struct peripherals_t &peripherals);
void print_scan_result(struct peripherals_t &peripherals, std::array<uint16_t, libsc::Tsl1401cl::kSensorW>& ccd_data);

#endif
