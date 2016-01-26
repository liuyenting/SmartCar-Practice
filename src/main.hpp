#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <cstdint>
#include <array>

#define UPDATE_INT 50 // 50ms, 20FPS

/* Type definitions */
struct peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;
};


/* Prototypes */
void init(struct peripherals_t &peripherals);
void print_scan_result(struct peripherals_t &peripherals, std::array<uint16_t, Tsl1401cl::kSensorW>& ccd_data);

#endif
