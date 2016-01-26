#include <cassert>
#include <cstring>
#include <cstdio>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libbase/k60/gpio.h>
#include <libsc/st7735r.h>  // LCD
#include <libsc/tsl1401cl.h> // CCD

#include "main.hpp"

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

using namespace libsc;
using namespace libbase::k60;

int main(void) {
	System::Init();

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	// Redirect the local buffer of the CCD object.
	ccd_buffer_t *ccd_data = &(peripherals.ccd->GetData());

	while(true) {
		// Reset the buffer index, and start the acquisition.
		peripherals.ccd->StartSample();
		while(!peripherals.ccd->SampleProcess());

		print_scan_result(ccd_data);

		System::DelayMs(UPDATE_INT);
	}

	return 0;
}

void init(struct peripherals_t &peripherals) {
	// Init the LCD hardware.
	St7735r::Config st7735r_config;
	st7735r_config.is_revert = false;
	st7735r_config.is_bgr = false;
	peripherals->lcd = new St7735r(st7735r_config);

	// Init the linear CCD.
	peripherals->ccd = new Tsl1401cl(0);
}

void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t *ccd_data) {
	// Clear the screen.
	peripherals.ccd->Clear();

	// Rect variable for reuse.
	Lcd::Rect region;

	// Start drawing the entire array,
	//  since we know that the screen width is the same as the sensor width.
	//  (X = 128, Y = 160)
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		region = { .x = i, .y = 0,
			          .w = 1, .h = (255-ccd_data[i])/2 };
		peripherals.lcd->SetRegion(region);
		peripherals.lcd->FillPixel(Lcd::kGray);
	}
}
