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
	std::array<uint16_t, Tsl1401cl::kSensorW> ccd_data;

	while(true) {
		// Reset the buffer index, and start the acquisition.
		peripherals.ccd->StartSample();
		while(!peripherals.ccd->SampleProcess());
		ccd_data = peripherals.ccd->GetData();

		print_scan_result(peripherals, ccd_data);

		System::DelayMs(UPDATE_INT);
	}

	return 0;
}

void init(struct peripherals_t &peripherals) {
	// Init the LCD hardware.
	St7735r::Config st7735r_config;
	st7735r_config.is_revert = false;
	st7735r_config.is_bgr = false;
	peripherals.lcd = new St7735r(st7735r_config);

	// Init the linear CCD.
	peripherals.ccd = new Tsl1401cl(0);
}

void print_scan_result(struct peripherals_t &peripherals, std::array<uint16_t, Tsl1401cl::kSensorW>& ccd_data) {
	// Clear the screen.
	peripherals.lcd->Clear();

	Lcd::Rect region;
	region.y = 0; // Start from the first row.
	region.w = 1; // Bar width = 1px.

	// Start drawing the entire array,
	//  since we know that the screen width is the same as the sensor width.
	//  (X = 128, Y = 160)
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		region.x = iㄤ
		
		float ratio = (float)ccd_data[i] / 4096;
		region.h = 160 * ratio;

		peripherals.lcd->SetRegion(region);
		peripherals.lcd->FillColor(Lcd::kGray);
	}
}
