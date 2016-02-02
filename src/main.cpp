#include "main.hpp"

using namespace libsc;
using namespace libbase::k60;

char str_buf[32];
const uint16_t color = 0xFFFF;

#define LEFT_POS 40
#define RIGHT_POS 60

int main(void) {
	System::Init();

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	double center_pos = 0;
	int steer_pos = STEERING_CENTER;

	ccd_buffer_t avg_ccd_data;

	// Set default steering position.
	peripherals.steering->SetDegree(steer_pos);

	// Set driving motor speed.
	peripherals.driving->SetClockwise(false);
	peripherals.driving->SetPower(DRIVING_POWER);

	while(true) {
		// Dummy read to wipe out the charges on the CCD.
		peripherals.ccd->StartSample();
		while(!peripherals.ccd->SampleProcess());

		for(int i = 0; i < AVERAGE_COUNTS; i++) {
			// Start the acquisition and grab the data.
			peripherals.ccd->StartSample();
			while(!peripherals.ccd->SampleProcess());

			// Average the result from the second sample.
			if(i == 0)
				avg_ccd_data = peripherals.ccd->GetData();
			else {
				for(int j = 0; j < Tsl1401cl::kSensorW; j++)
					avg_ccd_data[j] = (avg_ccd_data[j] + peripherals.ccd->GetData()[j]) / 2;
			}
		}

		print_scan_result(peripherals, avg_ccd_data);

		center_pos = calculate_center_pos(avg_ccd_data);

		sprintf(str_buf, "ERR = %.2f", center_pos);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 0, 128, 16));
		peripherals.typewriter->WriteString(str_buf);
		// print_error_pos(peripherals, error_val);

		// Change the steering position.
		if(center_pos < LEFT_POS)
			steer_pos = 700;
		else if(center_pos > RIGHT_POS)
			steer_pos = 1100;
		else
			steer_pos = 900;

		sprintf(str_buf, "POS = %d", steer_pos);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 16, 128, 16));
		peripherals.typewriter->WriteString(str_buf);

		// Set steering wheel position.
		peripherals.steering->SetDegree(steer_pos);

		System::DelayMs(REFRESH_INTERVAL);
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

	// Init the steering servo.
	FutabaS3010::Config steering_config;
	steering_config.id = 0;
	peripherals.steering = new FutabaS3010(steering_config);

	// Init the driving motor.
	AlternateMotor::Config driving_config;
	driving_config.id = 0;
	peripherals.driving = new AlternateMotor(driving_config);

	// Init the type writer for debug information.
	LcdTypewriter::Config typewriter_config;
	typewriter_config.lcd = peripherals.lcd;
	typewriter_config.text_color = Lcd::kWhite;
	typewriter_config.bg_color = Lcd::kBlack;
	peripherals.typewriter = new LcdTypewriter(typewriter_config);
}

double calculate_center_pos(ccd_buffer_t &ccd_data) {
	// Search for the minimum and maximum value.
	uint16_t min_val, max_val;
	min_val = max_val = ccd_dat[0]; // Default value is the first element.
	for(int i = 1; i < Tsl1401cl::kSensorW; i++) {
		if(ccd_data[i] < min_val)
			min_val = ccd_data[i];
		else if(ccd_data[i] > max_val)
			max_val = ccd_data[i];
	}

	uint16_t threshold = (min_val + max_val) / 2;
	// Perform binary operation on the values.
	for(int i = 0; i < Tsl1401cl::kSensorW; i++)
		ccd_data[i] = (ccd_data[i] > threshold);

	// Calculate the average position.
	double center = 0;
	for(int i = 0; i < Tsl1401cl::kSensorW; i++)
		center += i * ccd_data[i]; // The bins are weighted by their positions.
	center /= 128;

	// If you only wants to divide by the bins that are taken into account,
	// comment out the following section.
	/*
	   double center = 0;
	   uint16_t cnt = 0;
	   for(int i = 0; i < Tsl1401cl::kSensorW; i++) {
	   if(ccd_data[i]) {
	   ++cnt;
	   center += i;
	   }
	   }
	   center /= cnt;
	 */

	return center;
}

void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t &ccd_data) {
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		peripherals.lcd->SetRegion(Lcd::Rect(i, (255-ccd_data[i])/2.0, 1, 1));
		peripherals.lcd->FillColor(Lcd::kWhite);
	}

	// Wait 5ms to clear
	System::DelayMs(5);

	// Clear Region.
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++) {
		peripherals.lcd->SetRegion(Lcd::Rect(i, (255 - ccd_data[i])/2.0, 1, 1));
		peripherals.lcd->FillColor(Lcd::kBlack);
	}
}
