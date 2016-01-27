#include "main.hpp"

using namespace libsc;
using namespace libbase::k60;

int main(void) {
	System::Init();

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	Pid pid_model(REFRESH_INTERVAL,
	              STEERING_CENTER - STEERING_RANGE,
	              STEERING_CENTER + STEERING_RANGE,
	              KP, KI, KD);
	pid_model.set_target(0.0);

	double error_val = 0;
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
					avg_ccd_data[j] = (avg_ccd_data[j] + peripherals.ccd->GetData()) / 2;
			}
		}

		print_scan_result(peripherals, avg_ccd_data);

		error_val = calculate_error(avg_ccd_data);
		steer_pos = std::round(pid_model.calculate(error_val));

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
	servo_config.id = 0;
	peripherals.steering = new FutabaS3010(steering_config);

	// Init the driving motor.
	AlternateMotor::Config driving_config;
	driving_config.id = 0;
	peripherals.driving = new AlternateMotor(driving_config);
}

double calculate_error(ccd_buffer_t &ccd_data) {
	uint16_t ccd_min_val, ccd_max_val;

	// Directly assign the first value as kickstart.
	ccd_min_val = ccd_max_val = ccd_data[0];

	// Find out the upper and lower boundary of current batch of the signal.
	for(int i = 1; i < Tsl1401cl::kSensorW; i++) {
		if(ccd_data[i] > ccd_max_val)
			ccd_max_val = ccd_data[i];
		if(ccd_data[i] < ccd_min_val)
			ccd_min_val = ccd_data[i];
	}

	uint16_t threshold = (ccd_min_val + ccd_max_val) / 2;

	int left_pos = -1, right_pos = -1;
	bool state = (ccd_data[0] < threshold);
	for(int i = 1; i < Tsl1401cl::kSensorW; i++) {
		if(state ^ (ccd_data[i] < threshold)) {
			// State change.
			// Note: Record the first change as left side,
			//        while the last change as right side.
			if(left_pos == -1)
				left_pos = i;
			else
				right_pos = i;
		}
	}

	return (left_pos + right_pos) / 2.0;
}

void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t &ccd_data) {
	// Clear the screen.
	// peripherals.lcd->Clear(); delays too much.
	peripherals.lcd->ClearRegion();
	peripherals.lcd->FillColor(Lcd::kBlack);

	Lcd::Rect region;
	region.y = 0; // Start from the first row.
	region.w = 1; // Bar width = 1px.

	// Start drawing the entire array,
	//  since we know that the screen width is the same as the sensor width.
	//  (X = 128, Y = 160)
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		region.x = i, region.h = ccd_data[i];

		peripherals.lcd->SetRegion(region);
		peripherals.lcd->FillColor(Lcd::kWhite);
	}
}
