#include "main.hpp"

using namespace libsc;
using namespace libbase::k60;

char str_buf[32];
const uint16_t color = 0xFFFF;

int main(void) {
	System::Init();

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	Pid pid_model(STEERING_CENTER - STEERING_RANGE,
	              STEERING_CENTER + STEERING_RANGE,
	              KP, KI, KD);
	pid_model.set_target(64.0);

	uint16_t error_val = 0;
	int steer_pos = STEERING_CENTER;

	ccd_buffer_t avg_ccd_data;

	// Set default steering position.
	peripherals.steering->SetDegree(steer_pos);

	// Set driving motor speed.
	peripherals.driving->SetClockwise(false);
	peripherals.driving->SetPower(DRIVING_POWER);

	// Timer to contain the time stamp.
	Timer::TimerInt prev_time = System::Time();
	float dt;

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

		error_val = calculate_error(avg_ccd_data);

		sprintf(str_buf, "ERR = %d", error_val);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 0, 128, 16));
		peripherals.typewriter->WriteString(str_buf);
		// print_error_pos(peripherals, error_val);

		dt = Timer::TimeDiff(System::Time(), prev_time) / 1000.0f;
		steer_pos = pid_model.calculate(dt, (double)error_val);

		sprintf(str_buf, "POS = %d", steer_pos);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 16, 128, 16));
		peripherals.typewriter->WriteString(str_buf);

		// Set steering wheel position.
		peripherals.steering->SetDegree(steer_pos);

		System::DelayMs(REFRESH_INTERVAL);

		prev_time = System::Time();
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

uint16_t calculate_error(ccd_buffer_t &ccd_data) {
	uint32_t g, max = 0;
	uint16_t total = 0, total_low = 0;
	uint8_t u0 = 0, u1 = 0, count = 0, tr = 0, cnt = 0;
	uint8_t pc[256] = { 0 };
	for(int j = 5; j < 122; j++) {
		pc[ccd_data[j]]++;
		total += ccd_data[j];
	}
	for(int j = 0; j < 254; j++) {
		cnt = pc[j];
		if(cnt == 0)
			continue;
		count += pc[j];
		total_low += cnt * j;
		u0 = total_low / count;
		u1 = (total - total_low) / (118 - count);
		g = ((uint32_t)(u0 - u1) * (u0 - u1)) * ((count * (118 - count))) / 16384;
		if(g > max) {
			max = g;
			tr = j;
		}
		if(count >= 118)
			break;
	}
	return tr;
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

void print_error_pos(struct peripherals_t &peripherals, double error) {

}
