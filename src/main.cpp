#include "main.hpp"

using namespace libsc;
using namespace libbase::k60;

char str_buf[32];

#define USE_LCD

const pid_var_t pid_servo_var(18, 0, 0);
const pid_var_t pid_motor_var(17, 0, 0);

const range_t pid_servo_range(-600, 600);
const range_t pid_motor_range(-250, 250);

int main(void) {
	System::Init();

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	// init pid for servo and motor
	Pid pid_servo(pid_servo_range.min, pid_servo_range.max,
	              pid_servo_var.kp, pid_servo_var.ki, pid_servo_var.kd);
	pid_servo.set_target(63.5);

	Pid pid_motor(pid_motor_range.min, pid_motor_range.max,
	              pid_motor_var.kp, pid_motor_var.ki, pid_motor_var.kd);
	pid_motor.set_target(63.5);

	double center_pos = 0;

	// Variables to store the driving details.
	int steer_pos = STEERING_CENTER;
	int drive_pwr = INI_DRIVING_POWER;

	ccd_buffer_t avg_ccd_data;

	// Set default steering position.
	peripherals.steering->SetDegree(steer_pos);

	// Set initial driving motor speed.
	peripherals.driving->SetClockwise(false);
	peripherals.driving->SetPower(drive_pwr);

	// Start the timer.
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
		#ifdef USE_LCD
		print_scan_result(peripherals, avg_ccd_data);
		#endif

		center_pos = calculate_center_pos(avg_ccd_data);

		// Time elapsed since last calculation.
		dt = Timer::TimeDiff(System::Time(), prev_time) / 1000.0f;
		// Calculate the new steering wheel positoin and adapated driving speed.
		steer_pos = STEERING_CENTER + (int)pid_servo.calculate(dt, center_pos);
		int drive_pwr_diff = (int)pid_motor.calculate(dt, center_pos); // THIS SHOULD BE steer_pos
		if(drive_pwr_diff > 0)
			drive_pwr = INI_DRIVING_POWER - drive_pwr_diff;
		else
			drive_pwr = INI_DRIVING_POWER + drive_pwr_diff;

		// Apply the newly calculated result.
		peripherals.steering->SetDegree(steer_pos);
		peripherals.driving->SetPower(drive_pwr);

		#ifdef USE_LCD
		// Print the variables.
		sprintf(str_buf, "ERR = %.2f", center_pos);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 0, 128, 16));
		peripherals.typewriter->WriteString(str_buf);
		sprintf(str_buf, "POS = %d", steer_pos);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 16, 128, 16));
		peripherals.typewriter->WriteString(str_buf);
		sprintf(str_buf, "PWR = %d", drive_pwr);
		peripherals.lcd->SetRegion(Lcd::Rect(0, 32, 128, 16));
		peripherals.typewriter->WriteString(str_buf);
		#endif

		System::DelayMs(REFRESH_INTERVAL);
	}

	return 0;
}

void init(peripherals_t &peripherals) {
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
	min_val = max_val = ccd_data[0]; // Default value is the first element.
	for(int i = 0; i < Tsl1401cl::kSensorW; i++) {
		if(ccd_data[i] < min_val)
			min_val = ccd_data[i];
		else if(ccd_data[i] > max_val)
			max_val = ccd_data[i];
	}

	uint16_t threshold = (min_val + max_val) / 2;
	// Perform binary operation on the pixels.
	uint8_t count = 0;
	double center = 0;
	for(int i = 0; i < Tsl1401cl::kSensorW; i++) {
		if(ccd_data[i]) {
			center += i; // Weighted by the pixel's location.
			++count;
		}
	}
	center /= count;

	return center;
}

void print_scan_result(peripherals_t &peripherals, ccd_buffer_t &ccd_data) {
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		peripherals.lcd->SetRegion(Lcd::Rect(i, (225-ccd_data[i])/2.0, 1, 1));
		peripherals.lcd->FillColor(Lcd::kWhite);
	}

	// Wait 5ms to clear
	System::DelayMs(5);

	// Clear Region.
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++) {
		peripherals.lcd->SetRegion(Lcd::Rect(i, (225-ccd_data[i])/2.0, 1, 1));
		peripherals.lcd->FillColor(Lcd::kBlack);
	}
}
