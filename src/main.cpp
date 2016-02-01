#include "main.hpp"
#include "pVarManager.h"


using namespace libsc;
using namespace libbase::k60;

uint16_t color = 0xFFFF;

int main(void) {
	System::Init();

	char msg[30];

	// Bundle-up all of our peripheral devices.
	peripherals_t peripherals;
	init(peripherals);

	Pid pid_model(STEERING_CENTER - STEERING_RANGE,
	              STEERING_CENTER + STEERING_RANGE,
	              KP, KI, KD);
	pid_model.set_target(0.0);
float kp;
	pVarManager grapher;
	grapher.addSharedVar(&kp,"kp");
	grapher.sendWatchData();
	double error_val = 0;
	int steer_pos = STEERING_CENTER;
	int16_t speed =0;

	ccd_buffer_t avg_ccd_data;

	// Set default steering position.
	peripherals.steering->SetDegree(steer_pos);

	// Set driving motor speed.
	peripherals.driving->SetClockwise(false);
	peripherals.driving->SetPower(DRIVING_POWER);

	// Timer to contain the time stamp.
	Timer::TimerInt prev_time = System::Time();
	float dt;

	//TIME
	uint16_t t;

	while(true) {

		//Time variable
		t= System::Time() ;

		sprintf(msg, "%d", steer_pos);
		peripherals.writer->WriteString(msg);


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
		//sprintf(msg, "%d", error_val);
		//peripherals.writer->WriteString(msg);
		//System::DelayMs(2000);
		//error_val = otsu(avg_ccd_data);
		dt = Timer::TimeDiff(System::Time(), prev_time) / 1000.0f;
		steer_pos = (int)pid_model.calculate(dt, error_val);
		//sprintf(msg, "%d", steer_pos);
		//peripherals.writer->WriteString(msg);
		// Set steering wheel position.
		peripherals.steering->SetDegree(steer_pos);

		System::DelayMs(REFRESH_INTERVAL);
		uint16_t t = System::Time();
		//sprintf(msg, "%d", t);
		//peripherals.writer->WriteString(msg);


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

	// Init the LED.
	libsc::Led::Config led_config;
	led_config.id = 3;
	led_config.is_active_low = 0;
	peripherals.led = new Led(led_config);

	// Init the LCD console.
	libsc::LcdConsole::Config console_config;
	console_config.bg_color = 0;
	console_config.lcd = peripherals.lcd;
	console_config.text_color = 0xFFFF;
	peripherals.console = new LcdConsole(console_config);

	// Init the LCD Typewritter.
	libsc::LcdTypewriter::Config writer_config;
	writer_config.lcd = peripherals.lcd;
	writer_config.is_text_wrap = true;
	writer_config.text_color = 0xFFFF;
	peripherals.writer = new LcdTypewriter(writer_config);


	// Init the steering servo.
	FutabaS3010::Config steering_config;
	steering_config.id = 0;
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

//Threshold algo from otsu
uint16_t otsu(uint16_t *p){
    uint16_t g,max=0;
    uint16_t tt=0, ttlow=0;
    uint16_t u0=0, u1=0, count=0, cnt=0;
    uint16_t tr=0;
    uint16_t pc[256] = {0};
    uint16_t j;
    uint16_t pk = 0;

    for(j=5; j<123; j++){
        //pc[haha[j]]++;
        //tt+=haha[j];
        pc[*(p+j)]+=1;
        tt+=(*(p+j));
        pk++;
    }

    for(j=0; j< 256; j++){
        cnt=pc[j];
        if(cnt==0) continue;
        count+=pc[j];
        ttlow+=cnt*j;
        u0=ttlow/count;
        if(count==116){
            int y;
            y=5;
        }

        u1=(tt-ttlow)/(pk-count);
        g=((u0-u1)*(u0-u1))*((count*(118-count)))/16654;

        if(g>max){
            max=g;
            tr=j;
        }
        if(count > (pk-5)) break;
    }
    return tr;
}

void print_scan_result(struct peripherals_t &peripherals, ccd_buffer_t &ccd_data) {

	/*
	// Clear the screen.
	// peripherals.lcd->Clear(); delays too much.
	peripherals.lcd->ClearRegion();
	peripherals.lcd->FillColor(Lcd::kBlack);

	Lcd::Rect region;
	region.y = 0; // Start from the first row.
	region.w = 1; // Bar width = 1px.
	*/


	// Start drawing the entire array,
	//  since we know that the screen width is the same as the sensor width.
	//  (X = 128, Y = 160)
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
		peripherals.lcd->SetRegion(Lcd::Rect(i, (255-ccd_data[i])/2.0,1,1));
		peripherals.lcd->FillPixel(&color,1);
	}

	/*
	for(uint16_t i = 0; i < Tsl1401cl::kSensorW; i++) {
		region.x = i, region.h = ccd_data[i];

		peripherals.lcd->SetRegion(region);
		peripherals.lcd->FillColor(Lcd::kWhite);
	}
	*/

	//Wait 5ms to clear
	System::DelayMs(5);

	//Clear Region.
	for(uint16_t i=0; i<Tsl1401cl::kSensorW; i++){
		peripherals.lcd->SetRegion(Lcd::Rect(i, (255-ccd_data[i])/2.0,1,1));
		peripherals.lcd->FillColor(libsc::Lcd::kBlack);
	}


}
