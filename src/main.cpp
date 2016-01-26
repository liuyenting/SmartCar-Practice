#include <cassert>
#include <cstring>
#include <cstdio>
#include <libbase/k60/mcg.h>
#include <libsc/system.h>
#include <libbase/k60/gpio.h>
#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>

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

	while(true) {
		/* Put the state machine here.... */
	}

	return 0;
}

void init(struct peripherals_t &peripherals) {
	// Init bluetooth.
	libsc::k60::JyMcuBt106::Config bluetooth_config;
	bluetooth_config.id = 0;       // UART0
	bluetooth_config.baud_rate = Uart::Config::BaudRate::k115200;
	bluetooth_config.tx_buf_size = TX_BUF_SIZE;
	bluetooth_config.rx_isr = bluetooth_listener;
	peripherals.bluetooth = new libsc::k60::JyMcuBt106(bluetooth_config);
	//Init motor.
	AlternateMotor::Config motorConfig;
	motorConfig.id = 0;
	peripherals.driving = new AlternateMotor(motorConfig);
}

bool bluetooth_listener(const Byte *data, const size_t data_size) {
	// Take the first byte in the array as our current action to process.
	Byte action = data[0];
	switch(action) {
	case CMD_START:
		/* Start the car */

		break;
	case CMD_STOP:
		/* Stop the car */

		break;
	default:
		// Unknown command.
		break;
	}
return true;
// Discard rest of the data.

}
