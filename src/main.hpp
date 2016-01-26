#ifndef __MAIN_HPP__
#define __MAIN_HPP__

#include <libsc/k60/jy_mcu_bt_106.h>
#include <libsc/futaba_s3010.h>
#include <libsc/alternate_motor.h>
/* Bluetooth buffer related */
#define TX_BUF_SIZE 200

/* Control action related */
#define CMD_START   'j'
#define CMD_STOP    'k'
using namespace libsc;
using namespace libbase::k60;

struct peripherals_t {
	libsc::k60::JyMcuBt106 *bluetooth;
	FutabaS3010 *steering;
	AlternateMotor *driving;
};

/* Prototypes */
void init(struct peripherals_t &peripherals);
bool bluetooth_listener(const Byte *data, const size_t data_size);

#endif
