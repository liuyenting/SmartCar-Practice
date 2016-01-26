#ifndef __MAIN_HPP__
#define __MAIN_HPP__

/* Bluetooth buffer related */
#define TX_BUF_SIZE 200

/* Control action related */
#define CMD_START   'j'
#define CMD_STOP    'k'

struct peripherals_t {
	libsc::St7735r *lcd;
	libsc::Tsl1401cl *ccd;
};

/* Prototypes */
void init(struct peripherals_t &peripherals);
bool bluetooth_listener(const Byte *data, const size_t data_size);

#endif
