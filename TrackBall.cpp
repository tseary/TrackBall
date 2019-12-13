
#include "TrackBall.h"

TrackBall::TrackBall(uint8_t address, uint8_t interruptPin) {
	// Set up I2C connection
	_i2c_address = address;
	_interrupt_pin = interruptPin;

	// Not doing this is probably fine
	/*chip_id = struct.unpack("<H", bytearray(self.i2c_rdwr([REG_CHIP_ID_L], 2)))[0]
	if (chip_id != CHIP_ID) {
		raise RuntimeError("Invalid chip ID: 0x{:04X}, expected 0x{:04X}".format(chip_id, CHIP_ID));
	}*/

	if (_interrupt_pin != NO_PIN) {
		pinMode(_interrupt_pin, INPUT);
	}

	enable_interrupt();
}

// Write a new I2C address into flash.
void TrackBall::change_address(uint8_t new_address) {
	i2cWriteRegValue(REG_I2C_ADDR, new_address);
	_wait_for_flash();
}

void TrackBall::_wait_for_flash() {
	while (get_interrupt()) {}
	while (!get_interrupt()) {
		delay(1);
	}
}

// Enable/disable GPIO interrupt pin.
void TrackBall::enable_interrupt(bool interrupt) {
	uint8_t value = 0;
	//i2c_rdwr_2(&REG_INT, 1, &value, 1);

	value &= ~MSK_INT_OUT_EN;
	if (interrupt) {
		value |= MSK_INT_OUT_EN;
	}

	i2cWriteRegValue(REG_INT, value);
}

/*uint8_t TrackBall::i2c_rdwr_2(const uint8_t* data, uint8_t dataLen, const uint8_t* readBuf, uint8_t readLen) {
	Wire.beginTransmission(_i2c_address); // transmit to device
	for (uint8_t i = 0; i < dataLen; i++) {
		Wire.write(data[i]);        // send all data
	}
	Wire.endTransmission();    // stop transmitting

	uint8_t bytesRead = 0;
	while (bytesRead < readLen && Wire.available()) {
		readBuf[bytesRead++] = Wire.read(_i2c_address);
	}

	return bytesRead;
}*/

void TrackBall::i2cWriteRegValue(uint8_t reg, uint8_t value) {
	Wire.beginTransmission(_i2c_address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

// Get the trackball interrupt status.
bool TrackBall::get_interrupt() {
	if (_interrupt_pin != NO_PIN) {
		return !digitalRead(_interrupt_pin);
	} else {
		uint8_t value = 0;
		//i2c_rdwr_2(&REG_INT, 1, &value, 1);
		return value & MSK_INT_TRIGGERED;
	}
}

// Set all LED brightness as RGBW.
void TrackBall::setRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
	Wire.beginTransmission(_i2c_address);
	Wire.write(REG_LED_RED);
	Wire.write(r);
	Wire.write(g);
	Wire.write(b);
	Wire.write(w);
	Wire.endTransmission();
}

// Set brightness of trackball red LED.
void TrackBall::setRed(uint8_t value) {
	i2cWriteRegValue(REG_LED_RED, value);
}

// Set brightness of trackball green LED.
void TrackBall::setGreen(uint8_t value) {
	i2cWriteRegValue(REG_LED_GRN, value);
}

// Set brightness of trackball blue LED.
void TrackBall::setBlue(uint8_t value) {
	i2cWriteRegValue(REG_LED_BLU, value);
}

// Set brightness of trackball white LED.
void TrackBall::setWhite(uint8_t value) {
	i2cWriteRegValue(REG_LED_WHT, value);
}

// Read and store all directional information from the trackball.
bool TrackBall::read() {
	Wire.beginTransmission(_i2c_address);
	Wire.write(REG_LEFT);
	Wire.endTransmission();

	uint8_t timeout = 10;
	do {
		if (Wire.available() >= 5) {
			left = Wire.read();
			right = Wire.read();
			up = Wire.read();
			down = Wire.read();
			sw = Wire.read();

			swState = sw & MSK_SWITCH_STATE;	// top bit as bool
			sw &= ~MSK_SWITCH_STATE;				// bottom 7 bits

			return true;	// success
		}

		delay(1);
	} while (--timeout > 0);

	return false;
}

uint8_t TrackBall::getLeft() {
	return left;
}

uint8_t TrackBall::getRight() {
	return right;
}

uint8_t TrackBall::getUp() {
	return up;
}

uint8_t TrackBall::getDown() {
	return down;
}

uint8_t TrackBall::getSwitch() {
	return sw;
}

bool TrackBall::getSwitchState() {
	return swState;
}
