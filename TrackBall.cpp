
#include "TrackBall.h"

TrackBall::TrackBall(uint8_t address, uint8_t interruptPin) {
	Serial.print(address, HEX);
	Serial.print('\t');
	Serial.println(interruptPin);

	// Set up I2C connection
	_address = address;
	_interruptPin = interruptPin;

	// Not doing this is probably fine
	/*chip_id = struct.unpack("<H", bytearray(self.i2c_rdwr([REG_CHIP_ID_L], 2)))[0]
	if (chip_id != CHIP_ID) {
		raise RuntimeError("Invalid chip ID: 0x{:04X}, expected 0x{:04X}".format(chip_id, CHIP_ID));
	}*/

	if (_interruptPin != NO_PIN) {
		pinMode(_interruptPin, INPUT);
	}

	// Set default interrupt behaviour
	enableInterrupt();
}

// Enable/disable GPIO interrupt pin.
void TrackBall::enableInterrupt(bool interrupt) {
	Serial.println("enable_interrupt");

	uint8_t value = 0;
	//i2c_rdwr_2(&REG_INT, 1, &value, 1);

	value &= ~MSK_INT_OUT_EN;
	if (interrupt) {
		value |= MSK_INT_OUT_EN;
	}

	i2cWriteRegValue(REG_INT, value);
}

// Get the trackball interrupt status.
/*bool TrackBall::getInterrupt() {
	if (_interruptPin != NO_PIN) {
		return !digitalRead(_interruptPin);
	} else {
		uint8_t value = 0;
		//i2c_rdwr_2(&REG_INT, 1, &value, 1);
		return value & MSK_INT_TRIGGERED;
	}
}*/

// Write a new I2C address into flash.
/*void TrackBall::changeAddress(uint8_t new_address) {
	i2cWriteRegValue(REG_I2C_ADDR, new_address);
	waitForFlash();
}*/

// Set all LED brightness as RGBW.
void TrackBall::setRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
	Wire.beginTransmission(_address);
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
	Wire.beginTransmission(_address);
	Wire.write(REG_LEFT);
	Wire.endTransmission();

	Wire.requestFrom(_address, 5);

	if (Wire.available() >= 5) {
		left = Wire.read();
		right = Wire.read();
		up = Wire.read();
		down = Wire.read();
		sw = Wire.read();

		x += right - left;
		y += up - down;

		return true;	// success
	}

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
	return sw & ~MSK_SWITCH_STATE;	// bottom 7 bits
}

bool TrackBall::getSwitchState() {
	return sw & MSK_SWITCH_STATE;	// top bit as bool
}

long TrackBall::getX() {
	return x;
}

long TrackBall::getY() {
	return y;
}

bool TrackBall::isAnyInput() {
	return left || right || up || down || sw;
}

/*uint8_t TrackBall::i2c_rdwr_2(const uint8_t* data, uint8_t dataLen, const uint8_t* readBuf, uint8_t readLen) {
	Wire.beginTransmission(_address); // transmit to device
	for (uint8_t i = 0; i < dataLen; i++) {
		Wire.write(data[i]);        // send all data
	}
	Wire.endTransmission();    // stop transmitting

	uint8_t bytesRead = 0;
	while (bytesRead < readLen && Wire.available()) {
		readBuf[bytesRead++] = Wire.read(_address);
	}

	return bytesRead;
}*/

void TrackBall::i2cWriteRegValue(uint8_t reg, uint8_t value) {
	Serial.print(reg, HEX);
	Serial.print('\t');
	Serial.println(value);

	Wire.beginTransmission(_address);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

/*void TrackBall::waitForFlash() {
	while (getInterrupt()) {}
	while (!getInterrupt()) {
		delay(1);
	}
}*/
