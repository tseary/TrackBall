#pragma once

#include <Arduino.h>
#include <Wire.h>

/*
The Pimoroni trackball-python library, ported to Arduino.
https://github.com/pimoroni/trackball-python/
Created Dec. 2019 by Thomas Seary
Some features that are available in the python library are not implemented here:
- Checking the chip ID in the constructor
- Getting the interrupt status over I2C
- Changing the I2C address.
*/

class TrackBall {
public:
	// I2C Addresses
	static const uint8_t
		I2C_ADDRESS = 0x0A;
		//I2C_ADDRESS_ALTERNATIVE = 0x0B;

	static const uint8_t NO_PIN = 255;

private:
	// Chip identity
	/*static const uint16_t
		CHIP_ID = 0xBA11,
		VERSION = 1;*/

	// I2C Registers
	static const uint8_t
		REG_LED_RED = 0x00,
		REG_LED_GRN = 0x01,
		REG_LED_BLU = 0x02,
		REG_LED_WHT = 0x03;

	static const uint8_t
		REG_LEFT = 0x04,
		REG_RIGHT = 0x05,
		REG_UP = 0x06,
		REG_DOWN = 0x07,
		REG_SWITCH = 0x08,
		MSK_SWITCH_STATE = 0b10000000;

	static const uint8_t
		REG_USER_FLASH = 0xD0,
		REG_FLASH_PAGE = 0xF0,
		REG_INT = 0xF9,
		MSK_INT_TRIGGERED = 0b00000001,
		MSK_INT_OUT_EN = 0b00000010,
		REG_CHIP_ID_L = 0xFA,
		RED_CHIP_ID_H = 0xFB,
		REG_VERSION = 0xFC,
		REG_I2C_ADDR = 0xFD,
		REG_CTRL = 0xFE,
		MSK_CTRL_SLEEP = 0b00000001,
		MSK_CTRL_RESET = 0b00000010,
		MSK_CTRL_FREAD = 0b00000100,
		MSK_CTRL_FWRITE = 0b00001000;

	// self fields
	uint8_t _address;
	//TwoWire& _i2c_bus;
	uint8_t _interruptPin;

	// Readout
	uint8_t left, right, up, down, sw;
	bool swState;

	long x = 0, y = 0;

public:
	TrackBall(uint8_t address = I2C_ADDRESS, uint8_t interruptPin = NO_PIN);

	void enableInterrupt(bool interrupt = true);

	bool isInterrupt();

	// Not supported
	//void changeAddress(uint8_t new_address);

	// Set all colour components
	void setRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);

	// Set colour components
	void setRed(uint8_t value);
	void setGreen(uint8_t value);
	void setBlue(uint8_t value);
	void setWhite(uint8_t value);

	// Read and store the state of the trackball.
	// Returns a positive number if successful.
	bool read();

	// Get the stored state of the trackball
	// Returns the number of left "ticks" since the previous call to read().
	uint8_t getLeft();
	// Returns the number of right "ticks" since the previous call to read().
	uint8_t getRight();
	// Returns the number of up "ticks" since the previous call to read().
	uint8_t getUp();
	// Returns the number of down "ticks" since the previous call to read().
	uint8_t getDown();
	// Returns the number of switch toggles since the previous call to read().
	uint8_t getSwitch();
	// Returns true if the switch was pressed at the last call to read().
	bool getSwitchState();

	void resetOrigin();
	long getX();
	long getY();

	// Returns true if any movement was made as of the last call to read().
	bool isAnyInput();

private:
	// I2C helpers
	// Returns the nuymber of bytes read into readBuf.
	/*uint8_t i2c_rdwr_2(uint8_t* data, const uint8_t dataLen, uint8_t* readBuf, const uint8_t readLen);*/
	// Writes a value into a register.
	void i2cWriteRegValue(uint8_t reg, uint8_t value);

	//void waitForFlash();
};
