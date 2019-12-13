#pragma once

#include <Arduino.h>
#include <Wire.h>

/*

*/

class TrackBall {
protected:
	// I2C Addresses
	static const uint8_t
	I2C_ADDRESS = 0x0A,
	I2C_ADDRESS_ALTERNATIVE = 0x0B;
	
	static const uint16_t
	CHIP_ID = 0xBA11,
	VERSION = 1;
	
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
	
public:
	static const uint8_t NO_PIN = 255;
	
private:
	// self fields
	uint8_t _i2c_address;
	Wire *_i2c_bus;
	uint8_t _interrupt_pin;
	
	// Readout
	uint8_t left, right, up, down, enter;
	bool enter_state;

public:
	TrackBall(uint8_t address = I2C_ADDRESS, Wire* i2c_bus = Wire, interrupt_pin = 0xff);

    void change_address(uint8_t new_address);

    void _wait_for_flash();

    void enable_interrupt(bool interrupt = true);
	
	// I2C helpers
	// Returns the nuymber of bytes read into readBuf.
    uint8_t i2c_rdwr_2(uint8_t* data, uint8_t dataLength, uint8_t* readBuf, uint8_t readLength);
	// Writes a value into a register.
    void i2cWriteRegValue(uint8_t reg, uint8_t value);

    bool get_interrupt();

	// Set all colour components
    void set_rgbw(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
	
	// Set colour components
    void set_red(uint8_t value);
    void set_green(uint8_t value);
    void set_blue(uint8_t value);
    void set_white(uint8_t value);
	
	// Read and store the state of the trackball
	// Returns true if successful
	bool read(self);
	
	// Get the stored state of the trackball
	uint8_t getLeft();
	uint8_t getRight();
	uint8_t getUp();
	uint8_t getDown();
	bool getEnter();
	bool getEnterState();
};
