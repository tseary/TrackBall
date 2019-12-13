
#include <Wire.h>


/*
From https://shop.pimoroni.com/products/trackball-breakout

Notes
The trackball breakout only works reliably with I2C speeds up to 250kHz
due to limitations in the Raspberry Pi's clock-stretching implementation
*/

const uint8_t TRACKBALL_I2C_ADDRESS = 0x0A;

void setup() {
	Wire.begin(); // join i2c bus (address optional for master)
}

byte x = 0;

void loop() {
	Wire.beginTransmission(TRACKBALL_I2C_ADDRESS); // transmit to device #4
	Wire.write("x is ");        // sends five bytes
	Wire.write(x);              // sends one byte  
	Wire.endTransmission();    // stop transmitting

	x++;
	delay(500);
}
