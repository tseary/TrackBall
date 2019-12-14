#include <TrackBall.h>
#include <Wire.h>

/*
  From https://shop.pimoroni.com/products/trackball-breakout

  Notes
  The trackball breakout only works reliably with I2C speeds up to 250kHz
  due to limitations in the Raspberry Pi's clock-stretching implementation
*/

const uint8_t TRACKBALL_INT_PIN = 4;

TrackBall *trackball = NULL;

bool ledState = false;

void setup() {
  Wire.begin(); // join i2c bus (address optional for master)

  pinMode(13, OUTPUT);
  digitalWrite(13, ledState = true);
  
  Serial.begin(115200);
  
  trackball = new TrackBall(TrackBall::I2C_ADDRESS, TRACKBALL_INT_PIN);
}

void loop() {
//  Serial.println(millis());
  digitalWrite(13, ledState = !ledState); // Heartbeat
  
  Serial.print("success?\t");
  Serial.println(trackball->read());
  
  uint8_t
    r = trackball->getRight() ? 255 : 0,
    g = trackball->getLeft() ? 255 : 0,
    b = trackball->getUp() ? 255 : 0,
    w = trackball->getDown() ? 255 : 0;
  
  trackball->setRGBW(r, g, b, w);
  delay(1);
}
