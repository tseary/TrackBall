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
  
  while(!Serial);
  delay(2000);
  
  Serial.begin(115200);
  
  trackball = new TrackBall(TrackBall::I2C_ADDRESS, TRACKBALL_INT_PIN);
}

void loop() {
//  Serial.println(millis());
  digitalWrite(13, ledState = !ledState); // Heartbeat

  trackball->setRed(255);
  delay(200);
  trackball->setRed(0);

  trackball->setGreen(255);
  delay(200);
  trackball->setGreen(0);

  trackball->setBlue(255);
  delay(200);
  trackball->setBlue(0);

  trackball->setWhite(255);
  delay(200);
  trackball->setWhite(0);

  delay(200);
}
