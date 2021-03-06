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
  digitalWrite(13, ledState = !ledState); // Heartbeat
  
  if (!digitalRead(TRACKBALL_INT_PIN)) {
    trackball->read();
    
    // Print directional data
    Serial.print("L\t");
    Serial.println(trackball->getLeft());
    Serial.print("R\t");
    Serial.println(trackball->getRight());
    Serial.print("U\t");
    Serial.println(trackball->getUp());
    Serial.print("D\t");
    Serial.println(trackball->getDown());
    
    Serial.print("x\t");
    Serial.println(trackball->getX());
    Serial.print("y\t");
    Serial.println(trackball->getY());
    
    Serial.print("sw\t");
    Serial.println(trackball->getSwitch());
    Serial.print("sw_state \t");
    Serial.println(trackball->getSwitchState());
    Serial.println();

    // Set colour according to movement
    uint8_t
      r = trackball->getRight() ? 255 : 0,
      g = trackball->getLeft() ? 255 : 0,
      b = trackball->getUp() ? 255 : 0,
      w = trackball->getDown() ? 255 : 0;

    trackball->setRGBW(r, g, b, w);
  }
  
  delay(50);
}
