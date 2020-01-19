#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>



namespace {
  const uint8_t NONE        = 128;
  const uint8_t TURN180     = 129;
  const uint8_t TOGGLEDOORS = 130;
  const uint8_t OPENDOORS   = 131;
  const uint8_t CLOSEDOORS  = 132;

  const uint8_t OPEN        = 20;
  const uint8_t CLOSED      = 115;

  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

  Adafruit_DCMotor *motors[4] = {AFMS.getMotor(4), AFMS.getMotor(3), AFMS.getMotor(1), AFMS.getMotor(2)};
  Servo servo1;
  Servo servo2;

  void setWheel(uint8_t p) {
    if (p & 0x80) {
      return;
    }
    motors[p & 0x03]->setSpeed(((p >> 4) & 0x07) * 32);
    motors[p & 0x03]->run(((p >> 2) & 0x03) + 1);
  }
  void allWheels(uint8_t mode, uint8_t spd) {
    setWheel(0 | mode << 2 | spd << 4);
    setWheel(1 | mode << 2 | spd << 4);
    setWheel(2 | mode << 2 | spd << 4);
    setWheel(3 | mode << 2 | spd << 4);
  }

  void setDoors(int angle) {
    if (angle < OPEN) angle = OPEN;
    if (angle > CLOSED) angle = CLOSED;

    servo1.write(angle);
    servo2.write(180 - angle);
  }

  void turn(int degs) {
    auto mode = degs < 0 ? 1 : 0;

    setWheel((0 + mode) & 3 | 0 << 2| 7 << 4);
    setWheel((2 + mode) & 3 | 0 << 2| 7 << 4);
    setWheel((1 + mode) & 3 | 1 << 2| 7 << 4);
    setWheel((3 + mode) & 3 | 1 << 2| 7 << 4);

  }


}


void setup() {
  Serial1.begin(250000);

  servo1.attach(9);
  servo2.attach(10);

  AFMS.begin();

  setDoors(CLOSED);
}

void loop() {
  static volatile unsigned long long lastMillis;
  static volatile uint8_t maneuver = NONE;
  static volatile int doorAngle = 90;

  if (Serial1.available()) {
    maneuver = Serial1.read();
    setWheel(maneuver);
    lastMillis = millis();
  }

  // MANEUVER ACTIONS

  switch (maneuver) {
  case TURN180:
    // stop turn after 500 millis
    if (millis() - lastMillis > 500) {
      maneuver = NONE;
    } 
    else {
      turn(180);
    }
    break;

  case OPENDOORS:
    // control number of millis per degree
    if (millis() - lastMillis >= 10) {
      setDoors(doorAngle);
      doorAngle--;
      lastMillis = millis();
      if (doorAngle < OPEN) {
        maneuver = NONE;
      }
    } 
    break;

  case CLOSEDOORS: 
    if (millis() - lastMillis >= 10) {
      setDoors(doorAngle);
      doorAngle++;
      lastMillis = millis();
      if (doorAngle > CLOSED) {
        maneuver = NONE;
      }
    }
    break;

  default:
    if (millis() - lastMillis > 300) {
      allWheels(2, 0);
      lastMillis = millis();
    }
    break;
  }
}
