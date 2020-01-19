#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SoftwareSerial.h>
#include <Servo.h>

#define LEFTWHEEL 0
#define RIGHTWHEEL 1
#define FRONTWHEEL 0
#define BACKWHEEL 2

#define MFWD 0
#define MBCK 1
#define MBRK 2
#define MREL 3
#define TOPSPEED 7

#define TURN180 129

#define TOGGLEDOORS 130
#define OPENDOORS 131
#define CLOSEDOORS 132
#define OPEN 20
#define CLOSED 115

#define COLLISIONAVOIDANCE false

#define NONE 0

namespace {
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();

  Adafruit_DCMotor *motors[4] = {AFMS.getMotor(4), AFMS.getMotor(3), AFMS.getMotor(1), AFMS.getMotor(2)};
  Servo servo1;
  Servo servo2;

  struct control_packet {
    uint8_t wheel : 2;
    uint8_t mode  : 2;
    uint8_t speed : 3;
    bool special  : 1;
  };

  // spd is 0-7
  void setWheel(unsigned int wheel, unsigned int mode, unsigned int spd) {
    motors[wheel % 4]->setSpeed(spd * 32);
    motors[wheel % 4]->run(mode + 1);
  }
  void setWheel(control_packet p) {
    motors[p.wheel % 4]->setSpeed(p.speed * 32);
    motors[p.wheel % 4]->run(p.mode + 1);
  }

  void setDoors(int angle) {
    if (angle < OPEN) angle = OPEN;
    if (angle > CLOSED) angle = CLOSED;

    servo1.write(angle);
    servo2.write(180 - angle);
  }

  void turn(int degs) {
    int mode = degs < 0;
    setWheel(FRONTWHEEL + mode, MFWD, TOPSPEED);
    setWheel(BACKWHEEL + mode, MFWD, TOPSPEED);

    setWheel(FRONTWHEEL + mode + 1, MBCK, TOPSPEED);
    setWheel(BACKWHEEL + mode + 1, MBCK, TOPSPEED);

  }


  void allWheels(int mode, int spd) {
    setWheel(0, mode, spd);
    setWheel(1, mode, spd);
    setWheel(2, mode, spd);
    setWheel(3, mode, spd);
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
  static control_packet p;
  static volatile unsigned long long lastMillis;
  static volatile int maneuver = NONE;
  static volatile int doorAngle = 90;

  if (Serial1.available()) {
    Serial1.readBytes((uint8_t *) &p, 1);
    if (p.special) {
      maneuver = *((uint8_t *) &p);
    } 
    else { // normal wheel control packet
      setWheel(p);
    }
    lastMillis = millis();
  }

  // MANEUVER ACTIONS

  if (maneuver == TURN180) {
    // turn, stop after 500 millis
    if (millis() - lastMillis > 500) {
      maneuver = NONE;
    } else {
      turn(180);
    }
  }
  // open and close doors, stop at 0 or 90 degrees
  if (maneuver == OPENDOORS && millis() - lastMillis >= 10) {
    setDoors(doorAngle);
    doorAngle--;
    if (doorAngle < OPEN) {
      maneuver = NONE;
      lastMillis = millis();
    }
  } 
  else if (maneuver == CLOSEDOORS && millis() - lastMillis >= 10) {
    setDoors(doorAngle);
    doorAngle++;
    if (doorAngle > CLOSED) {
      maneuver = NONE;
      lastMillis = millis();

    }
  }

  // NO MANEUVER - RESET WHEELS EVERY 300 MILLIS

  if (millis() - lastMillis > 300 && maneuver == NONE) {
    allWheels(MREL, 0);
    lastMillis = millis();
  }
}
