# HeavyHusky
Object following code made with ChatGTP 
#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include <NewPing.h>

// Define SoftwareSerial pins for HuskyLens
#define HUSKY_TX 2
#define HUSKY_RX 3

SoftwareSerial huskySerial(HUSKY_TX, HUSKY_RX); // RX, TX
HUSKYLENS huskylens;

// Set the pin out for Motor 1
int RPWM_M1 = 5; // RPWM for Motor 1
int LPWM_M1 = 6; // LPWM for Motor 1
int L_EN_M1 = 7; // L_EN for Motor 1
int R_EN_M1 = 8; // R_EN for Motor 1

// Set the pin out for Motor 2
int RPWM_M2 = 9; // RPWM for Motor 2
int LPWM_M2 = 10; // LPWM for Motor 2
int L_EN_M2 = 11; // L_EN for Motor 2
int R_EN_M2 = 12; // R_EN for Motor 2

// Define pins for the ultrasonic sensor
#define TRIGGER_PIN 4
#define ECHO_PIN 13

// Define maximum distance (in cm) for the ultrasonic sensor
#define MAX_DISTANCE 200

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

const int desiredDistance = 30; // Desired distance from the object in cm

int lastLeftSpeed = 0;
int lastRightSpeed = 0;

void setup() {
  // Initialize all pins to output
  pinMode(RPWM_M1, OUTPUT);
  pinMode(LPWM_M1, OUTPUT);
  pinMode(L_EN_M1, OUTPUT);
  pinMode(R_EN_M1, OUTPUT);
  pinMode(RPWM_M2, OUTPUT);
  pinMode(LPWM_M2, OUTPUT);
  pinMode(L_EN_M2, OUTPUT);
  pinMode(R_EN_M2, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(1000); // Wait for a second
  Serial.begin(9600); // Initialize hardware serial for debugging

  huskySerial.begin(9600); // Initialize SoftwareSerial for HuskyLens

  // Enable "Right" and "Left" movement on the H-Bridge for both motors
  digitalWrite(R_EN_M1, HIGH);
  digitalWrite(L_EN_M1, HIGH);
  digitalWrite(R_EN_M2, HIGH);
  digitalWrite(L_EN_M2, HIGH);

  while (!huskylens.begin(huskySerial)) {
    delay(100);
  }
}

void loop() {
  delay(50); // Short delay to avoid overloading the serial communication

  // Get distance from ultrasonic sensor
  unsigned int distance = sonar.ping_cm();

  // Ensure distance measurement is valid
  if (distance == 0 || distance > MAX_DISTANCE) {
    distance = MAX_DISTANCE;
  }

  if (!huskylens.request()) {
    stopMoving();
  } else if (!huskylens.isLearned()) {
    stopMoving();
  } else if (!huskylens.available()) {
    stopMoving();
  } else {
    while (huskylens.available()) {
      HUSKYLENSResult result = huskylens.read();

      // Move the motors based on the object position and distance
      if (result.command == COMMAND_RETURN_BLOCK) {
        int center = 160; // Assuming a 320x240 screen resolution
        int error = result.xCenter - center;

        // Adjust speed based on the error to smoothly follow the object
        int turnSpeed = map(abs(error), 0, center, 0, 128);
        if (turnSpeed > 128) turnSpeed = 128; // Ensure max speed does not exceed 128

        if (distance < desiredDistance - 5) {
          // Object is too close, move backward immediately
          moveBackward();
        } else if (error < -10) {
          // Object is left of center, turn left to center it
          moveLeft(turnSpeed);
        } else if (error > 10) {
          // Object is right of center, turn right to center it
          moveRight(turnSpeed);
        } else {
          // Object is in the center and at the correct distance
          if (distance > desiredDistance + 5) {
            // Object is too far, move forward
            moveForward();
          } else {
            // Object is at the desired distance, stop
            stopMoving();
          }
        }
      }
    }
  }
}

void moveForward() {
  setMotorSpeed(128, 128);
}

void moveBackward() {
  setMotorSpeed(-128, -128);
}

void moveLeft(int speed) {
  setMotorSpeed(-speed, speed);
}

void moveRight(int speed) {
  setMotorSpeed(speed, -speed);
}

void stopMoving() {
  setMotorSpeed(0, 0);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Check if the new speeds are different from the last speeds
  if (leftSpeed != lastLeftSpeed || rightSpeed != lastRightSpeed) {
    if (leftSpeed >= 0) {
      analogWrite(RPWM_M1, leftSpeed); // Left motor forward
      analogWrite(LPWM_M1, 0);
    } else {
      analogWrite(RPWM_M1, 0);
      analogWrite(LPWM_M1, -leftSpeed); // Left motor backward
    }

    if (rightSpeed >= 0) {
      analogWrite(RPWM_M2, rightSpeed); // Right motor forward
      analogWrite(LPWM_M2, 0);
    } else {
      analogWrite(RPWM_M2, 0);
      analogWrite(LPWM_M2, -rightSpeed); // Right motor backward
    }

    // Save the last speeds
    lastLeftSpeed = leftSpeed;
    lastRightSpeed = rightSpeed;
  }
}
