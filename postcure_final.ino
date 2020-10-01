/*
   Using acceleration and flex sensors changes in posture are detected
   and trigger notifications via vibration motors and correction advise
   using servos which pull strings attached to the shoulder.
*/

#include <Servo.h>
#include <Wire.h>
//#include <Sparkfun_DRV2605L.h> //SparkFun Haptic Motor Driver Library
#include "SparkFun_MMA8452Q.h"

// sets information to debugging, i.e. printing information to std. output
bool verbosity = true;
int loopDelay = 500;  // 1000 = 1sec.
int servoDelay = 250; // delay to send signal to servo

Servo servo_1;    //left outer
Servo servo_2;    //left inner
Servo servo_3;    //middle
Servo servo_4;    //right inner
Servo servo_5;    //right outer

MMA8452Q accel;

int accReadingX = 0;
int accReadingY = 0;

const int acc_thresh_value_x = -100;    //slouching sideways
const int acc_thresh_value_y = -900;    //slouching forward
const int threshSlouchingLeft = -150;
const int threshSlouchingRight = 90;
//const int acc_thresh_value_z = 500;

// TODO: adjust thresholds for flex sensor
const int flex_left_thresh_value = 500;
const int flex_right_thresh_value = 500;
// const int flex_3_thresh_value = 35;

const int servoPin = 8;
//extra motor

const int switch_1 = 2;
const int switch_2 = 3;
const int switch_3 = 4;

int pos = 10;

int mode1 = 0;   //first switch high -- only motors
int mode2 = 0;   //second switch high -- only vibration
int mode3 = 0;   //third switch high -- motors and vibration

int flexSensorPin_left = A0;   // pin of flex sensor on the left side
int flexSensorPin_right = A1;   // pin of flex sensor on the right side
// int flexSensorPin_xxx = A2;   // pin of flex sensor on the xxx side

int flexSensorReading_left = 0;
int flexSensorReading_right = 0;
// int flexSensorReading_xxx = 0;

// pins for Vibration Motor
const int motorPin_1 = 6;
const int motorPin_2 = 7;
// const int motorPin_3 = 8;

void setup() {
  // setup code run once at the start

  // setup vibration motor output
  pinMode(motorPin_1, OUTPUT);
  pinMode(motorPin_2, OUTPUT);
  // pinMode(motorPin_3, OUTPUT);

  // setup switch input to select notifications
  pinMode(switch_1, INPUT);
  pinMode(switch_2, INPUT);
  pinMode(switch_3, INPUT);

  Serial.begin(9600);  // seting port for output
  Wire.begin();

  if (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    Serial.println();
    while (1);
  } else if (accel.begin() == true) {
    accel.setScale(SCALE_2G);  // scale ranges from 2G to 8G while 2G the most sensible
    Serial.println("Accelerometer started");
    Serial.println();
  }

  servo_1.attach(9);
  servo_2.attach(10);
  servo_3.attach(11);
  servo_4.attach(12);
  servo_5.attach(13);

  resetServosUprightPosition();  // put servos in neutral position
  turnOffMotors();  // stop all vibration motors

}


void loop() {

  // update mode settings
  mode1 = digitalRead(switch_1);  // pulling mode
  mode2 = digitalRead(switch_2);  // vibration mode
  mode3 = digitalRead(switch_3);  // TODO: what to do here?
  mode1 = 1;  // REMOVE
  mode2 = 1;  // REMOVE

  // read acceleration
  accReadingX = accel.getX();
  accReadingY = accel.getY();

  // read flex sensors
  //flexSensorReading_left = analogRead(flexSensorPin_left);
  //flexSensorReading_right = analogRead(flexSensorPin_right);

  /*
      detecting forward slouching using accelerometer and flex sensors
      if slouching is detected, then run servos and vibration motors
      in both cases the dedicated functions will decide if the action
      is executed, based on the mode
  */

  /*
    if (isSlouchingForwardLeft(accReadingY, flexSensorReading_left)) {
    // pull right outside servos as they're opposite to the left side
    pullFromSlouchingLeftForward(mode1);
    // vibrate middle motors
    vibrateMotor(mode2, motorPin_2);
    }

    else if (isSlouchingForwardRight(accReadingY, flexSensorReading_right)) {
    // pull right outside servos as they're opposite to the left side
    pullFromSlouchingRightForward(mode1);
    // vibrate middle motors
    vibrateMotor(mode2, motorPin_1);
    }

    else if (isSlouchingForward(accReadingY)) {
    // pull with middle servos
    pullFromSlouchingForward(mode1);
    // vibrate middle motors
    vibrateMotor(mode2, motorPin_1);
    vibrateMotor(mode2, motorPin_2);
    // vibrateMotor(mode2, motorPin_3);
    }
  */
  if (accReadingY >= acc_thresh_value_y && accReadingX >= threshSlouchingRight) {
    Serial.println("slouching RIGHT!");
    // pull right outside servos as they're opposite to the left side
    pullFromSlouchingRightForward(mode1);
    // vibrate middle motors
    vibrateMotor(mode2, motorPin_1);

  } else if (accReadingY >= acc_thresh_value_y && accReadingX <= threshSlouchingLeft) {
    Serial.println("slouching LEFT!");
    // pull right outside servos as they're opposite to the left side
    pullFromSlouchingLeftForward(mode1);
    // vibrate middle motors

    vibrateMotor(mode2, motorPin_2);
  } else if (accReadingY >= acc_thresh_value_y) {
    Serial.println("slouching FORWARD!");
    // pull with middle servos
    pullFromSlouchingForward(mode1);
    // vibrate middle motors
    vibrateMotor(mode2, motorPin_1);
    vibrateMotor(mode2, motorPin_2);
  }

  else {
    resetServosUprightPosition();
  }

  if (verbosity) Serial.println();
  // wait for some time to execute the next loop
  delay(loopDelay);
  turnOffMotors();  // TODO: find better position
  // TODO: send all sensor reading to phone via bluetooth

}

/*
   Functions for detecting which kind of slouching
*/

bool isSlouchingForward(int reading) {
  bool slouching = false;
  if (reading >= acc_thresh_value_y) {
    slouching = true;

    // printing the detection for debugging cases
    if (verbosity) {
      Serial.print("Currently slouching FORWARD: \t\t");
      Serial.print(slouching);
      Serial.print("\t");
      Serial.print(reading);
      Serial.println();
    }
  }

  return slouching;
}

/*
   detecting slouching to the front LEFT
   takes readings from the accelerometer
   and the flex sensor on the left side
*/
bool isSlouchingForwardLeft(int readingAcc, int readingFlex) {
  bool slouching = false;
  // if (readingAcc >= acc_thresh_value_y && readingFlex > flex_left_thresh_value) {
  if (readingFlex > flex_left_thresh_value) {
    slouching = true;

    // printing the detection for debugging cases
    if (verbosity) {
      Serial.print("Currently slouching LEFT: \t");
      Serial.print(slouching);
      Serial.print("\t");
      Serial.print(readingAcc);
      Serial.print("\t");
      Serial.print(readingFlex);
      Serial.println();
    }
  }

  return slouching;
}

/*
   detecting slouching to the front RIGHT
   takes readings from the accelerometer
   and the flex sensor on the left side
*/
bool isSlouchingForwardRight(int readingAcc, int readingFlex) {
  bool slouching = false;
  // if (readingAcc >= acc_thresh_value_y && readingFlex > flex_right_thresh_value) {
  if (readingFlex > flex_right_thresh_value) {
    slouching = true;

    // printing the detection for debugging cases
    if (verbosity) {
      Serial.print("Currently slouching RIGHT: \t");
      Serial.print(slouching);
      Serial.print("\t");
      Serial.print(readingAcc);
      Serial.print("\t");
      Serial.print(readingFlex);
      Serial.println();
    }
  }
  return slouching;
}

/*
   Functions for activating servos based on slouching position
*/

void resetServosUprightPosition() {
  // sit upright
  servo_1.write(10);
  servo_2.write(10);
  servo_3.write(10);
  servo_4.write(10);
  servo_5.write(10);

  delay(servoDelay);
}

void pullFromSlouchingForward(int mode1) {
  // pull into an upright position from a forward slouching position
  if (mode1 == HIGH) {
    if (verbosity) Serial.println("Pulling from slouching FORWARD ...");
    servo_1.write(170);
    servo_2.write(170);
    delay(servoDelay);
  }
  resetServosUprightPosition();
}

void pullFromSlouchingLeftForward(int mode1) {
  // pull into an upright position from a left-forward slouching position
  if (mode1 == HIGH) {
    if (verbosity) Serial.println("Pulling from slouching LEFT forward ...");
    //servo_1.write(170);
    servo_2.write(170);
    delay(servoDelay);
  }
  resetServosUprightPosition();
}

void pullFromSlouchingRightForward(int mode1) {
  // pull into an upright position from a right-forward slouching position
  if (mode1 == HIGH) {
    if (verbosity) Serial.println("Pulling pull from slouching RIGHT forward ...");
    servo_1.write(170);
    //servo_2.write(170);
    delay(servoDelay);
  }
  resetServosUprightPosition();
}

void pullFromLeaningLeftSide(int mode1) {
  // pull into an upright position from a left-side slouching position
  if (mode1 == HIGH) {
    servo_1.write(170);
    servo_2.write(170);
    servo_4.write(170);
    servo_5.write(170);
    delay(servoDelay);
  }
}

void pullFromLeaningRightSide(int mode1) {
  // pull into an upright position from a right-side slouching position
  if (mode1 == HIGH) {
    servo_1.write(170);
    servo_2.write(170);
    delay(servoDelay);
  }
}

void vibrateMotor(int mode2, int location) {
  // vibrate motor indicated by location if vibration mode is on
  if (mode2) {
    digitalWrite(location, HIGH);
    Serial.print("Vibration Motor Turned On: \t");
    if (location == motorPin_2) {
      Serial.print("RIGHT");
    } else {
      Serial.print("LEFT");
    }
    Serial.println();
  }
}

void turnOffMotors() {
  digitalWrite(motorPin_1, LOW);
  digitalWrite(motorPin_2, LOW);
  // digitalWrite(motorPin_3, LOW);
}
