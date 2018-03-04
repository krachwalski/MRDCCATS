const float pi = 3.141596;

#include <Servo.h>

//Xbox Controller **********************************************************************
#include <XBOXRECV.h>
// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

USB Usb;
XBOXRECV Xbox(&Usb);

//Motor**********************************************************************

//front left
const int en1 = 2;
//const int dir1 = 23;

//front right
const int en2 = 3;
//const int dir2 = 22;

//back left
const int en3 = 11;
//const int dir3 = 29;

//back right
const int en4 = 5;
//const int dir4 = 24;

//5th one (hopper movement)
const int en5 = 6;
//const int dir5 = 36;


//6th one (arm)
//const int en6 = 7;
//const int dir6 = 37;

//Servos*****************************************************************
Servo wall;
Servo arm;
const int wall_pin = 8;
const int arm_pin = 12;
bool wall_state; //setting wall state to 1 to indicate up
int arm_servo_state; //position of arm servo

//if RT or LT is pressed
int analogR2;
int analogL2;
int arm_motor_state = 0; //0 is off 1 is on
int initial_pos = 80;

void setup() {
  pinMode(en1, INPUT);
  pinMode(en2, INPUT);
  pinMode(en3, INPUT);
  pinMode(en4, INPUT);
  pinMode(en5, INPUT);
//  pinMode(en6, INPUT);
  pinMode(wall_pin, INPUT);
  pinMode(arm_pin, INPUT);

  delay(1000);

  pinMode(en1, OUTPUT);
  //pinMode(dir1, OUTPUT);

  pinMode(en2, OUTPUT);
  //pinMode(dir2, OUTPUT);

  pinMode(en3, OUTPUT);
  //pinMode(dir3, OUTPUT);

  pinMode(en4, OUTPUT);
  //pinMode(dir4, OUTPUT);

  pinMode(en5, OUTPUT);
  //pinMode(dir5, OUTPUT);

//  pinMode(en6, OUTPUT);
  //pinMode(dir6, OUTPUT);

  pinMode(wall_pin, OUTPUT);
  pinMode(arm_pin, OUTPUT);

  wall.attach(wall_pin, 900, 2100);
  wall.write(initial_pos); //set servo to initial position
  wall_state = 1; //set state to high

  arm.attach(arm_pin, 900, 2100);
  arm_servo_state = 90;
  arm.write(arm_servo_state);


  // enable diagnostic output
  Serial.println("\n\n\n");
  Serial.println("Ready.");

  Serial.begin(115200);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nXbox Wireless Receiver Library Started"));
}

void loop() {

  Usb.Task();



  if (Xbox.XboxReceiverConnected) {
    analogL2 = Xbox.getButtonPress(L2);
    analogR2 = Xbox.getButtonPress(R2);
    //move_hopper((analogR2 - analogL2) * 0.5);

    //arm_motion();

    move_all_motors();

    if (Xbox.getButtonClick(A)) {   //arm up
      arm_servo_state = 0;
      Serial.print("\r\nThe servo State is" + arm_servo_state);
      if (arm_servo_state < 0) {
        arm_servo_state = 0;
      }
      arm.write(arm_servo_state);
    }

    if (Xbox.getButtonClick(B)) {    //arm down
      arm_servo_state = 90;
      //Serial.print(F("\r\nThe servo State is" + arm_servo_state"));

      if (arm_servo_state > 180) {
        arm_servo_state = 180;
      }
      arm.write(arm_servo_state);
    }
    //    Serial.println(arm_servo_state);

    if (Xbox.getButtonClick(A)) {
//      release_wall();
    }

  } else {

    move_motor1(0);
    move_motor2(0);
    move_motor3(0);
    move_motor4(0);
    //move_hopper(0);
//    move_arm(0);
  }
}
int motor1 = en1;
int motor2 = en2;
int motor3 = en3;
int motor4 = en4;


//move left and right

//mapping the signal from the joystick to actual speeds
int map_analog(float val) {
  int ret_val;
  //  deadzone between 0 and 3200
  if (val > 100 && val < 4000) {
    ret_val = 0;
  } else {
    ret_val = map(val, -32768, 32767, -100, 100);
  }
  return ret_val;
}

void move_all_motors() {

  //left joystick
  float drive;
  float strafe;
  float rot;
  int drive_spd;
  int strafe_spd;
  int rot_spd;

  //analog signal for the left joystick, this is to move forward and back
  drive = Xbox.getAnalogHat(LeftHatY);
  strafe = Xbox.getAnalogHat(LeftHatX);
  rot = Xbox.getAnalogHat(RightHatX);


  drive_spd = map_analog(drive);
  strafe_spd =  map_analog(strafe);
  rot_spd =  map_analog(rot);


  move_motor1(-drive_spd + strafe_spd - rot_spd);
  move_motor2(-drive_spd - strafe_spd + rot_spd);
  move_motor3(-drive_spd - strafe_spd - rot_spd);
  move_motor4(-drive_spd + strafe_spd + rot_spd);

}

void move_left_right(int spd23, int spd14) {
  move_motor1(spd14);
  move_motor2(spd23);
  move_motor3(spd23);
  move_motor4(spd14);
}


//move forward and backward
void move_motors12(int spd) {
  move_motor1(spd);
  move_motor2(spd);
}

void move_motors34(int spd) {
  move_motor3(spd);
  move_motor4(spd);
}

int newSpeed(int spd) {
  if (spd == 0) {
    spd = 150;
  } else if (spd > 0) {
    spd = 150 + spd / (254 - 150);
  } else {
    spd = 150 - spd / (254 - 150);
  }
}
//backward full speed is -255, forward full speed is 255
void move_motor1(int spd) {
  spd = newSpeed(spd);
  analogWrite(motor1, spd);
  //if (spd >= 0) {
  //  analogWrite(en1, spd);
  //  digitalWrite(dir1, HIGH);
  //} else if (spd < 0) {
  // analogWrite(en1, -spd);
  //  digitalWrite(dir1, LOW);
  //}
}

void move_motor2(int spd) {
  spd = newSpeed(spd);

  analogWrite(motor2, spd);
  //0-255 is sent to spd
  //150 is stop
  //254 is full forward
  //70 is full backward

  //if (spd >= 0) {
  //  analogWrite(en2, spd);
  //  digitalWrite(dir2, HIGH);
  //} else if (spd < 0) {
  //  analogWrite(en2, -spd);
  // digitalWrite(dir2, LOW);
  //}
}

void move_motor3(int spd) {
  spd = newSpeed(spd);
  analogWrite(motor3, spd);
  //if (spd >= 0) {
  //  analogWrite(en3, spd);
  //  digitalWrite(dir3, HIGH);
  //} else if (spd < 0) {
  //  analogWrite(en3, -spd);
  //  digitalWrite(dir3, LOW);
  //}
}

void move_motor4(int spd) {
  spd = newSpeed(spd);
  analogWrite(motor4, spd);

  //if (spd >= 0) {
  //  analogWrite(en4, spd);
  //  digitalWrite(dir4, HIGH);
  //} else if (spd < 0) {
  //  analogWrite(en4, -spd);
  //  digitalWrite(dir4, LOW);
  //}
}

void move_hopper(int spd) {
  if (spd >= 0) {
    analogWrite(en5, spd);
  //  digitalWrite(dir5, HIGH);
  } else if (spd < 0) {
    analogWrite(en5, -spd);
//    digitalWrite(dir5, LOW);
  }
}

//void move_arm(int spd) {
//  if (spd >= 0) {
//    analogWrite(en6, spd);
//    digitalWrite(dir6, HIGH);
//  } else if (spd < 0) {
//    analogWrite(en6, -spd);
//    digitalWrite(dir6, LOW);
//  }
//}

//void arm_motion() {
///  if (Xbox.getButtonClick(L1)) {
 //   Serial.println("l1");
 //   if (arm_motor_state == 0) {
//      move_arm(50);
//      arm_motor_state = 1;
 //   } else {
//      move_arm(0);
  //    arm_motor_state = 0;
    //}
 // }

//}
