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

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


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
  pinMode(wall_pin, INPUT);
  pinMode(arm_pin, INPUT);

  delay(1000);

  pinMode(wall_pin, OUTPUT);
  pinMode(arm_pin, OUTPUT);

  wall.attach(wall_pin, 900, 2100);
  wall.write(initial_pos); //set servo to initial position
  wall_state = 1; //set state to high

  arm.attach(arm_pin, 900, 2100);
  arm_servo_state = 90;
  arm.write(arm_servo_state);

  motor1.attach(2,1000,2000);
  motor2.attach(3,1000,2000);
  motor3.attach(11,1000,2000);
  motor4.attach(5,1000,2000);


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



//move left and right

//mapping the signal from the joystick to actual speeds

//shaun did this and im not messing with it
int map_analog(float val) {
  int ret_val;
  //  deadzone between 0 and 3200
  if (val > 100 && val < 4000) {
    ret_val = 0;
  } else {
    ret_val = map(val, -32768, 32767, 0,180);
  }
  return ret_val;
}

void move_all_motors() {
  Serial.print("Move all motors called");
  //left joystick
  float drive;
  float strafe;
  float rot;
  int drive_spd;
  int strafe_spd;
  int rot_spd;

  //analog signal for the left joystick, this is to move forward and back VALS 0 - 180
  drive = Xbox.getAnalogHat(LeftHatY); //left joystick up and down
  strafe = Xbox.getAnalogHat(LeftHatX);
  rot = Xbox.getAnalogHat(RightHatX);
  

  drive_spd = map_analog(drive);
  strafe_spd =  map_analog(strafe);
  rot_spd =  map_analog(rot);
  /*if(drive_spd < 105 && drive_spd > 75) {
    drive_spd = 90;             //deadzone for stick in place
  } else if (drive_spd < 75) { //drive_spd 0-75
    drive_spd = drive_spd + 15; //now 15-90
    drive_spd = 60 + drive_spd/3; //now 65-90 
  } else { //105- 180
    drive_spd = drive_spd - 15; //90-165
    drive_spd = 60 + drive_spd/3; //90-115
  }
  if(strafe_spd < 105 && strafe_spd > 75) {
    strafe_spd = 90;            //deadzone for stick in place
  } else if (strafe_spd < 75) { //drive_spd 0-75
    strafe_spd = strafe_spd + 15; //now 15-90
    strafe_spd = 60 + strafe_spd/3; //now 65-90 
  } else { //105- 180
    strafe_spd = strafe_spd - 15; //90-165
    strafe_spd = 60 + strafe_spd/3; //90-115
  }
  */
  if(drive_spd < 150 && drive_spd > 30) {
    drive_spd = 90;
  }
  if(strafe_spd < 150 && strafe_spd > 30) {
    strafe_spd = 90;
  }
  if(rot_spd < 150 && rot_spd > 30) {
    rot_spd = 90;
  }
  drive_spd = (drive_spd - 90)/6;
  strafe_spd = (strafe_spd - 90)/6;
  rot_spd = (rot_spd - 90)/6;
  //move_motor1(-drive_spd + strafe_spd - rot_spd);
  //move_motor2(-drive_spd - strafe_spd + rot_spd);
  //move_motor3(-drive_spd - strafe_spd - rot_spd);
  //move_motor4(-drive_spd + strafe_spd + rot_spd);
  move_motor1(90 + drive_spd + strafe_spd + rot_spd);
  move_motor2(90 - drive_spd + strafe_spd + rot_spd);
  move_motor3(90 + drive_spd - strafe_spd + rot_spd);
  move_motor4(90 - drive_spd - strafe_spd + rot_spd);

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
    spd = 190;
  } else if (spd > 0) {
    spd = 190 + spd / (254 - 190);
  } else {
    spd = 150 - spd / (190 - 100);
  }
}
//backward full speed is -255, forward full speed is 255
void move_motor1(int spd) {
  Serial.print("BEFORE speed is");
  Serial.print(spd);
  Serial.print("\n");
  motor1.write(spd); //spd is between 0 (full reverse) 90 (stop) 180 (full fullward)
  Serial.print("AFTER speed is");
  Serial.print(spd);
  Serial.print("\n");
}

void move_motor2(int spd) {
  Serial.print("BEFORE speed is ");
  Serial.print(spd);
  Serial.print("\n");
  motor2.write(spd);
  Serial.print("AFTER speed is "); 
  Serial.print(spd);
  Serial.print("\n");

}

void move_motor3(int spd) {
  Serial.print("BEFORE speed is ");
  Serial.print(spd);
  Serial.print("\n");
  motor3.write(spd);
  Serial.print("AFTER speed is ");
  Serial.print(spd);
  Serial.print("\n");
}

void move_motor4(int spd) {
  Serial.print("BEFORE speed is");
  Serial.print(spd);
  Serial.print("\n");  
  motor4.write(spd);
  Serial.print("AFTER speed is");
  Serial.print(spd);
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


