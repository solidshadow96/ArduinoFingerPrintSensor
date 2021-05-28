#include <LowPower.h> //Lib used to put arduino to sleep while waiting
#include <Servo.h> //Servo controller lib
#include <Adafruit_Fingerprint.h> //Fingerprint sensor lib

Servo srvo; //Defines servo object as srvo

SoftwareSerial mySerial(2, 3); //Sets the serial ports used in controlling input from fingerprint sensor


//Begin Constants

const int servoPin = 9; //Pin used to control servo
const int buttonPin = 11; //Pin to measure button state (open or closed)
const int doorOpenAngle = 70; //Angle motor should move to to open door
const int doorClosedAngle = 0; //Angle motor should move to to close door

int buttonState = 0; //Button is 0 if open, and 1 if closed. Defined up here to make globaly available
bool doorOpen = true; //Current state of the door (on boot door is automatically opened, hence true)

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial); //Allows easy u

void setup(){ //This is run once at device boot
  //Serial.begin(9600); //Only used for debugging. Disable unless needed to conserve power
  
  digitalWrite(buttonPin, HIGH); //Solves floating pins
  srvo.attach(servoPin);
  delay(100);
  srvo.write(doorOpenAngle); //If motor is not in the open state, set it to be open
  delay(1300); //Provide time for motor to arrive at pos before sleep enables
  srvo.detach();
  
  finger.begin(57600); //Starts fingerprint sensor
}

void loop(){
  srvo.detach(); //Detaches servo to stop mixed signals from being sent
  LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF); //Puts device to sleep for X amount of time. This conserves ALOT of power, but sacrifices time between finger scans.
  delay(50);
  
  if(digitalRead(buttonPin) == LOW){ //If button is pressed
    if(doorOpen == true and srvo.read() > 60){ //If the door is open. Checks boolean and the servo state (incase servo is broken. Without this, it could lead to state looping)
      srvo.attach(servoPin);
      delay(100);
      srvo.write(doorClosedAngle);
      doorOpen = false; 
      delay(1500); //Without this delay, the motor would move a small amount, then the LowPower.powerDown would stop it, making it frozen.
    }else if(doorOpen == false and srvo.read() < 10){
      srvo.attach(servoPin);
      delay(100);
      srvo.write(doorOpenAngle);
      doorOpen = true;
      delay(1500);
    }
  }

  //Serial.println("FP: " + String(checkFP()));
  if(checkFP() == true){ //If fingerprint check function returns true, change the lock state
    if(doorOpen == true){
      srvo.attach(servoPin);
      delay(100);
      srvo.write(doorClosedAngle);
      doorOpen = false;
      delay(1500);
    }else if(doorOpen == false){
      srvo.attach(servoPin);
      delay(100);
      srvo.write(doorOpenAngle);
      doorOpen = true;
      delay(1500);
    }
  }
}


//Checks if 
bool checkFP() { //Modification of the sample fingerprint checking function. Returns true if match is found, else returns false.
  uint8_t p = finger.getImage();
  if (p != FINGERPRINT_OK)  return false;

  p = finger.image2Tz();
  if (p != FINGERPRINT_OK)  return false;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return false;
  return true;
}
