#include <Servo.h>
#include <SonarSRF08.h>
// STEERING AND MOTOR CONSTANTS
#define SERVO_PIN 10      // Steering servo pin
#define STRAIGHT_ANGLE 90 // 90 is when WHEELS ARE FACING FORWARD
#define MOTOR_PIN 11      // "Motor" servo pin 
#define MOTOR_NEUTRAL 1500 // Calibrated neutral value for ESC
// SENSOR CONSTANTS
#define IR_SENSOR_1 0
#define IR_SENSOR_2 1
#define IR_SENSOR_3 2
// RECIEVER CONSTANTS
#define CHAN1_RECEIVER 3 // Non-neutral value >0
#define CHAN2_RECEIVER 5// Non-neutral value 1900
#define ENCODER_INTERRUPT_PIN 2
// ULTRASONIC SENSOR CONSTANTS
#define FRONT_SENSOR_ADDRESS 0x70  // Address for front sensor 115
#define RIGHT_SENSOR_ADDRESS 0x73  // Address for front-right sensor 114
#define GAIN_REGISTER 0x09
#define LOCATION_REGISTER 0x8C
const unsigned short GAIN = 0x1F; //maximum gain
const unsigned short RANGE = 0x07; // 7 for 34 centimeters
// CODE CONSTANTS
volatile int encoderCount = 0;
int bufferCounter = 0;
int rcCounter = 0;
int sensorCounter = 1;
int rcFlag = 0;
Servo servo;
Servo motor;
SonarSRF08 frontSensor;
SonarSRF08 rightSensor;
String inputString = "";         // a string to hold incoming data
void turnRight(int turnAngle) {
  if((turnAngle > 30 || turnAngle < 0) == false) {
    servo.write(STRAIGHT_ANGLE + turnAngle);
  }
}
void turnLeft(int turnAngle) {
  if((turnAngle > 30 || turnAngle < 0) == false) {
    servo.write(STRAIGHT_ANGLE - turnAngle);
  }
}
void turnStraight() { // Turn NORTH
  servo.write(STRAIGHT_ANGLE);
}
void goForward(int velocity) { // Accelerate forward
  motor.writeMicroseconds(velocity);
}
void goBackward() { // Accelerate backwards
  motor.writeMicroseconds(1300);
}
void goNeutral() { // Neutral gear
  motor.writeMicroseconds(1500);
}
void setup() {
  frontSensor.connect(FRONT_SENSOR_ADDRESS, GAIN_REGISTER, LOCATION_REGISTER); // setup for front US sensor
  servo.attach(SERVO_PIN); // Tell servo module there is a servo on this pin
  servo.write(STRAIGHT_ANGLE); // Write to 'neutral' value (facing NORTH)s
  motor.attach(MOTOR_PIN); // Tell servo module there is a servo on this pin
  motor.writeMicroseconds(1500); // Write 'neutral' value (1500 = NEUTRAL GEAR)
  pinMode(IR_SENSOR_1, INPUT);
  pinMode(IR_SENSOR_2, INPUT);
  pinMode(IR_SENSOR_3, INPUT);
  
  pinMode(CHAN1_RECEIVER, INPUT);  // channel1 for the controller hsp receiver/transmitter
  pinMode(CHAN2_RECEIVER, INPUT);
  attachInterrupt(digitalPinToInterrupt(CHAN1_RECEIVER), rcActivate, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_INTERRUPT_PIN), wheelEncoder, RISING);
  Serial.begin(115200);
  inputString.reserve(50000);
}
void loop() {
  unsigned long pulseLength1; // Length of PULSE for WHEEL on remote
  unsigned long pulseLength2; // Length of PULSE for TRIGGER on remote
  pulseLength1 = pulseIn(CHAN1_RECEIVER, HIGH, 25000); // Read in the pulse
  pulseLength2 = pulseIn(CHAN2_RECEIVER, HIGH, 25000);
  
  if(rcFlag == 1) {
      rcController(pulseLength1, pulseLength2);  // call function for controlling with the rc controller
      
      if(pulseLength1 == 0) {  // 0 when controller is off
        rcCounter++;
      }
      if(rcCounter > 5) {  // safety check to make sure the controller is off
        rcFlag = 0;
        rcCounter = 0;
      } 
      
  }else{
    int startDelim = inputString.lastIndexOf('[');
    int endDelim = inputString.lastIndexOf(']') + 1;
    
    if(startDelim != -1 && endDelim != -1 && startDelim < endDelim){
      String command = inputString.substring(startDelim, endDelim);
      odroidControl(command);
      inputString = "";
      bufferCounter = 0;
    }
    
    if(sensorCounter == 1) {
      irValue();
      sensorCounter++;
    }
    else if(sensorCounter == 2) {
      usValue();
      sensorCounter++;
    }
    else if(sensorCounter == 3) {
      Serial.print("[E,"); Serial.print(encoderCount); Serial.println("]");
      sensorCounter = 1;
    }
    
    if (bufferCounter >= 16) {
      inputString = "";
      bufferCounter = 0;
    }
  }
}
void rcActivate() {
  rcFlag = 1;  // set flag to 1 when remote is on
}
void wheelEncoder(){
  encoderCount++;
}
int rcController(unsigned long pulse1, unsigned long pulse2){  
    if(pulse1 > 1300 && pulse1 < 1370) {
       turnStraight();
    }
    if(pulse1 > 1700) {
       turnRight(30);
    }
    if(pulse1 < 1000) {
       turnLeft(30);
    }
    if(pulse2 > 1400 && pulse2 < 1700) {
       goForward(1550);
    }
    if(pulse2 > 1700){
       goForward(1550);
    }  
    if(pulse2 < 1000){
       goBackward();   
    }
    if(pulse2 > 1300 && pulse2 < 1400) {
      if(pulse1 > 1300 && pulse1 < 1370) {
          goNeutral();
          turnStraight(); 
      }
    }
}
void serialEvent() {
  while (Serial.available() && bufferCounter < 16) {
    char inChar = (char) Serial.read();
    inputString += inChar;  
    if (inChar == ']') bufferCounter = 17;
    bufferCounter++;
  }
}
void odroidControl(String command) {
  if (command[3] == 'R' || command[3] == 'L') {
    int angle; // firstIndex is the last comma in the string & lastIndex is the "]" ending the string
    angle = command.substring(5, 7).toInt(); // save the last value in the packet which is the angle
    if (command[3] == 'R') {
      turnRight(angle);
    }
    else if (command[3] == 'L') {
      turnLeft(angle);
    }
  }
  else if (command[1] == 'M'){
    int velocity;
    velocity = command.substring(4,6).toInt();
    if(command[3] == 'B'){
      goBackward();
    }
    else if (command[3] == 'F'){
      if(command[5] == '0') goNeutral();
      else if(command[5] == 'F') goForward(1570);
      else if(command[5] == 'S') goForward(1570);
    }
  }
}
void irValue(){  // reading the IR values from the sensors
   int irVal1, irVal2, irVal3;
   String irStrVal1,irStrVal2, irStrVal3;
   
   irVal1 = analogRead(IR_SENSOR_1);
   irVal2 = analogRead(IR_SENSOR_2);
   irVal3 = analogRead(IR_SENSOR_3);
   irStrVal1 = String(irVal1);
   irStrVal2 = String(irVal2);
   irStrVal3 = String(irVal3);
   Serial.println("[IR," + irStrVal1 + "," + irStrVal2 + "," + irStrVal3 + "]");
}
void usValue(){ // read US value from the sensors
  int usVal1;
  String usStrVal1;
  usVal1 = frontSensor.getRange('c'); 
  usStrVal1 = String(usVal1);
  Serial.println("[US," + usStrVal1 + ",0]");
}
