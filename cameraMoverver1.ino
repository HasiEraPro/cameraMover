#include <RCSwitch.h>

RCSwitch mySwitch = RCSwitch();
//////////////Settings/////////////////////

 ////----------pins------///////
//motordriver ibt2
const uint8_t LPWM = 14; // H-bridge leg 1 ->LPWM
const uint8_t enL = 12;  // H-bridge enable pin 1 -> L_EN

const uint8_t RPWM = 5; // H-bridge leg 2 ->RPWM
const uint8_t enR = 4;  // H-bridge enable pin 2 -> R_EN

//Pin Kameratrigger
const uint8_t trig = 15; //camera trigger

const int reedSensorPin = A0; //without function, in the past trigger button for manual use
//wheel encoder GPIO16(used for inerrupts) digitally D0 
const uint8_t encoderPin = 16  ;
 //////////////


///////////////////////////////////////////


// this are my variable for the speed of the motor. At the moment it start moving at 240
uint16_t mspeed = 0;
int sensorValue = 0;
uint16_t encdCount = 0;
uint16_t currentPWMvalueMotor = 100;

bool initiateComplete = false;
uint16_t movingSpeed = 250;

//////////////Function prototype//////////////////////

void motor_clockwise(int speed);
void initiate();
void motorStopSlowly();

//////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);

  //433 MHz Empfänger
  //12 r 13, depending on the testsetting/jumper-position
  mySwitch.enableReceive(13);  

  // motor driver preparing the pins
  pinMode(LPWM, OUTPUT);
  pinMode(RPWM, OUTPUT);
  pinMode(enL, OUTPUT);
  pinMode(enR, OUTPUT);
  pinMode(trig, OUTPUT);

  //relay open when LOW, closed when high
  digitalWrite(trig, LOW); 
  
  // motor off (attention pins inverted)
  digitalWrite(enL, HIGH);
  digitalWrite(enR, HIGH);

  Serial.println("start");



  
    motor_clockwise(100);


  //attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, RISING);

}

void loop()
{

     

}








void motor_clockwise(int speed)
//move motor clockwise
{
  //pinMode(enL, HIGH);
  //pinMode(enR, LOW);
  //digitalWrite(LPWM, HIGH);
  if (speed> 1022)
  {
    digitalWrite(LPWM, HIGH); // 1023 is the max value, if it is reached set the pin on high simply
  }
  if (speed < 1023)
  {
    analogWrite(LPWM, speed); // any other speed value is a pwm signal
  }
  //analogWrite(LPWM,mspeed); //pwm value
  digitalWrite(RPWM, LOW);
}

void readReedContact()
{
sensorValue = analogRead(reedSensorPin);

}


ICACHE_RAM_ATTR void ISR()
{


  encdCount++;

}

void takepic()
// simple set trigger pin to high, wait a short time for camera to focus and take pic and set it to low
{
  digitalWrite(trig, HIGH);
  
  digitalWrite(trig, LOW);
}

void StillPhoto()
{





}

void MovingPhoto()
{
bool isWorkDone = false;
if(!initiateComplete)initiate();
while(!isWorkDone)
{


motor_clockwise(movingSpeed);


}



}

void motorStopSlowly()
{

for (int i = currentPWMvalueMotor ; i >=0 ;i=i-30 )
{

motor_clockwise(i);

}

Serial.print("m slow stop encode count:");Serial.println(encdCount);

}

void initiate()
{
bool isReachedStart = false;

while(!isReachedStart)
{
motor_clockwise(currentPWMvalueMotor);
sensorValue = analogRead(reedSensorPin);
if(sensorValue > 1022)
{

isReachedStart = true;
encdCount = 0;

}


}
motorStopSlowly();

initiateComplete = true;



}




