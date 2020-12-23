#include <RCSwitch.h>
#define debug  //activated debug mode,after done comment this
#define encoderPin 12
RCSwitch mySwitch = RCSwitch();

///counted steps without offset 372,offset is 5(5 steps goes inside magnet)

//////////////Settings/////////////////////

 ////----------pins------///////
//motordriver ibt2
const uint8_t LPWM = 14; // H-bridge leg 1 ->LPWM
const uint8_t enL = 16;  // H-bridge enable pin 1 -> L_EN

const uint8_t RPWM = 5; // H-bridge leg 2 ->RPWM
const uint8_t enR = 4;  // H-bridge enable pin 2 -> R_EN

//Pin Kameratrigger
const uint8_t trig = 15; //camera trigger

const int reedSensorPin = A0; //without function, in the past trigger button for manual use
//wheel encoder GPIO16(used for inerrupts) digitally D0 

//photos per round
uint16_t photosPerRound = 10;
uint16_t photosTaken = 0;
//steps per round
uint16_t  stepsPerRound = 600;
//steps to travel to take the next photo
uint16_t stepsToTravel  = stepsPerRound/photosPerRound;

//desired speed for moving photo from pwm 
uint16_t MovingPhotoSpeed = 500;
//how much jump steps to desired speed
uint16_t MovingPhotoAcceleration = 25;
//time to stay before going to take other picture from milliseconds
uint16_t timeToStayStopMode = 2000;
//minimum moving speed because emergency stop happens;
uint16_t minimumMovingSpeed = 100;
///////////////////////////////////////////

uint16_t mspeed = 0;
int sensorValue = 0;
volatile uint16_t encdCount = 0;
uint16_t currentPWMvalueMotor = 500;
uint16_t movingSpeed = 250;

bool initiateComplete = false;
bool isMotorRotates = false;
bool isEncoderRotated = false;
bool isCalibrated = false;

unsigned long previousMicros;

uint16_t OffsetStepCount =0,fullCalibratedStepCount=0;
//////////////Function prototype//////////////////////

void motor_clockwise(int speed);
void initiate();
void motorStopSlowly(uint16_t PwmSpeed);
void ICACHE_RAM_ATTR ISR();
void StillPhoto();
void calibrateSteps();
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

  Serial.println("start Program");

    //motor_clockwise(100);

  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, RISING);

  //initiate();
  //MovingPhoto();
  //StillPhoto();
calibrateSteps();

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

  isMotorRotates = true;
}

void readReedContact()
{
 
sensorValue = analogRead(reedSensorPin);

}

void ICACHE_RAM_ATTR ISR()
{
  if (micros() - previousMicros >= 500) {
    //if(encdCount > 60 )encdCount=0;
    encdCount++;
    isEncoderRotated = true;
    previousMicros = micros();
  }
 
    
  

}

void MovingPhoto()
{

Serial.println("Started moving photo procedure");
bool isWorkDone = false;

if(!initiateComplete)initiate();

for(int i = 0 ; i < MovingPhotoSpeed ; i+=MovingPhotoAcceleration )
{

  motor_clockwise(i);
  delay(500);

}

encdCount = 0;
#ifdef debug 
Serial.println("moving photo acceleration achived");
#endif

while(!isWorkDone)
{

if(encdCount >= stepsToTravel)
{
encdCount = 0;
photosTaken++;
#ifdef debug
Serial.print("Photos taken:= ");Serial.println(photosTaken);
#endif

}

if(photosTaken >= photosPerRound)
{

isWorkDone = true;
motorStopSlowly(movingSpeed);
}
yield();



}
initiateComplete = false;
Serial.println("End moving procedure");


}

void motorStopSlowly(uint16_t PwmSpeed)
{

#ifdef debug
Serial.println("Started slow stop procedure");
#endif
for (int i = PwmSpeed ; i > 20 ;i=i-10 )
{

motor_clockwise(i);
delayMicroseconds(1000);
}

#ifdef debug
Serial.print("Moving to slow stop encode count:");Serial.println(encdCount);
#endif

digitalWrite(LPWM, HIGH);
digitalWrite(RPWM, HIGH);
initiateComplete = false;
}

void initiate()
{
if(!isCalibrated)calibrateSteps();

  #ifdef debug 
  Serial.println("Initiation procedure start");
  #endif
bool isReachedStart = false;
bool firstContactHome =false;
for(int i = 100 ; i < minimumMovingSpeed ; i+=10)
{

motor_clockwise(i);
delay(1000);

}

while(!isReachedStart)
{
yield();

readReedContact();

//{
//#ifdef debug
//Serial.print("count:= ");Serial.println(encdCount);
//#endif
//isEncoderRotated = false;

//}

//sensorValue = analogRead(reedSensorPin);

//if(sensorValue > 1022)
//{

//isReachedStart = true;
//encdCount = 0;

//}

if(sensorValue > 1022)
{
  encdCount = 0;
  isReachedStart = true;

}





}


motorStopSlowly(minimumMovingSpeed);

Serial.println("initiation at home postion");
Serial.println("travel counted steps and confirm");
initiateComplete = true;






}

void StillPhoto()
{
#ifdef debug
Serial.println("Started Still photo procedure");
#endif

bool isWorkDone = false;

if(!initiateComplete)initiate();

for(int i =50 ; i < minimumMovingSpeed ; i+=10 )
{

  motor_clockwise(i);
  delay(500);

}
#ifdef debug
Serial.println("Still photo speed achived");
#endif
encdCount = 0;
photosTaken =0;

while(!isWorkDone)
{
yield();

if(encdCount >= stepsToTravel)
{
#ifdef debug
Serial.print("Count stoped at:-");Serial.println(encdCount);
#endif
encdCount = 0;
photosTaken++;
#ifdef debug
Serial.print("Photos taken from still:= ");Serial.println(photosTaken);
#endif
motorStopSlowly(movingSpeed);
#ifdef debug
Serial.println("stopped for flash light");
#endif
delay(timeToStayStopMode);
for(int i = 50 ; i < minimumMovingSpeed ; i+=10 )
{

  motor_clockwise(i);
  delay(500);

}
}

if(photosTaken >= photosPerRound)
{

isWorkDone = true;
motorStopSlowly(movingSpeed);
}




}

initiateComplete = false;
#ifdef debug
Serial.println("Still photo procedure finished");
#endif

motorStopSlowly(movingSpeed);
}

void calibrateSteps()
{

Serial.println("started calibration sequence");

for(int i = 100 ; i < minimumMovingSpeed ; i+=10)
{

motor_clockwise(i);
delay(1000);

}


bool isWorkDone = false;
bool FirstTimeContact = false;
bool gettingAwayHome = false;

while (!isWorkDone)
{
  yield();

  readReedContact();

  if(sensorValue >1022 && !FirstTimeContact && !gettingAwayHome)
  {
    #ifdef debug
    Serial.println("First contact with home sensor");
    #endif
      FirstTimeContact = true;
      encdCount = 0;

    
      

  }

  if(sensorValue >1022 && !FirstTimeContact && gettingAwayHome)
  {
      fullCalibratedStepCount = encdCount;
      encdCount = 0;

    #ifdef debug
    Serial.print("2nd contact with home sensor");Serial.println(fullCalibratedStepCount);
    #endif

isWorkDone = true;


  }
if(sensorValue < 1022 && FirstTimeContact)
{
    #ifdef debug
    Serial.print("Getting away from home sensor:= ");
    #endif
    FirstTimeContact = false;
    gettingAwayHome = true;
    OffsetStepCount = encdCount;

    #ifdef debug
    Serial.println(OffsetStepCount);
    #endif
    encdCount = 0;


}

  
}

motorStopSlowly(minimumMovingSpeed);


isCalibrated = true;


}

void testStepCount()
{
Serial.println("started Testing steps sequence");

encdCount = 0;

for(int i = 100 ; i <= minimumMovingSpeed ; i+=10)
{

motor_clockwise(i);
delay(1000);

}
bool isWorkDone = false;

while(!isWorkDone)
{

yield();

readReedContact();

if(encdCount == (fullCalibratedStepCount+OffsetStepCount))
{

Serial.print("steps travelled:-");Serial.println(encdCount);
Serial.print("Home sensor value: - ");Serial.println(sensorValue);
isWorkDone = true;
}






}

motorStopSlowly(minimumMovingSpeed);




}






