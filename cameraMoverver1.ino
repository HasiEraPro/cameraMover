#include <RCSwitch.h>
#define debug  //activated debug mode,after done comment this
#define encoderPin 12
#define delayinAcceleration 50 ///how much time per incresing step 
#define slowDownDelay 1000      //how much time for decreasing step consider the acceleration value too
#define defaultStartupSpeed 180 // default speed valu to startup the motor motor rotates after 200PWM
RCSwitch mySwitch = RCSwitch();



//////////////Settings/////////////////////

 ////----------pins------///////
//motordriver ibt2
const uint8_t LPWM = 14; // H-bridge leg 1 ->LPWM
const uint8_t enL = 16;  // H-bridge enable pin 1 -> L_EN

const uint8_t RPWM = 5; // H-bridge leg 2 ->RPWM
const uint8_t enR = 4;  // H-bridge enable pin 2 -> R_EN

//Pin Kameratrigger
const uint8_t trig = D8; //camera trigger

//reed contact pin
const int reedSensorPin = A0; 


/////////////---------You can change this to your taste----------------/////////////////////////////
//photos per one round
uint16_t photosPerRound = 32;

//desired speed for moving photo from pwm 
uint16_t MovingPhotoSpeed = 300;

//desired speed for Still photo from pwm 
uint16_t StillPhotoSpeed = 300;

//minimum moving speed because emergency stop happens;
uint16_t minimumMovingSpeed =250;

//how much jump steps to desired speed
uint16_t MovingPhotoAcceleration = 10;

//how much jump steps to desired speed
uint16_t stillPhotoAcceleration = 50;

//time to stay before going to take other picture from milliseconds
uint16_t timeToStayStopMode = 6000;

//////////////////--define remote control from left to right columns numbered (eg:-R1C1 >>> ROW 1,Coulmn 1) ---//////////////
#define R1C1  1381717  // take photo manual  
#define R1C2  1381716 //calibrate

#define R2C1  1394005 //initiate
#define R2C2  1394004 //moving picture procedure

#define R3C1  1397077 //till picture procedure
#define R3C2  1397076 //emergency stop  

#define R4C1  1397845 //not assigned
#define R4C2  1397844 //not assigned
///////////////////////////////////////////

///Global variables dont change////////////////////////////////

uint16_t photosTaken = 0;
//steps to travel to take the next photo
uint16_t stepsToTravel = 0;

uint16_t mspeed = 0;
int sensorValue = 0;
volatile uint16_t encdCount = 0;
uint16_t currentPWMvalueMotor = 500;
uint16_t movingSpeed = 250;

bool initiateComplete = false;
bool isMotorRotates = false;
bool isEncoderRotated = false;
bool isCalibrated = false;

bool initialLocked = false;
bool movingPhotoLocked = false;
bool stillPhotoLocked =false;
bool calibrationLocked = false;

unsigned long previousMicros;

uint16_t OffsetStepCount =0,fullCalibratedStepCount=0;


//////////////Function prototype->Add the function definition here//////////////////////

void motor_clockwise(int speed);
void initiate();
void motorStopSlowly(uint16_t PwmSpeed);
void ICACHE_RAM_ATTR ISR();
void StillPhoto();
void calibrateSteps();
void accelerateMotor(uint16_t startSpeed,uint16_t finalSpeed,uint16_t increment);
void takePhoto();
void MovingPhoto();
void emergencyStop();
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
  pinMode(LED_BUILTIN, OUTPUT);
  //relay open when LOW, closed when high
  digitalWrite(trig, LOW); 
  
  // motor off (attention pins inverted)
  digitalWrite(enL, HIGH);
  digitalWrite(enR, HIGH);

  Serial.println("start Program");
    
  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, RISING);

  //initiate();
  //MovingPhoto();
  //StillPhoto();

calibrateSteps();


}

void loop()
{

if (mySwitch.available()) 
{

      ///take photo manual 
   
    if (mySwitch.getReceivedValue() == R1C1)
      {

        Serial.println("Pressed manual photo 1381717");
        takePhoto();
      }
      ///calibrate
    if (mySwitch.getReceivedValue() == R1C2)
      {
        Serial.println("Pressed calibrate R1C2");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)calibrateSteps();
      }
      //initiate
    if (mySwitch.getReceivedValue() == R2C1)
      {
        Serial.println("Pressed initiate R2C1");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)initiate();
      }
      //moving picture
    if (mySwitch.getReceivedValue() == R2C2)
      {
        Serial.println("Pressed moving pic R2C2");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)MovingPhoto();
      }
        //still picture
    if (mySwitch.getReceivedValue() == R3C1)
      {
        Serial.println("Pressed Still pic R3C1");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)StillPhoto();
      }

      //Emergency stop
    if (mySwitch.getReceivedValue() == R3C2)
      {
        Serial.println("Pressed Still pic R3C2");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)emergencyStop();
      }

     mySwitch.resetAvailable();
  }
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

movingPhotoLocked = true;

Serial.println("Started moving photo procedure");
bool isWorkDone = false;

  if(!initiateComplete)initiate();

accelerateMotor(defaultStartupSpeed,MovingPhotoSpeed,MovingPhotoAcceleration);

encdCount = 0;
photosTaken =0;
#ifdef debug 
Serial.println("moving photo acceleration achived");
#endif

while(!isWorkDone)
{

  if(encdCount >= stepsToTravel-5)
    {
      #ifdef debug  
      Serial.print("Photos taken:= ");Serial.println(photosTaken);
      #endif
      encdCount = 0;
      takePhoto();
      photosTaken++;


    }

  if(photosTaken >= photosPerRound)
    { 

      isWorkDone = true;
      motorStopSlowly(movingSpeed);
    }
  yield();



}
movingPhotoLocked = false;
initiateComplete = false;
Serial.println("End moving procedure");


}

void motorStopSlowly(uint16_t PwmSpeed)
{

#ifdef debug
Serial.println("Started slow stop procedure");
#endif
for (int i = PwmSpeed ; i > 180 ;i=i-5 )
{

motor_clockwise(i);
delayMicroseconds(slowDownDelay);
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

initialLocked = true;
  #ifdef debug 
  Serial.println("Initiation procedure start");
  #endif
bool isReachedStart = false;
bool firstContactHome =false;

accelerateMotor(defaultStartupSpeed,minimumMovingSpeed,10);

while(!isReachedStart)
{
yield();

readReedContact();

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
initialLocked = false;

}

void StillPhoto()
{
#ifdef debug
Serial.println("Started Still photo procedure");
#endif



if(!initiateComplete)initiate();
delay((int)timeToStayStopMode/2);
takePhoto();
delay((int)timeToStayStopMode/2);
stillPhotoLocked =true;

accelerateMotor(defaultStartupSpeed,StillPhotoSpeed,stillPhotoAcceleration);

#ifdef debug
Serial.println("Still photo speed achived");
#endif

bool isFirstRoundGone = false;
sensorValue = 0;

while(!isFirstRoundGone)
{

  yield();
  readReedContact();
  if(sensorValue > 1022)
  {
      isFirstRoundGone = true;
      Serial.println("First round complete Still pics");

  }

}

stepsToTravel = (fullCalibratedStepCount+OffsetStepCount)/photosPerRound;
Serial.print("step distance:-");Serial.println(stepsToTravel);
bool isWorkDone = false;
encdCount = 0;
photosTaken =0;

while(!isWorkDone)
  {
  yield();
  
  if(encdCount >= stepsToTravel)
    {
    
     
      
      #ifdef debug
      Serial.print("Photos taken from still:= ");Serial.println(photosTaken);
      #endif
      motorStopSlowly(movingSpeed);
      #ifdef debug
      Serial.println("stopped for flash light");
      #endif
      delay(timeToStayStopMode/2);
      takePhoto();
      delay(timeToStayStopMode/2);
      photosTaken++;
      
      encdCount = 0;
      accelerateMotor(defaultStartupSpeed,StillPhotoSpeed,stillPhotoAcceleration);
      
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
stillPhotoLocked =false;
initiateComplete = false;
}

void calibrateSteps()
{
calibrationLocked = true;
Serial.println("started calibration sequence please wait...");

for(int i = 100 ; i < minimumMovingSpeed ; i+=10)
{

motor_clockwise(i);
delay(delayinAcceleration);

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
    OffsetStepCount = 6;

    #ifdef debug
    Serial.println(OffsetStepCount);
    #endif
    encdCount = 0;


}

  
}

motorStopSlowly(minimumMovingSpeed);


isCalibrated = true;

stepsToTravel =(fullCalibratedStepCount+OffsetStepCount)/photosPerRound;
calibrationLocked = false;
initiateComplete = false;

}

void testStepCount()
{
Serial.println("started Testing steps sequence");

encdCount = 0;

for(int i = 100 ; i <= minimumMovingSpeed ; i+=10)
  {

  motor_clockwise(i);
  delay(250);

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

void accelerateMotor(uint16_t startSpeed,uint16_t finalSpeed,uint16_t increment)
{

for(int i = startSpeed ; i <= finalSpeed ; i+=increment)
  {

  motor_clockwise(i);
  delay(delayinAcceleration);

  }


}


void takePhoto()
{

digitalWrite(trig, HIGH);
delay(500);
 digitalWrite(trig, LOW);
  

}

void emergencyStop()
{



if(stillPhotoLocked)
{

  motorStopSlowly(StillPhotoSpeed);
  stillPhotoLocked = false;
  initiateComplete = false;


}
else if(movingPhotoLocked)
{

motorStopSlowly(MovingPhotoSpeed);
movingPhotoLocked = false;
initiateComplete = false;

}
else
{
  motorStopSlowly(minimumMovingSpeed);
initiateComplete = false;
movingPhotoLocked = false;
stillPhotoLocked = false;
}




}

