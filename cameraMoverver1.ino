#include <RCSwitch.h>

#define debug  //activated debug mode,after done comment this
#define encoderPin 12
#define delayinAcceleration 50 ///how much time per incresing step 
#define slowDownDelay 1000      //how much time for decreasing step consider the acceleration value too
#define defaultStartupSpeed 200 // default speed valu to startup the motor motor rotates after 200PWM
#define stillTriggertime 500
#define MovingTriggertime 200
RCSwitch mySwitch = RCSwitch();





/////////////---------You can change this to your taste----------------/////////////////////////////
//photos per
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
uint16_t photosPerRound = 31;

//desired speed for moving photo from pwm 
uint16_t MovingPhotoSpeed = 300;

//desired speed for Still photo from pwm 
uint16_t StillPhotoSpeed = 300;

//minimum moving speed because emergency stop happens;
uint16_t minimumMovingSpeed =300;

//how much jump steps to desired speed
uint16_t MovingPhotoAcceleration = 50;

//how much jump steps to desired speed
uint16_t stillPhotoAcceleration = 50;

//time to stay before going to take other picture from milliseconds
uint16_t timeToStayStopMode = 4000;

//////////////////--define remote control from left to right columns numbered (eg:-R1C1 >>> ROW 1,Coulmn 1) ---//////////////
#define R1C1  1381717  // initialise  
#define R1C2  1381716 //take pic

#define R2C1  1394005 //still pic mode
#define R2C2  1394004 //moving picture procedure

#define R3C1  1397077 //till picture procedure
#define R3C2  1397076 //emergency stop  

#define R4C1  1397845 //not assigned
#define R4C2  1397844 //emergency stop
///////////////////////////////////////////

///Global variables dont change////////////////////////////////

uint16_t photosTaken = 0;
//steps to travel to take the next photo
uint16_t stepsToTravel = 0;
uint16_t movingSpeed = 250;

int sensorValue = 0;
volatile int encdCount = 0;

bool initiateComplete = false;
bool isMotorRotates = false;
bool isEncoderRotated = false;
bool isCalibrated = false;

bool initialLocked = false;
bool movingPhotoLocked = false;
bool stillPhotoLocked =false;
bool calibrationLocked = false;

unsigned long previousMicros;

/////////////Function prototype->Add the function definition here//////////////////////

void motor_clockwise(int speed);
void initiate();
void motorStopSlowly(uint16_t PwmSpeed);
void ICACHE_RAM_ATTR ISR();
void StillPhoto();
void accelerateMotor(uint16_t startSpeed,uint16_t finalSpeed,uint16_t increment);
void takePhoto();
void MovingPhoto();
void turnSteps(uint16_t steps);
void checkEmergency(bool & flag);
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


Serial.println("waiting to power on");

 delay(15000);

initiate();


}

void loop()
{

if (mySwitch.available()) 
{

      //initialise
       if (mySwitch.getReceivedValue() == R1C1)
      {

        Serial.println("Pressed initiate R1C1");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)initiate();
      }
   
      //manul shot
    if (mySwitch.getReceivedValue() == R1C2)
      {
        Serial.println("Pressed manual shot  R2C1");
        takePhoto();
      }

      //still pic
    if (mySwitch.getReceivedValue() == R2C1)
      {
        Serial.println("Pressed Still pic R3C1");
        if(!calibrationLocked && !initialLocked && !movingPhotoLocked && !stillPhotoLocked)StillPhoto();
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
        Serial.println("Pressed R3C1 not assigned");
      }
           //still picture
    if (mySwitch.getReceivedValue() == R3C2)
      {
        Serial.println("Pressed R3C2 not assigned");
      }
   //Emergency stop
    if (mySwitch.getReceivedValue() == R4C1)
      {
        Serial.println("Pressed R3C2 not assigned ");
      
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

if(!initiateComplete)initiate();

movingPhotoLocked = true;

Serial.println("Started moving photo procedure");


  Serial.print("Full steps:");Serial.println("380");
 stepsToTravel = round((380.0)/(photosPerRound+1));
Serial.print("step distance moving photo:-");Serial.println(stepsToTravel);

encdCount = 0;
photosTaken =0;
takePhoto();
delay(1000);
accelerateMotor(defaultStartupSpeed,MovingPhotoSpeed,MovingPhotoAcceleration);

bool isWorkDone = false;

while(!isWorkDone)
{
yield();

if(isEncoderRotated)
{
Serial.print("count:-");Serial.println(encdCount);
isEncoderRotated = false;
}
  if(encdCount == stepsToTravel)
    {
      
      encdCount = 0;
      takePhoto();
      photosTaken++;
      Serial.print("Photo count:= ");Serial.println(photosTaken);
      


    }
     if(encdCount > stepsToTravel)
    {
      int error = encdCount - stepsToTravel;
      
      
      takePhoto();
      photosTaken++;
      Serial.print("Photo(greater count) count:= ");Serial.println(photosTaken);
      encdCount = 0 - error ;


    }

  if(photosTaken >= photosPerRound)
    { 

      isWorkDone = true;
      //motorStopSlowly(movingSpeed);
    }
  



}
motorStopSlowly(movingSpeed);
takePhoto();
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

digitalWrite(LPWM, HIGH);
digitalWrite(RPWM, HIGH);
initiateComplete = false;
}

void initiate()
{

//if(!isCalibrated)calibrateSteps();

initialLocked = true;
  #ifdef debug 
  Serial.println("Initiation procedure start");
  #endif
bool isReachedStart = false;
//before start see if that actualy we are inside the MagnetField

sensorValue = 0;
bool insideMagnetField = false;
readReedContact();
delay(500);
readReedContact();

if(sensorValue > 1000)
{
  insideMagnetField = true;

}
accelerateMotor(defaultStartupSpeed,minimumMovingSpeed,10);

while(!isReachedStart)
{
yield();

readReedContact();

checkEmergency(&isReachedStart);
//if you found a magnetic field
if(sensorValue > 1000)
  {
    //check you already inside a one, if it is not, this is the initial position
    if(!insideMagnetField)
      { 
        //encdCount = 0;
        isReachedStart = true;
      }

  }

  //if you got no magnetic field,
if(sensorValue < 1000)
  {
//check did you inside a magentic field
    if(insideMagnetField)
      {
        //if it is you got out from magnetic field,but not in intial position yet
        insideMagnetField = false;

      }


  }   


}


motorStopSlowly(minimumMovingSpeed);

encdCount = 0;

Serial.println("initiation at home postion");

initiateComplete = true;
initialLocked = false;


}

void StillPhoto()
{
#ifdef debug
Serial.println("Started Still photo procedure");
#endif

if(!initiateComplete)initiate();

stillPhotoLocked =true;

delay((int)timeToStayStopMode);
takePhoto();
delay((int)timeToStayStopMode/2);
photosTaken = 0;
encdCount = 0;
stepsToTravel = round(380.0/(photosPerRound+1));
Serial.print("step distance:-");Serial.println(stepsToTravel);
accelerateMotor(defaultStartupSpeed,StillPhotoSpeed,stillPhotoAcceleration);
bool isWorkDone = false;



while(!isWorkDone)
  {
  yield();


  
  if(encdCount > stepsToTravel)
  {
   
    uint16_t howMuchFarError = encdCount - stepsToTravel;
   
      //plusError = true;
    
      #ifdef debug
      Serial.print("Photos taken from still:= ");Serial.println(photosTaken);
      #endif
      motorStopSlowly(movingSpeed);
      #ifdef debug
      Serial.println("stopped for flash light");
      #endif
      delay(timeToStayStopMode);
      takePhoto();
      delay(timeToStayStopMode/2);
      photosTaken++;
      
       encdCount = 0 - howMuchFarError;  
        
     accelerateMotor(defaultStartupSpeed,StillPhotoSpeed,stillPhotoAcceleration);
    
    }
    
    
  if(encdCount == stepsToTravel)
    {
    
     
      
      #ifdef debug
      Serial.print("Photos taken from still:= ");Serial.println(photosTaken);
      #endif
      motorStopSlowly(movingSpeed);
      #ifdef debug
      Serial.println("stopped for flash light");
      #endif
      delay(timeToStayStopMode);
      takePhoto();
      delay(timeToStayStopMode/2);
      photosTaken++;
      
       encdCount = 0;    
     accelerateMotor(defaultStartupSpeed,StillPhotoSpeed,stillPhotoAcceleration);
      
    }

  if(photosTaken > (photosPerRound-1))
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
if(stillPhotoLocked) {delay(stillTriggertime);}
if(movingPhotoLocked){delay(MovingTriggertime);}
if(!stillPhotoLocked && !movingPhotoLocked){{delay(stillTriggertime);}}
digitalWrite(trig, LOW);
  

}


void turnSteps(uint16_t steps)
{
Serial.println("turn steps procedure");
Serial.print("turn steps:-");Serial.println(steps);

encdCount = 0;
accelerateMotor(defaultStartupSpeed,MovingPhotoSpeed,MovingPhotoAcceleration);
bool workdone = false;

while (!workdone)
{
  yield();

if(encdCount >= steps)
{
workdone = true;

}

}

motorStopSlowly(MovingPhotoSpeed);



}

void checkEmergency(bool *flag)
{

if (mySwitch.available()) 
{

if (mySwitch.getReceivedValue() == R4C2)
      {
        Serial.println("Pressed Still pic R3C2");

        *flag = true;
        initiateComplete = false;
        movingPhotoLocked = false;
        stillPhotoLocked = false;
      }

     mySwitch.resetAvailable();



}


}
