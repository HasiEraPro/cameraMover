#include <RCSwitch.h>
#define encoderPin 12
RCSwitch mySwitch = RCSwitch();
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




///////////////////////////////////////////



uint16_t mspeed = 0;
int sensorValue = 0;
volatile uint16_t encdCount = 0;
uint16_t currentPWMvalueMotor = 500;

bool initiateComplete = false;
uint16_t movingSpeed = 250;
bool isMotorRotates = false;
bool isEncoderRotated = false;
unsigned long previousMicros;
//////////////Function prototype//////////////////////

void motor_clockwise(int speed);
void initiate();
void motorStopSlowly();
void ICACHE_RAM_ATTR ISR();
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

    //motor_clockwise(100);

  attachInterrupt(digitalPinToInterrupt(encoderPin), ISR, RISING);

  initiate();

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
bool isWorkDone = false;
if(!initiateComplete)initiate();
while(!isWorkDone)
{


motor_clockwise(movingSpeed);


}



}

void motorStopSlowly()
{

for (int i = currentPWMvalueMotor ; i > 20 ;i=i-50 )
{

motor_clockwise(i);
delayMicroseconds(1000);
}


Serial.print("m slow stop encode count:");Serial.println(encdCount);

}

void initiate()
{
bool isReachedStart = false;
motor_clockwise(currentPWMvalueMotor);

while(!isReachedStart)
{
  yield();
if(isEncoderRotated)
{

Serial.print("count:= ");Serial.println(encdCount);
isEncoderRotated = false;

}
//sensorValue = analogRead(reedSensorPin);

//if(sensorValue > 1022)
//{

//isReachedStart = true;
//encdCount = 0;

//}

if(encdCount > 200)
{
encdCount = 0;
isReachedStart = true;

}


}

motorStopSlowly();

initiateComplete = true;



}




