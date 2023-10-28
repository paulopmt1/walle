#include <Servo.h> 

/**
 * Head direction pins and motors
 */
int headUpDownDirectionSensorPin = 2;
volatile long headUpDownStartTime = 0;
volatile long headUpDownCurrentTime = 0;
volatile long headUpDownPulses = 0;
int headUpDownPulseWidth = 0;
const byte headUpDownDirectionMotorRightPin = 10;
const byte headUpDownDirectionMotorLeftPin = 11;
Servo headUpDownDirectionServoRight;
Servo headUpDownDirectionServoLeft;

/**
 * Back forward direction pins and motors
 */
int backForwardSensorPin = 3;
volatile long backForwardStartTime = 0;
volatile long backForwardCurrentTime = 0;
volatile long backForwardPulses = 0;
volatile long lastSpeedChangeTime = 0;
int backForwardPulseWidth = 0;
const byte backForwardDirectionMotorFrontPin = 12;
const byte backForwardDirectionMotorRearPin = 13;
const byte backForwardL_enable = 52;
const byte backForwardR_enable = 53;
const byte MAX_SPEED_PWM_VALUE = 150;

/**
 * Oil pump
 */
const byte oilPumpPWMPin = 4;
const byte oilPumpStartupSpeed = 200;
const byte oilPumpOperationSpeed = 90;
bool oilPumpIsOn = false;
const int reduceOilPumpPressureAfterMS = 3000;
const int stopOilPumpPressureAfterMS = 10000;
Servo oilPump;
unsigned long pumpStartTime;


void setup(){

  // Head up down
  pinMode(headUpDownDirectionSensorPin, INPUT_PULLUP);
  headUpDownDirectionServoRight.attach(headUpDownDirectionMotorRightPin);
  headUpDownDirectionServoLeft.attach(headUpDownDirectionMotorLeftPin);
  attachInterrupt(digitalPinToInterrupt(headUpDownDirectionSensorPin), headUpDownPulseTimer, CHANGE);
  
  // Back forward
  pinMode(backForwardL_enable, OUTPUT);
  pinMode(backForwardR_enable, OUTPUT);
  digitalWrite(backForwardL_enable, HIGH);
  digitalWrite(backForwardR_enable, HIGH);
  pinMode(backForwardDirectionMotorFrontPin, OUTPUT);
  pinMode(backForwardDirectionMotorRearPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(backForwardSensorPin), backForwardPulseTimer, CHANGE);

  // Oil PUMP starts disabled
  oilPump.attach( oilPumpPWMPin, 1000, 2000 );
  oilPump.write(0);



  Serial.begin(115200);
  Serial.println("Wall-e started!");

  delay(100);
}

void headUpDownPulseTimer() {
  headUpDownCurrentTime  = micros();
  if (headUpDownCurrentTime > headUpDownStartTime) {
    headUpDownPulses = headUpDownCurrentTime - headUpDownStartTime;
    headUpDownStartTime = headUpDownCurrentTime;
  }
}

void backForwardPulseTimer() {
  backForwardCurrentTime  = micros();
  if (backForwardCurrentTime > backForwardStartTime) {
    backForwardPulses = backForwardCurrentTime - backForwardStartTime;
    backForwardStartTime = backForwardCurrentTime;
  }
}

void loop() {
  modifyOilPumpSpeed();

  if (headUpDownPulses < 2000){
    headUpDownPulseWidth = headUpDownPulses;
  }
  if (backForwardPulses < 2000){
    backForwardPulseWidth = backForwardPulses;
  }

  /**
   * Head up down
   */
  int headUpDownMotorLeftValueAngle = constrain( map(headUpDownPulseWidth, 985, 1800, 140, 40), 45, 130) +2;
  int headUpDownMotorRightValueAngle = constrain( map(headUpDownPulseWidth, 985, 1800, 40, 140), 50, 135);
  headUpDownDirectionServoRight.write(headUpDownMotorRightValueAngle);
  headUpDownDirectionServoLeft.write(headUpDownMotorLeftValueAngle);
  // Serial.print( "Head puslse: ");
  // Serial.print( headUpDownPulseWidth);
  // Serial.print( " - left: ");
  // Serial.print(headUpDownMotorLeftValueAngle);
  // Serial.print( " - right: ");
  // Serial.print(headUpDownMotorRightValueAngle);
  // Serial.print( " - ");

  /**
   * Going forward and backwards
   */
  Serial.print( "Back forward puslse: ");
  Serial.print( backForwardPulseWidth);
  const int frontPWMValue = constrain( map( backForwardPulseWidth, 1490, 1980, 0, MAX_SPEED_PWM_VALUE ), 0, MAX_SPEED_PWM_VALUE );
  const int backwardsPWMValue = constrain( map( backForwardPulseWidth, 1450, 990, 0, MAX_SPEED_PWM_VALUE ), 0, MAX_SPEED_PWM_VALUE );

  if ( frontPWMValue > 5 || backwardsPWMValue > 5 ) {
    lastSpeedChangeTime = millis();
  }

  if ( frontPWMValue > 0 ) {
    digitalWrite( backForwardDirectionMotorRearPin, LOW );
    analogWrite( backForwardDirectionMotorFrontPin, frontPWMValue );

  } else {
    digitalWrite( backForwardDirectionMotorFrontPin, LOW );
    analogWrite( backForwardDirectionMotorRearPin, backwardsPWMValue );
  }
  Serial.print( " - frontPWMValue: ");
  Serial.print( frontPWMValue );
  Serial.print( " - backwardsPWMValue: ");
  Serial.print( backwardsPWMValue );


  Serial.println();

}


void oilPumpOn() {
  if ( ! oilPumpIsOn ) {
    oilPumpIsOn = true;
    oilPump.write( oilPumpStartupSpeed );
    pumpStartTime = millis();
  }
}

void oilPumpOff() {
  oilPumpIsOn = false;
  oilPump.write( 0 );
  pumpStartTime = millis();
}

void modifyOilPumpSpeed() {
  if ( millis() > pumpStartTime + reduceOilPumpPressureAfterMS ) {
    oilPump.write( oilPumpOperationSpeed );
  }

  if ( millis() > lastSpeedChangeTime + stopOilPumpPressureAfterMS ) {
    oilPumpOff();
  }

  if ( lastSpeedChangeTime + 50 > millis() ) {
    oilPumpOn();
  }
}
