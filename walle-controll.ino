#include <Servo.h> 

bool isWalleEnabled = false;
const byte detectWalleEnabledPin = 7;


/**
 * Head UpDown direction pins and motors
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
int lastHeadUpDownValue = 0;
long int lastHeadUpDownChangeTime = 0;

/**
 * Head RightLeft direction pins and motors
 */
int headRightLeftDirectionSensorPin = 18;
volatile long headRightLeftStartTime = 0;
volatile long headRightLeftCurrentTime = 0;
volatile long headRightLeftPulses = 0;
int headRightLeftPulseWidth = 0;
const byte headRightLeftDirectionMotorPin = 9;
Servo headRightLeftDirectionServo;
int lastHeadRightLeftValue = 0;
long int lastHeadRightLeftChangeTime = 0;

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
const int stopOilPumpPressureAfterMS = 5000;
Servo oilPump;
unsigned long pumpStartTime;


void setup(){

  // Detect if walle is enabled
  pinMode( detectWalleEnabledPin, INPUT );

  // Head up down
  pinMode(headUpDownDirectionSensorPin, INPUT_PULLUP);
  headUpDownDirectionServoRight.attach(headUpDownDirectionMotorRightPin);
  headUpDownDirectionServoLeft.attach(headUpDownDirectionMotorLeftPin);
  attachInterrupt(digitalPinToInterrupt(headUpDownDirectionSensorPin), headUpDownPulseTimer, CHANGE);
  
  // Head right left
  pinMode( headRightLeftDirectionSensorPin, INPUT_PULLUP );
  headRightLeftDirectionServo.attach( headRightLeftDirectionMotorPin );
  attachInterrupt(digitalPinToInterrupt( headRightLeftDirectionSensorPin ), headRightLeftPulseTimer, CHANGE);

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

}

void loop() {

  detectIfWalleIsEnabled();

  if ( isWalleEnabled ) {
    modifyOilPumpSpeed();
    headUpDown();
    headLeftRight();
    goingBackForward();
  }
  
}


void detectIfWalleIsEnabled() {
  byte walleIsEnabledInputValue = GetPWM(detectWalleEnabledPin);
  
  // Button from controll return 5 for disable and 10 e enable
  if ( walleIsEnabledInputValue > 7 ) {
    isWalleEnabled = 1;
  } else {
    isWalleEnabled = 0;
  }

  Serial.print("is enabled: ");
  Serial.println(isWalleEnabled);
}

void oilPumpOn() {
  Serial.print("oilPumOn function call. isOn: " );
  Serial.print(oilPumpIsOn);
  Serial.println();

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
  if ( oilPumpIsOn && millis() > pumpStartTime + reduceOilPumpPressureAfterMS ) {
    oilPump.write( oilPumpOperationSpeed );
  }

  if ( millis() > lastSpeedChangeTime + stopOilPumpPressureAfterMS ) {
    oilPumpOff();
  }

  if ( lastSpeedChangeTime + 200 > millis() ) {
    oilPumpOn();
  }
}

/**
 * Head up down
 */
void headUpDown() {
  if (headUpDownPulses < 2000){
    headUpDownPulseWidth = headUpDownPulses;
  }

  // Remove noise
  int headUpDownPulseNormalized = round( headUpDownPulseWidth * 0.01);
  int headUpDownPulseWidthReverse = map( headUpDownPulseNormalized, 10, 20, 20, 10 );
  int headUpDownMotorLeftValueAngle = constrain( map(headUpDownPulseWidthReverse, 10, 20, 140, 40), 45, 130) +2;
  int headUpDownMotorRightValueAngle = constrain( map(headUpDownPulseWidthReverse, 10, 20, 40, 140), 50, 135);
 
  if ( lastHeadUpDownValue != headUpDownPulseNormalized ) {
    lastHeadUpDownValue = headUpDownPulseNormalized;
    headUpDownDirectionServoRight.attach(headUpDownDirectionMotorRightPin);
    headUpDownDirectionServoLeft.attach(headUpDownDirectionMotorLeftPin);
    headUpDownDirectionServoRight.write(headUpDownMotorRightValueAngle);
    headUpDownDirectionServoLeft.write(headUpDownMotorLeftValueAngle);
    lastHeadUpDownChangeTime = millis();
  }

  if ( millis() > lastHeadUpDownChangeTime + 500 ) {
    headUpDownDirectionServoRight.detach();
    headUpDownDirectionServoLeft.detach();
  }

  // Serial.print( "Head puslse: ");
  // Serial.print( headUpDownPulseWidthReverse );
  // Serial.print( " - left: ");
  // Serial.print(headUpDownMotorLeftValueAngle);
  // Serial.print( " - right: ");
  // Serial.print(headUpDownMotorRightValueAngle);
  // Serial.print( " - ");
  // Serial.println();
}

/**
 * Head Right Left
 */
void headLeftRight() {
  if (headRightLeftPulses < 2000){
    headRightLeftPulseWidth = headRightLeftPulses;
  }

  // Remove noise
  int headRightLeftPulseNormalized = round( headRightLeftPulseWidth * 0.01);
  int headRightLeftPulseWidthReverse = map( headRightLeftPulseNormalized, 10, 20, 20, 10 );
  int headRightLeftMotorValueAngle = constrain( map(headRightLeftPulseWidthReverse, 10, 20, 20, 160), 20, 160);
 
  if ( lastHeadRightLeftValue != headRightLeftPulseNormalized ) {
    lastHeadRightLeftValue = headRightLeftPulseNormalized;
    headRightLeftDirectionServo.attach(headRightLeftDirectionMotorPin);
    headRightLeftDirectionServo.write(headRightLeftMotorValueAngle);
    lastHeadRightLeftChangeTime = millis();

    // Serial.print( "Head Left Right pulse normalized: ");
    // Serial.print( headRightLeftPulseNormalized );
    // Serial.print( " - angle: ");
    // Serial.print( headRightLeftMotorValueAngle);
    // Serial.print( " - ");
    // Serial.println();
  }

  if ( millis() > lastHeadRightLeftChangeTime + 500 ) {
    headRightLeftDirectionServo.detach();
  }

}

/**
 * Going forward and backwards
 */
void goingBackForward() {
  if (backForwardPulses < 2000){
    backForwardPulseWidth = backForwardPulses;
  }

  const int backForwardPulseWidthNormalized = round( backForwardPulseWidth * 0.01 );
  const int frontPWMValue = constrain( map( backForwardPulseWidthNormalized, 15, 20, 0, MAX_SPEED_PWM_VALUE ), 0, MAX_SPEED_PWM_VALUE );
  const int backwardsPWMValue = constrain( map( backForwardPulseWidthNormalized, 15, 10, 0, MAX_SPEED_PWM_VALUE ), 0, MAX_SPEED_PWM_VALUE );

  if ( frontPWMValue > 10 || backwardsPWMValue > 10 ) {
    lastSpeedChangeTime = millis();
  }

  if ( frontPWMValue > 0 ) {
    digitalWrite( backForwardDirectionMotorRearPin, LOW );
    analogWrite( backForwardDirectionMotorFrontPin, frontPWMValue );
  } else {
    digitalWrite( backForwardDirectionMotorFrontPin, LOW );
    analogWrite( backForwardDirectionMotorRearPin, backwardsPWMValue );
  }
  // Serial.print( "Back forward puslse: ");
  // Serial.print( backForwardPulseWidthNormalized );
  // Serial.print( " - frontPWMValue: ");
  // Serial.print( frontPWMValue );
  // Serial.print( " - backwardsPWMValue: ");
  // Serial.print( backwardsPWMValue );
  // Serial.println();
}



/**
 * INTERRUPTS
 */
void headUpDownPulseTimer() {
  headUpDownCurrentTime  = micros();
  if (headUpDownCurrentTime > headUpDownStartTime) {
    headUpDownPulses = headUpDownCurrentTime - headUpDownStartTime;
    headUpDownStartTime = headUpDownCurrentTime;
  }
}

void headRightLeftPulseTimer() {
  headRightLeftCurrentTime  = micros();
  if (headRightLeftCurrentTime > headRightLeftStartTime) {
    headRightLeftPulses = headRightLeftCurrentTime - headRightLeftStartTime;
    headRightLeftStartTime = headRightLeftCurrentTime;
  }
}

void backForwardPulseTimer() {
  backForwardCurrentTime  = micros();
  if (backForwardCurrentTime > backForwardStartTime) {
    backForwardPulses = backForwardCurrentTime - backForwardStartTime;
    backForwardStartTime = backForwardCurrentTime;
  }
}

byte GetPWM(byte pin) {
  unsigned long highTime = pulseIn(pin, HIGH, 50000UL);  // 50 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 50000UL);    // 50 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}
