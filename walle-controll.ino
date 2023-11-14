// #include <Servo.h> 
#include <RCSwitch.h>
#include <VarSpeedServo.h> 

RCSwitch rfControl = RCSwitch();
bool isWalleEnabled = false;
const byte detectWalleEnabledPin = 7;

/**
 * Wall-e sounds
 *
 * 01.wav => waleeee happy
 * 02.wav => call attention
 * 03.wav => grgrgruggrugrg
 * 04.wav => look at the ring
 * 05.wav => oooooohhhh
 * 06.wav => kisses
 * 07.wav => kisses 2
 * 08.wav => waleee long
 */

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
VarSpeedServo headUpDownDirectionServoRight;
VarSpeedServo headUpDownDirectionServoLeft;
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
VarSpeedServo headRightLeftDirectionServo;
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
 * Right arm
 */
const byte rightArmMotorPin = 8;
VarSpeedServo rightArmServo;
volatile long timeSinceItWasUp = 0;

/**
 * Oil pump
 */
const byte oilPumpPWMPin = 4;
const byte oilPumpStartupSpeed = 200;
const byte oilPumpOperationSpeed = 90;
bool oilPumpIsOn = false;
const int reduceOilPumpPressureAfterMS = 3000;
const int stopOilPumpPressureAfterMS = 5000;
VarSpeedServo oilPump;
unsigned long pumpStartTime;


/**
 * Prepare rf433 pin
 * Codes:
 * 174102677
 * 174102693
 * 130809621
 */
const byte rf433Pin = 19;
volatile long lastRfSignal;


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

  // Starts RF433Mhz module
  rfControl.enableReceive( digitalPinToInterrupt( rf433Pin ) );
  lastRfSignal = 0;

  // Right arm
  pinMode( rightArmMotorPin, OUTPUT );
  rightArmServo.attach( rightArmMotorPin );
  rightArmServo.write(0, 20, true);

  // Prepare communication with waveshield
  Serial2.begin( 9600 );

  // Prepare communication with PC
  Serial.begin( 115200 );
  Serial.println("Wall-e started!");

}

void loop() {

  detectIfWalleIsEnabled();

  if ( isWalleEnabled ) {
    modifyOilPumpSpeed();
    headUpDown();
    headLeftRight();
    goingBackForward();
    checkShouldFallHand();
  }

  if ( rfControl.available() ) {
    Serial.print("Received ");
    String music = "";
    int receivedValue = rfControl.getReceivedValue();
    Serial.print( receivedValue );

    if ( receivedValue == -26459 ) {
      music = "01.wav";
    }
    else if ( receivedValue == -26475 ) {
      showRings();
      //music = "02.wav";
    }
    else if ( receivedValue == -235 ) {
      music = "03.wav";
    }

    // Serial.print(" / ");
    // Serial.print( rfControl.getReceivedBitlength() );
    // Serial.print("bit ");
    // Serial.print("Protocol: ");
    // Serial.println( rfControl.getReceivedProtocol() );
    if ( music != "" ) {
      playMusic( music );
    }

    lastRfSignal = millis();
    rfControl.resetAvailable();
  }
  
}

void playMusic( String music ) {
    Serial.print( "Playing " );
    Serial.println( music );

    Serial2.write( music.c_str(), 11 );
    while ( Serial2.available() > 0 ) {
      Serial.print( Serial2.read() );
      delay(10);
    }

    Serial.println( "Music played");
}


void detectIfWalleIsEnabled() {
  byte walleIsEnabledInputValue = GetPWM(detectWalleEnabledPin);
  
  // Button from controll return 5 for disable and 10 e enable
  if ( walleIsEnabledInputValue > 7 ) {
    isWalleEnabled = 1;
  } else {
    isWalleEnabled = 0;
  }

  // Serial.print("is enabled: ");
  // Serial.println(isWalleEnabled);
}

void oilPumpOn() {
  // Serial.print("oilPumOn function call. isOn: " );
  // Serial.print(oilPumpIsOn);
  // Serial.println();

  // if ( ! oilPumpIsOn ) {
  //   oilPumpIsOn = true;
  //   oilPump.write( oilPumpStartupSpeed );
  //   pumpStartTime = millis();
  // }
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

  if ( detectIfIsMoving() ) {
    oilPumpOn();
  }
}

bool detectIfIsMoving() {
  if ( lastSpeedChangeTime + 200 > millis() ) {
    return true;
  }

  return false;
}

bool movedInThePastFiveSeconds() {
  if ( lastSpeedChangeTime + 5000 > millis() ) {
    return true;
  }

  return false;
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

  // if ( millis() > lastHeadUpDownChangeTime + 500 && ! movedInThePastFiveSeconds() ) {
  //   headUpDownDirectionServoRight.detach();
  //   headUpDownDirectionServoLeft.detach();
  // }

  // Serial.print( "Head puslse: ");
  // Serial.print( headUpDownPulseWidthReverse );
  // Serial.print( " - left: ");
  // Serial.print(headUpDownMotorLeftValueAngle);
  // Serial.print( " - right: ");
  // Serial.print(headUpDownMotorRightValueAngle);
  // Serial.print( " - ");
  // Serial.println();
}

// value should be between 10 - 15 - 20
void moveHeadUPDown( int value ) {
  int headUpDownMotorLeftValueAngle = constrain( map(value, 10, 20, 140, 40), 45, 130) +2;
  int headUpDownMotorRightValueAngle = constrain( map(value, 10, 20, 40, 140), 50, 135);

  headUpDownDirectionServoRight.attach(headUpDownDirectionMotorRightPin);
  headUpDownDirectionServoLeft.attach(headUpDownDirectionMotorLeftPin);
  headUpDownDirectionServoRight.write(headUpDownMotorRightValueAngle, 30, true);
  headUpDownDirectionServoLeft.write(headUpDownMotorLeftValueAngle, 30, true);
  headUpDownDirectionServoRight.wait();
  headUpDownDirectionServoLeft.wait();
  lastHeadUpDownChangeTime = millis();
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
    // headRightLeftDirectionServo.write(headRightLeftMotorValueAngle, 40, true);
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

void showRings() {
  // music call attention x 2
  playMusic( "02.wav" );
  playMusic( "02.wav" );

  // look up
  moveHeadUPDown( 10 );

  // music call attention x 2
  playMusic( "02.wav" );

  // look at right hand
  moveHeadUPDown( 10 );
  const byte rightHandAngle = 40;
  headRightLeftDirectionServo.attach(headRightLeftDirectionMotorPin);
  headRightLeftDirectionServo.write( rightHandAngle, 30, true);
  headRightLeftDirectionServo.wait();

  // make right hand up
  rightArmServo.write(150, 20, true);
  timeSinceItWasUp = millis();

  // look up
  moveHeadUPDown( 20 );
  
  // music ohhhh
  playMusic( "05.wav" );

  // Look up and centered
  headRightLeftDirectionServo.attach(headRightLeftDirectionMotorPin);
  headRightLeftDirectionServo.write( 90, 30, true);
}

void checkShouldFallHand() {
  if ( millis() > timeSinceItWasUp + 10000 ) {
    rightArmServo.write(0, 20, true);
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
