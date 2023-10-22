const byte motorSpeedPwmPin = 2;
const byte motorDirectionPwmPin = 3;
const byte motorSpeedOutputPin = 9;
const byte motorDirectionOutputPin = 8;
unsigned long directionLastChange = 0;
int directionLastValue = 0;
const int directionChangeSeconds = 1;

void setup(){

  pinMode(motorSpeedPwmPin, INPUT);
  pinMode(motorDirectionPwmPin, INPUT);

  pinMode(13, OUTPUT);
  pinMode(motorSpeedOutputPin, OUTPUT);
  pinMode(motorDirectionOutputPin, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  byte motorSpeedInputValue = GetPWM(motorSpeedPwmPin);
  byte motorDirectionInputValue = GetPWM(motorDirectionPwmPin);


  if ( motorDirectionInputValue < 8 ) {
    Serial.println( "Bachoe Disabled");
    digitalWrite(13, LOW);
    analogWrite( motorSpeedOutputPin, 0 );
  } else {
    digitalWrite(13, HIGH);

    int motorSpeedOutputValue = map(motorSpeedInputValue, 5, 11, 0, 100);
    int motorDirectionOutputValue = map(motorDirectionInputValue, 5, 11, 0, 100);

    if ( millis() > directionLastChange + (directionChangeSeconds * 1000) ) {
      analogWrite( motorSpeedOutputPin, motorSpeedOutputValue );
    } else {
      analogWrite( motorSpeedOutputPin, 0);
      delay(1000);
    }

    if ( motorDirectionInputValue == 8) {
      // Go front
      digitalWrite( motorDirectionOutputPin, 0 );
    } else {
      digitalWrite( motorDirectionOutputPin, 1 );
    }


/*
    Serial.print( "Direction: " );
    Serial.print( motorDirectionInputValue );
    Serial.print( " - speed input: " );
    Serial.print(motorSpeedInputValue);
    Serial.print( " - speed output: " );
    Serial.println(motorSpeedOutputValue);
*/

  }

  if ( directionLastValue != motorDirectionInputValue ) {
    Serial.println("directionLastValue != motorDirectionInputValue");
    directionLastValue = motorDirectionInputValue;
    directionLastChange = millis();
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


