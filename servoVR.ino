#include <Servo.h>
#include <IRremote.hpp>

#define PIN_RECV 2

Servo passiveServo;
Servo activeServo;

int passiveServoAnalogOut = A2;  // passiveServo
int activeServoAnalogOut = A1;   // activeServo
unsigned long previousMillis = 0;
unsigned long interval = 10;
unsigned long currentMillis;
bool reversePos = 1;
int pS, aS;

// unsigned int servoValue0Deg, servoValue180Deg; // Vaiables to store min and max values of servo's pot


void setup() {
  passiveServo.attach(12);
  activeServo.attach(11);
  Serial.begin(9600);
  passiveServo.detach();
  IrReceiver.begin(PIN_RECV);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    pS = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
    aS = map(analogRead(activeServoAnalogOut), 54, 607, 0, 180);
    // Serial.print(pS);
    // Serial.print(", ");
    // Serial.println(aS);

    if (abs(pS - aS) > 4) {
      activeServo.attach(11);
      reversePos ? activeServo.write(180 - pS) : activeServo.write(pS);
      // activeServo.write(pS);
    } else {
      activeServo.detach();
    }
  }

  if (IrReceiver.decode()) {
    // Serial.println("Received something...");
    // IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data
    // Serial.println();
    // Serial.println(IrReceiver.decodedIRData.address);
    // Serial.println(IrReceiver.decodedIRData.command);
    int prePassivePos = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
    int preActivePos = map(analogRead(activeServoAnalogOut), 54, 607, 0, 180);
    if (IrReceiver.decodedIRData.address == 0 && IrReceiver.decodedIRData.command == 24 && prePassivePos < 176) {
      Serial.println("V+");
      remoteMove(prePassivePos, prePassivePos + 5);
    } else if (IrReceiver.decodedIRData.address == 0 && IrReceiver.decodedIRData.command == 82 && prePassivePos > 4) {
      Serial.println("V-");
      remoteMove(prePassivePos, prePassivePos - 5);
    }
    IrReceiver.resume();  // Important, enables to receive the next IR signal
  }
}

// void calibration() {
//   myservo.write(0); //set the servo to 0 position
//   delay(2000); //wait for the servo to reach there
//   servoValue0Deg= analogRead(passiveServoAnalogOut); // Pot value at 0 degrees
//   Serial.println("Pot value for 0 deg is " + String(servoValue0Deg)); // Print it!
//   delay(500); //fancy delay
//   myservo.write(180); //go to 180 degrees
//   delay(2000); //wait for the servo to reach there
//   servoValue180Deg= analogRead(passiveServoAnalogOut); //pot value at 180 deg
//   Serial.println("Pot value for 180 deg is " + String(servoValue180Deg));
//   delay(500); //fancy delay
//   Serial.println("Now going to 90 Degrees"); //It does what it says
//   myservo.write(90);// going to 90 degrees
//   delay(2000);// wait for it to reach there
//   myservo.detach(); // stop the PWM so that we can freely move the servo
//   delay(1000);

// }

void remoteMove(int prePost, int targetPos) {
  passiveServo.attach(12);
  activeServo.attach(11);
  // prePost = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
  if (targetPos > prePost) {
    for (int i = prePost; i < targetPos; i++) {
      passiveServo.write(i);
      // delay(30);
      reversePos ? activeServo.write(180 - i) : activeServo.write(i);
      delay(15);
    }
  } else if (targetPos < prePost) {
    for (int i = prePost; i > targetPos; i--) {
      passiveServo.write(i);
      // delay(30);
      reversePos ? activeServo.write(180 - i) : activeServo.write(i);
      delay(15);
    }
  }
  delay(5);
  activeServo.detach();
  passiveServo.detach();
}
