#include <Servo.h>
#include <EEPROM.h>
#include <IRremote.hpp>
// #include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.

#define PIN_RECV 2
// #define DEBUG
#define IR_LEARN_KEY 5
Servo passiveServo;
Servo activeServo;

int eeAddress = 0;               //EEPROM address to start reading from
int passiveServoAnalogOut = A2;  // passiveServo
int activeServoAnalogOut = A1;   // activeServo
unsigned long previousMillis = 0;
unsigned long interval = 10;
// unsigned long interval2 = 1000;
unsigned long currentMillis;
bool reversePos = 1;
bool IRLearnState;
int IRLearnSE = 1;
int pS, aS;

void IRLearningProcess();
void remoteMove(int, int);
void readFromEEPROM();

// unsigned int servoValue0Deg, servoValue180Deg; // Vaiables to store min and max values of servo's pot

struct IRremoteCode {
  long IRProtocol;
  long IRAddress;
  long IRCommand;
};
struct IRremoteCode IRVolumeUp;
struct IRremoteCode IRVolumeDown;
struct IRremoteCode IRMainPower;

void setup() {
  passiveServo.attach(12);
  activeServo.attach(11);
  Serial.begin(9600);
  passiveServo.detach();
  IrReceiver.begin(PIN_RECV);
  pinMode(IR_LEARN_KEY, INPUT_PULLUP);
  delay(30);
  IRLearnState = digitalRead(IR_LEARN_KEY);
  readFromEEPROM();
}

void loop() {
  if (!IRLearnState) IRLearningProcess();
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    pS = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
    aS = map(analogRead(activeServoAnalogOut), 54, 607, 0, 180);
#ifdef DEBUG
    // Serial.print(pS);
    // Serial.print(", ");
    // Serial.println(aS);
#endif

    if (abs(pS - aS) > 4) {
      activeServo.attach(11);
      reversePos ? activeServo.write(180 - pS) : activeServo.write(pS);
    } else {
      activeServo.detach();
    }
  }

  if (IrReceiver.decode()) {
#ifdef DEBUG
    IrReceiver.printIRResultShort(&Serial);  // Prints a summary of the received data
    Serial.println();
    Serial.println(IrReceiver.decodedIRData.decodedRawData);
    Serial.println(IrReceiver.decodedIRData.protocol);
    Serial.println(IrReceiver.decodedIRData.address);
    Serial.println(IrReceiver.decodedIRData.command);
#endif

    int prePassivePos = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
    int preActivePos = map(analogRead(activeServoAnalogOut), 54, 607, 0, 180);
    if (
      IrReceiver.decodedIRData.protocol == IRVolumeUp.IRProtocol && IrReceiver.decodedIRData.address == IRVolumeUp.IRAddress && IrReceiver.decodedIRData.command == IRVolumeUp.IRCommand && prePassivePos < 176) {
#ifdef DEBUG
      Serial.println("V+");
#endif
      remoteMove(prePassivePos, prePassivePos + 5);
    } else if (
      IrReceiver.decodedIRData.protocol == IRVolumeDown.IRProtocol && IrReceiver.decodedIRData.address == IRVolumeDown.IRAddress && IrReceiver.decodedIRData.command == IRVolumeDown.IRCommand && prePassivePos > 4) {
#ifdef DEBUG
      Serial.println("V-");
#endif
      remoteMove(prePassivePos, prePassivePos - 5);
    }
  }
  IrReceiver.resume();  // Important, enables to receive the next IR signal
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

void IRLearningProcess() {
  while (!IRLearnState) {
    switch (IRLearnSE) {
      case 1:
        Serial.println("VOLUME+");  // can be replace by the status of LED
        if (IrReceiver.decode()) {
          IRVolumeUp = (IRremoteCode){ IrReceiver.decodedIRData.protocol, IrReceiver.decodedIRData.address, IrReceiver.decodedIRData.command };
          IrReceiver.resume();
          IRLearnSE = IRLearnSE + 1;
        }
        break;

      case 2:
        Serial.println("VOLUME-");  // can be replace by the status of LED
        if (IrReceiver.decode()) {
          IRVolumeDown = (IRremoteCode){ IrReceiver.decodedIRData.protocol, IrReceiver.decodedIRData.address, IrReceiver.decodedIRData.command };
          IrReceiver.resume();
          IRLearnSE = IRLearnSE + 1;
        }
        break;

      case 3:
        Serial.println("POWER");  // can be replace by the status of LED
        if (IrReceiver.decode()) {
          IRMainPower = (IRremoteCode){ IrReceiver.decodedIRData.protocol, IrReceiver.decodedIRData.address, IrReceiver.decodedIRData.command };
          IrReceiver.resume();
          IRLearnSE = IRLearnSE + 1;
        }
        break;
    }
    if (IRLearnSE == 4) {
      eeAddress = 0;
#ifdef DEBUG
      Serial.println("VOLUME+");
      Serial.println(IRVolumeUp.IRProtocol);
      Serial.println(IRVolumeUp.IRAddress);
      Serial.println(IRVolumeUp.IRCommand);
      Serial.println("VOLUME-");
      Serial.println(IRVolumeDown.IRProtocol);
      Serial.println(IRVolumeDown.IRAddress);
      Serial.println(IRVolumeDown.IRCommand);
      Serial.println("POWER");
      Serial.println(IRMainPower.IRProtocol);
      Serial.println(IRMainPower.IRAddress);
      Serial.println(IRMainPower.IRCommand);
#endif
      EEPROM.put(eeAddress, IRVolumeUp);
      eeAddress += sizeof(IRVolumeUp);
      EEPROM.put(eeAddress, IRVolumeDown);
      eeAddress += sizeof(IRVolumeDown);
      EEPROM.put(eeAddress, IRMainPower);
      delay(30);
      Serial.println("done.");
      IRLearnSE = IRLearnSE + 1;
    }
  }
}

void readFromEEPROM() {
  eeAddress = 0; 
  EEPROM.get(eeAddress, IRVolumeUp);
  eeAddress += sizeof(IRVolumeUp);
  delay(20);
  EEPROM.get(eeAddress, IRVolumeDown);
  eeAddress += sizeof(IRVolumeDown);
  delay(20);
  EEPROM.get(eeAddress, IRMainPower);
  eeAddress += sizeof(IRMainPower);
#ifdef DEBUG
  Serial.println(IRVolumeUp.IRProtocol);
  Serial.println(IRVolumeUp.IRAddress);
  Serial.println(IRVolumeUp.IRCommand);
  Serial.println(IRVolumeDown.IRProtocol);
  Serial.println(IRVolumeDown.IRAddress);
  Serial.println(IRVolumeDown.IRCommand);
  Serial.println(IRMainPower.IRProtocol);
  Serial.println(IRMainPower.IRAddress);
  Serial.println(IRMainPower.IRCommand);
#endif
}
