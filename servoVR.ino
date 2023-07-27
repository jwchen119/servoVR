#include <Servo.h>
#include <EEPROM.h>
#include <IRremote.hpp>

// #include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.

#define PIN_RECV 2
#define DEBUG
#define IR_LEARN_KEY 5
#define IR_VOLUME_UP
#define IR_VOLUME_DOWN
#define IR_POWER
Servo passiveServo;
Servo activeServo;

int eeAddress = 0; //EEPROM address to start reading from
int passiveServoAnalogOut = A2;  // passiveServo
int activeServoAnalogOut = A1;   // activeServo
unsigned long previousMillis = 0;
unsigned long interval = 10;
unsigned long interval2 = 1000;
unsigned long currentMillis;
bool reversePos = 1;
bool IRLearnState;
int IRLearnSE = 1;
int pS, aS;

struct {
  int IRProtocol;
  int IRAddress;
  int IRCommand;
} IRremoteCode[10];

// unsigned int servoValue0Deg, servoValue180Deg; // Vaiables to store min and max values of servo's pot


void setup() {
  passiveServo.attach(12);
  activeServo.attach(11);
  Serial.begin(9600);
  passiveServo.detach();
  IrReceiver.begin(PIN_RECV);
  pinMode(IR_LEARN_KEY, INPUT_PULLUP);
  delay(30);
  IRLearnState = (digitalRead(IR_LEARN_KEY) == LOW);
  for (int i = 0; i < 3; i++) {
    EEPROM.get(eeAddress, IRremoteCode[i]);
    Serial.println(IRremoteCode[i].IRProtocol);
    Serial.println(IRremoteCode[i].IRAddress);
    Serial.println(IRremoteCode[i].IRCommand);
    eeAddress += sizeof(IRremoteCode[i]);
  }
  
}

void loop() {
  while (IRLearnState) {
      switch (IRLearnSE) {
        case 1:
          // Serial.println("VOLUME+");
          if (IrReceiver.decode()) {
            IRremoteCode[0].IRProtocol = IrReceiver.decodedIRData.protocol;
            IRremoteCode[0].IRAddress = IrReceiver.decodedIRData.address;
            IRremoteCode[0].IRCommand = IrReceiver.decodedIRData.command;
            IrReceiver.resume();
            IRLearnSE = IRLearnSE + 1;
          }
        break;

        case 2:
          // Serial.println("VOLUME-");
          if (IrReceiver.decode()) {
            IRremoteCode[1].IRProtocol = IrReceiver.decodedIRData.protocol;
            IRremoteCode[1].IRAddress = IrReceiver.decodedIRData.address;
            IRremoteCode[1].IRCommand = IrReceiver.decodedIRData.command;
            IrReceiver.resume();
            IRLearnSE = IRLearnSE + 1;
          }
        break;

        case 3:
        // Serial.println("POWER");
        if (IrReceiver.decode()) {
            IRremoteCode[2].IRProtocol = IrReceiver.decodedIRData.protocol;
            IRremoteCode[2].IRAddress = IrReceiver.decodedIRData.address;
            IRremoteCode[2].IRCommand = IrReceiver.decodedIRData.command;
            IrReceiver.resume();
            IRLearnSE = IRLearnSE + 1;
          }
        break;
      }
    if (IRLearnSE == 4) {
      // Serial.println("VOLUME+");
      // Serial.println(IRremoteCode[0].IRProtocol);
      // Serial.println(IRremoteCode[0].IRAddress);
      // Serial.println(IRremoteCode[0].IRCommand);
      // Serial.println("VOLUME-");
      // Serial.println(IRremoteCode[1].IRProtocol);
      // Serial.println(IRremoteCode[1].IRAddress);
      // Serial.println(IRremoteCode[1].IRCommand);
      // Serial.println("POWER");
      // Serial.println(IRremoteCode[2].IRProtocol);
      // Serial.println(IRremoteCode[2].IRAddress);
      // Serial.println(IRremoteCode[2].IRCommand);
      // Serial.println(sizeof(IRremoteCode[0]));
      // Serial.println(sizeof(IRremoteCode[1]));
      // Serial.println(sizeof(IRremoteCode[2]));
      for (int i = 0; i < 3; i++) {
        EEPROM.put(eeAddress, IRremoteCode[i]);
        eeAddress += sizeof(IRremoteCode[i]);
      }
      Serial.println("done.");
      IRLearnSE = IRLearnSE+ 1;
    }
  }

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
      IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data
      Serial.println();
      // Serial.println(IrReceiver.decodedIRData.decodedRawData);
      // Serial.println(IrReceiver.decodedIRData.protocol);
      // Serial.println(IrReceiver.decodedIRData.address);
      // Serial.println(IrReceiver.decodedIRData.command);
    #endif

    if(!IRLearnState) {
      int prePassivePos = map(analogRead(passiveServoAnalogOut), 54, 607, 0, 180);
      int preActivePos = map(analogRead(activeServoAnalogOut), 54, 607, 0, 180);
      if (IrReceiver.decodedIRData.address == 0 && IrReceiver.decodedIRData.command == 24 && prePassivePos < 176) {
        // if (IrReceiver.decodedIRData.address == 4 && IrReceiver.decodedIRData.command == 2 && prePassivePos < 176) {
        #ifdef DEBUG
          Serial.println("V+");
        #endif
        remoteMove(prePassivePos, prePassivePos + 5);
      } else if (IrReceiver.decodedIRData.address == 0 && IrReceiver.decodedIRData.command == 82 && prePassivePos > 4) {
        // } else if (IrReceiver.decodedIRData.address == 4 && IrReceiver.decodedIRData.command == 3 && prePassivePos > 4) {
        #ifdef DEBUG
          Serial.println("V-");
        #endif
        remoteMove(prePassivePos, prePassivePos - 5);
      }
    } else {
      IrReceiver.printIRResultShort(&Serial); // Prints a summary of the received data
      Serial.println();
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
