#include <Scheduler.h>
#include "LSM6DS3.h"
#include <Dps310.h>
#include "Wire.h"
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
Dps310 myDPS310 = Dps310();


#define mySerial SerialUSB
//#define mySerial Serial1
#define myRelay D1

//Status
bool sLock = true;
byte RelayCount = 0;
byte SensorCount=0;

void setup() {
  pinMode(myRelay, OUTPUT);
  digitalWrite(myRelay, LOW);

  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);

  mySerial.begin(115200);
  
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
  
    myDPS310.begin(Wire);

  // put your setup code here, to run once:
  Scheduler.startLoop(loop_alive_LED);
  Scheduler.startLoop(loop_Rcv);
  Scheduler.startLoop(loop_Relay);
}

void loop() {
  //Sensor用
 Serial.print(myIMU.readFloatAccelX(), 3);
    Serial.print(',');
    Serial.print(myIMU.readFloatAccelY(), 3);
    Serial.print(',');
    Serial.print(myIMU.readFloatAccelZ(), 3);
    Serial.print(',');
    Serial.print(myIMU.readFloatGyroX(), 3);
    Serial.print(',');
    Serial.print(myIMU.readFloatGyroY(), 3);
    Serial.print(',');
    Serial.print(myIMU.readFloatGyroZ(), 3);
    Serial.println();

  if(SensorCount>0){
    delay(100);
    SensorCount--;
  }else{
    delay(10000);
    }

}
// Task no.2: blink LED with 0.1 second delay.
void loop_alive_LED() {
  if (sLock) {  //ロックされている場合
    digitalWrite(LEDG, LOW);
    delay(100);
    digitalWrite(LEDG, HIGH);
    delay(100);
  }
  else {  //ロックが解除されている場合
    digitalWrite(LEDR, LOW);
    delay(100);
    digitalWrite(LEDR, HIGH);
    delay(100);
  }
}

// Task no.3: accept commands from Serial port
// '0' turns off LED
// '1' turns on LED
void loop_Rccv() {
  if (Serial.available()) {
    String str = Serial.readString();
    str.trim();
    if (str.length() = 3) {
      if (str.
    }
  if
    if (c == '0') {
        //  digitalWrite(led3, LOW);
        Serial.println("Led turned off!");
      }
    if (c == '1') {
      //  digitalWrite(led3, HIGH);
      Serial.println("Led turned on!");
    }
  }

  // IMPORTANT:
  // We must call 'yield' at a regular basis to pass
  // control to other tasks.
  yield();
}
void loop_Relay() {
  if ((!sLock)&&(RelayCount > 0)) {
    digitalWrite(myRelay, HIGH);
    RelayCount--;
  }
  else {
    digitalWrite(myRelay, LOW);
  }
  delay(100);
}
