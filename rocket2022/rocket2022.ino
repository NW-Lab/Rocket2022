#include <Scheduler.h>
#include "LSM6DS3.h"
#include <Adafruit_DPS310.h>
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

Adafruit_DPS310 myDps;
Adafruit_Sensor *dps_temp = myDps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = myDps.getPressureSensor();

float pressureInit;

#define mySerial SerialUSB
//#define mySerial Serial1
#define myRelay D1


//Status
bool sLock = true;
byte RelayCount = 0;
byte SensorCount = 0;
byte PcCount = 0;

//Function
float P2High(float pressure, float pressureInit, float temp) {
  return ((pow((pressureInit / pressure), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void setup() {
  int16_t ret;
  mySerial.begin(115200);
  pinMode(myRelay, OUTPUT); digitalWrite(myRelay, LOW);

  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);

  if (myIMU.begin() != 0) {
    Serial.println("Device(IMU) error");
  } else {
    Serial.println("Device(IMU) OK!");
  }

  if (! myDps.begin_I2C()) {
    Serial.println("Failed to find DPS");
    while (1) yield();
  }
  Serial.println("DPS310 OK!");
  sensors_event_t  pressure_event;
  myDps.configurePressure(DPS310_128HZ, DPS310_8SAMPLES);
  myDps.configureTemperature(DPS310_128HZ, DPS310_8SAMPLES);
  while ( !myDps.pressureAvailable()) {
    delay(1);
  }
  delay(1000);
  dps_pressure->getEvent(&pressure_event);
  pressureInit = pressure_event.pressure;

  // put your setup code here, to run once:
  Scheduler.startLoop(loop_alive_LED);
  Scheduler.startLoop(loop_Rcv);
  Scheduler.startLoop(loop_Relay);
}

void loop() {
  int16_t ret;
  sensors_event_t temp_event, pressure_event;
  float pressure, temp;
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

  while (!myDps.temperatureAvailable() || !myDps.pressureAvailable()) {
    delay(1);
  }
  myDps.getEvents(&temp_event, &pressure_event);
  temp = temp_event.temperature;
  pressure = pressure_event.pressure;
  Serial.print("Temperature: ");
  Serial.print(temp);
  Serial.println(" degrees of Celsius");
  Serial.print("/ Pressoure: ");
  Serial.print(pressure);
  Serial.print("/ height: ");
  Serial.print(P2High(pressure, pressureInit, temp));

  if (SensorCount > 0) {
    delay(100 - 62);
    SensorCount--;
  } else {
    delay(1000 - 62);
  }

}
void loop_alive_LED() {
  // 内蔵LEDは、PullUpされているので反転動作
  if (sLock) {  //ロックされている場合は緑
    digitalWrite(LEDG, LOW);
    delay(100);
    if (PcCount != 0)digitalWrite(LEDG, HIGH);
    delay(100);
  }
  else {  //ロックが解除されている場合は赤
    digitalWrite(LEDR, LOW);
    delay(100);
    if (PcCount != 0)digitalWrite(LEDR, HIGH);
    delay(100);
  }

  if (PcCount != 0)PcCount--;
}

void loop_Rcv() {

  if (Serial.available()) {
    String str = Serial.readStringUntil('\n');
    str.trim();
    if (str.length() > 0) {
      PcCount = 150; //受信したらPcCountを150(1.5秒)を追加
      str.toLowerCase();//小文字に変換
      if (str.equals("reset")) {
        Serial.println("\n!!!Reset!!!");
        sensors_event_t  pressure_event;
        while ( !myDps.pressureAvailable()) {
          delay(1);
        }
        dps_pressure->getEvent(&pressure_event);
        pressureInit = pressure_event.pressure;
      }
      else if (str.equals("unlock")) {
        Serial.println("\n***unLock***");
        sLock = false;
      }
      else if (str.equals("lock")) {
        Serial.println("\n!!!Lock!!!");
        sLock = true;
      }
      else if (str.equals("relayon")) {
        Serial.println("\n***Relay On***");
        SensorCount = 150; //15秒
        RelayCount = 30; //3秒
      }
      else if (str.equals("relayoff")) {
        Serial.println("\n!!!Relay Off!!!");
        RelayCount = 0; //0秒-->Off
        digitalWrite(myRelay, LOW);

      }

    }
  }
  yield();
}
void loop_Relay() {
  if ((!sLock) && (RelayCount > 0)) {
    digitalWrite(myRelay, HIGH);
    RelayCount--;
  }
  else {
    digitalWrite(myRelay, LOW);
  }

  delay(100);
}
