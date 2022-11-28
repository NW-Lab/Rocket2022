#include <Scheduler.h>
#include "LSM6DS3.h"
#include <Adafruit_DPS310.h>
#include "Wire.h"

LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

Adafruit_DPS310 myDps;
Adafruit_Sensor *dps_temp = myDps.getTemperatureSensor();
Adafruit_Sensor *dps_pressure = myDps.getPressureSensor();

float pressureInit;

//#define mySerial2 SerialUSB
#define mySerial Serial1
#define myRelay D1

//Status
bool sLock = true;
byte RelayCount = 0;
int SensorCount = 0;
byte PcCount = 0;
float sRSSI = 0.0;
float sBattery = 0.0;

//Function
float P2High(float pressure, float pressureInit, float temp) {
  return ((pow((pressureInit / pressure), 1 / 5.257) - 1.0) * (temp + 273.15)) / 0.0065;
}

void setup() {
  int16_t ret;
  mySerial.begin(115200);
  //mySerial2.begin(115200);
  pinMode(myRelay, OUTPUT); digitalWrite(myRelay, LOW);

  pinMode(LEDR, OUTPUT); digitalWrite(LEDR, HIGH);
  pinMode(LEDG, OUTPUT); digitalWrite(LEDG, HIGH);
  pinMode(LEDB, OUTPUT); digitalWrite(LEDB, HIGH);

  if (myIMU.begin() != 0) {
    //mySerial2.println("Device(IMU) error");
  } else {
    //mySerial2.println("Device(IMU) OK!");
  }

  if (! myDps.begin_I2C()) {
    //mySerial2.println("Failed to find DPS");
    while (1) yield();
  }
  //mySerial2.println("DPS310 OK!");
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

/***  
  float ax=myIMU.readFloatAccelX();
  float ay=myIMU.readFloatAccelY();
  float az=myIMU.readFloatAccelZ();
  float gx=myIMU.readFloatGyroX();
  float gy=myIMU.readFloatGyroY();
  float gz=myIMU.readFloatGyroZ();
**/
  float ax=-1.0*myIMU.readFloatAccelY();
  float ay=myIMU.readFloatAccelZ();
  float az=-1.0*myIMU.readFloatAccelX();
  float gx=-1.0*myIMU.readFloatGyroY();
  float gy=myIMU.readFloatGyroZ();
  float gz=-1.0*myIMU.readFloatGyroX();
  
  mySerial.print("{");
  mySerial.print("\"lock\":");
  if (sLock)mySerial.print("true");
  else mySerial.print("false");
  mySerial.print(",\"RelayOn\":");
  if (RelayCount > 0)mySerial.print("true");
  else mySerial.print("false");
  mySerial.println("}");

  mySerial.print("{\"AccelX\":");
  mySerial.print(ax, 3);
  mySerial.print(',');

  mySerial.print("\"AccelY\":");
  mySerial.print(ay, 3);
  mySerial.print(',');

  mySerial.print("\"AccelZ\":");
  mySerial.print(az, 3);
  mySerial.println("}");

  mySerial.print("{\"GyroX\":");
  mySerial.print(gx, 3);
  mySerial.print(',');

  mySerial.print("\"GyroY\":");
  mySerial.print(gy, 3);
  mySerial.print(',');

  mySerial.print("\"GyroZ\":");
  mySerial.print(gz, 3);
  mySerial.println("}");

  while (!myDps.temperatureAvailable() || !myDps.pressureAvailable()) {
    delay(1);
  }
  myDps.getEvents(&temp_event, &pressure_event);
  temp = temp_event.temperature;
  pressure = pressure_event.pressure;

  mySerial.print("{\"Temperature\":");
  mySerial.print(temp, 1);
  mySerial.print(',');

  mySerial.print("\"Pressure\":");
  mySerial.print(pressure, 2);
  mySerial.print(',');

  mySerial.print("\"Height\":");
  mySerial.print(P2High(pressure, pressureInit, temp), 2);
  mySerial.println("}");
  /**
    mySerial.print("{\"RSSI\":");
    mySerial.print(sRSSI, 2);
    mySerial.print(',');

    mySerial.print("\"Battery\":");
    mySerial.print(sBattery, 2);

    mySerial.println("}");
    **/
  /*
    //Sensor用
    Serial.print("[{");
    Serial.print("\"lock\":");
    Serial.print(sLock);
    Serial.print(',');

    Serial.print("\"AccelX\":");
    Serial.print(myIMU.readFloatAccelX(),3);
    Serial.print(',');

    Serial.print("\"AccelY\":");
    Serial.print(myIMU.readFloatAccelY()),3;
    Serial.print(',');

     Serial.print("\"AccelZ\":");
    Serial.print(myIMU.readFloatAccelZ(),3);
    Serial.print(',');

     Serial.print("\"GyroX\":");
    Serial.print(myIMU.readFloatGyroX(),3);
    Serial.print(',');

     Serial.print("\"GyroY\":");
    Serial.print(myIMU.readFloatGyroY(),3);
    Serial.print(',');

     Serial.print("\"GyroZ\":");
    Serial.print(myIMU.readFloatGyroZ(),3);
    Serial.print(',');


    while (!myDps.temperatureAvailable() || !myDps.pressureAvailable()) {
     delay(1);
    }
    myDps.getEvents(&temp_event, &pressure_event);
    temp = temp_event.temperature;
    pressure = pressure_event.pressure;

     Serial.print("\"Temperature\":");
    Serial.print(temp,1);
    Serial.print(',');

     Serial.print("\"Pressure\":");
    Serial.print(pressure,2);
    Serial.print(',');

     Serial.print("\"Height\":");
    Serial.print(P2High(pressure, pressureInit, temp),2);
    Serial.print(',');

     Serial.print("\"RSSI\":");
    Serial.print(sRSSI,2);
    Serial.print(',');

     Serial.print("\"Battery\":");
    Serial.print(sBattery,2);

    Serial.println("}]");
  */
  /*****
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
  *****/
  /**
    if (SensorCount > 0) {
    //    delay(100 - 62);
    delay(100);
    SensorCount--;
    } else {
    delay(1000);
    }
  **/
  delay(100);
}
void loop_alive_LED() {
  // 内蔵LEDは、PullUpされているので反転動作
  if (sLock) {  //ロックされている場合は緑
    digitalWrite(LEDG, LOW);
    digitalWrite(LEDR, HIGH);
    delay(100);
    if (PcCount != 0)digitalWrite(LEDG, HIGH);
    delay(100);
  }
  else {  //ロックが解除されている場合は赤
    digitalWrite(LEDR, LOW);
    digitalWrite(LEDG, HIGH);
    delay(100);
    if (PcCount != 0)digitalWrite(LEDR, HIGH);
    delay(100);
  }
  if (PcCount != 0)PcCount--;
}

void loop_Rcv() {

  if (mySerial.available()) {
    String str = mySerial.readStringUntil('\n');
    str.trim();
    if (str.length() > 0) {
      PcCount = 150; //受信したらPcCountを150(1.5秒)を追加
      str.toLowerCase();//小文字に変換
      if (str.equals("reset")) {
        mySerial.println("!reset");
        //mySerial2.println("**Reset**");
        sensors_event_t  pressure_event;
        while ( !myDps.pressureAvailable()) {
          delay(1);
        }
        dps_pressure->getEvent(&pressure_event);
        pressureInit = pressure_event.pressure;
      }
      else if (str.equals("unlock")) {
        mySerial.println("!unlock");
        //mySerial2.println("**unlock**");
        sLock = false;
      }
      else if (str.equals("lock")) {
        mySerial.println("!lock");
        //mySerial2.println("**lock**");
        sLock = true;
        RelayCount = 0;
      }
      else if (str.equals("relayon")) {
        mySerial.println("!RelayOn");
        //mySerial2.println("**RelayOn**");
        if (!sLock) {
          SensorCount = 150; //15秒
          RelayCount = 30; //3秒
        }
      }
      else if (str.equals("relayoff")) {
        mySerial.println("!RelayOff");
        //mySerial2.println("**RelayOff**");
        RelayCount = 0; //0秒-->Off
        digitalWrite(myRelay, LOW);
      }
      else if (str.equals("fast")) {
        mySerial.println("!fast");
        //mySerial2.println("**fast**");
        SensorCount = 300; //30秒
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
