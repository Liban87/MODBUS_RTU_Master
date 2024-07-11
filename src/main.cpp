/*
  Modbus Library for Arduino Example - Modbus RTU Client
  Read Holding Registers from Modbus RTU Server in blocking way
  ESP8266 Example
  
  (c)2020 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266
*/
#include <Arduino.h>
#include <ModbusRTU.h>
//#include <SoftwareSerial.h>

#define SLAVE_ID 1
#define FIRST_REG 0
#define REG_COUNT 22
#define EN 21

//SoftwareSerial S(D2, D1);
ModbusRTU mb;

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS) {
    Serial.print("Request result: 0x");
    Serial.print(event, HEX);
  }
  return true;
}

void setup() {

  Serial.begin(9600);
 #if defined(ESP8266)
  S.begin(9600, SWSERIAL_8N1);
  mb.begin(&S);
 #elif defined(ESP32)
  Serial1.begin(9600, SERIAL_8N1);
  //mb.setBaudrate(9600);
  mb.begin(&Serial1);
  //mb.setInterFrameTime(40000);
 #else
  Serial1.begin(9600, SERIAL_8N1);
  mb.begin(&Serial1);
  mb.setBaudrate(9600);
 #endif
  mb.master();
  pinMode(EN,OUTPUT);
  digitalWrite(EN,LOW);
}

uint16_t Hregs[REG_COUNT];
void loop() {
  digitalWrite(EN,HIGH);
  mb.readHreg(SLAVE_ID, FIRST_REG, Hregs, REG_COUNT, cb); // Send Read Hreg from Modbus Server
  mb.task();
  vTaskDelay(25);
  digitalWrite(EN,LOW);
  float frec=(float)(Hregs[13]<<16|Hregs[12]);
  //Serial.println(frec);
  delay(5000);
}