/*
  Modbus Library for Arduino Example - Modbus RTU Client
  Read Holding Registers from Modbus RTU Server in blocking way
  ESP8266 Example
  
  (c)2020 Alexander Emelianov (a.m.emelianov@gmail.com)
  https://github.com/emelianov/modbus-esp8266
*/
#include <Arduino.h>
//#include <ModbusRTU.h>
//#include <SoftwareSerial.h>

#define SLAVE_ID 1
#define FIRST_REG 0
#define REG_COUNT 22
#define EN 25
#define GP   0xA001 //generating polinomial

uint16_t Hregs[REG_COUNT];
void ReadHreg(uint8_t addr, uint16_t offset){
  char msg[8];
  msg[0] = addr;
  msg[1] = 0x03;
  msg[2] = (uint8_t)offset;
  msg[3] = offset>>8;
  msg[4] = 0x00;
  msg[5] = 0x06;
  msg[6] = 0xC5;
  msg[7] = 0xC8;
  // for(int i = 0; i<8;i++){
  //   Serial1.write(msg[i]);
  // }
  digitalWrite(EN,HIGH);
  Serial1.write(msg,8);
  vTaskDelay(9);
  digitalWrite(EN,LOW);
}

void setup() {

  Serial.begin(115200);
  Serial1.begin(9600);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,LOW);
}

void loop() {
  ReadHreg(0x01,0x0000);
  //Serial1.write('a');
  Serial.flush();
  Serial.println("a");
  delay(5000);
}