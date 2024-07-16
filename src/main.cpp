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
//Tratar de dejar el EN automático

#define SLAVE_ID 1
#define FIRST_REG 0 //FUNCIONANDO COMO OFFSET
#define REG_COUNT 6
#define EN 25

ModbusRTU mb;

bool cbWrite(Modbus::ResultCode event, uint16_t transactionId, void* data) {
  Serial.printf_P("Request result: 0x%02X, Mem: %d\n", event, ESP.getFreeHeap());
  return true;
}

bool cb(Modbus::ResultCode event, uint16_t transactionId, void* data) { // Callback to monitor errors
  if (event != Modbus::EX_SUCCESS) {
    Serial.print("Request result: 0x");
    Serial.print(event, HEX);
  }
  return true;
}

void setup() {
  
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1);
  mb.begin(&Serial1, EN);
  mb.master(); //Inicializar el maestro

}

//bool coils[20];

void loop() {
  uint16_t res[REG_COUNT];
  if (!mb.slave()) {    // Check if no transaction in progress
    mb.readHreg(SLAVE_ID, FIRST_REG, res, REG_COUNT, cb); // Send Read Hreg from Modbus Server
    while(mb.slave()) { // Check if transaction is active
      mb.task();
     // delay(10);
    }
    Serial.println(res[0]);
    Serial.println(res[1]);
  }
  delay(5000);
}