#include <Arduino.h>
#include <ModbusRTU.h>
#include "driver/timer.h"

#define SLAVE_ID 1
#define FIRST_REG 0x00
#define REG_COUNT 0x0B
#define EN 25

ModbusRTU mb;
xSemaphoreHandle xMutex;
Modbus::ResultCode err;

typedef struct MeterData{
  float Va;
  float Vb;
  float Vc;
  float Vab;
  float Vbc;
  float Vca;
}MeterData;

MeterData mtDt;

typedef union unt162float{
  uint16_t dt16[2];
  float fp;
}unt162float;

Modbus::ResultCode readSync(uint8_t address, uint16_t start, uint16_t num, uint16_t* buf) {
  xSemaphoreTake(xMutex, portMAX_DELAY);
  if (mb.slave()) {
    xSemaphoreGive(xMutex);
    return Modbus::EX_GENERAL_FAILURE;
  }
  Serial.printf("SlaveID: %d Hreg %d\r\n", address, start);
  mb.readHreg(address, start, buf, num, [](Modbus::ResultCode event, uint16_t, void*) {
    err = event;
    return true;
  });
  
  unsigned long startTime = millis();
  while (mb.slave()) {
    vTaskDelay(1);
    mb.task();
    
    // Check if we've been waiting too long
    if (millis() - startTime > 5000) { // 5000 ms timeout
      Serial.println("Timeout waiting for Modbus response");
      xSemaphoreGive(xMutex);
      return Modbus::EX_TIMEOUT;
    }
  }
  
  Modbus::ResultCode res = err;
  xSemaphoreGive(xMutex);
  return res;
}

void loop1(void *pvParameters);

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1);
  mb.begin(&Serial2, EN);
  mb.client(); // Initialize the Modbus master
  xMutex = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(
    loop1,   /* Task function. */
    "Task1", /* name of task. */
    10000,   /* Stack size of task */
    NULL,    /* parameter of the task */
    10,      /* priority of the task */
    NULL,    /* Task handle to keep track of created task */
    0        /* pin task to core 1 */
  );

  // Timer Group 1 Watchdog Timer configuration
  timer_config_t config = {
      .alarm_en = TIMER_ALARM_DIS,
      .counter_en = TIMER_PAUSE,
      .intr_type = TIMER_INTR_LEVEL,
      .counter_dir = TIMER_COUNT_UP,
      .auto_reload = TIMER_AUTORELOAD_DIS,
      .divider = 80   // 80 MHz / 80 = 1 MHz (1 tick per microsecond)
  };

  // Initialize the timer
  timer_init(TIMER_GROUP_1, TIMER_0, &config);

  // Set counter value to 0
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);

  // Set alarm value to a longer period (e.g., 10 seconds)
  timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 10 * 1000000);  // 10 seconds

  // Enable the interrupt
  timer_enable_intr(TIMER_GROUP_1, TIMER_0);

  // Start the timer
  timer_start(TIMER_GROUP_1, TIMER_0);
}

uint16_t res[REG_COUNT];

void loop1(void *pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (readSync(SLAVE_ID, FIRST_REG, REG_COUNT, res) == Modbus::EX_SUCCESS)
      {
        Serial.println("OK 1");
        //memcpy(&mtDt,res,REG_COUNT*2);
        Serial.print("res= ");
        Serial.print(res[0],HEX);
        Serial.println(res[1],HEX);
        float Va;
        unsigned long* p;
        p = (unsigned long*)&Va;
        *p = (unsigned long)res[0]<<16 | res[1];
        // float Va = (res[0]<<16 | res[1]);
        Serial.print("Va= ");
        Serial.println(Va,1);
        // Serial.print("Vb= ");
        // Serial.println(mtDt.Vb);
        // Serial.print("Vc= ");
        // Serial.println(mtDt.Vc);
      }
    else
      Serial.println("Error 1");
  }
}

void loop() {
  // Reset the Timer Group 1 Watchdog Timer
  timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0x00000000ULL);

  // Your existing loop code
  delay(100);
}