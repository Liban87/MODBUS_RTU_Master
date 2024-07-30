//Incluir librerías

#include <Arduino.h>
#include <ModbusRTU.h>
#include "driver/timer.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT_Client.h"
#include "WiFi.h"

//Definir constantes

#define SLAVE_ID 1
#define FIRST_REG 0x00
#define REG_COUNT 0x0B
#define EN 25

//CONSTANTES DE PUBLICACIÓN POR MQTT
#define WIFI_SSID "Anita"
#define WIFI_PASSWORD "anasofia12"
#define HOST "192.168.118.13" //Direccion IP de la raspberry pi dentro de la red
#define PORT 1883 //Puerto por defecto para MQTT
#define USERNAME "anas.valenciae"
#define PASSWORD "anasofia12"

//CREACIÓN DEL CLIENTE
WiFiClient client;
Adafruit_MQTT_Client mqtt(&client, HOST, PORT, USERNAME, PASSWORD);

//----------------------------------------------------

// -------------
ModbusRTU mb;
xSemaphoreHandle xMutex;
Modbus::ResultCode err;

//Definir la estructura para los datos del medidor

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

//DEFINICIÓN DE OBJETOS MQTT
Adafruit_MQTT_Publish VaPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Va");
Adafruit_MQTT_Publish VbPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Vb");
Adafruit_MQTT_Publish VcPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Vc");
Adafruit_MQTT_Publish VabPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Vab");
Adafruit_MQTT_Publish VbcPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Vbc"); 
Adafruit_MQTT_Publish VcaPub = Adafruit_MQTT_Publish(&mqtt, "meterData/Vca");

//MUTEX --------------------

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

//DECLARACIÓN DE FUNCIONES 
void loop1(void *pvParameters);

void Hex2Float(uint16_t* res, MeterData* data);

void MQTT_connect();

//--------------------------------------------

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

  //CONFIGURACIÓN WIFI --------------------------------------------------
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");

    //CONFIGURACIÓN BROKER MQTT -------------------------------------------------
    MQTT_connect();
}

uint16_t res[REG_COUNT];

void loop1(void *pvParameters) {
  while (true) {
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if (readSync(SLAVE_ID, FIRST_REG, REG_COUNT, res) == Modbus::EX_SUCCESS)
      {
        Serial.println("OK 1");
        Serial.print("res= ");
        Serial.print(res[0],HEX);
        Serial.println(res[1],HEX);

        //Llamar la función 
        Hex2Float(res, &mtDt);

        //Imprmir valores
        Serial.print("Va: ");
        Serial.println(mtDt.Va);
        Serial.print("Vb: ");
        Serial.println(mtDt.Vb);
        Serial.print("Vc: ");
        Serial.println(mtDt.Vc);
        Serial.print("Vab: ");
        Serial.println(mtDt.Vab);
        Serial.print("Vbc: ");
        Serial.println(mtDt.Vbc);
        Serial.print("Vca: ");
        Serial.println(mtDt.Vca);

        // Publicar los valores a través de MQTT
        VaPub.publish(mtDt.Va);
        VbPub.publish(mtDt.Vb);
        VcPub.publish(mtDt.Vc);
        VabPub.publish(mtDt.Vab);
        VbcPub.publish(mtDt.Vbc);
        VcaPub.publish(mtDt.Vca);

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

//Función para pasar de Hexa a flotante 

void Hex2Float(uint16_t* res, MeterData* data) {
    int temp;
    float* components[] = { &data->Va, &data->Vb, &data->Vc, &data->Vab, &data->Vbc, &data->Vca };

    for (int i = 0; i < 6; ++i) {
        temp = (int)res[2 * i] << 16 | res[2 * i + 1];
        *components[i] = *((float*)&temp);
    }
}

//function for connecting esp32 to mqtt broker.
void MQTT_connect(){
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 10;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 1 second...");
       mqtt.disconnect();
       delay(1000);
       retries--;
       if (retries == 0)
         Serial.println("No Conectado");
  }
  Serial.println("MQTT Connected!");
}