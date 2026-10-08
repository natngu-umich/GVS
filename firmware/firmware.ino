#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

// Define pin for onboard LED
#define LED_PIN 2

// Define pins for I2C Bus 2 (Wire1)
const int SDA_2 = 18;
const int SCL_2 = 19;

// Declare task handle
TaskHandle_t BlinkTaskHandle = NULL;
TaskHandle_t DACTaskHandle = NULL;
TaskHandle_t ReadCurrentTaskHandle = NULL;

// Declare objects
Adafruit_MCP4725 dac;
Adafruit_INA219 ina219;

// Tasks!
void BlinkTask(void *parameter) {
  for (;;) { // Infinite loop
    digitalWrite(LED_PIN, HIGH);
    Serial.println("BlinkTask: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    Serial.println("BlinkTask: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print("BlinkTask running on core ");
    Serial.println(xPortGetCoreID());
  }
}

void DACTask(void *parameter) {
  for (;;) { // Infinite loop
    unsigned long startTime = micros();
    dac.setVoltage(2048, false); 
    unsigned long duration = micros() - startTime;
    Serial.print("Task took: ");
    Serial.print(duration);
    Serial.println(" microseconds");
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms
    
    // Set output to max scale (3.3V)
    dac.setVoltage(4095, false); 
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms
  }
}

void ReadCurrentTask(void *parameter) {
  for (;;) { // Infinite loop
    unsigned long startTime = micros();
    Serial.println(ina219.getCurrent_mA());
    unsigned long duration = micros() - startTime;
    Serial.print("Task took: ");
    Serial.print(duration);
    Serial.println(" microseconds");
    vTaskDelay(50 / portTICK_PERIOD_MS); // 100ms
  }
}

void setup() {
  Serial.begin(115200);
  
  // LED Setup
  pinMode(LED_PIN, OUTPUT);

  // INA219 Current Sensor Setup
  Wire.setClock(400000);
  ina219.begin();
  ina219.setCalibration_16V_400mA();

  // MCP4725 External DAC Setup
  Wire1.begin(SDA_2, SCL_2);
  dac.begin(0x60, &Wire1);

  xTaskCreatePinnedToCore(
    BlinkTask,         // Task function
    "BlinkTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &BlinkTaskHandle,  // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    DACTask,         // Task function
    "DACTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    2,                 // Priority
    &DACTaskHandle,  // Task handle
    1                  // Core 1
  );

  xTaskCreatePinnedToCore(
    ReadCurrentTask,         // Task function
    "ReadCurrentTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    3,                 // Priority
    &ReadCurrentTaskHandle,  // Task handle
    1                  // Core 1
  );
}

void loop() {
}