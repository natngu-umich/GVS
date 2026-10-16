#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

// Define pin for onboard LED
#define LED_PIN 2

// Define pins for I2C Bus 2 (Wire1)
const int SDA_2 = 18;
const int SCL_2 = 19;

// MCP4725 is 12-bit
const long DAC_MAX = 4095;
const float DAC_VREF = 3.3;  // DAC supply voltage (used only for the printed estimate)

// Declare task handles
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
    // Serial.println("BlinkTask: LED ON");
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000ms
    digitalWrite(LED_PIN, LOW);
    // Serial.println("BlinkTask: LED OFF");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    // Serial.print("BlinkTask running on core ");
    Serial.println(xPortGetCoreID());
  }
}

// Reads a line from the Serial Monitor and sets the DAC to that value (0-4095)
void DACTask(void *parameter) {
  char buf[16];
  size_t len = 0;
  bool overflow = false;

  Serial.println("DACTask: type a value 0-4095 and press Enter to set the DAC");

  for (;;) { // Infinite loop
    while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
        if (overflow) {
          Serial.println("DACTask: input too long, ignored");
        } else if (len > 0) {
          buf[len] = '\0';

          char *end;
          long value = strtol(buf, &end, 10);

          if (*end != '\0') {
            Serial.print("DACTask: invalid input \"");
            Serial.print(buf);
            Serial.println("\" (enter an integer 0-4095)");
          } else if (value < 0 || value > DAC_MAX) {
            Serial.print("DACTask: ");
            Serial.print(value);
            Serial.println(" is out of range (0-4095)");
          } else {
            unsigned long startTime = micros();
            dac.setVoltage((uint16_t)value, false);
            unsigned long duration = micros() - startTime;

            Serial.print("DACTask: DAC set to ");
            Serial.print(value);
            Serial.print(" (~");
            Serial.print(value * DAC_VREF / DAC_MAX, 3);
            Serial.print(" V), took ");
            Serial.print(duration);
            Serial.println(" us");
          }
        }
        len = 0;
        overflow = false;
      } else if (len < sizeof(buf) - 1) {
        buf[len++] = c;
      } else {
        overflow = true;  // discard the rest of this line
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Poll serial every 10ms
  }
}

void ReadCurrentTask(void *parameter) {
  for (;;) { // Infinite loop
    // Serial.println(ina219.getCurrent_mA());
    vTaskDelay(50 / portTICK_PERIOD_MS);
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
  dac.setVoltage(0, false);  // Start at 0V

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
