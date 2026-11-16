#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

//we are a server / peripheral (slave), computer is client / central (master)

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define DEVICE_NAME "ESP32-Brocoli"   // matches GVSController.targetDeviceName
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define DAC_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pDACCharacteristic = NULL;

QueueHandle_t cmdQueue = NULL; 

enum : uint8_t { CMD_STOP = 0, CMD_START = 1, CMD_DATA = 2 };

struct __attribute__((packed)) GVSPacket {
  uint8_t command;     // 0 = STOP, 1 = START, 2 = DATA
  uint8_t direction;   // 0 / 1 polarity
  int32_t frequency;   // currently unused (DC)
  int32_t amplitude;   // microamps
};
static_assert(sizeof(GVSPacket) == 10, "GVSPacket must be exactly 10 bytes");

// ---------------- Hardware mapping: FILL THESE IN FOR YOUR CIRCUIT ----------------
// Microamps delivered per DAC count through the Howland pump. Measure it (e.g. with the INA219)
// before enabling. Compilation is blocked until you set it, so a guess can't reach a person.
#define UA_PER_DAC_COUNT_CALIBRATED 1
const float UA_PER_DAC_COUNT =  0.000805;     // <-- your measured value (isn't this just 3.3/4096??? idk)
#if !UA_PER_DAC_COUNT_CALIBRATED
#error "Set UA_PER_DAC_COUNT to your measured value, then set UA_PER_DAC_COUNT_CALIBRATED to 1."
#endif


const int32_t HARD_MAX_UA = 3000;        // independent of whatever Unity sends
const uint32_t WATCHDOG_MS = 200;        // no packet for this long -> output zero (Unity sends at 30-100 Hz)


const int32_t RAMP_VAL = 25; //max change of 25uA in 2.5 ms (aka 100uA in 10ms, matching Unity limits)

//const int ledPin = 2; // Use the appropriate GPIO pin for your setup

// Define pins for I2C Bus 2 (Wire1)
const int SDA_2 = 18;
const int SCL_2 = 19;

//ADC/Potentiometer Pin
const int POT_PIN = 33;

// MCP4725 is 12-bit
const long DAC_MAX = 4095;
const float DAC_VREF = 3.3;  // DAC supply voltage (used only for the printed estimate)

const int   ADC_MAX     = 4095;  // 12-bit resolution
const float ADC_REF_VOLT = 3.3;  // approximate; ESP32 ADC is not perfectly linear



// // Declare task handles
// //TaskHandle_t BlinkTaskHandle = NULL;
// TaskHandle_t BLETaskHandle = NULL;
// TaskHandle_t DACTaskHandle = NULL;
// //TaskHandle_t ReadCurrentTaskHandle = NULL;

// Declare objects
Adafruit_MCP4725 dac;
Adafruit_INA219 ina219;

//SemaphoreHandle_t dac_sem = NULL;
//SemaphoreHandle_t ina_sem = NULL;

// Shared state (written by one task, read by StatusTask for printing only)
volatile bool deviceConnected = false;
volatile bool armed = false;
volatile int32_t output_uA = 0;
volatile uint32_t rxCount = 0, rejectCount = 0, watchdogTrips = 0;


class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) {
    deviceConnected = true;
    // Ask for a 7.5-15 ms connection interval so 100 Hz writes fit. The PC may choose otherwise.
    s->updateConnParams(param->connect.remote_bda, 6, 12, 0, 400);
  }

  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    // Tell the DAC task to zero NOW (it is the only one allowed to touch the DAC).
    GVSPacket stop = {};
    stop.command = CMD_STOP;
    xQueueOverwrite(cmdQueue, &stop);
    BLEDevice::startAdvertising();
  }
};

class DacCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    // Validate and hand off. No Serial (a println takes ~1 ms per 10 chars at 115200 baud),
    // no I2C, no String allocation.
    if (c->getLength() != sizeof(GVSPacket)) { rejectCount++; return; }

    GVSPacket p;
    memcpy(&p, c->getData(), sizeof(p));   // ESP32 and Unity are both little-endian

    if (p.command > CMD_DATA || p.direction > 1 || p.amplitude < 0) { rejectCount++; return; }

    rxCount++;
    xQueueOverwrite(cmdQueue, &p);
  }
};

static int last_dac_out = -1; // 0 - 4095
static int lastDir = 0;
static float last_pot = 1; // default unscaled

static int32_t prev_unity_amp = 0; //previous value actually sent from unity (scaled)
static int32_t prev_actual_amp = 0; //previous/latest value (-3000 to 3000) used for DAC (aka ramp up/down) (scaled)
static int32_t unity_amp = 0; //scaled target (newest value actually sent from unity)
static uint8_t unity_dir = 0;


void writeHardwareTask(void *parameter) {
  for (;;) {
    int dir = unity_dir > 0 ? 1 : -1;
    int32_t scaled_sign_val = (unity_amp + 0.5f) * dir; // -3000 to 3000 

    if (abs(scaled_sign_val - prev_actual_amp) > RAMP_VAL) {
      if (scaled_sign_val < prev_actual_amp) {
        scaled_sign_val = prev_actual_amp - RAMP_VAL;
      }

      else {
        scaled_sign_val = prev_actual_amp + RAMP_VAL;
      }

    }

    prev_actual_amp = scaled_sign_val;

    uint16_t dac_out = map(scaled_sign_val, -3000, 3000, 0, 4095);

    Serial.print("scaled unity val: ");
    Serial.print(unity_amp);
    Serial.print(" scaled signed val: ");
    Serial.print(scaled_sign_val );
    Serial.print(" dac_out: ");
    Serial.print(dac_out);
    Serial.print(" dir: ");
    Serial.println(unity_dir);

    if (dac_out != last_dac_out) {                // skip redundant I2C writes
      dac.setVoltage(dac_out, false);
      last_dac_out = dac_out;
      lastDir = dir;
    }
      
    vTaskDelay(pdMS_TO_TICKS(2.5));

  }
}

// Reads a line from the Serial Monitor and sets the DAC to that value (0-4095)
void UnityReadTask (void *parameter) {
  GVSPacket p;
  int32_t target = 0;

  for(;;) { //infinite loop
    if (xQueueReceive(cmdQueue, &p, pdMS_TO_TICKS(WATCHDOG_MS)) == pdTRUE) {
      switch (p.command) {
        case CMD_STOP:
          armed = false;
          target = 0;
          break;
        case CMD_START:
          armed = true;
          target = 0;
          break;
        case CMD_DATA: 
          int32_t amp = p.amplitude;                       // copy out of the packed struct
          target = amp < HARD_MAX_UA ? amp : HARD_MAX_UA;
          //target = armed ? (p.direction ? a : -a) : 0;   // DATA before START is ignored
          break;
      }
    }
    else {
      // Watchdog: heartbeat lost. Zero now; stay armed so output resumes (ramped) when packets return.
      if (output_uA != 0) {
        watchdogTrips++;
        target = 0;
      }
    }
    //applyOutput(target);
    //writeHardware(target, p.direction);
    prev_unity_amp = unity_amp;
    unity_amp = target * last_pot; //scaled!
    unity_dir = p.direction;

    vTaskDelay(pdMS_TO_TICKS(5)); //5 ms

  }
}

void PotTask (void *parameter) {
  for (;;) {
    int pot_raw = analogRead(POT_PIN);
    float pot_percent = (pot_raw * 100.0) / ADC_MAX;
    if (pot_percent < 0.1) {
      pot_percent = 0.1;
    }

    last_pot = pot_percent;

    vTaskDelay(pdMS_TO_TICKS(10)); //10 ms

  }

}


void StatusTask(void *parameter) {
  bool wasConnected = false;
  uint32_t lastRx = 0;
  //vTaskDelay(pdMS_TO_TICKS(1000));

  bool connected = deviceConnected;
  if (connected != wasConnected) {
    Serial.println(connected ? "Device connected" : "Device disconnected, advertising");
    wasConnected = connected;
  }
  
  for(;;){

    //if (!connected) continue;

    float curr = (float)readCurrent_uA();
    output_uA = curr;

    float scaled_pot = last_pot * 100;
    GVSPacket p;
    p = {CMD_DATA, lastDir, (int32_t) scaled_pot , (int32_t) curr}; //maybe have issue w/ lastDir since it's updated in another task but idk
    // struct __attribute__((packed)) GVSPacket {
    //   uint8_t command;     // 0 = STOP, 1 = START, 2 = DATA
    //   uint8_t direction;   // 0 / 1 polarity
    //   int32_t frequency;   // currently unused (DC)
    //   int32_t amplitude;   // microamps
    // };


   // float curr = (float)readCurrent_uA();
    pSensorCharacteristic->setValue((uint8_t *)&p, sizeof(GVSPacket));
    pSensorCharacteristic->notify();

    uint32_t rx = rxCount;
    // Serial.printf("rx %lu pkt/s | rejected %lu | wd trips %lu | armed %d | out %ld uA  | dir %d \n  |   pot strength %f %"),
    //               (unsigned long)(rx - lastRx), (unsigned long)rejectCount, (unsigned long)watchdogTrips,
    //               (int)armed, (long)output_uA, (int)lastDir, last_pot);
    lastRx = rx;
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(ledPin, OUTPUT);

  //INA219 Current Sensor Setup
  ina219Init();

  //ADC and Potentiometer Setup
  analogReadResolution(12);  // 0-4095
  pinMode(POT_PIN, INPUT);

  // MCP4725 External DAC Setup
  Wire1.begin(SDA_2, SCL_2);
  dac.begin(0x60, &Wire1);
  dac.setVoltage(0, false);  // Start at 0V
  last_dac_out = 0;
  last_pot = 1;

  cmdQueue = xQueueCreate(1, sizeof(GVSPacket));

  // Create the BLE Device
  BLEDevice::init(DEVICE_NAME);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);


  // Create the ON button Characteristic
  pDACCharacteristic = pService->createCharacteristic(
    DAC_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE |  BLECharacteristic::PROPERTY_WRITE_NR);

  // Register the callback for the DAC set characteristic
  pDACCharacteristic->setCallbacks(new DacCharCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  //pDACCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  xTaskCreatePinnedToCore(
    writeHardwareTask,         // Task function
    "writeHardwareTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    4,                 // Priority
    NULL,  // Task handle
    1                  // Core 1 //double check this
  );

  xTaskCreatePinnedToCore(
    UnityReadTask,         // Task function
    "UnityReadTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    3,                 // Priority
    NULL,  // Task handle
    1                  // Core 1 //double check this
  );

  xTaskCreatePinnedToCore(
    PotTask,         // Task function
    "PotTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    2,                 // Priority
    NULL,  // Task handle
    1                  // Core 1 //double check this
  );
  
  xTaskCreatePinnedToCore(
    StatusTask,         // Task function
    "StatusTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    NULL,  // Task handle
    1                  // Core 1 //double check this
  );
}

void loop() {
  //nothing, things run in tasks
}
