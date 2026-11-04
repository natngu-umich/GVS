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
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define DAC_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pDACCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint16_t dac_val;

//const int ledPin = 2; // Use the appropriate GPIO pin for your setup

// Define pins for I2C Bus 2 (Wire1)
const int SDA_2 = 18;
const int SCL_2 = 19;

// MCP4725 is 12-bit
const long DAC_MAX = 4095;
const float DAC_VREF = 3.3;  // DAC supply voltage (used only for the printed estimate)

// Declare task handles
//TaskHandle_t BlinkTaskHandle = NULL;
TaskHandle_t BLETaskHandle = NULL;
TaskHandle_t DACTaskHandle = NULL;
TaskHandle_t ReadCurrentTaskHandle = NULL;

// Declare objects
Adafruit_MCP4725 dac;
Adafruit_INA219 ina219;

SemaphoreHandle_t dac_sem = NULL;
SemaphoreHandle_t ina_sem = NULL;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {  //on dac, do this
  void onWrite(BLECharacteristic* pDACCharacteristic) {
    String value = pDACCharacteristic->getValue();
    Serial.print("pre-conversion value: ");
    Serial.println(value);
    if (value.length() > 0) {
      dac_val = static_cast<uint16_t>(value[1] << 8);
      dac_val += static_cast<uint16_t>(value[0]);
      Serial.print("converted value: ");
      Serial.println(dac_val);
      xSemaphoreGive(dac_sem);
    }
  }
};

// Reads a line from the Serial Monitor and sets the DAC to that value (0-4095)
void DACTask(void *pvParameters) {
  //Serial.println("DACTask: type a value 0-4095 and press Enter to set the DAC");
  
  for(;;) { //infinite loop
    if(xSemaphoreTake(dac_sem, ( TickType_t ) 2 ) == pdTRUE) {
      int val = dac_val;
      if (val < 0 || val > DAC_MAX) {
        Serial.print("DACTask: "); ///
        Serial.print(val); //////
        Serial.println(" is out of range (0-4095)"); /////
      }
      else {
        dac.setVoltage(val, false);
        Serial.print("DACTask: DAC set to "); ///
        Serial.print(val); ////
        Serial.print(" (~"); ////
        Serial.print(val * DAC_VREF / DAC_MAX, 3); ///
        Serial.print(" V)"); ////
      }
    }
    else {
      //can't do anything here
    }
  }
}


void ReadCurrentTask(void *parameter) {
  for (;;) { // Infinite loop
    if(xSemaphoreTake(ina_sem, ( TickType_t ) 2 ) == pdTRUE) {
      float curr = (float) readCurrent_uA();
      pSensorCharacteristic->setValue(curr);
      pSensorCharacteristic->notify();
      Serial.print("New value notified: ");
      Serial.println(curr);
    }
    else {
      //can't do anything here
    }
  }
}

// // continuously running task that basically waits for an interrupt to run something useful
void BLETask(void *pvParameter) {
  for (;;) { //infinite loop
    // send current value
    if (deviceConnected) {
      //Serial.print("device connected");
      xSemaphoreGive(ina_sem);
      vTaskDelay(1000 / portTICK_PERIOD_MS); // 1s
    }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      //safety first - anything else we need to do here?
      dac.setVoltage(0,false); //safe to do this while bypassing the semaphore?

      Serial.println("Device disconnected.");
      delay(500);                   // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising();  // restart advertising
      Serial.println("Start advertising");
      oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      //also safety things here?

      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
      Serial.println("Device Connected");
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //pinMode(ledPin, OUTPUT);

  // INA219 Current Sensor Setup
  ina219Init();

  // MCP4725 External DAC Setup
  Wire1.begin(SDA_2, SCL_2);
  dac.begin(0x60, &Wire1);
  dac.setVoltage(0, false);  // Start at 0V
  dac_val = 0;

  //semaphores for ISR handling
  dac_sem = xSemaphoreCreateBinary();
  ina_sem = xSemaphoreCreateBinary();

  // Create the BLE Device
  BLEDevice::init("ESP32");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  // Create the ON button Characteristic
  pDACCharacteristic = pService->createCharacteristic(
    DAC_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE 
  );

  // Register the callback for the DAC set characteristic
  pDACCharacteristic->setCallbacks(new MyCharacteristicCallbacks());

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pSensorCharacteristic->addDescriptor(new BLE2902());
  pDACCharacteristic->addDescriptor(new BLE2902());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  xTaskCreatePinnedToCore( //highest priority
    BLETask,         // Task function
    "BLETask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    3,                 // Priority
    &BLETaskHandle,  // Task handle
    1                  // Core 0 //double check this!!!!!! seems to read current faster when on same core as rest
  );

  xTaskCreatePinnedToCore(
    DACTask,         // Task function
    "DACTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    2,                 // Priority
    &DACTaskHandle,  // Task handle
    1                  // Core 1 //double check this
  );

  
  xTaskCreatePinnedToCore(
    ReadCurrentTask,         // Task function
    "ReadCurrentTask",       // Task name
    10000,             // Stack size (bytes)
    NULL,              // Parameters
    1,                 // Priority
    &ReadCurrentTaskHandle,  // Task handle
    1                  // Core 1 //double check this
  );
}

void loop() {
  //nothing, things run in tasks
}
