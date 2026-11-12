#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

// We are the peripheral (server); the PC running Unity is the central (client).
//
// Work split:
//   BLE stack task (onWrite / onConnect / onDisconnect callbacks)
//       -> validate the 10-byte packet, drop it in a 1-slot mailbox, return immediately.
//          No Serial, no I2C, no heap allocation, no blocking.
//   DACTask (highest app priority, ONLY task that touches the DAC)
//       -> waits on the mailbox with a timeout. Packet: apply it. Timeout: zero the output
//          (watchdog). Applies the hard cap, ramp-up limit and polarity-through-zero.
//   StatusTask (1 Hz, lowest priority)
//       -> reads the INA219, sends the notify, prints stats. Never prints per packet.

#define DEVICE_NAME "ESP32-Brocoli"   // matches GVSController.targetDeviceName
#define SERVICE_UUID "19b10000-e8f2-537e-4f6c-d104768a1214"
#define SENSOR_CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#define DAC_CHARACTERISTIC_UUID "19b10002-e8f2-537e-4f6c-d104768a1214"

// ---------------- Protocol: must match GVSLink.GVSPacket in Unity ----------------
enum : uint8_t { CMD_STOP = 0, CMD_START = 1, CMD_DATA = 2 };

struct __attribute__((packed)) GVSPacket {
  uint8_t command;     // 0 = STOP, 1 = START, 2 = DATA
  uint8_t direction;   // 0 / 1 polarity
  int32_t frequency;   // currently unused (DC)
  int32_t amplitude;   // microamps
};
static_assert(sizeof(GVSPacket) == 10, "GVSPacket must be exactly 10 bytes");

// ---------------- Safety limits ----------------
const int32_t HARD_MAX_UA = 3000;        // independent of whatever Unity sends
const uint32_t WATCHDOG_MS = 200;        // no packet for this long -> output zero (Unity sends at 30-100 Hz)
const int32_t RAMP_UP_UA_PER_MS = 30;    // increases limited to 3 mA per 100 ms; decreases are instant

// ---------------- Hardware mapping: FILL THESE IN FOR YOUR CIRCUIT ----------------
// Microamps delivered per DAC count through the Howland pump. Measure it (e.g. with the INA219)
// before enabling. Compilation is blocked until you set it, so a guess can't reach a person.
#define UA_PER_DAC_COUNT_CALIBRATED 0
const float UA_PER_DAC_COUNT = 1.0f;     // <-- your measured value
#if !UA_PER_DAC_COUNT_CALIBRATED
#error "Set UA_PER_DAC_COUNT to your measured value, then set UA_PER_DAC_COUNT_CALIBRATED to 1."
#endif

// GPIO that selects polarity (relay / H-bridge / analog switch). -1 = not wired yet: direction ignored.
const int DIR_PIN = -1;

// ---------------- Pins / devices ----------------
const int SDA_2 = 18;
const int SCL_2 = 19;
const long DAC_MAX = 4095;               // MCP4725 is 12-bit

Adafruit_MCP4725 dac;                    // on Wire1, owned exclusively by DACTask
Adafruit_INA219 ina219;                  // on Wire (bus 0). If it is ever moved to Wire1, add a mutex.

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
BLECharacteristic* pDACCharacteristic = NULL;

QueueHandle_t cmdQueue = NULL;           // 1-slot mailbox: newest packet wins

// Shared state (written by one task, read by StatusTask for printing only)
volatile bool deviceConnected = false;
volatile bool armed = false;
volatile int32_t outputSignedUA = 0;
volatile uint32_t rxCount = 0, rejectCount = 0, watchdogTrips = 0;

// ============================================================================
// BLE callbacks: run in the BLE stack's task. Keep them tiny.
// ============================================================================

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

// ============================================================================
// DAC task: the only code that writes the DAC / polarity pin.
// ============================================================================

static int lastDacCode = -1;
static int lastDir = -1;
static uint32_t lastApplyMs = 0;

static void writeHardware(int32_t signedUA) {
  int32_t mag = abs(signedUA);
  int code = constrain((int)lroundf(mag / UA_PER_DAC_COUNT), 0, (int)DAC_MAX);
  int dir = signedUA > 0 ? 1 : 0;

  // Never switch polarity while current is flowing: DAC to zero first, then the pin.
  if (DIR_PIN >= 0 && code > 0 && dir != lastDir) {
    if (lastDacCode != 0) { dac.setVoltage(0, false); lastDacCode = 0; }
    digitalWrite(DIR_PIN, dir);
    lastDir = dir;
  }

  if (code != lastDacCode) {                // skip redundant I2C writes
    dac.setVoltage(code, false);
    lastDacCode = code;
  }
}

static void applyOutput(int32_t target) {
  uint32_t now = millis();
  int32_t elapsed = constrain((int32_t)(now - lastApplyMs), 1, 50);
  lastApplyMs = now;

  int32_t cur = outputSignedUA;
  int32_t next;
  if ((cur > 0 && target < 0) || (cur < 0 && target > 0)) {
    next = 0;                                // polarity change: pass through zero first
  } else if (abs(target) <= abs(cur)) {
    next = target;                           // decreasing: immediately (the safe direction)
  } else {
    int32_t step = RAMP_UP_UA_PER_MS * elapsed;
    next = cur + constrain(target - cur, -step, step);
  }

  writeHardware(next);
  outputSignedUA = next;
}

void DACTask(void* pvParameters) {
  GVSPacket p;
  int32_t target = 0;

  for (;;) {
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
        case CMD_DATA: {
          int32_t amp = p.amplitude;                       // copy out of the packed struct
          int32_t a = amp < HARD_MAX_UA ? amp : HARD_MAX_UA;
          target = armed ? (p.direction ? a : -a) : 0;   // DATA before START is ignored
          break;
        }
      }
    } else {
      // Watchdog: heartbeat lost. Zero now; stay armed so output resumes (ramped) when packets return.
      if (outputSignedUA != 0) watchdogTrips++;
      target = 0;
    }
    applyOutput(target);
  }
}

// ============================================================================
// Status task: current sensing + notify + stats, once a second.
// ============================================================================

void StatusTask(void* pvParameters) {
  bool wasConnected = false;
  uint32_t lastRx = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(1000));

    bool connected = deviceConnected;
    if (connected != wasConnected) {
      Serial.println(connected ? "Device connected" : "Device disconnected, advertising");
      wasConnected = connected;
    }
    if (!connected) continue;

    float curr = (float)readCurrent_uA();
    pSensorCharacteristic->setValue(curr);
    pSensorCharacteristic->notify();

    uint32_t rx = rxCount;
    Serial.printf("rx %lu pkt/s | rejected %lu | wd trips %lu | armed %d | out %ld uA | measured %.0f uA\n",
                  (unsigned long)(rx - lastRx), (unsigned long)rejectCount, (unsigned long)watchdogTrips,
                  (int)armed, (long)outputSignedUA, curr);
    lastRx = rx;
  }
}

// ============================================================================

void setup() {
  Serial.begin(115200);

  ina219Init();                              // your existing INA219 setup (Wire)

  Wire1.begin(SDA_2, SCL_2);
  Wire1.setClock(400000);                    // ~0.1 ms per DAC write
  dac.begin(0x60, &Wire1);
  dac.setVoltage(0, false);
  lastDacCode = 0;

  if (DIR_PIN >= 0) {
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(DIR_PIN, 0);
    lastDir = 0;
  }

  cmdQueue = xQueueCreate(1, sizeof(GVSPacket));

  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);
  pSensorCharacteristic->addDescriptor(new BLE2902());

  // WRITE_NR is what Unity uses (write-without-response). WRITE kept for other tools.
  pDACCharacteristic = pService->createCharacteristic(
    DAC_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pDACCharacteristic->setCallbacks(new DacCharCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as " DEVICE_NAME);

  // The BLE stack (Bluedroid) runs on core 0; keep the output path on core 1.
  xTaskCreatePinnedToCore(DACTask, "DACTask", 4096, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(StatusTask, "StatusTask", 4096, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);                         // everything runs in tasks
}
