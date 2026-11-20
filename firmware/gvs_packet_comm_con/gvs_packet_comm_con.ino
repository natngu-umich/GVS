#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <esp_sleep.h>
#include "esp_system.h"
#include "driver/rtc_io.h"
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA219.h>

//we are a server / peripheral (slave), computer is client / central (master)

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
// NOTE: 0.000805 is 3.3/4096 = VOLTS per DAC count, not microamps, and this constant is not used.
// The real scaling is map(-3000..3000 -> 0..4095) in writeHardwareTask (~1.465 uA/count,
// assuming full-scale DAC = exactly +/-3000 uA). Measure that before a person is connected.
#define UA_PER_DAC_COUNT_CALIBRATED 1
const float UA_PER_DAC_COUNT =  0.000805;
const int32_t DAC_ZERO = 2047;                      // DAC code that gives 0 uA (mid-scale). Trim after measuring.
#if !UA_PER_DAC_COUNT_CALIBRATED
#error "Set UA_PER_DAC_COUNT to your measured value, then set UA_PER_DAC_COUNT_CALIBRATED to 1."
#endif

// One-time: store DAC_ZERO in the MCP4725's EEPROM so the DAC *powers up* at 0 uA.
// (With the bipolar mapping, DAC code 0 = full-scale NEGATIVE current.)
// Set to 1, upload, wait for "EEPROM written", set back to 0 and upload again.
#define PROGRAM_DAC_EEPROM_ZERO 0

const int32_t HARD_MAX_UA = 3000;        // independent of whatever Unity sends
const uint32_t WATCHDOG_MS = 200;        // no packet for this long -> output zero (Unity sends at 30-100 Hz)

const int32_t RAMP_VAL = 25; //max change of 25uA in 2.5 ms (aka 100uA in 10ms, matching Unity limits)
const float POT_MIN = 0.1f;              // pot scale floor (fraction of full strength)

// Define pins for I2C Bus 2 (Wire1)
const int SDA_2 = 18;
const int SCL_2 = 19;

//ADC/Potentiometer Pin
const int POT_PIN = 33;

// ---------------- Buttons ----------------
// E-stop: any GPIO. Sleep: must be an RTC GPIO (0, 2, 4, 12-15, 25-27, 32-39) to wake from deep sleep.
// DO NOT move the sleep button to GPIO 0 while it is active-HIGH with a pull-down: GPIO 0 is a boot
// strapping pin, and if it is LOW at reset the ESP32 boots into download mode instead of your sketch.
// (GPIO 0 only works as an active-LOW button with a pull-up, like the board's BOOT button.)
const int ESTOP_PIN = 23;
const int ESTOP_ACTIVE_LEVEL = HIGH;   // HIGH when pressed, as you have it wired now
#define SLEEP_PIN_NUM 32
const int SLEEP_PIN = SLEEP_PIN_NUM;
const gpio_num_t SLEEP_WAKE_GPIO = (gpio_num_t)SLEEP_PIN_NUM;   // the cast fixes "doesn't like gpio_num"
const int SLEEP_ACTIVE_LEVEL = HIGH;

const uint32_t BUTTON_POLL_MS = 2;
const uint8_t  PRESS_SAMPLES = 5;      // 5 x 2 ms = must read pressed for 10 ms (rejects noise/bounce)
const uint8_t  RELEASE_SAMPLES = 25;   // 50 ms released before the sleep button counts again

// MCP4725 is 12-bit
const long DAC_MAX = 4095;
const float DAC_VREF = 3.3;

const int   ADC_MAX     = 4095;
const float ADC_REF_VOLT = 3.3;

#define STATUS_TASK_PERIOD_MS 12   // ms

Adafruit_MCP4725 dac;
Adafruit_INA219 ina219;

// Shared state
volatile bool deviceConnected = false;
volatile bool armed = false;
volatile int32_t output_uA = 0;
volatile uint32_t rxCount = 0, rejectCount = 0, watchdogTrips = 0;
volatile uint32_t dacWriteFails = 0;

volatile bool outputInhibit = false;   // set by e-stop/sleep: writeHardwareTask forces DAC_ZERO
volatile bool dacAtZero = false;       // set by writeHardwareTask once the zero write SUCCEEDED
volatile bool shuttingDown = false;    // stops BLE activity / advertising
volatile bool statusParked = false;    // StatusTask has stopped touching BLE/Serial
bool bleStarted = false;

// E-stop latch. Survives a crash/software reset (so a crash during shutdown can't "undo" the e-stop),
// cleared only by a power-on reset (power cycle or the EN button).
#define ESTOP_MAGIC 0xE5709E57u
RTC_NOINIT_ATTR uint32_t estopLatch;


class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) {
    deviceConnected = true;
    s->updateConnParams(param->connect.remote_bda, 6, 12, 0, 400);
  }

  void onDisconnect(BLEServer* s) {
    deviceConnected = false;
    armed = false;   // a new connection must send START again
    GVSPacket stop = {};
    stop.command = CMD_STOP;
    xQueueOverwrite(cmdQueue, &stop);
    if (!shuttingDown) {
      BLEDevice::startAdvertising();
    }
  }
};

class DacCharCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    if (shuttingDown) return;
    if (c->getLength() != sizeof(GVSPacket)) { rejectCount++; return; }

    GVSPacket p;
    memcpy(&p, c->getData(), sizeof(p));   // ESP32 and Unity are both little-endian

    if (p.command > CMD_DATA || p.direction > 1 || p.amplitude < 0) { rejectCount++; return; }

    rxCount++;
    if (p.command == CMD_START) armed = true;
    else if (p.command == CMD_STOP) armed = false;
    xQueueOverwrite(cmdQueue, &p);
  }
};

static volatile int last_dac_out = -1; // 0 - 4095
static volatile int lastDir = 0;
static volatile float last_pot = 1;

static volatile int32_t prev_unity_amp = 0;
static volatile int32_t prev_actual_amp = 0;
static volatile int32_t unity_amp = 0;
static volatile uint8_t unity_dir = 0;


// ---------------- Boot helpers ----------------

// If the ESP32 resets mid-I2C-transfer, the MCP4725 can be left holding SDA low and every later
// write fails silently. Clock SCL up to 9 times and send a STOP to free the bus.
static void i2cBusRecover(int sda, int scl) {
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
  delayMicroseconds(10);
  if (digitalRead(sda) == HIGH && digitalRead(scl) == HIGH) return;

  pinMode(scl, OUTPUT_OPEN_DRAIN);
  digitalWrite(scl, HIGH);
  for (int i = 0; i < 9 && digitalRead(sda) == LOW; i++) {
    digitalWrite(scl, LOW);  delayMicroseconds(5);
    digitalWrite(scl, HIGH); delayMicroseconds(5);
  }
  digitalWrite(scl, LOW);  delayMicroseconds(5);
  pinMode(sda, OUTPUT_OPEN_DRAIN);
  digitalWrite(sda, LOW);  delayMicroseconds(5);
  digitalWrite(scl, HIGH); delayMicroseconds(5);
  digitalWrite(sda, HIGH); delayMicroseconds(5);
  pinMode(sda, INPUT_PULLUP);
  pinMode(scl, INPUT_PULLUP);
}

static bool dacZeroAtBoot() {
  for (int attempt = 0; attempt < 20; attempt++) {
    i2cBusRecover(SDA_2, SCL_2);
    Wire1.begin(SDA_2, SCL_2);
    if (dac.begin(0x60, &Wire1) && dac.setVoltage(DAC_ZERO, false)) return true;
    Wire1.end();
    delay(10);
  }
  return false;
}

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "power-on / EN button";
    case ESP_RST_SW:        return "software restart";
    case ESP_RST_PANIC:     return "PANIC (crash) - scroll up for Guru Meditation / Backtrace";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:       return "OTHER WATCHDOG (RTC/MWDT)";
    case ESP_RST_DEEPSLEEP: return "woke from deep sleep";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (supply dipped)";
    default:                return "other";
  }
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0:  Serial.println("Wakeup caused by sleep button (EXT0)"); break;
    case ESP_SLEEP_WAKEUP_EXT1:  Serial.println("Wakeup caused by EXT1"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    default:                     Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}


// ---------------- E-stop / sleep ----------------
// One routine for both. Never returns.
//   isSleepMode = false -> e-stop: latch, no wake source, needs a power cycle (or EN button)
//   isSleepMode = true  -> sleep:  wake on the sleep button; waking reboots through setup()
void shutdownAndSleep(bool isSleepMode) {
  if (!isSleepMode) estopLatch = ESTOP_MAGIC;

  // 1. Zero the output. Only writeHardwareTask touches the DAC, so it can't overwrite our zero
  //    with a stale ramp value afterwards. Wait until it reports a SUCCESSFUL zero write.
  armed = false;
  dacAtZero = false;
  outputInhibit = true;
  shuttingDown = true;

  uint32_t waited = 0;
  while (!dacAtZero) {
    vTaskDelay(pdMS_TO_TICKS(2));
    waited += 2;
    if (waited % 500 == 0) {
      Serial.println("WARNING: DAC zero write not confirmed - still retrying. Use the hardware cutoff.");
    }
  }
  Serial.println(isSleepMode ? "SLEEP: output at zero" : "E-STOP: output at zero");

  // 2. Let StatusTask park itself so it isn't mid-notify when BLE is shut down.
  for (int i = 0; i < 50 && !statusParked; i++) vTaskDelay(pdMS_TO_TICKS(2));

  // 3. Tell Unity, then shut BLE down cleanly (scheduler stays RUNNING - BLE needs it).
  if (bleStarted) {
    if (deviceConnected && pSensorCharacteristic != nullptr) {
      GVSPacket p = { CMD_STOP, 0, 0, 0 };
      pSensorCharacteristic->setValue((uint8_t *)&p, sizeof(GVSPacket));
      pSensorCharacteristic->notify();
      vTaskDelay(pdMS_TO_TICKS(30));   // give the notification time to go out
    }
    BLEDevice::stopAdvertising();
    if (deviceConnected && pServer != nullptr) {
      pServer->disconnect(pServer->getConnId());
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    BLEDevice::deinit(false);
  }

  // 4. Wake source (sleep mode only).
  if (isSleepMode) {
    // Wait until the button has been released for 50 ms, or we'd wake straight back up.
    uint8_t released = 0;
    while (released < RELEASE_SAMPLES) {
      released = (digitalRead(SLEEP_PIN) != SLEEP_ACTIVE_LEVEL) ? released + 1 : 0;
      vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
    }
    rtc_gpio_pullup_dis(SLEEP_WAKE_GPIO);
    rtc_gpio_pulldown_en(SLEEP_WAKE_GPIO);
    esp_sleep_enable_ext0_wakeup(SLEEP_WAKE_GPIO, SLEEP_ACTIVE_LEVEL);
    Serial.println("Entering sleep - press the sleep button to wake (device restarts, Unity must reconnect).");
  } else {
    Serial.println("E-STOP latched - power cycle (or press EN) to restart.");
  }

  Serial.flush();
  esp_deep_sleep_start();
  for (;;) {}   // not reached
}

// Polls both buttons. No ISRs: nothing here can fire before setup is done, it's debounced,
// and all the heavy work happens in a normal task where I2C and BLE calls are allowed.
void buttonTask(void *parameter) {
  uint8_t estopCount = 0, sleepCount = 0, sleepReleasedCount = 0;
  bool sleepArmed = false;   // sleep button must be seen released first (it's still held right after waking)

  for (;;) {
    // E-stop is level-triggered: if it's held, we stop, every time.
    if (digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_LEVEL) {
      if (++estopCount >= PRESS_SAMPLES) shutdownAndSleep(false);
    } else {
      estopCount = 0;
    }

    // Sleep is edge-triggered: released for 50 ms, then pressed for 10 ms.
    if (digitalRead(SLEEP_PIN) == SLEEP_ACTIVE_LEVEL) {
      sleepReleasedCount = 0;
      if (sleepArmed && ++sleepCount >= PRESS_SAMPLES) shutdownAndSleep(true);
    } else {
      sleepCount = 0;
      if (sleepReleasedCount < RELEASE_SAMPLES) sleepReleasedCount++;
      else sleepArmed = true;
    }

    vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
  }
}


// ---------------- Output ----------------

void writeHardwareTask(void *parameter) {
  for (;;) {
    if (outputInhibit) {
      // E-stop / sleep: straight to zero (no ramp) and stay there.
      prev_actual_amp = 0;
      if (!dacAtZero) {
        if (dac.setVoltage(DAC_ZERO, false)) { last_dac_out = DAC_ZERO; dacAtZero = true; }
        else dacWriteFails++;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
      continue;
    }

    int dir = unity_dir > 0 ? 1 : -1;
    int32_t scaled_sign_val = (unity_amp + 0.5f) * dir; // -3000 to 3000

    if (abs(scaled_sign_val - prev_actual_amp) > RAMP_VAL) {
      if (scaled_sign_val < prev_actual_amp) {
        scaled_sign_val = prev_actual_amp - RAMP_VAL;
      } else {
        scaled_sign_val = prev_actual_amp + RAMP_VAL;
      }
    }

    uint16_t dac_out = map(scaled_sign_val, -3000, 3000, 0, DAC_MAX);

    // Only record the new value / advance the ramp if the I2C write actually succeeded.
    if (dac_out == last_dac_out) {
      prev_actual_amp = scaled_sign_val;
    } else if (dac.setVoltage(dac_out, false)) {
      prev_actual_amp = scaled_sign_val;
      last_dac_out = dac_out;
      lastDir = dir;
    } else {
      dacWriteFails++;   // retry next pass
    }

    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Takes packets from the BLE callback and turns them into a target current
void UnityReadTask (void *parameter) {
  GVSPacket p = {};
  int32_t target = 0;

  for(;;) {
    if (xQueueReceive(cmdQueue, &p, pdMS_TO_TICKS(WATCHDOG_MS)) == pdTRUE) {
      switch (p.command) {
        case CMD_STOP:  target = 0; break;
        case CMD_START: target = 0; break;
        case CMD_DATA: {
          int32_t amp = armed ? p.amplitude : 0;
          target = amp < HARD_MAX_UA ? amp : HARD_MAX_UA;
          break;
        }
      }
    }
    else {
      // Watchdog: heartbeat lost. Zero now; stay armed so output resumes (ramped) when packets return.
      if (output_uA != 0 || target != 0) {
        watchdogTrips++;
        target = 0;
      }
    }
    prev_unity_amp = unity_amp;
    unity_amp = target * last_pot; //scaled!
    unity_dir = p.direction;

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void PotTask (void *parameter) {
  for (;;) {
    int pot_raw = analogRead(POT_PIN);
    float pot_percent = pot_raw / (float)ADC_MAX;
    if (pot_percent < POT_MIN) pot_percent = POT_MIN;
    last_pot = pot_percent;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}


void StatusTask(void *parameter) {
  bool wasConnected = false;
  TickType_t lastWake = xTaskGetTickCount();

  for(;;){
    if (shuttingDown) {          // stop touching BLE/Serial so shutdown can take them over
      statusParked = true;
      vTaskSuspend(NULL);
    }

    bool connected = deviceConnected;
    if (connected != wasConnected) {
      Serial.println(connected ? "Device connected" : "Device disconnected, advertising");
      wasConnected = connected;
    }

    float curr = (float)readCurrent_uA();
    output_uA = curr;

    if (connected){
      float scaled_pot = last_pot * 100;
      GVSPacket p = {
        CMD_DATA,
        (uint8_t)(lastDir > 0 ? 1 : 0),
        (int32_t) scaled_pot,     // frequency field reused for pot %
        (int32_t) curr            // measured microamps
      };
      pSensorCharacteristic->setValue((uint8_t *)&p, sizeof(GVSPacket));
      pSensorCharacteristic->notify();

      Serial.print("unity val: ");     Serial.print(unity_amp);
      Serial.print(" signed val: ");   Serial.print(prev_actual_amp);
      Serial.print(" dac_out: ");      Serial.print(last_dac_out);
      Serial.print(" scaled pot: ");   Serial.print(scaled_pot);
      Serial.print(" dir: ");          Serial.print(unity_dir);
      Serial.print(" dacFails: ");     Serial.println(dacWriteFails);
    }

    vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(STATUS_TASK_PERIOD_MS));
  }
}


void setup() {
  // 1. Zero the DAC before ANYTHING else. The MCP4725 doesn't reset with the ESP32.
  bool dacOk = dacZeroAtBoot();
  last_dac_out = dacOk ? DAC_ZERO : -1;

  Serial.begin(115200);
  esp_reset_reason_t rr = esp_reset_reason();
  Serial.printf("\nReset reason: %s\n", resetReasonStr(rr));
  print_wakeup_reason();   // must come after Serial.begin, or it prints nothing

  while (!dacOk) {
    Serial.println("FATAL: MCP4725 not responding on Wire1 - output NOT confirmed at zero. Retrying...");
    delay(500);
    dacOk = dacZeroAtBoot();
    if (dacOk) last_dac_out = DAC_ZERO;
  }
  Serial.println("DAC confirmed at zero.");

#if PROGRAM_DAC_EEPROM_ZERO
  if (dac.setVoltage(DAC_ZERO, true)) {
    delay(50);
    Serial.println("EEPROM written: MCP4725 will power up at DAC_ZERO. Set PROGRAM_DAC_EEPROM_ZERO back to 0.");
  } else {
    Serial.println("EEPROM write FAILED");
  }
#endif

  // 2. Buttons. Pull-downs so a floating pin can't read as "pressed" (add external 10k pull-downs too).
  rtc_gpio_deinit(SLEEP_WAKE_GPIO);    // hand the pin back from the RTC domain after a deep-sleep wake
  pinMode(ESTOP_PIN, INPUT_PULLDOWN);
  pinMode(SLEEP_PIN, INPUT_PULLDOWN);

  // 3. E-stop latch: if we e-stopped and then crashed/reset without a power cycle, stay stopped.
  if (rr == ESP_RST_POWERON) estopLatch = 0;
  delay(5);
  if (estopLatch == ESTOP_MAGIC || digitalRead(ESTOP_PIN) == ESTOP_ACTIVE_LEVEL) {
    estopLatch = ESTOP_MAGIC;
    Serial.println("E-STOP active/latched at boot - output held at zero. Release e-stop and power cycle.");
    Serial.flush();
    esp_deep_sleep_start();   // no wake source; BLE never started
  }

  //INA219 Current Sensor Setup
  ina219Init();

  //ADC and Potentiometer Setup
  analogReadResolution(12);
  pinMode(POT_PIN, INPUT);
  last_pot = 1;

  cmdQueue = xQueueCreate(1, sizeof(GVSPacket));

  // BLE
  BLEDevice::init(DEVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  BLEService* pService = pServer->createService(SERVICE_UUID);

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_INDICATE);

  pDACCharacteristic = pService->createCharacteristic(
    DAC_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pDACCharacteristic->setCallbacks(new DacCharCallbacks());

  pSensorCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  bleStarted = true;
  Serial.println("Waiting a client connection to notify...");

  // Tasks (all core 1). buttonTask is highest so a press is seen within ~10 ms.
  xTaskCreatePinnedToCore(buttonTask,        "buttonTask",        8192,  NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(writeHardwareTask, "writeHardwareTask", 10000, NULL, 4, NULL, 1);
  xTaskCreatePinnedToCore(UnityReadTask,     "UnityReadTask",     10000, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(PotTask,           "PotTask",           10000, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(StatusTask,        "StatusTask",        10000, NULL, 1, NULL, 1);
}

void loop() {
  vTaskDelete(NULL);   // everything runs in tasks; don't leave an empty loop spinning
}