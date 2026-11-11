/*
  ESP32 I/O Demo
  - Cycles through 4 LEDs, one on at a time
  - Reads 1 button, 2 switches, and 1 potentiometer (ADC)
  - Prints all statuses to the Serial Monitor once per second

  Serial Monitor baud rate: 115200
*/

// ---------- Pin definitions ----------
const int LED_PINS[4]    = {13, 16, 14, 15};
const int BUTTON_PINS[1] = {23};
const int SWITCH_PINS[2] = {26, 27};
const int POT_PIN        = 33;   // ADC1 pin, input-only (safe to use with WiFi)

// ---------- Timing (milliseconds) ----------
const unsigned long LED_INTERVAL_MS   = 500;   // how often the active LED advances
const unsigned long PRINT_INTERVAL_MS = 1000;  // how often status is printed

// ---------- ADC settings ----------
const int   ADC_MAX     = 4095;  // 12-bit resolution
const float ADC_REF_VOLT = 3.3;  // approximate; ESP32 ADC is not perfectly linear

// ---------- State ----------
int currentLed = 0;
unsigned long lastLedChange = 0;
unsigned long lastPrint = 0;

// Set only one LED high, all others low
void setActiveLed(int index) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_PINS[i], (i == index) ? HIGH : LOW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  for (int i = 0; i < 4; i++) {
    pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], LOW);
  }

  for (int i = 0; i < 2; i++) {
    pinMode(BUTTON_PINS[i], INPUT_PULLUP);
    pinMode(SWITCH_PINS[i], INPUT_PULLUP);
  }

  analogReadResolution(12);  // 0-4095
  pinMode(POT_PIN, INPUT);

  setActiveLed(currentLed);

  Serial.println("ESP32 I/O demo started");
}

void loop() {
  unsigned long now = millis();

  // Advance to the next LED (non-blocking)
  if (now - lastLedChange >= LED_INTERVAL_MS) {
    lastLedChange = now;
    currentLed = (currentLed + 1) % 4;
    setActiveLed(currentLed);
  }

  // Print status once per second
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    // Inputs are active-low because of the internal pull-ups
    bool btn1 = (digitalRead(BUTTON_PINS[0]) == LOW);
    bool sw1  = (digitalRead(SWITCH_PINS[0]) == LOW);
    bool sw2  = (digitalRead(SWITCH_PINS[1]) == LOW);

    int   potRaw     = analogRead(POT_PIN);
    float potPercent = (potRaw * 100.0) / ADC_MAX;
    float potVolts   = (potRaw * ADC_REF_VOLT) / ADC_MAX;

    Serial.println("------------------------------");
    Serial.printf("Active LED : %d (GPIO %d)\n", currentLed + 1, LED_PINS[currentLed]);
    Serial.printf("Button 1   : %s\n", btn1 ? "PRESSED" : "released");
    Serial.printf("Switch 1   : %s\n", sw1 ? "ON" : "OFF");
    Serial.printf("Switch 2   : %s\n", sw2 ? "ON" : "OFF");
    Serial.printf("Pot        : %d / %d  (%.1f%%, ~%.2f V)\n",
                  potRaw, ADC_MAX, potPercent, potVolts);
  }
}
