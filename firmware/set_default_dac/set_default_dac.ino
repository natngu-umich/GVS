/*
  MCP4725 - Program power-on default to mid-scale (2048)
  Run this ONCE, then re-flash your main sketch.
  Serial Monitor baud rate: 115200
*/

#include <Adafruit_MCP4725.h>

const int SDA_2 = 18;
const int SCL_2 = 19;
const uint16_t DEFAULT_VALUE = 2048;  // halfway on a 12-bit DAC (0-4095)

Adafruit_MCP4725 dac;

void setup() {
  Serial.begin(115200);
  delay(500);

  Wire1.begin(SDA_2, SCL_2);

  if (!dac.begin(0x60, &Wire1)) {
    Serial.println("MCP4725 not found. Check wiring/address (try 0x61).");
    return;
  }

  // 'true' writes the value to the DAC register AND the EEPROM
  dac.setVoltage(DEFAULT_VALUE, true);

  Serial.printf("Done. Power-on default set to %u (about half of VCC).\n", DEFAULT_VALUE);
}

void loop() {
  // nothing to do
}