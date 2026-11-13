// ---- INA219 minimal driver (Wire, address 0x40) ----
const uint8_t  INA219_ADDR        = 0x40;
const float    SHUNT_OHMS         = 33.00f;     // use your measured resistor value
const uint16_t INA219_CONFIG_REG  = 0x00; 
const uint16_t INA219_CONFIG      = 0x19F5;    // PGA /8 (±320 mV), shunt 12-bit x64 avg, shunt-only continuous
const uint16_t INA219_RESET       = 0x8000;    // System reset

void ina219Write(uint8_t reg, uint16_t val) {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write(reg);
  Wire.write((uint8_t)(val >> 8));
  Wire.write((uint8_t)(val & 0xFF));
  Wire.endTransmission();
}

void ina219Init() {
  Wire.begin();
  Wire.setClock(400000);
  ina219Write(INA219_CONFIG_REG, INA219_RESET);    // reset
  delay(1);
  ina219Write(INA219_CONFIG_REG, INA219_CONFIG);

  Wire.beginTransmission(INA219_ADDR);
  Wire.write((uint8_t)0x01);        // point at shunt voltage register once
  Wire.endTransmission();
}

int16_t ina219ReadShuntRaw() {
  Wire.beginTransmission(INA219_ADDR);
  Wire.write((uint8_t)0x01);              // shunt voltage register
  Wire.endTransmission();
  Wire.requestFrom(INA219_ADDR, (uint8_t)2);
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

float readCurrent_uA() {
  return ina219ReadShuntRaw() * 10.0f / SHUNT_OHMS;   // 10 µV/LSB ÷ ohms = µA
}