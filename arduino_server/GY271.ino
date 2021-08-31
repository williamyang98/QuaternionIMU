#define GY271_ADDR 0x1E

// check to see that GY271 compass is still on the i2c bus
uint8_t gy271_ping() {
  Wire.beginTransmission(GY271_ADDR);
  byte err = Wire.endTransmission();
  return (err == 0);
}

// readings are located in address 0x03
// structured as 3 int16_t representing Mx,My,Mz
uint8_t gy271_get_readings(uint8_t *buf) {
  return i2c_read_reg(GY271_ADDR, 0x03, buf, 6);
}
