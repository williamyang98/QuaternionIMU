uint8_t i2c_write_reg(const uint8_t addr, uint8_t r, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(r);
  Wire.write(buf, n);
  return Wire.endTransmission();
}

uint8_t i2c_read_reg(const uint8_t addr, uint8_t r, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(r);
  int err = Wire.endTransmission();
  if (!err) {
    uint8_t n_actual = Wire.requestFrom(addr, n);
    if (n_actual != n) {
      return 0x05;
    }
    
    for (int i = 0; i < n; i++) {
      buf[i] = Wire.read();  
    }
    return 0x00;
  }
  return 0x06;
}
