#define MPU6050_ADDR 0x68

// make it run continuously or not
uint8_t mpu6050_set_clock_source(uint8_t v) {
  uint8_t d;
  uint8_t rv = i2c_read_reg(MPU6050_ADDR, 0x6B, &d, 1);
  if (rv != 0) return rv;
  
  d = (d & 0xf8) | (v & 0x07);
  return i2c_write_reg(MPU6050_ADDR, 0x6B, &d, 1);
}

// sensitivity of the gyroscope
uint8_t mpu6050_set_gyro_fullscale_range(uint8_t v) {
  v = v & 0x03;
  uint8_t d;
  uint8_t rv = i2c_read_reg(MPU6050_ADDR, 0x1B, &d, 1);
  if (rv != 0) return rv;

  d = (d & 0b11100111) | (v << 3);
  return i2c_write_reg(MPU6050_ADDR, 0x1B, &d, 1);
}

// sensitivity of the accelerometer
uint8_t mpu6050_set_accel_fullscale_range(uint8_t v) {
  v = v & 0x03;
  uint8_t d;
  uint8_t rv = i2c_read_reg(MPU6050_ADDR, 0x1C, &d, 1);
  if (rv != 0) return rv;
  
  d = (d & 0b11100111) | (v << 3);
  return i2c_write_reg(MPU6050_ADDR, 0x1C, &d, 1);
}

// make the sensor chip idle or active
uint8_t mpu6050_set_sleep(uint8_t is_sleep) {
  uint8_t d;
  uint8_t rv = i2c_read_reg(MPU6050_ADDR, 0x6B, &d, 1);
  if (rv != 0) return rv;

  if (!is_sleep) {
    d = d & ~(1u << 6);    
  } else {
    d = d |  (1u << 6);
  }
  return i2c_write_reg(MPU6050_ADDR, 0x6B, &d, 1);
}

uint8_t mpu6050_is_ready(void) {
  uint8_t d;
  uint8_t rv = i2c_read_reg(MPU6050_ADDR, 0x3A, &d, 1);
  if (rv != 0) return 0;

  return (d & (1u << 0)) != 0;
}

// readings are located at 0x3b 
// structured as Ax,Ay,Az,temperature,p,q,r
// Axyz = acceleration
// pqr = body angular velocities (degrees/second)
uint8_t mpu6050_get_readings(uint8_t *buf) {
  return i2c_read_reg(MPU6050_ADDR, 0x3B, buf, 14);
}

uint8_t mpu6050_begin(void) {
  mpu6050_set_clock_source(0x00);
  mpu6050_set_gyro_fullscale_range(0x00);
  mpu6050_set_accel_fullscale_range(0x00);
  mpu6050_set_sleep(0);
}
