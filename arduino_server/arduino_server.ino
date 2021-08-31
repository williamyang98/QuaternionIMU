#include <Wire.h> 

// packet headers
#define RX_MEASUREMENT_START_HEADER 0x01
#define RX_MEASUREMENT_STOP_HEADER  0x02
#define RX_I2C_READ_HEADER          0x03
#define RX_I2C_WRITE_HEADER         0x04

#define TX_ALIVE_HEADER                 0xFF
#define TX_GY271_DATA_HEADER            0x01
#define TX_GY271_TIMEOUT_HEADER         0x02
#define TX_MPU6050_DATA_HEADER          0x03
#define TX_MEASUREMENT_START_ACK_HEADER 0x04
#define TX_MEASUREMENT_STOP_ACK_HEADER  0x05
#define TX_UNKNOWN_RX_PACKET_HEADER     0x06
#define TX_I2C_READ_INFO_HEADER         0x07
#define TX_I2C_WRITE_INFO_HEADER        0x08

#define MAX_I2C_RW 32
#define COBS_BUFFER_LEN 64

// packet buffers
uint8_t alive_packet[]             = {TX_ALIVE_HEADER,0x0A};                       // device id
uint8_t gy271_timeout_packet[]     = {TX_GY271_TIMEOUT_HEADER, 0xFF};              // timeout_reason
uint8_t measurement_start_packet[] = {TX_MEASUREMENT_START_ACK_HEADER, 0x00};      // tag_byte
uint8_t measurement_stop_packet[]  = {TX_MEASUREMENT_STOP_ACK_HEADER, 0x00};       // tag_byte
uint8_t unknown_rx_packet[1+1+COBS_BUFFER_LEN+1];
uint8_t tx_i2c_write_ack[4]        = {TX_I2C_WRITE_INFO_HEADER, 0x00, 0x00, 0x00}; // addr, reg, length

// packet structs
union {
  uint8_t buf[1+4+6];
  struct {
    uint8_t header;
    uint32_t uS;
    uint8_t buf[6];
  } data;
} gy271_packet;

union {
  uint8_t buf[1+4+14];
  struct {
    uint8_t header;
    uint32_t uS;
    uint8_t buf[14];
  } data;
} mpu6050_packet;

// reply to i2c read
union {
  uint8_t buf[1+1+1+MAX_I2C_RW];
  struct {
    uint8_t header;
    uint8_t addr;
    uint8_t reg;
    uint8_t len;
    uint8_t buf[MAX_I2C_RW];
  } data;
} i2c_read_ack_packet;

// encoding and decoding cobs data from serial
uint8_t encode_buf[COBS_BUFFER_LEN];
uint8_t decode_buf[COBS_BUFFER_LEN];
uint8_t rx_buf[COBS_BUFFER_LEN];
int rx_i = 0;

// send packet over serial
void send_packet(const uint8_t *data, int n_data) {
  int n = cobsEncode(data, n_data, (uint8_t*)encode_buf);
  encode_buf[n] = 0x00;
  Serial.write(encode_buf, n+1);
}

// server flags
uint8_t measurements_running = 0;   // send packets if running
int cnt_alive = 0;                  // every second send an alive packet to server

void setup() {
  measurements_running = 0;
  rx_i = 0;
  gy271_packet.data.header = TX_GY271_DATA_HEADER;
  mpu6050_packet.data.header = TX_MPU6050_DATA_HEADER;
  i2c_read_ack_packet.data.header = TX_I2C_READ_INFO_HEADER;
  
  Wire.begin();
  Serial.begin(38400);
  delay(100);
  mpu6050_begin();
}

void continuous_running(void);
void rx_commands(void);
void rx_i2c_commands(uint8_t *buf, int n);

void loop() {
  // send data if running
  rx_commands();

  if (measurements_running) {
    continuous_running();  
  }

  // send alive packet every second
  cnt_alive++;
  if (cnt_alive == 100) {
    alive_packet[1] = (uint8_t)(0xFF & Serial.available());
    send_packet((const uint8_t *)alive_packet, sizeof(alive_packet));  
    cnt_alive = 0;
  }
  
  delay(10);  
}

// recieve i2c commands
void rx_commands(void) {
  uint8_t d;
  
  while (Serial.available() > 0) {
    d = Serial.read();
    rx_buf[rx_i++] = d;
    // prevent overflow in case this happens
    rx_i = (rx_i % COBS_BUFFER_LEN);

    if (d != 0x00) continue;
    
    // if we read end of packet, decode it and reset rx_buf
    int n = cobsDecode(rx_buf, rx_i, decode_buf);
    n--; // ignore null terminator
    uint8_t encoded_size = (uint8_t)(rx_i & 0xFF);
    rx_i = 0;

    if (encoded_size == 0) continue;
    if (n == 0) continue;
    if (encoded_size == 1 && rx_buf[0] == 0x00) continue;

    uint8_t header = decode_buf[0];

    // start gy271 and mpu
    if (header == RX_MEASUREMENT_START_HEADER && n == 2) 
    {
      measurements_running = 1;
      // second byte is the tag of start command
      measurement_start_packet[1] = decode_buf[1];
      send_packet((const uint8_t *)measurement_start_packet, sizeof(measurement_start_packet));
    // stop gy271 and mpu
    } 
    else if (header == RX_MEASUREMENT_STOP_HEADER && n == 2) 
    {
      measurements_running = 0;
      // second byte is the tag of start command
      measurement_stop_packet[1] = decode_buf[1];
      send_packet((const uint8_t *)measurement_stop_packet, sizeof(measurement_stop_packet));
    }
    // i2c read
    else if (header == RX_I2C_READ_HEADER && n == 4)
    {
      uint8_t addr = decode_buf[1];
      uint8_t reg = decode_buf[2];
      uint8_t read_len = decode_buf[3];
      
      i2c_read_ack_packet.data.addr = addr;
      i2c_read_ack_packet.data.reg = reg;
      i2c_read_ack_packet.data.len = 0x00;
      
      // invalid length 
      if (n > MAX_I2C_RW) {
        i2c_read_ack_packet.data.len = 0x00;
        send_packet(i2c_read_ack_packet.buf, 4);
        continue;
      }
      uint8_t rv = i2c_read_reg(addr, reg, i2c_read_ack_packet.data.buf, read_len);
      // error on read
      if (rv != 0) {
        i2c_read_ack_packet.data.len = 0x00;
        send_packet(i2c_read_ack_packet.buf, 4);
        continue;
      }

      i2c_read_ack_packet.data.len = read_len;
      send_packet(i2c_read_ack_packet.buf, 4+read_len);
    } 
    // i2c write
    else if (header == RX_I2C_WRITE_HEADER && n >= 5)
    {
      uint8_t addr = decode_buf[1];
      uint8_t reg = decode_buf[2];
      uint8_t write_len = decode_buf[3];

      tx_i2c_write_ack[1] = addr;
      tx_i2c_write_ack[2] = reg;

      // above max length
      if (write_len > MAX_I2C_RW || write_len != (n-4)) {
        tx_i2c_write_ack[3] = 0x00;  
        send_packet(tx_i2c_write_ack, sizeof(tx_i2c_write_ack));
        continue;
      }

      // write to i2c address
      uint8_t rv = i2c_write_reg(addr, reg, &decode_buf[4], write_len);
      // error on  write
      if (rv != 0) {
        tx_i2c_write_ack[3] = 0x00;  
        send_packet(tx_i2c_write_ack, sizeof(tx_i2c_write_ack));
        continue;
      }

      tx_i2c_write_ack[3] = write_len;  
      send_packet(tx_i2c_write_ack, sizeof(tx_i2c_write_ack));
    }
    // unknown packet
    else 
    {
      unknown_rx_packet[0] = TX_UNKNOWN_RX_PACKET_HEADER;
      unknown_rx_packet[1] = encoded_size;
      for (int i = 0; i < encoded_size; i++) {
        unknown_rx_packet[2+i] = rx_buf[i];
      }
      send_packet((const uint8_t *)unknown_rx_packet, encoded_size+2);
    }
  }
}

void continuous_running(void) {
  int err;
  // compass
  if (gy271_ping()) {
    err = gy271_get_readings((uint8_t *)gy271_packet.data.buf);
    if (err == 0) {
      gy271_packet.data.uS = micros();
      send_packet((const uint8_t *)gy271_packet.buf, sizeof(gy271_packet.buf));
    }
  } else {
    send_packet((const uint8_t *)gy271_timeout_packet, sizeof(gy271_timeout_packet));
  }

  if (mpu6050_is_ready()) {
    // send gyro data
    err = mpu6050_get_readings((uint8_t *)mpu6050_packet.data.buf);
    if (err == 0) {
      mpu6050_packet.data.uS = micros();
      send_packet((const uint8_t *)mpu6050_packet.buf, sizeof(mpu6050_packet.buf));
    }
  }
}
