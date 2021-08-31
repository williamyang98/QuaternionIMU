Uses an AtMega328p to manage UART and I2C communications between the python-client and sensors.
Sensors used:
- MPU6050: Accelerometer and gyroscope
- GY271: Magnetometer (compass)

Communicates via bluetooth using the HC06 uart module to a python client.