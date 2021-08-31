import asyncio

# asynchronously control the mpu6050 combined accelerometer and gyroscope
# over the arduino's i2c bus
class MPU6050:
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = 0x68
    
        self.gyro_sensitivity = [131, 65.5, 32.8, 16.4]
        self.accel_sensitivity = [16384, 8192, 4096, 2048]
        
    async def set_clock_source(self, source):
        data = await self.i2c.read(self.addr, 0x6B, 1)
        if data is None:
            return False
        data = data[0]
        data = (data & 0xf8) | (source & 0x07)
        n = await self.i2c.write(self.addr, 0x6B, [data])
        return n == 1
    
    # page 14
    async def set_gyro_fullscale_range(self, mode):
        # 0 = 250 deg/s
        # 1 = 500
        # 2 = 1000
        # 3 = 2000
        mode = mode & 0x03
        data = await self.i2c.read(self.addr, 0x1b, 1)
        if data is None:
            return False
        data = data[0]
        data = (data & int("11100111", 2)) | (mode << 3)
        n  = await self.i2c.write(self.addr, 0x1b, [data])
        return n == 1
        
    # page 14
    async def set_accel_fullscale_range(self, mode):
        # 0 = 2g
        # 1 = 4g
        # 2 = 8g
        # 3 = 16g
        mode = mode & 0x03
        data = await self.i2c.read(self.addr, 0x1c, 1)
        if data is None:
            return False
        data = data[0]
        data = (data & int("11100111", 2)) | (mode << 3)
        n  = await self.i2c.write(self.addr, 0x1c, [data])
        return n == 1
            
    async def set_sleep(self, is_sleep):
        data = await self.i2c.read(self.addr, 0x6B, 1)
        if data is None:
            return False
        data = data[0]
        if not is_sleep:
            n = await self.i2c.write(self.addr, 0x6B, [data & ~(1 << 6)])
        else:
            n = await self.i2c.write(self.addr, 0x6B, [data | (1 << 6)])
        return n == 1

    async def get_gyro_sensitivity(self):
        data = await self.i2c.read(self.addr, 0x1b, 1)
        if data is None:
            return None
        data = data[0]
        mode = (data & ~int("11100111", 2)) >> 3
        return self.gyro_sensitivity[mode]
    
    async def get_accel_sensitivity(self):
        data = await self.i2c.read(self.addr, 0x1c, 1)
        if data is None:
            return None
        data = data[0]
        mode = (data & ~int("11100111", 2)) >> 3
        return self.accel_sensitivity[mode]
    
    async def is_ready(self):
        data = await self.i2c.read(self.addr, 0x3a, 1)
        if data is None:
            return data
        data = data[0]
        is_ready = (data & 0x01) != 0x00
        return is_ready
