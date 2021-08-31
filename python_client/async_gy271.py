import asyncio
from collections import namedtuple

GY271_Config_A = namedtuple('GY271_Config_A', ["averages", "data_rate", "measure_bias"])
GY271_Config_B = namedtuple('GY271_Config_B', ["gain", "range"])
GY271_Status = namedtuple("GY271_Status", ["locked", "ready"])

# asynchronously control the GY271 compass
class GY271:
    # control A mappings
    AVG_COUNT = [1,2,4,8]
    DATA_RATE = [0.75,1.5,3,7.5,15,30,75,"Unused"]  # (Hz)
    MEASUREMENT_BIAS = ["Normal", "Positive", "Negative", "Reserved"]
    
    # control B mappings
    MEASUREMENT_GAIN  = [1370,1090,820,660,440,390,330,230]
    MEASUREMENT_RANGE = [0.88,1.3,1.9,2.5,4.0,4.7,5.6,8.1]
    
    # mode register
    MODE_SELECT = ["Continuous", "Single", "Idle", "Idle"]
    
    def __init__(self, i2c):
        self.i2c = i2c
        self.addr = 0x1E
    
    # index of option to select
    async def set_config_A(self, average_count, data_output_rate, measurement_bias):
        MA = average_count & 0b11
        DO = data_output_rate & 0b111
        MS = measurement_bias & 0b11
        bits = 0x00 | (MA << 5) | (DO << 2) | (MS << 0)
        return await self.i2c.write(self.addr, 0x00, [bits]) == 1
    
    # get config description
    async def get_config_A(self):
        bits = await self.i2c.read(self.addr, 0x00, 1)
        if not bits:
            return None
        
        bits = bits[0]
        MA = (bits & (0b11 << 5)) >> 5
        DO = (bits & (0b111 << 2)) >> 2
        MS = bits & 0b11

        return GY271_Config_A(
            self.AVG_COUNT[MA], self.DATA_RATE[DO], self.MEASUREMENT_BIAS[MS])
    
    # index of option to select
    async def set_config_B(self, gain):
        GN = gain & 0b111
        bits = 0x00 | (GN << 5)
        return await self.i2c.write(self.addr, 0x01, [bits]) == 1
    
    async def get_config_B(self):
        bits = await self.i2c.read(self.addr, 0x01, 1)
        if not bits:
            return None
        
        bits = bits[0]
        GN = (bits & (0b111 << 5)) >> 5

        return GY271_Config_B(
            self.MEASUREMENT_GAIN[GN], self.MEASUREMENT_RANGE[GN])

    # index of option to select
    async def set_mode(self, mode):
        # 0=continuous, 1=single, 2=idle, 3=idle
        MD = mode & 0b11
        bits = 0x00 | (MD << 0)
        return await self.i2c.write(self.addr, 0x02, [bits]) == 1
    
    async def get_mode(self):
        bits = await self.i2c.read(self.addr, 0x02, 1)
        if not bits:
            return None
        
        bits = bits[0]
        MD = bits & 0b11

        return self.MODE_SELECT[MD]
    
    # lock indicates if it is busy
    async def get_status(self):
        bits = await self.i2c.read(self.addr, 0x09, 1)
        if bits is None: 
            return None
        
        bits = bits[0]
        sr_lock = bool(bits & (1 << 1))
        sr_ready = bool(bits & (1 << 0))

        return GY271_Status(sr_lock, sr_ready)
    
    # get identity marker (should be H43)
    async def get_identity(self):
        identity_reg = await self.i2c.read(self.addr, 0x0A, 3)
        if identity_reg is None:
            return None
        return ''.join(map(chr, identity_reg))