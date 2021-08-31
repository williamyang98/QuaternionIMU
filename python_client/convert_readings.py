import numpy as np

# convert our raw sensor measurements into their actual SI values and units
# All our incoming sensor values are stored as quantized integers
# We need to remove their gain and convert to floating point
class MeasurementConverter:
    def __init__(self):
        self.gain_magnetometer = 1090
        self.gain_accelerometer = 16384
        self.gain_gyroscope = 131

        self.bias_magnetometer = np.array([0,0,0]).reshape((3,1))
        self.normalise_magnetometer = False

    # m_xyz.shape: (3,N)
    def convert_magnetometer(self, m_xyz):
        m_xyz /= self.gain_magnetometer
        m_xyz = m_xyz[[0,2,1]]
        m_xyz -= self.bias_magnetometer
        return m_xyz

    # a_xyz.shape: (3,N) 
    def convert_accelerometer(self, a_xyz):
        a_xyz /= self.gain_accelerometer
        a_xyz *= 9.81
        a_xyz = a_xyz[[1,2,0]]
        a_xyz[0] *= -1
        return a_xyz
    
    # pqr.shape: (3,N) 
    def convert_gyroscope(self, pqr):
        pqr /= self.gain_gyroscope
        pqr *= np.pi/180
        pqr = pqr[[1,2,0]]
        pqr[[1,2]] *= -1
        return pqr