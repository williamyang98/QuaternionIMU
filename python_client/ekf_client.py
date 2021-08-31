from ekf import MultiRateExtendedKalmanFilter
import numpy as np

# add calibration support for multirate extended kalman filter
class EKFClient:
    def __init__(self, use_paired_measurements=False):
        self.ekf = MultiRateExtendedKalmanFilter()
        self.last_gyro_dt = None
        self.last_dt = 0

        self.a_xyz_0 = np.zeros((3,1)).reshape((3,1))
        self.m_xyz_0 = np.zeros((3,1)).reshape((3,1))
        self.pqr_0   = np.zeros((3,1)).reshape((3,1))

        self.use_paired_measurements = use_paired_measurements
        self.a_xyz_stored = None
        self.m_xyz_stored = None

        # when it is calibrating, load all measurements into buffer
        self.is_calibrating = True
        self.calib_m_xyz_buf = []
        self.calib_a_xyz_buf = []
        self.calib_pqr_buf = []

        self.Ra = 0.1*np.eye(3)
        self.Rm = 0.01*np.eye(3)

        # our magnetometer has some bias
        # this means as we rotate the sensor the magnitude of the magnetometer may change
        # we can normalise this so that our ekf doesn't break when it changes
        # even if the magnetometer is in the correctly predicted direction, the difference in 
        # magnitude will cause the ekf to try to correct this error and rotate forever
        self.normalise_magnetometer = True

    # when calibration is over we update our internal bias measurements 
    def set_calibrate(self, is_calibrating):
        if is_calibrating == self.is_calibrating:
            return
        
        if not self.is_calibrating:
            self.is_calibrating = True
            return

        # on exiting calibration
        # 1) Calculate the mean values of our sensor measurements 
        # 2) Update the external vector references for Axyz and Mxyz
        # 3) Update the pqr bias
        # 4) Set the orientation back to [1,0,0,0]
        self.is_calibrating = False
        if len(self.calib_pqr_buf) == 0 or len(self.calib_m_xyz_buf) == 0:
            return

        self.m_xyz_0 = np.array(self.calib_m_xyz_buf).squeeze().mean(axis=0).reshape((3,1))
        self.a_xyz_0 = np.array(self.calib_a_xyz_buf).squeeze().mean(axis=0).reshape((3,1))
        self.pqr_0   = np.array(self.calib_pqr_buf).squeeze().mean(axis=0).reshape((3,1))

        print(f"Calculated means of A={self.a_xyz_0.flatten()} M={self.m_xyz_0.flatten()} PQR={self.pqr_0.flatten()}")

        self.calib_m_xyz_buf = []
        self.calib_a_xyz_buf = []
        self.calib_pqr_buf = []

        self.ekf.E  = np.array([1,0,0,0]).reshape((4,1))
        self.ekf.Pk = 1e-6*np.eye(4)

        # ignore the first gyro reading after calibration
        # since we might not have an accurate Ts
        self.last_gyro_dt = None
    
    def on_compass(self, dt, m_xyz):
        m_xyz = np.array(m_xyz).reshape((3,1))
        if self.normalise_magnetometer:
            m_xyz = m_xyz / np.linalg.norm(m_xyz, axis=0)

        self.last_dt = dt
        if self.is_calibrating:
            self.calib_m_xyz_buf.append(m_xyz)
            return

        
        self.m_xyz_stored = m_xyz

        if self.use_paired_measurements and self.a_xyz_stored is not None:
            z = np.zeros((3,3))
            R = np.block([[self.Rm, z], [z, self.Ra]])
            Ek, Pk = self.ekf.measure([m_xyz, self.a_xyz_stored], [self.m_xyz_0, self.a_xyz_0], self.ekf.E, self.ekf.Pk, R)
        else:
            R = self.Rm
            Ek, Pk = self.ekf.measure([m_xyz], [self.m_xyz_0], self.ekf.E, self.ekf.Pk, R)

        self.ekf.E = Ek
        self.ekf.Pk = Pk

    def on_gyro(self, dt, a_xyz, pqr):
        a_xyz = np.array(a_xyz).reshape((3,1))
        pqr = np.array(pqr).reshape((3,1))

        self.last_dt = dt

        if self.last_gyro_dt is None:
            self.last_gyro_dt = dt

        if self.is_calibrating:
            self.calib_a_xyz_buf.append(a_xyz)
            self.calib_pqr_buf.append(pqr)
            return 

        Ts = dt-self.last_gyro_dt
        self.last_gyro_dt = dt
        
        pqr = pqr - self.pqr_0

        Ek, Pk = self.ekf.predict(pqr, self.ekf.E, self.ekf.Pk, Ts)

        self.a_xyz_stored = a_xyz

        if self.use_paired_measurements and self.m_xyz_stored is not None:
            z = np.zeros((3,3))
            R = np.block([[self.Ra, z], [z, self.Rm]])
            Ek, Pk = self.ekf.measure([a_xyz, self.m_xyz_stored], [self.a_xyz_0, self.m_xyz_0], Ek, Pk, R)
        else:
            R = self.Ra
            Ek, Pk = self.ekf.measure([a_xyz], [self.a_xyz_0], Ek, Pk, R)
        
        self.ekf.E = Ek
        self.ekf.Pk = Pk

        




