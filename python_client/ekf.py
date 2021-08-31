import numpy as np

# Extended kalman filter
# We can sample measurements at different rates
# We dynamically construct the required observation Jacobians 
class MultiRateExtendedKalmanFilter:
    def __init__(self):
        self.E = np.array([1,0,0,0]).reshape((4,1))
        self.Pk = 1e-6*np.eye(4)
        self.Q = 1e-2*np.eye(4)

    # Project Ek and Pk ahead using pqr measurements
    def predict(self, pqr, Ek, Pk, Ts):
        e0,e1,e2,e3 = Ek.flatten()
        p,q,r = pqr.flatten()

        Wk = 0.5*Ts*np.array([[-e1,-e2,-e3],[e0,-e3,e2],[e3,e0,-e1],[-e2,e1,e0]])
        Ak = 0.5*np.array([[0,-p,-q,-r],[p,0,r,-q],[q,-r,0,p],[r,q,-p,0]])

        Qk = self.Q @ (Wk@Wk.T)

        Fk = np.eye(4) + Ak*Ts

        Ek = Fk@Ek
        Pk = Fk@self.Pk@Fk.T + Qk

        Ek /= np.linalg.norm(Ek)

        return Ek, Pk
    
    # update Ek and Pk based on measurements of body and external vectors
    # Vb = body vectors
    # Ve = external vectors
    # R = covariance matrix noise of measurements
    def measure(self, Vb, Ve, Ek, Pk, R):
        hk = self.find_observation_matrix(Ek).squeeze()
        zk_pred = np.vstack([hk@ve for ve in Ve])

        Hks = [self.find_observation_jacobian(Ek, ve).squeeze() for ve in Ve]
        Hk = np.vstack(Hks)
        zk = np.vstack([vb for vb in Vb])

        Sk = Hk@Pk@Hk.T + R
        Kf = (Pk@Hk.T)@np.linalg.inv(Sk)

        Ek = Ek + Kf@(zk - zk_pred)
        Pk = (np.eye(4) - Kf@Hk)@Pk

        Ek /= np.linalg.norm(Ek)

        return Ek, Pk

    # calculate the non-linear observation matrix for body relative (x,y,z) vector
    # C is our direction cosine matrix which we can calculate using x (our state)
    # z = h(x,e) = C(x)*e
    def find_observation_matrix(self, e):
        e0 = e[0]
        e1 = e[1]
        e2 = e[2]
        e3 = e[3]

        return np.array([
            [(e0**2+e1**2-e2**2-e3**2), 2*(e1*e2+e0*e3), 2*(e1*e3-e0*e2)],
            [2*(e1*e2-e0*e3), (e0**2-e1**2+e2**2-e3**2), 2*(e2*e3+e0*e1)],
            [2*(e1*e3+e0*e2), 2*(e2*e3-e0*e1), (e0**2-e1**2-e2**2+e3**2)],
        ])

    # calculate the Jacobian of the non-linear observation function
    # where h is non-linear observation function, and H is Jacobian
    # z = h(x,v), dz/dx = H(x,v)
    # z is our body vector, v is our external vector, x is our current quaternion orientation
    def find_observation_jacobian(self, e, v):
        e1 = e[0]
        e2 = e[1]
        e3 = e[2]
        e4 = e[3]

        v1 = v[0]
        v2 = v[0]
        v3 = v[0]

        return np.array([
            [2*e1*v1 - 2*e3*v3 + 2*e4*v2, 2*e2*v1 + 2*e3*v2 + 2*e4*v3, 2*e2*v2 - 2*e1*v3 - 2*e3*v1, 2*e1*v2 + 2*e2*v3 - 2*e4*v1],
            [2*e1*v2 + 2*e2*v3 - 2*e4*v1, 2*e1*v3 - 2*e2*v2 + 2*e3*v1, 2*e2*v1 + 2*e3*v2 + 2*e4*v3, 2*e3*v3 - 2*e1*v1 - 2*e4*v2],
            [2*e1*v3 - 2*e2*v2 + 2*e3*v1, 2*e4*v1 - 2*e2*v3 - 2*e1*v2, 2*e1*v1 - 2*e3*v3 + 2*e4*v2, 2*e2*v1 + 2*e3*v2 + 2*e4*v3], 
        ])

# Specific single rate extended kalman filter
class ExtendedKalmanFilter(MultiRateExtendedKalmanFilter):
    def __init__(self):
        super().__init__()
        self.a_xyz_0 = None
        self.m_xyz_0 = None
        self.R = np.diag([0.1,0.1,0.1,0.05,0.05,0.05])

    # If we have the full update step 
    def update(self, Ts, a_xyz, m_xyz, pqr):
        Ek, Pk = self.predict(pqr, self.E, self.Pk, Ts)
        Ek, Pk = self.measure(a_xyz, m_xyz, Ek, Pk)
        self.E = Ek
        self.Pk = Pk
    
    # update Ek and Pk based on measurements
    def measure(self, a_xyz, m_xyz, Ek, Pk):
        Vb = [a_xyz, m_xyz]
        Ve = [self.a_xyz_0, self.m_xyz_0]
        return super().measure(Vb, Ve, Ek, Pk, self.R)

# adding logging to extended kalman filter
class LoggedExtendedKalmanFilter(ExtendedKalmanFilter):
    def __init__(self):
        super().__init__()
        self.quats = []
        self.Pks = []
        self.last_dt = None

    def update(self, dt, a_xyz, m_xyz, pqr, measure_step=True):
        Ts = dt-self.last_dt
        self.last_dt = dt 

        Ek, Pk = self.predict(pqr, self.E, self.Pk, Ts)
        if measure_step:
            Ek, Pk = self.measure(a_xyz, m_xyz, Ek, Pk)

        self.E = Ek
        self.Pk = Pk

        self.quats.append(Ek)
        self.Pks.append(Pk)
