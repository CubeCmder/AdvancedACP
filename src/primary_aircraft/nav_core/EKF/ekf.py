import numpy as np
from numpy.linalg import linalg
class ekf():
    """
    This class is an implementation of the Kalman Filter to track the position and orientation of the
    primary aircraft. The following (total of 15) states are tracked:

        - Aircraft position: X, Y, Z position expressed in [m], velocity expressed in [m/s] and accelerations
        expressed in [m/s2] (9 states) in the GLOBAL coordinate system.

        - Aircraft orientation: Bearing (or heading), pitch and roll angles expressed in [rad] and angular velocities
        expressed in [rad/s] (6 states) in the GLOBAL coordinate system.

    The required sensors are:

        - Accelerometer -> Provides acceleration along the LOCAL x/y/z axes expressed in [m/s2]. 3

        - Gyroscope -> Provides angular velocities about the Yaw/Pitch/Roll (about the LOCAL axes) 3
        angular velocities expressed in [rad/s].

        - Magnetometer -> Provides measurements of the local magnetic field about the x/y/z axes, and thus magnetic
        bearing expressed in [rads]. The bearing is measured relative to the GLOBAL Y axis, and is positive clockwise.1

        - Altimeter -> Provides absolute altitude measurements expressed in [m] in the GLOBAL coordinate system.1

        - GPS -> Provides absolute positioning of the aircraft in the geodetic coordinate system. Needs to be converted
        to meters about the GLOBAL coordinate system origin. 2

    Coordinate systems:

        - LOCAL coordinate system: Sensor's coordinate system, which is aligned with the aircraft's coordinate system
        and axes. "x" is oriented along the aircraft fuselage (positive towards the nose), "y" is oriented spanwise
        (positive LHS) and "z" is oriented following the right hand rule. Noted (x/y/z).

        - GLOBAL / WORLD coordinate system: The global coordinate system, where X is oriented West to East and the
        Y axis is oriented South to North. Z is the altitude. Note that the origin of this coordinate system needs to be
        set near the intended flight zone, before every flight, because this coordinate system is expressed in meters.
        Nom. (X/Y/Z).
    """

    def __int__(self, n=15, m=6, p=4, x_i=None, gps_origin=None):


        # Number of state variables
        self.n = 15
        # Number of control inputs: Accelerations and Gyro angular rates
        self.m = 6
        # Number of measurement inputs: Heading, Altitude (Z position), GPS position
        self.p = 4

        if gps_origin is None:
            self.origin = np.array([45.5182539409515, -73.7843097082072])
        else:
            self.origin = gps_origin

        # Find magnetic declination at origin
        self.mag_declination = 0

        # System State Vector
        self.x = np.zeros(n, 1)
        # Control Input Vector
        self.u = np.zeros(m, 1)

        # Error Covariance Matrix
        self.P = np.zeros(n)
        # State Transition Matrix (To Be Defined)
        self.F = np.zeros(n)
        # Control Input Matrix
        self.B = np.zeros(n, m)
        # Measurement Matrix H
        self.H = np.zeros(p, n)
        # Process Noise Covariance Matrix
        self.Q = np.zeros(n)
        # Measurement Noise Covariance Matrix
        self.R = np.zeros(p)

        # Kalman Gain
        self.K = np.zeros(n, p)

    def ricatti(self):
        pass

    def update_matrix_F(self, dt):
        F = np.zeros(self.n)

        return F
    def predict(self, dt=None):
        """

        Returns:

        """
        if dt is not None:
            F = self.update_matrix_F(dt)
        else:
            F = self.F

        # Define matrix F based on dt
        self.x = F @ self.x + self.B @ self.u
        self.P = (F @ self.x) @ F.T + self.Q

        self.K = self.P @ self.H.T @ linalg.inv(self.H @ self.P @ self.H.T + self.R)


    def update(self):
        '''
        Returns the next best state estimate at time 't'

        Returns:

        '''
        z = np.zeros(self.P)
        self.x = self.x + self.K @ (z - self.H @ self.x)

        return self.x
