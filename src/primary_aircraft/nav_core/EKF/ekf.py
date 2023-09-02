from time import time
import numpy as np
from numpy.linalg import linalg

"""
This module is an implementation of the Kalman Filter to track the position and orientation of the
primary aircraft. The following (total of 15) states are tracked by two separate filters:

    - Aircraft position: X, Y, Z position expressed in [m], velocity expressed in [m/s] and accelerations
    expressed in [m/s2] (9 states) in the GLOBAL coordinate system.

    - Aircraft orientation: Bearing (or heading), pitch and roll angles expressed in [rad] and angular velocities
    expressed in [rad/s] (6 states) in the GLOBAL coordinate system.

The required sensors are:

    - Accelerometer -> Provides acceleration along the LOCAL x/y/z axes expressed in [m/s2]. 

    - Gyroscope -> Provides angular velocities about the Yaw/Pitch/Roll (about the LOCAL axes) 
    angular velocities expressed in [rad/s].

    - Magnetometer -> Provides measurements of the local magnetic field about the x/y/z axes, and thus magnetic
    bearing expressed in [rads]. The bearing is measured relative to the GLOBAL Y axis, and is positive clockwise.

    - Altimeter -> Provides absolute altitude measurements expressed in [m] in the GLOBAL coordinate system.

    - GPS -> Provides absolute positioning of the aircraft in the geodetic coordinate system. Needs to be converted
    to meters about the GLOBAL coordinate system origin. 

Coordinate systems:

    - LOCAL coordinate system: Sensor's coordinate system, which is aligned with the aircraft's coordinate system
    and axes. "x" is oriented along the aircraft fuselage (positive towards the nose), "y" is oriented spanwise
    (positive LHS) and "z" is oriented following the right hand rule. Noted (x/y/z).

    - GLOBAL / WORLD coordinate system: The global coordinate system, where X is oriented West to East and the
    Y axis is oriented South to North. Z is the altitude. Note that the origin of this coordinate system needs to be
    set near the intended flight zone, before every flight, because this coordinate system is expressed in meters.
    Nom. (X/Y/Z).
"""

class ekf_pos():


    def __int__(self, n=9, m=3, p=3, x_i=None, gps_origin=None):


        # Number of state variables
        self.n = n
        # Number of control inputs: Accelerations
        self.m = m
        # Number of measurement inputs: Altitude (Z position), GPS position
        self.p = p

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


class ekf_ahrs():
    def __init__(self,
                n=3,
                m=3,
                p=3,
                dt=0.200,
                x_i=None):


        # Number of state variables
        self.n = n
        # Number of control inputs: Gyro angular rates
        self.m = m
        # Number of measurement inputs: Heading, Pitch and Yaw? from Accelerometer
        self.p = p

        self.t0 = 0

        # System State Vector
        if x_i is None:
            self.x = np.zeros(n)
        else:
            self.x = x_i

        # Control Input Vector
        self.u = np.zeros(m)

        # Error Covariance Matrix - We assume initial conditions are known with high confidence
        self.P = np.eye(n)/8
        # State Transition Matrix (To Be Defined)
        self.F = self.update_matrix_F()
        # Control Input Matrix
        self.B = self.update_matrix_B(dt)
        # Measurement Matrix H
        self.H = np.eye(p, n, 0)
        # Process Noise Covariance Matrix - This can be obtained in the noise level in the states of the system at steady state
        #q_vals = np.array([0.39653, 3.45907, 9.56484]) * self.dt
        q_vals = np.array([1, 1, 1])*0.005625
        self.Q = np.diag(q_vals)
        # Measurement Noise Covariance Matrix - Instrument covariances
        r_vals = np.array([1, 1, 1])*0.075/6*10#**2
        self.R = np.diag(r_vals)

        # Kalman Gain
        self.K = np.zeros((n, p))

    def update_matrix_F(self):
        F = np.zeros((self.n, self.n))
        F[0, 0] = 1
        F[1, 1] = 1
        F[2, 2] = 1

        return F

    def update_matrix_B(self, dt):
        B = np.zeros((self.n, self.n))
        B[0, 0] = dt
        B[1, 1] = dt
        B[2, 2] = dt

        return B
    def predict(self, gyr, dt=None):
        """
        Use angular rate from gyro to predict new orientation in space.
        Returns:

        """
        self.u = np.array(gyr).T

        F = self.F

        if dt is not None:
            B = np.diag([dt, dt, dt])
            #B = self.update_matrix_B(dt)
        else:
            B = self.B

        # Define matrix F based on dt
        self.x = F @ self.x + B @ self.u
        self.P = (F @ self.P) @ F.T + self.Q




        return self.x


    def update(self, z):
        '''
        Returns the next best state estimate at time "t" by correcting the latest prediction with accelerometer
        measurements of the YAW/PITCH/ROLL angles (which could be biased by external accelerations such as wind gusts
        or pilot input).

        Returns:

        '''

        self.K = self.P @ self.H.T @ linalg.inv(self.H @ self.P @ self.H.T + self.R)
        self.x = self.x + self.K @ (z - self.H @ self.x)

        # Update the estimate uncertainty
        I = np.eye(self.n)
        Y = (I - self.K @ self.H)
        self.P = Y @ self.P @ Y.T + self.K @ self.R @ self.K.T

        return self.x
