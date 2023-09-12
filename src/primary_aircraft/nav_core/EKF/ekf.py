"""
This module is an implementation of the Extended Kalman Filter (EKF) to track the position and orientation of the
primary aircraft. The following (total of 10) states are tracked by two separate filters:

    - Aircraft position: X, Y, Z position expressed in [m], velocity expressed in [m/s] and accelerations expressed
    in [m/s2] (9 states) in the GLOBAL coordinate system.

    - Aircraft orientation: One single state, a quaternion that represents the orientation of the aircraft (
    unitless), expressed in the GLOBAL coordinate system .

The required sensors are:

    - Accelerometer -> Provides acceleration along the LOCAL x/y/z axes expressed in [m/s2] or [g].

    - Gyroscope     -> Provides angular velocities about the LOCAL x/y/z axes expressed in [rad/s].

    - Magnetometer  -> Provides measurements of the local magnetic field about the LOCAL x/y/z axes expressed in [Gauss]

    - Altimeter     -> Provides absolute altitude measurements expressed in [m] in the GLOBAL coordinate system.

    - GPS           -> Provides absolute positioning of the aircraft in the geodetic coordinate system. Needs to be
    converted to meters about the GLOBAL coordinate system origin.

Coordinate systems:

    - LOCAL coordinate system: Sensor's coordinate system, which is aligned with the aircraft's coordinate system
    and axes. "x" is oriented along the aircraft fuselage (positive towards the nose), "y" is oriented spanwise
    (positive RHS) and "z" is oriented following the right hand rule. Noted (x/y/z).

    - GLOBAL / WORLD coordinate system: The global coordinate system, where Y is oriented West to East and the X axis
    is oriented South to North. Z is the altitude (positive downwards). Note that the origin of this coordinate
    system needs to be set near the intended flight zone, before every flight, because this coordinate system is
    expressed in meters (while the GPS coordinate system is a spherical coordinate system). Noted (X/Y/Z). This model
    corresponds to the 'NED' (North-East-Down) coordinate system (see
    https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates).

"""


class PositionEKF:

    def __int__(self):
        pass


class AhrsEKF:
    """
    This class is an implementation of the Extended Kalman Filter (EKF) for the estimation of the Primary Aircraft's
    attitude and heading. It uses the concept of the quaternion for orientation representation. The aircraft's
    orientation is a necessary variable in order to be able to fuse GPS data with accelerometer readings.

    NOTE: This implementation is a slightly modified version of the EKF implementation found in the AHRS python
    library created by Mayitzin (github: https://github.com/Mayitzin/ahrs).

    """
    def __init__(self):
        pass
