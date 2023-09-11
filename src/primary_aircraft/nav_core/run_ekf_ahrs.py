import sys
import numpy as np
import matplotlib.pyplot as plt
from src.primary_aircraft.nav_core.nav_core import NAVCore, SensorReport
from math import pi, atan2
from modules.ahrs.common.orientation import am2angles
from src.modules.ahrs.filters import EKF
from src.modules.ahrs.common.orientation import acc2q, q2euler, am2q, q2rpy
from src.modules.ahrs.utils.wmm import WMM


P = np.array([[ 3.84830773e-03,-7.56085038e-05, 3.66641139e-04,-5.57215587e-04],
 [-7.56085038e-05, 6.33500666e-04,-9.96881488e-06, 2.02551025e-05],
 [ 3.66641139e-04,-9.96881488e-06, 6.81227140e-04,-4.71979784e-05],
 [-5.57215587e-04, 2.02551025e-05,-4.71979784e-05, 9.85962205e-05]])

def load_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for i, line in enumerate(file):
            if i != 0:
                values = line.strip().split(',')
                data.append(list(map(float, values)))
    return np.array(data)


if __name__ == '__main__':

    data = load_data_from_file('semi_static_various_motion_data_2.csv')  # Open the file containing the results here into a numpy array (nx10)

    t = 0

    # Aircraft attitude measured solely with accelerometer
    m_roll = []
    m_pitch = []
    m_yaw = []

    # Aircraft attitude measured solely with integrated gyro readings
    i_roll = [0]
    i_pitch = [0]
    i_yaw = [0]

    # Sensor measurements at each timestep
    gyr_data = []
    mag_data = []
    acc_data = []

    # The total acceleration measured at each timestep
    acc_norm = []

    # A vector containing all the timesteps (delta t) as well as the total time
    dt_v = []
    t_v = []

    for idx, data_point in enumerate(data):
        if idx != len(data) - 1:
            dt = (data[idx + 1][0] - data_point[0])
            t += dt
        else:
            break

        AccX = data_point[1]
        AccY = data_point[2]#
        AccZ = data_point[3]
        GyrX = data_point[4]#*1.175#
        GyrY = data_point[5]#*1.175
        GyrZ = data_point[6]#*1.175
        MagX = data_point[7]
        MagY = data_point[8]
        MagZ = -data_point[9]#

        mag_data.append([MagX, MagY, MagZ])
        acc_data.append([AccX, AccY, AccZ])
        gyr_data.append([np.radians(GyrX), np.radians(GyrY), np.radians(GyrZ)])

        dt_v.append(dt)
        t_v.append(t)

        #roll, pitch, yaw = am2angles(np.array([AccX, AccY, AccZ]), np.array([MagX, MagY, MagZ]), in_deg=True)[0]
        q_est = am2q(np.array([-AccX, -AccY, -AccZ]), np.array([MagX, MagY, MagZ]),frame='NED')
        roll, pitch, yaw = q2rpy(q_est, in_deg=True)

        m_roll.append(roll)
        m_pitch.append(pitch)
        m_yaw.append(yaw)

        i_roll.append(i_roll[-1]+GyrX*dt)
        i_pitch.append(i_pitch[-1]+GyrY*dt)
        i_yaw.append(i_yaw[-1]+GyrZ*dt)

        acc_norm.append(np.linalg.norm(acc_data[-1]))


    acc_data = np.array(acc_data)
    mag_data = np.array(mag_data)
    gyr_data = np.array(gyr_data)

    i_yaw = np.array(i_yaw)
    i_roll = np.array(i_roll)
    i_pitch = np.array(i_pitch)


    wmm = WMM(latitude=45.513149172533765, longitude=-73.62900722758671)
    ekf = EKF(magnetic_ref=wmm.I, noises=[0.5**2, 0.1**2, 0.3**2],frame='NED')
    Q = np.zeros((len(acc_data), 4))  # Allocate array for quaternions

    a0 = np.array([-acc_data[0][0], -acc_data[0][1], -acc_data[0][2]])
    m0 = mag_data[0]


    print(f'ACC0 = {acc_data[0]}')
    print(f'MAG0 = {mag_data[0]}')

    roll, pitch, yaw = am2angles(acc_data[0], mag_data[0], in_deg=True)[0]
    print([roll, pitch, yaw])
    Q[0] = am2q(a0, m0, frame='NED')
    roll, pitch, yaw = np.degrees(q2rpy(Q[0]))
    print([roll, pitch, yaw])

    i_yaw += yaw
    i_roll += roll
    i_pitch += pitch

    for t in range(1, len(acc_data)):
        if acc_norm[t] > 1.05:
            correct = False
        else:
            correct = True

            ekf.update_measurement_noise_covariance(
                noises=[(0.5) ** 2,
                        (0.1 * (acc_norm[t]-1)*150) ** 2 ,
                        0.3 ** 2])

        Q[t] = ekf.update(Q[t - 1], gyr=gyr_data[t], acc=acc_data[t], mag=mag_data[t], dt=dt_v[t], correct=correct)


    print(np.array2string(ekf.P, separator=','))
    eulers = np.array([np.degrees(q2euler(i))[::-1] for i in Q])
    eulers = np.array([np.degrees(q2rpy(i))[::-1] for i in Q])


    # Create a figure and a grid of 3 rows and 1 column
    fig, axes = plt.subplots(nrows=4, ncols=1, figsize=(8, 10))

    # Plotting on each subplot
    axes[0].axhline(0, color='gray')
    axes[0].plot(t_v[:], eulers[:, 0], label='Filtered Yaw', color='blue')
    axes[0].plot(t_v[:], m_yaw[:], '--', label='Measured Yaw', color='blue')
    axes[0].plot(t_v[:], i_yaw[1:], '-.', label='Integrated Yaw', color='blue')

    axes[1].axhline(0, color='gray')
    axes[1].plot(t_v[:], eulers[:, 1], label='Filtered Pitch', color='orange')
    axes[1].plot(t_v[:], m_pitch[:], '--', label='Measured Pitch', color='orange')
    axes[1].plot(t_v[:], i_pitch[1:], '-.', label='Integrated Pitch', color='orange')

    axes[2].axhline(0, color='gray')
    axes[2].plot(t_v[:], eulers[:, 2], label='Filtered Roll', color='green')
    axes[2].plot(t_v[:], m_roll[:], '--', label='Measured Roll', color='green')
    axes[2].plot(t_v[:], i_roll[1:], '-.', label='Integrated Roll', color='green')

    axes[3].axhline(1.1, color='gray')
    axes[3].plot(t_v[:], acc_norm[:], label='Acceleration Norm', color='red')

    # Add titles, labels, legends, etc. for each subplot
    labels = ['Yaw', 'Pitch', 'Roll', 'Acceleration Norm']
    for i, ax in enumerate(axes):
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(labels[i])
        ax.legend()

        if i != 0 and i != len(labels)-1:
            ax.set_ylim(-90, 90)
        elif i == len(labels)-1:
            ax.set_ylim(0.5, 1.5)
        else:
            ax.set_ylim(0, 360)

    # Optionally save or display the figure
    # plt.savefig('subplots.png')
    print(f"Means Measured: {np.mean(gyr_data[:,0]):0.5f}, {np.mean(gyr_data[:,1]):0.5f}, {np.mean(gyr_data[:,2]):0.5f}")
    print(f"Variance Measured: {np.var(gyr_data[:, 0]):0.5f}, {np.var(gyr_data[:,1]):0.5f}, {np.var(gyr_data[:, 2]):0.5f}")

    print(f"Means Measured: {np.mean(acc_data[:, 0]):0.5f}, {np.mean(acc_data[:, 1]):0.5f}, {np.mean(acc_data[:, 2]):0.5f}")
    print(f"Variance Measured: {np.var(acc_data[:, 0]):0.5f}, {np.var(acc_data[:, 1]):0.5f}, {np.var(acc_data[:, 2]):0.5f}")

    print(f"Means Filtered: {np.mean(mag_data[:, 0]):0.5f}, {np.mean(mag_data[:, 1]):0.5f}, {np.mean(mag_data[:, 2]):0.5f}")
    print(f"Variance Filtered: {np.var(mag_data[:, 0]):0.5f}, {np.var(mag_data[:, 1]):0.5f}, {np.var(mag_data[:, 2]):0.5f}")
    plt.tight_layout()  # Ensures that subplots are nicely spaced
    plt.show()

    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from scipy.spatial.transform import Rotation as R
    import numpy as np
    from matplotlib.animation import FuncAnimation

    # Create a figure and 3D axis
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')


    # Function to update the coordinate system's orientation using quaternions
    def update(q):
        ax.cla()  # Clear the previous frame

        quaternion = [q[1], q[2], q[3], q[0]]

        # Define the axis vectors
        axis_x = np.array([1, 0, 0])
        axis_y = np.array([0, 1, 0])
        axis_z = np.array([0, 0, 1])

        # Rotate the axis vectors using the quaternion
        #rotated_axis_x = quaternion_rotation(quaternion, axis_x)
        #rotated_axis_y = quaternion_rotation(quaternion, axis_y)
        #rotated_axis_z = quaternion_rotation(quaternion, axis_z)
        rotated_axis_x = R.from_quat(quaternion).apply(axis_x)
        rotated_axis_y = R.from_quat(quaternion).apply(axis_y)
        rotated_axis_z = R.from_quat(quaternion).apply(axis_z)

        # Plot the rotated coordinate system
        ax.quiver(0, 0, 0, rotated_axis_x[0], rotated_axis_x[1], rotated_axis_x[2], color='r', label='X-axis')
        ax.quiver(0, 0, 0, rotated_axis_y[0], rotated_axis_y[1], rotated_axis_y[2], color='g', label='Y-axis')
        ax.quiver(0, 0, 0, rotated_axis_z[0], rotated_axis_z[1], rotated_axis_z[2], color='b', label='Z-axis')

        
        ax.legend()



    # Create an animation
    ani = FuncAnimation(fig, update, frames=Q, interval=50)

    # Display the animation
    plt.show()

