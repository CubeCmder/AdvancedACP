import numpy as np
import matplotlib.pyplot as plt
from src.primary_aircraft.nav_core.nav_core import NAVCore, SensorReport
from math import pi, atan2
from modules.ahrs.common.orientation import am2angles
from src.modules.ahrs.filters import EKF
from src.modules.ahrs.common.orientation import acc2q, q2euler, am2q, q2rpy
from src.modules.ahrs.utils.wmm import WMM


def get_quaternion_from_euler(roll, pitch, yaw):
    """
    Convert an Euler angle to a quaternion.

    Input
      :param roll: The roll (rotation around x-axis) angle in radians.
      :param pitch: The pitch (rotation around y-axis) angle in radians.
      :param yaw: The yaw (rotation around z-axis) angle in radians.

    Output
      :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)

    return [qw, qx, qy, qz]

def load_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for i, line in enumerate(file):
            if i != 0:
                values = line.strip().split(',')
                data.append(list(map(float, values)))
    return np.array(data)


if __name__ == '__main__':

    data = loaded_data = load_data_from_file('data.csv')  # Open the file containing the results here into a numpy array (nx10)

    t = 0

    m_roll = []
    m_pitch = []
    m_yaw = []

    i_roll = [0]
    i_pitch = [0]
    i_yaw = [0]


    gyr_data = []
    mag_data = []
    acc_data = []
    dt_v = []
    t_v = []
    for idx, data_point in enumerate(data):
        if idx != len(data) - 1:
            dt = (data[idx + 1][0] - data_point[0])
            t += dt
        else:
            break

        AccX = data_point[1]
        AccY = data_point[2]
        AccZ = data_point[3]
        GyrX = data_point[4]
        GyrY = data_point[5]
        GyrZ = data_point[6]
        MagX = data_point[7]
        MagY = data_point[8]
        MagZ = data_point[9]#

        mag_data.append([MagX, MagY, MagZ])
        acc_data.append([AccX, AccY, AccZ])
        gyr_data.append([np.radians(GyrX), np.radians(GyrY), np.radians(GyrZ)])
        dt_v.append(dt)
        t_v.append(t)

        #roll, pitch, yaw = am2angles(np.array([AccX, AccY, AccZ]), np.array([MagX, MagY, MagZ]), in_deg=True)[0]
        q_est = am2q(np.array([-AccX, -AccY, -AccZ]), np.array([MagX, MagY, MagZ]),frame='NED')
        roll, pitch, yaw = q2rpy(q_est,in_deg=True)

        m_roll.append(roll)
        m_pitch.append(pitch)
        m_yaw.append(yaw)

        i_roll.append(i_roll[-1]+GyrX*dt)
        i_pitch.append(i_pitch[-1]+GyrY*dt)
        i_yaw.append(i_yaw[-1]+GyrZ*dt)

    acc_data = np.array(acc_data)
    mag_data = np.array(mag_data)
    gyr_data = np.array(gyr_data)

    wmm =WMM(latitude=45.513149172533765, longitude=-73.62900722758671)
    ekf = EKF(magnetic_ref=wmm.I, noises=[0.01**2, 0.5**2, 0.8**2],frame='NED')
    Q = np.zeros((len(acc_data), 4))  # Allocate array for quaternions

    a0 = np.array([-acc_data[0][0], -acc_data[0][1], -acc_data[0][2]])
    m0 = mag_data[0]


    print(f'ACC0 = {acc_data[0]}')
    print(f'MAG0 = {mag_data[0]}')
    roll, pitch, yaw = am2angles(acc_data[0], mag_data[0], in_deg=True)[0]
    print([roll, pitch, yaw])
    Q[0] = am2q(a0, m0, frame='NED')
    print(np.degrees(q2rpy(Q[0])))
    #Q[0] = acc2q(acc_data[0])  # First sample of tri-axial accelerometer
    #print(np.degrees(q2euler(Q[0])))
    for t in range(1, len(acc_data)):
        Q[t] = ekf.update(Q[t - 1], gyr=gyr_data[t], acc=acc_data[t], mag=mag_data[t], dt=dt_v[t])

    eulers = np.array([np.degrees(q2euler(i))[::-1] for i in Q])
    eulers = np.array([np.degrees(q2rpy(i))[::-1] for i in Q])


    # Create a figure and a grid of 3 rows and 1 column
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 10))
    # Plotting on each subplot
    axes[0].plot(t_v[:], eulers[:, 0], label='Filtered Yaw', color='blue')
    axes[0].plot(t_v[:], m_yaw[:], '--', label='Measured Yaw', color='blue')
    axes[0].plot(t_v[:], i_yaw[1:], '-.', label='Integrated Yaw', color='blue')

    axes[1].plot(t_v[:], eulers[:, 1], label='Filtered Pitch', color='orange')
    axes[1].plot(t_v[:], m_pitch[:], '--', label='Measured Pitch', color='orange')
    axes[1].plot(t_v[:], i_pitch[1:], '-.', label='Integrated Pitch', color='orange')

    axes[2].plot(t_v[:], eulers[:, 2], label='Filtered Roll', color='green')
    axes[2].plot(t_v[:], m_roll[:], '--', label='Measured Roll', color='green')
    axes[2].plot(t_v[:], i_roll[1:], '-.', label='Integrated Roll', color='green')

    # Add titles, labels, legends, etc. for each subplot
    labels = ['Yaw', 'Pitch', 'Roll']
    for i, ax in enumerate(axes):
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(labels[i])
        ax.legend()

        if i != 0:
            ax.set_ylim(-90, 90)
        else:
            ax.set_ylim(-90, 360)

    # Optionally save or display the figure
    # plt.savefig('subplots.png')
    print(f"Means Measured: {np.mean(data[:, 4]):0.5f}, {np.mean(data[:, 5]):0.5f}, {np.mean(data[:, 6]):0.5f}")
    print(f"Variance Measured: {np.var(data[:, 4]):0.5f}, {np.var(data[:, 5]):0.5f}, {np.var(data[:, 6]):0.5f}")

    print(f"Means Filtered: {np.mean(data[:, 1]):0.5f}, {np.mean(data[:, 2]):0.5f}, {np.mean(data[:, 3]):0.5f}")
    print(f"Variance Filtered: {np.var(data[:, 1]):0.5f}, {np.var(data[:, 2]):0.5f}, {np.var(data[:, 3]):0.5f}")
    plt.tight_layout()  # Ensures that subplots are nicely spaced
    plt.show()
