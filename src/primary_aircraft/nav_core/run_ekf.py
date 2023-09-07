import numpy as np
import matplotlib.pyplot as plt
from src.primary_aircraft.nav_core.nav_core import NAVCore, SensorReport
from math import pi, atan2
from modules.ahrs.common.orientation import am2angles



def load_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for i, line in enumerate(file):
            if i != 0:
                values = line.strip().split(',')
                data.append(list(map(float, values)))
    return np.array(data)


if __name__ == '__main__':

    data = loaded_data = load_data_from_file('static_axes_data.csv')  # Open the file containing the results here into a numpy array (nx10)

    AccX = -data[0][1]
    AccY = data[0][2]
    AccZ = data[0][3]
    GyrX = -data[0][4]
    GyrY = data[0][5]
    GyrZ = -data[0][6]
    MagX = data[0][7]
    MagY = -data[0][8]
    MagZ = -data[0][9]

    yaw_int, pitch_int, roll_int = am2angles(np.array([AccX, AccY, AccZ]), np.array([MagX, MagY, MagZ]), in_deg=True)[
                                       0][::-1]
    # if yaw_int < 0:
    #     yaw_int += 360

    yaw_int -= 13.7
    nav_core = NAVCore(x0_ahrs=np.array([yaw_int, pitch_int, roll_int]))
    res = []
    t = 0

    for idx, data_point in enumerate(data):
        if idx != len(data) - 1:
            dt = (data[idx + 1][0] - data_point[0])
            t += dt
        else:
            break

        AccX = -data_point[1]
        AccY = data_point[2]
        AccZ = data_point[3]
        GyrX = -data_point[4]
        GyrY = data_point[5]
        GyrZ = -data_point[6]
        MagX = data_point[7]
        MagY = -data_point[8]
        MagZ = data_point[9]

        report = SensorReport(t=t,
                              acc_x=AccX,
                              acc_y=AccY,
                              acc_z=AccZ,
                              gyr_x=GyrX,
                              gyr_y=GyrY,
                              gyr_z=GyrZ,
                              mag_x=MagX,
                              mag_y=MagY,
                              mag_z=MagZ,
                              gps_lat=0,
                              gps_lon=0,
                              gps_alt=0,
                              baro_temp=0,
                              baro_press=0,
                              baro_alt=0,
                              mag_declination=nav_core.mag_declination)

        if np.linalg.norm([AccX, AccY, AccZ]) > 1.2:
            nav_core.update_ekf(dt, sensors=report, update=False)
        else:
            nav_core.update_ekf(dt, sensors=report, update=True)

        roll_int += GyrX * dt
        yaw_int += GyrZ * dt

        # if yaw_int < 0:
        #     yaw_int += 360
        # if yaw_int > 360:
        #     yaw_int -= 360
        yaw_measured = atan2(-MagY, MagX) * 180 / pi

        # yaw_measured = np.degrees(ecompass([AccX, AccY, AccZ], [MagX, MagY, MagZ],'NED','rpy'))[2]
        yaw_measured = am2angles(np.array([AccX, AccY, AccZ]), np.array([MagX, MagY, MagZ]), in_deg=True)[0][2] -13.7
        #if yaw_measured < 0:
            #yaw_measured += 360
        res.append(np.concatenate(
            ([t], nav_core.ekf_ahrs.x, [-report.roll_raw, -report.pitch_raw], [roll_int, yaw_int], [yaw_measured])))

    print(f'\n\n\nKalman Gain: \n{nav_core.ekf_ahrs.K}')
    res = np.array(res)
    # Create a figure and a grid of 3 rows and 1 column
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 10))
    # Plotting on each subplot
    axes[0].plot(res[:, 0], res[:, 1], label='Filtered Yaw', color='blue')
    axes[0].plot(res[:, 0], res[:, 8], '--', label='Measured Yaw', color='blue')
    axes[0].plot(res[:, 0], res[:, 7], '-.', label='Integrated Yaw', color='blue')
    axes[1].plot(res[:, 0], res[:, 2], label='Filtered Pitch', color='orange')
    axes[1].plot(res[:, 0], res[:, 5], '--', label='Measured Pitch', color='orange')
    axes[2].plot(res[:, 0], res[:, 3], label='Filtered Roll', color='green')
    axes[2].plot(res[:, 0], res[:, 4], '--', label='Measured Roll', color='green')
    axes[2].plot(res[:, 0], res[:, 6], '-.', label='Integrated Roll', color='green')

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
