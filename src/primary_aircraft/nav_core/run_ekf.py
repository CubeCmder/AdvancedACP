import numpy as np
import matplotlib.pyplot as plt
from src.primary_aircraft.nav_core.nav_core import NAVCore, SensorReport
def load_data_from_file(filename):
    data = []
    with open(filename, 'r') as file:
        for line in file[1::]:
            values = line.strip().split(',')
            data.append(list(map(float, values)))
    return np.array(data)

if __name__ == '__main__':

    data = loaded_data = load_data_from_file('data.csv') # Open the file containing the results here into a numpy array (nx10)

    nav_core = NAVCore(declination = )
    res = []

    for idx, data_point in enumerate(data):
        if idx != len(data)-1:
            dt = data[idx+1][0] - data_point[0]
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
        MagZ = data_point[9]

        report = SensorReport(dt=dt,
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
                              mag_declination=mag_declination)

        nav_core.update_ekf(dt, sensors=report)


        res.append(nav_core.x_ahrs)

    # Create a figure and a grid of 3 rows and 1 column
    fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 10))
    # Plotting on each subplot
    axes[0].plot(res[:, 0], res[:, 1], label='Filtered Yaw', color='blue')
    axes[0].plot(res[:, 0], res[:, 4], '--',label='Measured Yaw', color='blue')
    axes[1].plot(res[:, 0], res[:, 2], label='Filtered Pitch', color='orange')
    axes[1].plot(res[:, 0], res[:, 5], '--', label='Measured Pitch', color='orange')
    axes[2].plot(res[:, 0], res[:, 3], label='Filtered Roll', color='green')
    axes[2].plot(res[:, 0], res[:, 6], '--', label='Measured Roll', color='green')

    # Add titles, labels, legends, etc. for each subplot
    labels = ['Yaw', 'Pitch', 'Roll']
    for i, ax in enumerate(axes):
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(labels[i])
        ax.legend()

        if i != 0:
            ax.set_ylim(-90, 90)
        else:
            ax.set_ylim(0, 360)

    # Optionally save or display the figure
    # plt.savefig('subplots.png')
    print(f"Means Measured: {np.mean(data[:,4]):0.5f}, {np.mean(data[:,5]):0.5f}, {np.mean(data[:,6]):0.5f}")
    print(f"Variance Measured: {np.var(data[:,4]):0.5f}, {np.var(data[:,5]):0.5f}, {np.var(data[:,6]):0.5f}")

    print(f"Means Filtered: {np.mean(data[:,1]):0.5f}, {np.mean(data[:,2]):0.5f}, {np.mean(data[:,3]):0.5f}")
    print(f"Variance Filtered: {np.var(data[:,1]):0.5f}, {np.var(data[:,2]):0.5f}, {np.var(data[:,3]):0.5f}")
    plt.tight_layout()  # Ensures that subplots are nicely spaced
    plt.show()