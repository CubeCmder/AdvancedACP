import numpy as np
import matplotlib.pyplot as plt

def load_angles_from_file(filename):
    angles = []
    with open(filename, 'r') as file:
        for line in file:
            values = line.strip().split(',')
            angles.append(list(map(float, values)))
    return np.array(angles)

angles = loaded_angles = load_angles_from_file('angles_data.csv')# Open the file containing the results here into a numpy array (nx4)

# Create a figure and a grid of 3 rows and 1 column
fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(8, 10))
# Plotting on each subplot
axes[0].plot(angles[:, 0], angles[:, 1], label='Yaw', color='blue')
axes[0].plot(angles[:, 0], angles[:, 4], '--',label='Measured Yaw', color='blue')
axes[1].plot(angles[:, 0], angles[:, 2], label='Pitch', color='orange')
axes[1].plot(angles[:, 0], angles[:, 5], '--', label='Measured Yaw', color='orange')
axes[2].plot(angles[:, 0], angles[:, 3], label='Roll', color='green')
axes[2].plot(angles[:, 0], angles[:, 6], '--', label='Measured Yaw', color='green')

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
print(f"Means Measured: {np.mean(angles[:,4]):0.5f}, {np.mean(angles[:,5]):0.5f}, {np.mean(angles[:,6]):0.5f}")
print(f"Variance Measured: {np.var(angles[:,4]):0.5f}, {np.var(angles[:,5]):0.5f}, {np.var(angles[:,6]):0.5f}")

print(f"Means Filtered: {np.mean(angles[:,1]):0.5f}, {np.mean(angles[:,2]):0.5f}, {np.mean(angles[:,3]):0.5f}")
print(f"Variance Filtered: {np.var(angles[:,1]):0.5f}, {np.var(angles[:,2]):0.5f}, {np.var(angles[:,3]):0.5f}")
plt.tight_layout()  # Ensures that subplots are nicely spaced
plt.show()