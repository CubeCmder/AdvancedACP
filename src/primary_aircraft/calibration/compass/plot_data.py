import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg.linalg

INTERVAL = 0.01

fig, ax = plt.subplots(1, 1)
ax.set_aspect(1)

def read_points_file(filename):
    points = []
    with open(filename, 'r') as file:
        for line in file:
            values = line.strip().split('\t')
            if len(values) == 3:
                x, y, z = map(float, values)
                points.append((x, y, z))
    return np.array(points)

ret = read_points_file('points.txt')

# Calculate the hard iron offset (average of min and max values for each axis)
hard_iron_offset = np.mean(ret, axis=0)

# Center the data by subtracting the hard iron offset
centered_data = ret - hard_iron_offset

# Calculate the scaling matrix
scaling_matrix = np.diag(1.0 / np.max(centered_data, axis=0))

# Apply the scaling to get the data points on a unit sphere
scaled_data = centered_data.dot(scaling_matrix)

# Calculate the covariance matrix
covariance_matrix = np.cov(scaled_data, rowvar=False)

# Calculate the eigenvalues and eigenvectors
eigenvalues, eigenvectors = np.linalg.eig(covariance_matrix)

# The soft iron matrix is formed by the eigenvectors
soft_iron_matrix = eigenvectors ## ===> WRONG


print("Scaling Matrix:")
print(scaling_matrix)

print("Hard Iron Offset:", hard_iron_offset)
print("Soft Iron Matrix:")
print(soft_iron_matrix)

# Create some example data
x = ret[:,0]
y = ret[:,1]
z = ret[:,2]

# Create a figure and a 3D axes
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


# Create the scatter plot
ax.scatter(x, y, z, c='b', marker='o')

# Add labels and title
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.set_title('3D Surface Plot')
ax.set_aspect('equal')



# Show the plot
plt.show()





