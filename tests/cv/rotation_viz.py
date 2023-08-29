import math
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# Heading = 90 - theta
# Tilt = Tilt
# Roll = - yaw
yaw = 70
pitch = -15  # negative is nose up
roll = 25    # positive is left wing down


# Define the original vector
original_vector = np.array([0.0, 0.0, -1.0])

# Define the rotation matrix (example rotation about y-axis by 45 degrees)
rot_mat0 = Rotation.from_euler('z', yaw, degrees=True).as_matrix()
rot_mat2 = Rotation.from_euler('x', roll, degrees=True).as_matrix()
rot_mat1 = Rotation.from_euler('y', pitch, degrees=True).as_matrix()

rots = [rot_mat0, rot_mat1, rot_mat2]
# Create a 3D plot
fig = plt.figure()
rot_mat = np.eye(3)
for i, m in enumerate(rots):
    rot_mat = np.linalg.linalg.matmul(rot_mat, m)
    z_prime = np.linalg.linalg.matmul(rot_mat, original_vector)
    tilt = math.degrees(np.pi / 2 - np.arccos(np.abs(np.linalg.linalg.dot(z_prime, [0.0, 0.0, -1.0])) / (np.linalg.norm(z_prime))))
    print(90-tilt)
    # Apply the rotation to the vector
    rotated_vector = z_prime


    # Calculate the projection of z_p onto the X-Y plane
    z_p_proj = rotated_vector - np.dot(rotated_vector, np.array([0, 0, 1])) * np.array([0, 0, 1])

    # Calculate the heading angle (angle between z_p_proj and the Y-axis)
    cos_theta_heading = np.dot(z_p_proj, np.array([1, 0, 0])) / np.linalg.norm(z_p_proj)
    theta_heading = np.degrees(np.arccos(cos_theta_heading))
    print(theta_heading)



    ax = fig.add_subplot(101+len(rots)*10+i, projection='3d')

    # Plot the original and rotated vectors
    origin = np.array([0, 0, 0])
    ax.quiver(*origin, *original_vector, color='b', label='Original Vector')
    ax.quiver(*origin, *rotated_vector, color='r', label='Rotated Vector')
    ax.quiver(*origin, *z_p_proj, color='black', label='Projected Vector')

    # Plot the main axes (x, y, z)
    ax.quiver(0, 0, 0, 1, 0, 0, color='g', label='X Axis')
    ax.quiver(0, 0, 0, 0, 1, 0, color='g', label='Y Axis')
    ax.quiver(0, 0, 0, 0, 0, 1, color='g', label='Z Axis')

    # Define the vertices of the rotated rectangle
    rectangle_length = 2/2
    rectangle_width = 1.5/2
    rectangle_z_off = -1/3*0
    rectangle_vertices = [
        [-rectangle_width / 2, -rectangle_length / 2, rectangle_z_off],
        [-rectangle_width / 2, rectangle_length / 2, rectangle_z_off],
        [rectangle_width / 2, rectangle_length / 2, rectangle_z_off],
        [rectangle_width / 2, -rectangle_length / 2, rectangle_z_off]
    ]



    # Apply the same rotation matrix to the rectangle vertices
    rotated_rectangle_vertices = np.dot(rot_mat, np.array(rectangle_vertices).T).T

    ax.quiver(0, 0, 0,
              rotated_rectangle_vertices[2][0] - rotated_rectangle_vertices[1][0],
              rotated_rectangle_vertices[2][1] - rotated_rectangle_vertices[1][1],
              rotated_rectangle_vertices[2][2] - rotated_rectangle_vertices[1][2],
              color='purple', label='Camera Y')

    # Plot the rotated rectangle
    rectangle = Poly3DCollection([rotated_rectangle_vertices], color='purple', alpha=0.7)
    ax.add_collection3d(rectangle)

    # Set plot limits and labels
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Add legend
    ax.legend()

# Show the plot
plt.show()
