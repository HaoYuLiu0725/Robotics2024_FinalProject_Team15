import numpy as np
import matplotlib.pyplot as plt

def create_grid(p1, p2, grid_num=5):
    p1 = np.array(p1)
    p2 = np.array(p2)
    center = (p1 + p2) / 2
    theta_line = np.atan2(abs(p1[0] - p2[0]), abs(p1[1] - p2[1]))
    theta = theta_line - np.pi / 4  # rotate 45 degrees
    
    # Vectors from center to each point
    vector1 = np.array([p1[0] - center[0], p1[1] - center[1]])
    vector2 = np.array([p2[0] - center[0], p2[1] - center[1]])
    
    # Function to rotate a vector
    def rotate_2d(vector, angle):
        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
        return np.dot(rotation_matrix, vector)
    
    # Rotate both vectors by the calculated angle
    rotated_vector1 = rotate_2d(vector1, -theta * 2)
    rotated_vector2 = rotate_2d(vector2, -theta * 2)
    
    # Get the rotated points
    p3 = center + rotated_vector1
    p4 = center + rotated_vector2

    print(f"p3: {p3}")
    print(f"p4: {p4}")

    # Create line segments
    L14 = np.linspace(p1, p4, grid_num)
    L32 = np.linspace(p3, p2, grid_num)
    grid_points = []
    for i in range(grid_num):
        L_temp = np.linspace(L14[i], L32[i], grid_num)
        if(i%2 == 0):
            grid_points.extend(L_temp)
        else:
            grid_points.extend(np.flip(L_temp, axis=0))
        
    # print(grid_points)
    return grid_points
    
# Example usage with two points
p1 = (535, 257)  # Example point 1
p2 = (100, 334)  # Example point 2

grid_points = create_grid(p1, p2)

# Convert list of points into an array for easier manipulation
grid_points = np.array(grid_points)

# Plotting the grid
plt.figure(figsize=(6, 6))
plt.scatter(grid_points[:, 0], grid_points[:, 1], c='r', marker='o')
# plt.plot([p1[0], p3[0]], [p1[1], p3[1]], 'g--')
# plt.plot([p1[0], p4[0]], [p1[1], p4[1]], 'b--')
plt.gca().set_aspect('equal', adjustable='box')
plt.show()

# #               [ X ,  Y ]
# BLOCK_POINTS = [[535, 257], 
#                 [469, 205], 
#                 [398, 129], 
#                 [350, 65], 
#                 [323, 31], 
#                 [295, 134], 
#                 [348, 180], 
#                 [435, 243], 
#                 [489, 290], 
#                 [402, 381], 
#                 [383, 332], 
#                 [334, 281], 
#                 [294, 212], 
#                 [245, 198], 
#                 [182, 270], 
#                 [235, 345], 
#                 [323, 392], 
#                 [343, 432],
#                 [272, 510], 
#                 [214, 457], 
#                 [143, 391], 
#                 [100, 334], 
#                 [45, 332], 
#                 [182, 268], 
#                 [407, 250]]
