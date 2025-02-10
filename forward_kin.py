import numpy as np

def franka_fk(q_degrees):
    # Convert joint angles from degrees to radians
    q = np.deg2rad(q_degrees)
    
    # Modified DH parameters for Franka Research 3/Panda (alpha, a, d, theta_offset)
    # Source: Franka Documentation
    dh_params = [
        {'alpha': 0,      'a': 0,       'd': 0.333, 'theta_offset': 0},     # Joint 1
        {'alpha': -np.pi/2, 'a': 0,       'd': 0,     'theta_offset': 0},     # Joint 2
        {'alpha': np.pi/2,  'a': 0,       'd': 0.316, 'theta_offset': 0},     # Joint 3
        {'alpha': np.pi/2,  'a': 0.0825,  'd': 0,     'theta_offset': 0},     # Joint 4
        {'alpha': -np.pi/2, 'a': -0.0825, 'd': 0.384, 'theta_offset': 0},     # Joint 5
        {'alpha': np.pi/2,  'a': 0,       'd': 0,     'theta_offset': 0},     # Joint 6
        {'alpha': np.pi/2,  'a': 0.088,   'd': 0.107, 'theta_offset': 0},     # Joint 7
    ]
    
    # Initialize the transformation matrix (base to end-effector)
    T = np.eye(4)
    
    for i in range(7):
        alpha = dh_params[i]['alpha']
        a = dh_params[i]['a']
        d = dh_params[i]['d']
        theta_offset = dh_params[i]['theta_offset']
        
        # Joint angle (theta = q[i] + offset)
        theta = q[i] + theta_offset
        
        # Compute transformation matrix for the current joint
        Ti = np.array([
            [np.cos(theta), -np.sin(theta), 0, a],
            [np.sin(theta)*np.cos(alpha), np.cos(theta)*np.cos(alpha), -np.sin(alpha), -d*np.sin(alpha)],
            [np.sin(theta)*np.sin(alpha), np.cos(theta)*np.sin(alpha), np.cos(alpha), d*np.cos(alpha)],
            [0, 0, 0, 1]
        ])
        
        # Cumulative transformation
        T = T @ Ti
    
    # Extract position (x, y, z) and rotation matrix
    position = T[:3, 3]
    rotation = T[:3, :3]
    
    return position, rotation

# Example usage:
joint_angles = [0, 0, 0, 0, 0, 0, 0]  # All joints at 0 degrees (home position)
position, rotation = franka_fk(joint_angles)
print("End-effector position (meters):", position)
print("Rotation matrix:\n", rotation)
