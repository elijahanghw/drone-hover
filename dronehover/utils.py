import numpy as np
from numpy import sin, cos, pi
from numpy.linalg import norm

def rotation_matrix(roll, pitch, yaw):
    """
    Compute rotation matrix for yaw, pitch, roll rotations (ZYX Euler angles).
    
    Args:
        yaw: Rotation around Z-axis (radians)
        pitch: Rotation around Y-axis (radians)
        roll: Rotation around X-axis (radians)
        
    Returns:
        3x3 rotation matrix
    """
    # Rotation around Z-axis (yaw)
    Rz = np.array([[cos(yaw), -sin(yaw), 0],
                    [sin(yaw),  cos(yaw), 0],
                    [0,         0,        1]])
    
    # Rotation around Y-axis (pitch)
    Ry = np.array([[cos(pitch),  0, sin(pitch)],
                    [0,           1, 0],
                    [-sin(pitch), 0, cos(pitch)]])
    
    # Rotation around X-axis (roll)
    Rx = np.array([[1, 0,          0],
                    [0, cos(roll), -sin(roll)],
                    [0, sin(roll),  cos(roll)]])
    
    # Combined rotation: R = Rz * Ry * Rx
    return Rz @ Ry @ Rx


def align_vectors(v1, v2):
    """
    Compute rotation matrix that aligns vector v1 to vector v2.
    Uses Rodrigues' rotation formula.
    
    Args:
        v1: Initial vector (3D array or list)
        v2: Target vector (3D array or list)
        
    Returns:
        3x3 rotation matrix R such that R @ v1 is aligned with v2
    """
    # Normalize vectors
    v1 = np.array(v1, dtype=float)
    v2 = np.array(v2, dtype=float)
    v1 = v1 / norm(v1)
    v2 = v2 / norm(v2)
    
    # Compute cross product (rotation axis)
    v = np.cross(v1, v2)
    
    # Compute dot product (cosine of angle)
    c = np.dot(v1, v2)
    
    # Handle special cases
    if c < -0.9999:  # Vectors are opposite (180 degrees)
        # Find an orthogonal vector
        orthogonal = np.array([1, 0, 0]) if abs(v1[0]) < 0.9 else np.array([0, 1, 0])
        v = np.cross(v1, orthogonal)
        v = v / norm(v)
        # Return 180-degree rotation around v
        return 2 * np.outer(v, v) - np.eye(3)
    
    if c > 0.9999:  # Vectors are already aligned
        return np.eye(3)
    
    # Rodrigues' rotation formula
    s = norm(v)  # sine of angle
    K = np.array([[0, -v[2], v[1]],
                  [v[2], 0, -v[0]],
                  [-v[1], v[0], 0]])  # Skew-symmetric matrix
    
    R = np.eye(3) + K + (K @ K) * ((1 - c) / (s ** 2))
    
    return R