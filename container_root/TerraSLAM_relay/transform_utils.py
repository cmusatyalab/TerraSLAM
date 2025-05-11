import numpy as np

def read_transform_matrix(filepath):
    """Read transformation matrix from file.
    
    Args:
        filepath: Path to the file containing the transformation matrix
        
    Returns:
        4x4 transformation matrix as numpy array
    """
    try:
        with open(filepath, 'r') as f:
            lines = f.readlines()
            matrix = []
            for line in lines:
                row = [float(x) for x in line.strip().split()]
                if row:  # Skip empty lines
                    matrix.append(row)
            return np.array(matrix)
    except Exception as e:
        print(f"Error reading transform matrix from {filepath}: {e}")
        return np.eye(4)  # Return identity matrix as fallback

def transform_point(transform_matrix, point):
    """Transform a 3D point using a 4x4 transformation matrix.
    
    Args:
        transform_matrix: 4x4 transformation matrix as numpy array
        point: 3D point as a list or array [x, y, z]
        
    Returns:
        Transformed point as a list [x', y', z']
    """
    # Convert point to homogeneous coordinates
    homogeneous_point = np.array([point[0], point[1], point[2], 1.0])
    
    # Apply transformation
    transformed_point = transform_matrix.dot(homogeneous_point)
    
    # Convert back to 3D coordinates
    if transformed_point[3] != 0:
        transformed_point = transformed_point / transformed_point[3]
    
    return transformed_point[0:3].tolist() 