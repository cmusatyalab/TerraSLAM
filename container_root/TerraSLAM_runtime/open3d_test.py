import open3d as o3d
import numpy as np

def execute_global_registration(source_down, target_down, voxel_size):
    """Execute global registration (coarse registration)"""
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_icp(
        source_down, target_down, distance_threshold,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
    return result
    
# Simple point clouds for testing
source_test = o3d.geometry.PointCloud()
source_test.points = o3d.utility.Vector3dVector(np.random.rand(10, 3))
target_test = o3d.geometry.PointCloud()
target_test.points = o3d.utility.Vector3dVector(np.random.rand(10, 3))

# Execute simplified test
result_test = execute_global_registration(source_test, target_test, 0.1)
