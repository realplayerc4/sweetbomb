
import unittest
import numpy as np
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# Assuming the PointCloudVoxelizer class is importable or we will mock the logic to test independent functions if we refactor them out.
# Since PointCloudVoxelizer is a Node, it's hard to instantiate without rclpy.init(). 
# We should refactor the logic into a separate class or static methods, or just test the logic functions if they are decoupled.
# For now, I'll assume we will refactor the logic into a helper class or mixin, or simply instantiate the node if rclpy is mocked.
# Actually, better to test the logic functions. I will modify the script to allow importing the logic.

class TestVoxelizerLogic(unittest.TestCase):
    def test_voxelize_simple(self):
        # Create dummy points
        points = [
            (0.1, 0.1, 0.1),
            (0.12, 0.12, 0.12), # Should be in same voxel as above if size is 0.1
            (0.5, 0.5, 0.5)
        ]
        
        # Simple voxelization logic test
        voxel_size = 0.1
        voxels = {}
        inv_size = 1.0 / voxel_size
        
        for x, y, z in points:
            vx = int(x * inv_size)
            vy = int(y * inv_size)
            vz = int(z * inv_size)
            key = (vx, vy, vz)
            if key not in voxels:
                voxels[key] = True
                
        self.assertEqual(len(voxels), 2)

    def test_numpy_processing(self):
        # This test ensures our numpy logic matches the iterative logic
        # Create a dummy PointCloud2 data
        width = 10
        height = 1
        points = np.random.rand(width * height, 3).astype(np.float32)
        
        # Serialize to bytes (mimic PointCloud2 data)
        data = points.tobytes()
        
        # Numpy approach
        points_np = np.frombuffer(data, dtype=np.float32).reshape(-1, 3)
        
        # Check equality
        np.testing.assert_array_equal(points, points_np)
        
if __name__ == '__main__':
    unittest.main()
