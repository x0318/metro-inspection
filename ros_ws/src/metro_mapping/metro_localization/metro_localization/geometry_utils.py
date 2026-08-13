#坐标转换 LiDAR:lidar_frame->Camera:camera_frame->世界:odom
import numpy as np


def quaternion_to_rotation_matrix(quaternion):
    """Return a 3x3 rotation matrix for a geometry_msgs Quaternion."""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3, dtype=np.float64)
    scale = 2.0 / norm
    xx, yy, zz = x * x * scale, y * y * scale, z * z * scale
    xy, xz, yz = x * y * scale, x * z * scale, y * z * scale
    wx, wy, wz = w * x * scale, w * y * scale, w * z * scale
    return np.array(
        [
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ],
        dtype=np.float64,
    )


def transform_points(points_xyz, transform_stamped):
    """Apply target<-source TransformStamped to an Nx3 point array."""
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    transform = transform_stamped.transform
    rotation = quaternion_to_rotation_matrix(transform.rotation)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z],
        dtype=np.float64,
    )
    return points @ rotation.T + translation


def transform_point(point_xyz, transform_stamped):
    return transform_points(np.asarray(point_xyz).reshape(1, 3), transform_stamped)[0]