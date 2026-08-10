"""点云投影算法
 提取激光雷达XYZ
→ 转换到相机坐标系
→ 删除相机后方的点
→ 使用相机内参投影
→ 得到图像像素(u,v)
"""

import numpy as np
from sensor_msgs_py import point_cloud2


def pointcloud2_to_xyz(cloud_msg):
    """Convert x/y/z fields of PointCloud2 to a finite Nx3 float array."""
    points = point_cloud2.read_points_numpy(
        cloud_msg,
        field_names=["x", "y", "z"],
        skip_nans=True,
    )
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        return points
    return points[np.isfinite(points).all(axis=1)]


def project_camera_points(points_camera, camera_matrix, image_width, image_height):
    """Project optical-frame points and return visible xyz, uv and source indices."""
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        return points, np.empty((0, 2)), np.empty((0,), dtype=np.int64)

    fx = float(camera_matrix[0])
    fy = float(camera_matrix[4])
    cx = float(camera_matrix[2])
    cy = float(camera_matrix[5])
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError("CameraInfo contains invalid focal lengths")

    positive = points[:, 2] > 0.05
    indices = np.flatnonzero(positive)
    visible = points[positive]
    if visible.size == 0:
        return visible, np.empty((0, 2)), indices

    uv = np.column_stack(
        (
            fx * visible[:, 0] / visible[:, 2] + cx,
            fy * visible[:, 1] / visible[:, 2] + cy,
        )
    )
    in_image = (
        (uv[:, 0] >= 0.0)
        & (uv[:, 0] < float(image_width))
        & (uv[:, 1] >= 0.0)
        & (uv[:, 1] < float(image_height))
    )
    return visible[in_image], uv[in_image], indices[in_image]


def select_bbox_points(points_camera, uv, bbox, roi_scale=0.75):
    """Select projected 3D points from a centered, shrunken bbox region."""
    center_x = float(bbox.center.position.x)
    center_y = float(bbox.center.position.y)
    half_width = max(1.0, 0.5 * float(bbox.size_x) * float(roi_scale))
    half_height = max(1.0, 0.5 * float(bbox.size_y) * float(roi_scale))
    inside = (
        (uv[:, 0] >= center_x - half_width)
        & (uv[:, 0] <= center_x + half_width)
        & (uv[:, 1] >= center_y - half_height)
        & (uv[:, 1] <= center_y + half_height)
    )
    return points_camera[inside], uv[inside]

#鲁棒深度估计Median + MAD异常值剔除
def robust_median_point(points_xyz, max_depth_deviation=0.08):
    """Estimate surface position after rejecting points far from median depth."""
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        raise ValueError("Cannot estimate a point from an empty set")

    median_depth = float(np.median(points[:, 2]))
    absolute_deviation = np.abs(points[:, 2] - median_depth)
    mad = float(np.median(absolute_deviation))
    threshold = max(float(max_depth_deviation), 3.0 * 1.4826 * mad)
    inliers = points[absolute_deviation <= threshold]
    if inliers.size == 0:
        inliers = points
    return np.median(inliers, axis=0), inliers

def dbscan_median_point(points_xyz, eps=0.15, min_samples=3):
    """Estimate point with DBSCAN spatial clustering followed by XYZ median.

    The input is the candidate point set after bbox/mask projection. DBSCAN is
    used to keep the dominant dense 3D cluster and reject isolated points or
    small background clusters. The final coordinate is the XYZ median of that
    dominant cluster. This implementation avoids an extra sklearn dependency.
    """
    points = np.asarray(points_xyz, dtype=np.float64).reshape(-1, 3)
    if points.size == 0:
        raise ValueError("Cannot estimate a point from an empty set")

    point_count = len(points)
    eps = max(float(eps), 1e-6)
    min_samples = max(1, int(min_samples))
    if point_count < min_samples:
        return np.median(points, axis=0), points

    eps2 = eps * eps
    diff = points[:, None, :] - points[None, :, :]
    distances2 = np.einsum("ijk,ijk->ij", diff, diff)
    neighbors = [np.flatnonzero(distances2[i] <= eps2) for i in range(point_count)]

    unvisited = -99
    noise = -1
    labels = np.full(point_count, unvisited, dtype=np.int32)
    cluster_id = 0

    for start_idx in range(point_count):
        if labels[start_idx] != unvisited:
            continue

        start_neighbors = neighbors[start_idx]
        if len(start_neighbors) < min_samples:
            labels[start_idx] = noise
            continue

        labels[start_idx] = cluster_id
        queue = list(int(idx) for idx in start_neighbors if idx != start_idx)
        queue_pos = 0
        while queue_pos < len(queue):
            idx = queue[queue_pos]
            queue_pos += 1

            if labels[idx] == noise:
                labels[idx] = cluster_id
            if labels[idx] != unvisited:
                continue

            labels[idx] = cluster_id
            idx_neighbors = neighbors[idx]
            if len(idx_neighbors) >= min_samples:
                for neighbor_idx in idx_neighbors:
                    neighbor_idx = int(neighbor_idx)
                    if labels[neighbor_idx] in (unvisited, noise):
                        queue.append(neighbor_idx)

        cluster_id += 1

    valid_cluster_ids = sorted(set(int(label) for label in labels if label >= 0))
    if not valid_cluster_ids:
        return np.median(points, axis=0), points

    all_median_depth = float(np.median(points[:, 2]))
    best_cluster_points = None
    best_score = None
    for cid in valid_cluster_ids:
        cluster_points = points[labels == cid]
        cluster_median = np.median(cluster_points, axis=0)
        # Prefer the largest cluster; if tied, prefer the one whose depth is
        # closest to the overall median depth.
        score = (len(cluster_points), -abs(float(cluster_median[2]) - all_median_depth))
        if best_score is None or score > best_score:
            best_score = score
            best_cluster_points = cluster_points

    return np.median(best_cluster_points, axis=0), best_cluster_points

def select_mask_points(points_camera, uv, mask_image, bbox=None, roi_scale=1.0, min_value=1):
    """Select projected 3D points whose image projection falls inside a binary mask.

    mask_image must be in image pixel coordinates. If bbox is provided, the mask
    selection is additionally constrained to the bbox region so that unrelated
    mask pixels cannot pull points from other image areas.
    """
    points = np.asarray(points_camera, dtype=np.float64).reshape(-1, 3)
    uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
    mask = np.asarray(mask_image)
    if points.size == 0 or uv.size == 0 or mask.size == 0:
        return points[:0], uv[:0]
    if mask.ndim == 3:
        mask = mask[..., 0]

    height, width = mask.shape[:2]
    u = np.rint(uv[:, 0]).astype(np.int64)
    v = np.rint(uv[:, 1]).astype(np.int64)
    inside_image = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    inside = inside_image.copy()
    if bbox is not None:
        center_x = float(bbox.center.position.x)
        center_y = float(bbox.center.position.y)
        half_width = max(1.0, 0.5 * float(bbox.size_x) * float(roi_scale))
        half_height = max(1.0, 0.5 * float(bbox.size_y) * float(roi_scale))
        inside &= (
            (uv[:, 0] >= center_x - half_width)
            & (uv[:, 0] <= center_x + half_width)
            & (uv[:, 1] >= center_y - half_height)
            & (uv[:, 1] <= center_y + half_height)
        )

    valid_indices = np.flatnonzero(inside)
    if valid_indices.size == 0:
        return points[:0], uv[:0]

    selected = mask[v[valid_indices], u[valid_indices]] >= int(min_value)
    valid_indices = valid_indices[selected]
    return points[valid_indices], uv[valid_indices]
