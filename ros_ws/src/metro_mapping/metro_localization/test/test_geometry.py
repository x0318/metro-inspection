from types import SimpleNamespace

import numpy as np
from geometry_msgs.msg import TransformStamped

from metro_localization.cloud_projector import (
    project_camera_points,
    robust_median_point,
    select_bbox_points,
)
from metro_localization.geometry_utils import transform_points


def test_transform_points_applies_translation():
    transform = TransformStamped()
    transform.transform.rotation.w = 1.0
    transform.transform.translation.x = 1.0
    transform.transform.translation.y = -2.0
    transform.transform.translation.z = 0.5

    result = transform_points(np.array([[2.0, 3.0, 4.0]]), transform)
    np.testing.assert_allclose(result, [[3.0, 1.0, 4.5]])


def test_project_camera_points_filters_behind_and_outside_image():
    points = np.array(
        [
            [0.0, 0.0, 2.0],
            [2.0, 0.0, 2.0],
            [0.0, 0.0, -1.0],
            [10.0, 0.0, 1.0],
        ]
    )
    camera_matrix = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]

    visible, uv, _ = project_camera_points(points, camera_matrix, 120, 80)

    np.testing.assert_allclose(visible, [[0.0, 0.0, 2.0]])
    np.testing.assert_allclose(uv, [[50.0, 40.0]])


def test_project_camera_points_applies_camera_skew():
    points = np.array([[1.0, 2.0, 4.0]])
    camera_matrix = [100.0, 2.0, 50.0, 0.0, 120.0, 40.0, 0.0, 0.0, 1.0]

    visible, uv, _ = project_camera_points(points, camera_matrix, 200, 140)

    np.testing.assert_allclose(visible, points)
    np.testing.assert_allclose(uv, [[76.0, 100.0]])


def test_bbox_selection_and_robust_depth_rejection():
    bbox = SimpleNamespace(
        center=SimpleNamespace(position=SimpleNamespace(x=50.0, y=40.0)),
        size_x=20.0,
        size_y=20.0,
    )
    xyz = np.array(
        [
            [0.00, 0.00, 3.00],
            [0.02, 0.01, 3.01],
            [-0.01, 0.02, 2.99],
            [0.00, 0.00, 8.00],
            [1.00, 1.00, 3.00],
        ]
    )
    uv = np.array(
        [
            [50.0, 40.0],
            [51.0, 41.0],
            [49.0, 42.0],
            [52.0, 39.0],
            [100.0, 70.0],
        ]
    )

    selected, _ = select_bbox_points(xyz, uv, bbox, roi_scale=1.0)
    estimate, inliers = robust_median_point(selected, max_depth_deviation=0.08)

    assert len(selected) == 4
    assert len(inliers) == 3
    np.testing.assert_allclose(estimate, [0.0, 0.01, 3.0], atol=1e-9)
