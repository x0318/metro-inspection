"""Camera interfaces for the two existing simulation robots."""

import os


PROFILES = {
    'subway_v2': [
        {'id': 'xj1', 'label': '前向相机 XJ1', 'topic': '/subway_v2/xj1/image_raw/compressed'},
        {'id': 'xj2', 'label': '后向相机 XJ2', 'topic': '/subway_v2/xj2/image_raw/compressed'},
        {'id': 'xj3', 'label': '左侧相机 XJ3', 'topic': '/subway_v2/xj3/image_raw/compressed'},
        {'id': 'xj4', 'label': '右侧相机 XJ4', 'topic': '/subway_v2/xj4/image_raw/compressed'},
        {'id': 'pitch', 'label': '云台相机 Pitch', 'topic': '/subway_v2/pitch_camera/image_raw/compressed'},
    ],
    'route_training': [
        {'id': 'front', 'label': '前向相机', 'topic': '/zed2i_depth/image_raw/compressed'},
        {'id': 'left', 'label': '左前轨道相机', 'topic': '/gazebo_train/left_side/image_raw/compressed'},
        {'id': 'right', 'label': '右前轨道相机', 'topic': '/gazebo_train/right_side/image_raw/compressed'},
        {'id': 'ground', 'label': '轨道相机', 'topic': '/gazebo_train/ground/image_raw/compressed'},
    ],
}


def camera_config():
    profile = os.environ.get('PLATFORM_CAMERA_PROFILE', 'route_training')
    if profile not in PROFILES:
        raise ValueError(f'Unknown PLATFORM_CAMERA_PROFILE: {profile}')
    return [dict(camera) for camera in PROFILES[profile]]
