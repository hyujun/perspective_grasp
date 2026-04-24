# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for perception_launch_utils.fanout.

Doesn't spin up rclcpp — just asserts the right count and type of
launch actions come back, and that topic_overrides is called with the
expected namespace strings.
"""

import os
import tempfile
import textwrap

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

from perception_launch_utils import fanout_lifecycle_nodes


def _write_yaml(text: str) -> str:
    fd, path = tempfile.mkstemp(suffix='.yaml')
    os.write(fd, textwrap.dedent(text).encode())
    os.close(fd)
    return path


def _overrides(namespaces_seen: list):
    def fn(ns: str):
        namespaces_seen.append(ns)
        return {'image_topic': f'/{ns}/camera/color/image_raw'}
    return fn


def test_empty_camera_config_spawns_single_node():
    seen = []
    actions = fanout_lifecycle_nodes(
        package='fake_pkg',
        executable='fake_exec',
        name='fake_name',
        params_file='/fake/params.yaml',
        camera_config_path='',
        topic_overrides=_overrides(seen),
        autostart=None,
    )
    lifecycle_nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert len(lifecycle_nodes) == 1
    assert seen == []  # not called for single-node


def test_two_camera_yaml_spawns_two_nodes_and_calls_overrides():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            cameras:
              - id: 0
                namespace: /cam0
                type: eye_in_hand
                frame_id: cam0_color_optical_frame
              - id: 1
                namespace: /cam1
                type: eye_to_hand
                frame_id: cam1_color_optical_frame
    ''')
    try:
        seen = []
        actions = fanout_lifecycle_nodes(
            package='fake_pkg',
            executable='fake_exec',
            name='fake_name',
            params_file='/fake/params.yaml',
            camera_config_path=path,
            topic_overrides=_overrides(seen),
            autostart=None,
        )
        lifecycle_nodes = [a for a in actions if isinstance(a, LifecycleNode)]
        assert len(lifecycle_nodes) == 2
        assert seen == ['cam0', 'cam1']
    finally:
        os.unlink(path)


def test_autostart_adds_event_actions_per_node():
    path = _write_yaml('''
        perception_system:
          ros__parameters:
            cameras:
              - id: 0
                namespace: /cam0
                type: eye_in_hand
                frame_id: cam0_color_optical_frame
    ''')
    try:
        actions = fanout_lifecycle_nodes(
            package='fake_pkg',
            executable='fake_exec',
            name='fake_name',
            params_file='/fake/params.yaml',
            camera_config_path=path,
            topic_overrides=lambda ns: {},
            autostart=LaunchConfiguration('autostart'),
        )
        # 1 LifecycleNode + 2 autostart actions (register_handler + emit).
        assert len(actions) == 3
        assert sum(isinstance(a, LifecycleNode) for a in actions) == 1
    finally:
        os.unlink(path)


def test_missing_camera_config_falls_back_to_single_node():
    actions = fanout_lifecycle_nodes(
        package='fake_pkg',
        executable='fake_exec',
        name='fake_name',
        params_file='/fake/params.yaml',
        camera_config_path='/nonexistent/camera_config.yaml',
        topic_overrides=lambda ns: {},
        autostart=None,
    )
    lifecycle_nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert len(lifecycle_nodes) == 1
