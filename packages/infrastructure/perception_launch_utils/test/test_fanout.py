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

from perception_launch_utils import HostProfile, fanout_lifecycle_nodes


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


# ----- host_profile integration -------------------------------------------

def _node_parameters(node: LifecycleNode):
    """Pull the resolved ``parameters=[...]`` list off a LifecycleNode.

    launch_ros stores it under ``_Node__parameters`` (name-mangled
    private). This is purely a unit-test affordance — we're asserting
    the exact list the framework will hand to ROS at spawn time.
    """
    for attr in ('_Node__parameters', '_LifecycleNode__parameters'):
        if hasattr(node, attr):
            return getattr(node, attr)
    raise AssertionError(
        'Could not introspect LifecycleNode parameters; '
        'launch_ros may have changed its private layout.'
    )


def test_host_profile_overrides_appended_to_single_node():
    profile = HostProfile(
        profile_name='dev_8gb', description='', overrides={
            'fake_name': {'model_checkpoint': '/foo/small.pt'},
        }
    )
    actions = fanout_lifecycle_nodes(
        package='fake_pkg',
        executable='fake_exec',
        name='fake_name',
        params_file='/fake/params.yaml',
        camera_config_path='',
        topic_overrides=lambda ns: {},
        autostart=None,
        host_profile=profile,
    )
    nodes = [a for a in actions if isinstance(a, LifecycleNode)]
    assert len(nodes) == 1
    params = _node_parameters(nodes[0])
    # base YAML first, profile dict last
    assert params[0] == '/fake/params.yaml'
    assert {'model_checkpoint': '/foo/small.pt'} in params


def test_host_profile_overrides_appended_to_each_namespaced_node():
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
    profile = HostProfile(
        profile_name='dev_8gb', description='', overrides={
            'fake_name': {'bsz_images': 32},
        }
    )
    try:
        actions = fanout_lifecycle_nodes(
            package='fake_pkg',
            executable='fake_exec',
            name='fake_name',
            params_file='/fake/params.yaml',
            camera_config_path=path,
            topic_overrides=lambda ns: {'image_topic': f'/{ns}/img'},
            autostart=None,
            host_profile=profile,
        )
        nodes = [a for a in actions if isinstance(a, LifecycleNode)]
        assert len(nodes) == 2
        for node in nodes:
            params = _node_parameters(node)
            # base YAML, then per-camera topic remap, then profile last
            assert params[0] == '/fake/params.yaml'
            assert {'bsz_images': 32} == params[-1]
    finally:
        os.unlink(path)


def test_host_profile_without_match_for_node_is_noop():
    """A profile that mentions other nodes shouldn't append a stray
    empty-dict to this node's parameters."""
    profile = HostProfile(
        profile_name='dev_8gb', description='', overrides={
            'some_other_node': {'foo': 'bar'},
        }
    )
    actions = fanout_lifecycle_nodes(
        package='fake_pkg',
        executable='fake_exec',
        name='fake_name',
        params_file='/fake/params.yaml',
        camera_config_path='',
        topic_overrides=lambda ns: {},
        autostart=None,
        host_profile=profile,
    )
    node = next(a for a in actions if isinstance(a, LifecycleNode))
    params = _node_parameters(node)
    # exactly the base yaml — no trailing override dict
    assert params == ['/fake/params.yaml']


def test_no_host_profile_preserves_legacy_behavior():
    """Callers that don't pass host_profile should see exactly the same
    parameter list as before — single-node case is just [params_file]."""
    actions = fanout_lifecycle_nodes(
        package='fake_pkg',
        executable='fake_exec',
        name='fake_name',
        params_file='/fake/params.yaml',
        camera_config_path='',
        topic_overrides=lambda ns: {},
        autostart=None,
    )
    node = next(a for a in actions if isinstance(a, LifecycleNode))
    assert _node_parameters(node) == ['/fake/params.yaml']
