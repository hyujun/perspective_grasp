# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""Per-camera LifecycleNode fan-out factory.

Collapses ``cosypose``/``bundlesdf``/``megapose``/``sam2``/``foundationpose``
launch files (~140 lines of identical code each) down to a single call
parameterized by the four things that actually differ between them:
``(package, executable, name, topic_overrides)``.

Usage (inside an ``OpaqueFunction``)::

    from launch.substitutions import LaunchConfiguration
    from perception_launch_utils import fanout_lifecycle_nodes

    def _expand(context, *_a, **_kw):
        return fanout_lifecycle_nodes(
            package='sam2_instance_segmentor',
            executable='sam2_segmentor_node',
            name='sam2_segmentor',
            params_file=LaunchConfiguration('params_file').perform(context),
            camera_config_path=(
                LaunchConfiguration('camera_config').perform(context)
            ),
            topic_overrides=lambda ns: {
                'image_topic':      f'/{ns}/camera/color/image_raw',
                'detections_topic': f'/{ns}/yolo/detections',
                'masks_topic':      f'/{ns}/sam2/masks',
            },
            autostart=LaunchConfiguration('autostart'),
        )
"""

from __future__ import annotations

from typing import Any, Callable, Dict, List, Optional

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode

from .camera_config import load_config
from .lifecycle import autostart_lifecycle_actions


TopicOverrideFn = Callable[[str], Dict[str, Any]]


def _single_node(
    package: str,
    executable: str,
    name: str,
    params_file: str,
) -> LifecycleNode:
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace='',
        parameters=[params_file],
        output='screen',
    )


def _namespaced_node(
    package: str,
    executable: str,
    name: str,
    params_file: str,
    namespace: str,
    overrides: Dict[str, Any],
) -> LifecycleNode:
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=[params_file, overrides],
        output='screen',
    )


def fanout_lifecycle_nodes(
    *,
    package: str,
    executable: str,
    name: str,
    params_file: str,
    camera_config_path: str,
    topic_overrides: TopicOverrideFn,
    autostart: Optional[LaunchConfiguration] = None,
) -> List:
    """Build per-camera (or single-camera) LifecycleNodes with autostart wiring.

    Parameters
    ----------
    package, executable, name:
        The ROS 2 ``LifecycleNode`` fields, identical for every camera.
    params_file:
        Path to the node's YAML params (already resolved — pass the
        output of ``LaunchConfiguration('params_file').perform(ctx)``
        or ``config_path(pkg)``).
    camera_config_path:
        Path to a ``camera_config*.yaml``. Empty string → single node
        with global (non-namespaced) topics. Missing file → also falls
        back to single node (via ``load_config``'s 1-cam default).
    topic_overrides:
        Callable ``ns -> {param: value}`` that builds the per-camera
        topic remap dict. ``ns`` is the stripped namespace
        (e.g. ``"cam0"``). Not called for the single-node case.
    autostart:
        ``LaunchConfiguration('autostart')``. When ``None``, lifecycle
        autostart wiring is skipped and only the nodes are returned.

    Returns
    -------
    List of launch actions ready to extend a ``LaunchDescription``:
    the ``LifecycleNode`` instances plus their autostart event handlers.
    """
    cfg = load_config(camera_config_path if camera_config_path else None)

    nodes: List[LifecycleNode] = []
    for cam in cfg.cameras:
        ns = cam.ns_clean
        if not ns:
            nodes.append(_single_node(package, executable, name, params_file))
        else:
            nodes.append(_namespaced_node(
                package, executable, name, params_file,
                namespace=ns,
                overrides=topic_overrides(ns),
            ))

    actions: List = []
    for node in nodes:
        actions.append(node)
        if autostart is not None:
            actions.extend(autostart_lifecycle_actions(node, autostart))
    return actions
