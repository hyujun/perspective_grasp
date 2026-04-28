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
from .host_profile import HostProfile, overrides_for_node
from .lifecycle import autostart_lifecycle_actions


TopicOverrideFn = Callable[[str], Dict[str, Any]]


def _single_node(
    package: str,
    executable: str,
    name: str,
    params_file: str,
    profile_overrides: Dict[str, Any],
) -> LifecycleNode:
    parameters: List[Any] = [params_file]
    if profile_overrides:
        parameters.append(profile_overrides)
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace='',
        parameters=parameters,
        output='screen',
    )


def _namespaced_node(
    package: str,
    executable: str,
    name: str,
    params_file: str,
    namespace: str,
    overrides: Dict[str, Any],
    profile_overrides: Dict[str, Any],
) -> LifecycleNode:
    # Profile overrides go LAST so they win on conflict with both the
    # base YAML and the per-camera topic remap. (Topic remaps name node
    # IO; profiles tune model size / batch — they shouldn't collide in
    # practice, but explicit ordering avoids surprises.)
    parameters: List[Any] = [params_file, overrides]
    if profile_overrides:
        parameters.append(profile_overrides)
    return LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        parameters=parameters,
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
    host_profile: Optional[HostProfile] = None,
) -> List:
    """Build per-camera (or single-camera) LifecycleNodes with autostart wiring.

    Parameters
    ----------
    package, executable, name:
        The ROS 2 ``LifecycleNode`` fields, identical for every camera.
        ``name`` doubles as the lookup key into ``host_profile.overrides``.
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
    host_profile:
        Optional :class:`HostProfile` from
        :func:`resolve_host_profile`. When provided, the slice for
        ``name`` is appended to each node's ``parameters=[...]`` so it
        overrides both the base YAML and any topic remap. ``None``
        (default) means "no profile overrides" — every spawned node
        runs purely from its packaged ``<pkg>_params.yaml``.

    Returns
    -------
    List of launch actions ready to extend a ``LaunchDescription``:
    the ``LifecycleNode`` instances plus their autostart event handlers.
    """
    cfg = load_config(camera_config_path if camera_config_path else None)
    profile_overrides = (
        overrides_for_node(host_profile, name) if host_profile else {}
    )

    nodes: List[LifecycleNode] = []
    for cam in cfg.cameras:
        ns = cam.ns_clean
        if not ns:
            nodes.append(_single_node(
                package, executable, name, params_file, profile_overrides))
        else:
            nodes.append(_namespaced_node(
                package, executable, name, params_file,
                namespace=ns,
                overrides=topic_overrides(ns),
                profile_overrides=profile_overrides,
            ))

    actions: List = []
    for node in nodes:
        actions.append(node)
        if autostart is not None:
            actions.extend(autostart_lifecycle_actions(node, autostart))
    return actions
