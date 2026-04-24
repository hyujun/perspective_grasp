# Copyright 2026 perspective_grasp
# SPDX-License-Identifier: Apache-2.0
"""LifecycleNode autostart wiring.

Extracted from the five Phase 4 launch files (cosypose, bundlesdf,
megapose, sam2, foundationpose) which each re-implemented the same
``UNCONFIGURED → INACTIVE → ACTIVE`` sequence.
"""

from __future__ import annotations

from typing import List

import lifecycle_msgs.msg
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState


def autostart_lifecycle_actions(
    node: LifecycleNode,
    autostart: LaunchConfiguration,
) -> List:
    """Drive ``node`` UNCONFIGURED → INACTIVE → ACTIVE when autostart is true.

    The configure event is emitted once the node is up; the activate
    event is chained off the ``configuring → inactive`` transition so we
    don't race the node's lifecycle service becoming available.
    """
    cond = IfCondition(autostart)
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        ),
        condition=cond,
    )
    activate_on_inactive = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=(
                        lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
                    ),
                )),
            ],
        ),
        condition=cond,
    )
    return [activate_on_inactive, configure]
