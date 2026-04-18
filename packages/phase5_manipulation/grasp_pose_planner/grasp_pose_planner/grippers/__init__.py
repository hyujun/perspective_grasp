"""Robot-specific gripper adapters.

Planners stay robot-agnostic; concrete end-effectors (parallel jaw,
multi-finger hand, etc.) implement :class:`BaseGripper` to expose the
surface the planners need.
"""
