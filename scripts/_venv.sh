#!/bin/bash
# =============================================================================
# _venv.sh - Shared helper: create + activate workspace Python venv
#
# Sourced by install_host.sh, install_dependencies.sh, and build.sh.
# Creates `.venv` next to build/install/log at the colcon workspace root,
# with --system-site-packages so ROS 2 Python bindings (rclpy, cv_bridge,
# numpy from apt, etc.) stay visible.
#
# After `ensure_venv`:
#   - $VENV_DIR points at <workspace>/.venv
#   - venv is active ($VIRTUAL_ENV set, `python`/`pip` point into it)
#   - pip installs go into the venv, NOT into ~/.local or system site-packages
#
# Why system-site-packages: rclpy/cv_bridge are shipped as apt packages
# under /opt/ros/jazzy + /usr/lib/python3/dist-packages. Isolating them would
# require pip-installing ROS bindings (not supported upstream), so we inherit.
# =============================================================================

# Resolve the colcon workspace root (three levels above scripts/).
# scripts/ -> perspective_grasp/ -> src/ -> <ws>
_VENV_HELPER_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$_VENV_HELPER_DIR/../../.." && pwd)"
VENV_DIR="$WS_DIR/.venv"
export WS_DIR VENV_DIR

ensure_venv() {
    local created=0
    if [ ! -f "$VENV_DIR/bin/activate" ]; then
        echo ">>> Creating Python venv at $VENV_DIR (--system-site-packages)"
        if ! python3 -c 'import venv' >/dev/null 2>&1; then
            echo ">>> Installing python3-venv (required to create the venv)"
            sudo apt update
            sudo apt install -y python3-venv python3-full
        fi
        python3 -m venv --system-site-packages "$VENV_DIR"
        created=1
    fi
    # shellcheck disable=SC1091
    source "$VENV_DIR/bin/activate"
    # Only upgrade pip on first creation. Avoid bumping setuptools/wheel since
    # apt colcon pins setuptools<80 and the venv inherits system site-packages.
    if [ "$created" = "1" ]; then
        python -m pip install --quiet --upgrade pip
    fi
    echo ">>> venv active: $VIRTUAL_ENV"
}

activate_venv_if_exists() {
    if [ -f "$VENV_DIR/bin/activate" ]; then
        # shellcheck disable=SC1091
        source "$VENV_DIR/bin/activate"
        return 0
    fi
    return 1
}
