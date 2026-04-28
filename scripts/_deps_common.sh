#!/bin/bash
# =============================================================================
# _deps_common.sh - Shared installation helpers.
#
# Sourced by install_host.sh (fresh Ubuntu path) and install_dependencies.sh
# (ROS 2 already present path). Each function is idempotent — safe to re-run.
#
# Functions defined here:
#   install_ros_apt_deps     ROS 2 Jazzy pkgs + system libs (apt only)
#   install_teaserpp         TEASER++ from source (HAS_TEASERPP gate)
#   install_manif            manif headers (pose_filter_cpp dep)
#   install_gtsam            GTSAM 4.2.0 from source (borglab PPA has no noble)
#   install_host_python      workspace venv + scripts/requirements-host.txt
#
# Callers are responsible for `set -euo pipefail` and any shell safety flags.
# This file intentionally does not set them so sourcing is side-effect-light.
# =============================================================================

_COMMON_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_venv.sh
source "$_COMMON_SCRIPT_DIR/_venv.sh"

install_ros_apt_deps() {
    echo ""
    echo "=== ROS 2 Jazzy packages + system libraries ==="
    # Mirrors the union of <depend>/<exec_depend>/<test_depend> declared across
    # packages/*/*/package.xml. rosdep is not invoked from this script, so this
    # apt list is the SSOT for what gets installed — keep it in sync with the
    # package.xml dep keys when adding or removing a node.
    #
    # Naming note: rosdep keys differ from apt names in two places:
    #   - package.xml `libpcl-all-dev` (rosdep key) → apt `libpcl-dev`
    #   - package.xml `eigen` (rosdep key)          → apt `libeigen3-dev`
    # The apt names below are what dpkg ships on Ubuntu 24.04 noble.
    #
    # rmw_cyclonedds_cpp is NOT a package.xml dep (no node imports it directly)
    # but the runtime forces RMW_IMPLEMENTATION=rmw_cyclonedds_cpp via .env.live
    # to lockstep with the Phase 4 Docker containers (anti-pattern (m) in
    # CLAUDE.md). ros-jazzy-desktop ships Fast DDS only — without the cyclone
    # RMW package every `ros2` command on the host fails with "RMW
    # implementation 'rmw_cyclonedds_cpp' is not installed."
    #
    # universe must be enabled (libpcl-dev / libceres-dev / libopenmpi-dev /
    # nvidia-driver-* live there). install_host.sh enables it before this
    # function runs; install_dependencies.sh assumes the host already has it.
    sudo apt update
    sudo apt install -y \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip \
        build-essential \
        cmake \
        git \
        ros-jazzy-rclcpp \
        ros-jazzy-rclcpp-lifecycle \
        ros-jazzy-rclpy \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-std-msgs \
        ros-jazzy-sensor-msgs \
        ros-jazzy-sensor-msgs-py \
        ros-jazzy-geometry-msgs \
        ros-jazzy-action-msgs \
        ros-jazzy-lifecycle-msgs \
        ros-jazzy-diagnostic-msgs \
        ros-jazzy-std-srvs \
        ros-jazzy-tf2 \
        ros-jazzy-tf2-ros \
        ros-jazzy-tf2-eigen \
        ros-jazzy-tf2-sensor-msgs \
        ros-jazzy-tf2-geometry-msgs \
        ros-jazzy-cv-bridge \
        ros-jazzy-image-transport \
        ros-jazzy-pcl-conversions \
        ros-jazzy-message-filters \
        ros-jazzy-launch \
        ros-jazzy-launch-ros \
        ros-jazzy-rqt-image-view \
        ros-jazzy-rviz2 \
        ros-jazzy-ament-cmake-gtest \
        ros-jazzy-ament-cmake-python \
        ros-jazzy-ament-index-python \
        ros-jazzy-rosidl-default-generators \
        python3-yaml \
        python3-numpy \
        python3-pytest \
        libpcl-dev \
        libeigen3-dev \
        libopencv-dev \
        libopenmpi-dev \
        libceres-dev
}

install_teaserpp() {
    echo ""
    echo "=== TEASER++ (from source) ==="
    if ldconfig -p | grep -q teaserpp; then
        echo "TEASER++ already installed."
        return 0
    fi
    local src=/tmp/TEASER-plusplus
    [ ! -d "$src" ] && git clone --depth 1 https://github.com/MIT-SPARK/TEASER-plusplus.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DBUILD_TESTS=OFF \
                 -DBUILD_PYTHON_BINDINGS=OFF \
                 -DBUILD_DOC=OFF
        make -j"$(nproc)"
        sudo make install && sudo ldconfig
    )
}

install_manif() {
    echo ""
    echo "=== manif (header-only, from source) ==="
    if [ -f /usr/local/include/manif/manif.h ]; then
        echo "manif already installed."
        return 0
    fi
    local src=/tmp/manif
    [ ! -d "$src" ] && git clone --depth 1 https://github.com/artivis/manif.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
        sudo make install
    )
}

install_gtsam() {
    echo ""
    echo "=== GTSAM 4.2.0 (from source) ==="
    # borglab PPA does not publish for Ubuntu 24.04 (noble), so we source-build
    # on every host. Check for either the installed header or the shared lib.
    if [ -f /usr/local/include/gtsam/base/Matrix.h ] || ldconfig -p | grep -q libgtsam.so; then
        echo "GTSAM already installed."
        return 0
    fi
    local src=/tmp/gtsam
    [ ! -d "$src" ] && git clone --depth 1 --branch 4.2.0 https://github.com/borglab/gtsam.git "$src"
    (
        cd "$src" && mkdir -p build && cd build
        cmake .. -DCMAKE_BUILD_TYPE=Release \
                 -DGTSAM_BUILD_TESTS=OFF \
                 -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
                 -DGTSAM_BUILD_UNSTABLE=ON \
                 -DGTSAM_USE_SYSTEM_EIGEN=ON \
                 -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
        make -j"$(nproc)"
        sudo make install && sudo ldconfig
    )
}

install_host_python() {
    echo ""
    echo "=== Host Python deps (workspace venv) ==="
    ensure_venv
    # Pinned via scripts/requirements-host.txt. numpy<2 keeps the venv aligned
    # with apt's numpy 1.26.4 (cv_bridge ABI). Skip opencv-python: use apt's
    # python3-opencv / cv_bridge's bundled cv2.
    #
    # --no-user is load-bearing: with --system-site-packages and PEP 668 on
    # noble, pip can opportunistically route transitive deps into ~/.local
    # instead of the venv (memory: pip_venv_shadow_trap.md). Forcing --no-user
    # keeps every install inside <ws>/.venv where uninstall semantics match.
    #
    # Caveat: --no-user only fixes future installs. Pre-existing pollution
    # under ~/.local/lib/python3.12/site-packages still shadows the venv via
    # --system-site-packages, so a host with prior `pip install --user`
    # history may need manual cleanup (look for unexpected versions of
    # numpy/torch in `python -c 'import X; print(X.__file__)'` and rm by
    # hand). Fresh PCs are unaffected.
    #
    # PERSPECTIVE_TORCH_CUDA picks which torch wheel index to use. Map:
    #   cu126            matches Phase 4 Docker baseline; works on driver 560+
    #   cu128            for driver 570+/CUDA 12.8 hosts (e.g. RTX 40-series boxes)
    #   cu130            for driver 580+/CUDA 13.x hosts
    #   cpu              for headless CI / no GPU
    # When unset, _detect_torch_cuda probes nvidia-smi and picks the matching
    # cuXXX (or cpu) automatically. Explicit env var still wins — it's the
    # escape hatch for "I know what I want" cases (e.g. forcing cpu for CI).
    # CLAUDE.md anti-pattern (p) is the bug this exists to prevent.
    local torch_cuda
    if [ -n "${PERSPECTIVE_TORCH_CUDA:-}" ]; then
        torch_cuda="$PERSPECTIVE_TORCH_CUDA"
        echo ">>> PERSPECTIVE_TORCH_CUDA='$torch_cuda' (explicit override)"
    else
        torch_cuda="$(_detect_torch_cuda)"
        echo ">>> PERSPECTIVE_TORCH_CUDA unset — auto-detected '$torch_cuda' from nvidia-smi"
    fi
    case "$torch_cuda" in
        cu126|cu128|cu130|cpu) ;;
        *)
            echo "ERROR: PERSPECTIVE_TORCH_CUDA='$torch_cuda' not supported." >&2
            echo "       Use one of: cu126, cu128, cu130, cpu." >&2
            return 1
            ;;
    esac
    local torch_index="https://download.pytorch.org/whl/${torch_cuda}"
    echo ">>> torch wheel index: $torch_index"
    pip install --no-user --upgrade \
        --extra-index-url "$torch_index" \
        -r "$_COMMON_SCRIPT_DIR/requirements-host.txt"
    pip uninstall -y opencv-python >/dev/null 2>&1 || true

    _verify_torch_install "$torch_cuda"
}

# Pick a cuXXX wheel index by parsing `nvidia-smi`'s "CUDA Version:" line
# (the runtime CUDA the deployed driver advertises). Prints the chosen tag
# (cu126 / cu128 / cu130 / cpu) on stdout — never fails: if nvidia-smi is
# missing or unparseable, we fall back to cu126 (the Phase 4 Docker baseline,
# which works on the broadest range of supported drivers). Diagnostics go to
# stderr so the stdout capture stays clean.
#
# Mapping rationale (matches _verify_torch_install's WARNING text and
# docs/installation.md): the CUDA Version reported by nvidia-smi is the max
# runtime the driver supports, so a wheel built for that exact version (or
# lower) will work. We pick the highest cuXXX <= reported.
_detect_torch_cuda() {
    local nvsmi
    nvsmi="$(command -v nvidia-smi || true)"
    if [ -z "$nvsmi" ]; then
        echo "nvidia-smi not found — assuming no GPU, picking 'cpu'." >&2
        echo "cpu"
        return 0
    fi
    # `nvidia-smi` (no flags) prints a header containing e.g.
    #   "| NVIDIA-SMI 570.86.15  Driver Version: 570.86.15  CUDA Version: 12.8 |"
    # The query-gpu form does not expose CUDA Version, so we parse the header.
    local raw cuda_ver
    raw="$("$nvsmi" 2>/dev/null | grep -oE 'CUDA Version: *[0-9]+\.[0-9]+' | head -n1 || true)"
    cuda_ver="${raw##*: }"
    if [ -z "$cuda_ver" ]; then
        echo "nvidia-smi present but 'CUDA Version' line not found — falling back to cu126." >&2
        echo "cu126"
        return 0
    fi
    echo "nvidia-smi reports CUDA Version: $cuda_ver" >&2
    # Compare as "major*100 + minor" so 12.10 > 12.8 sorts correctly.
    local major minor score
    major="${cuda_ver%%.*}"
    minor="${cuda_ver##*.}"
    score=$(( 10#$major * 100 + 10#$minor ))
    if   [ "$score" -ge 1300 ]; then echo "cu130"
    elif [ "$score" -ge 1208 ]; then echo "cu128"
    elif [ "$score" -ge 1206 ]; then echo "cu126"
    else
        echo "Driver-advertised CUDA $cuda_ver is below cu126 — picking 'cpu'." >&2
        echo "cpu"
    fi
}

# Probe the freshly-installed torch and warn if its CUDA build cannot
# actually drive the host's NVIDIA driver. Non-fatal — emits a clear
# diagnostic and exits 0 so the install script keeps going (the runtime
# helper resolve_torch_device() will still fall back to CPU at node
# startup; this just surfaces the misconfiguration earlier).
_verify_torch_install() {
    local picked="$1"
    if ! python -c 'import torch' 2>/dev/null; then
        echo "ERROR: torch failed to import after install" >&2
        return 1
    fi
    python - "$picked" <<'PYEOF'
import shutil, subprocess, sys
picked = sys.argv[1]
import torch
print(f">>> torch={torch.__version__}  torch.version.cuda={torch.version.cuda}")
if picked == 'cpu':
    print(">>> CPU build requested; no GPU probe.")
    sys.exit(0)
ok = torch.cuda.is_available()
print(f">>> torch.cuda.is_available()={ok}")
if ok:
    try:
        print(f">>>   device 0: {torch.cuda.get_device_name(0)}")
    except Exception as exc:
        print(f">>>   device 0 name unavailable: {exc}")
# Probe the deployed driver if nvidia-smi is reachable.
nvsmi = shutil.which('nvidia-smi')
if nvsmi:
    try:
        out = subprocess.check_output(
            [nvsmi, '--query-gpu=driver_version', '--format=csv,noheader'],
            text=True, timeout=4,
        ).strip().splitlines()[0]
        print(f">>> nvidia-smi driver={out}")
    except Exception as exc:
        print(f">>> nvidia-smi probe failed: {exc}")
if not ok:
    print(
        "WARNING: torch.cuda.is_available() is False after install.\n"
        "         Likely PERSPECTIVE_TORCH_CUDA was set higher than the\n"
        "         deployed driver supports. Re-run install with the value\n"
        "         that matches `nvidia-smi`'s CUDA Version (12.6→cu126,\n"
        "         12.8→cu128, 13.x→cu130, no GPU→cpu)."
    )
PYEOF
}
