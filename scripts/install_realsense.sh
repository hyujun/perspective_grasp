#!/bin/bash
# =============================================================================
# install_realsense.sh - Intel RealSense SDK 2.0 on Ubuntu 24.04
#
# Installs the librealsense2 apt packages needed to run RealSense cameras as
# RGB-D inputs for the perspective_grasp pipeline.
#
# Installs:
#   - Build/runtime deps (libusb, libudev, GLFW, GTK, OpenGL)
#   - Intel librealsense apt repo signing key + source
#   - librealsense2-udev-rules : userspace USB access permissions
#   - librealsense2-utils      : realsense-viewer + demo tools
#   - librealsense2-dev        : headers + libs for C++ consumers
#   - /etc/modules-load.d/uvcvideo.conf : ensure uvcvideo is loaded at boot
#
# Skips (intentionally):
#   - librealsense2-dkms : the out-of-tree V4L2 kernel module. It ships
#     unsigned, so on Secure Boot machines (our target) the kernel refuses
#     to load it, leaving userland with no driver. The stock uvcvideo driver
#     in noble's 6.x kernel already exposes RealSense streams and is signed
#     by the distro, so skipping DKMS is both safer and more reliable on
#     newer kernels. This script purges any leftover DKMS install from
#     earlier attempts.
#   - librealsense2-dbg : debug symbols (pull manually if you need them).
#
# Usage:
#   chmod +x scripts/install_realsense.sh
#   ./scripts/install_realsense.sh
#
# Notes:
#   - Run as a regular user; sudo is invoked per command.
#   - The script is idempotent — re-running after a partial install is safe.
#   - If you later hit UVC metadata / hardware-timestamp limitations, build
#     librealsense from source instead per
#     https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md
# =============================================================================
set -euo pipefail

echo "============================================================"
echo " perspective_grasp - Intel RealSense SDK 2.0 Setup"
echo " Target: Ubuntu 24.04 (noble), kernel 6.x, Secure Boot OK"
echo "============================================================"

# ---- 1. System update + build deps ----
echo ""
echo "=== [1/7] System update + build dependencies ==="
sudo apt-get update
sudo apt-get install -y \
    curl gnupg ca-certificates lsb-release software-properties-common \
    git pkg-config \
    libssl-dev libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

# ---- 2. Intel repo signing key ----
echo ""
echo "=== [2/7] Intel librealsense signing key ==="
# The .pgp file served at librealsense.intel.com does not reliably contain the
# key that actually signs the noble Release file (apt reports NO_PUBKEY
# FB0B24895113F120). Pull the key by ID from the Ubuntu keyserver instead,
# then install the resulting binary keyring into /etc/apt/keyrings/ where
# signed-by= can find it.
#
# Run gpg as the regular user in a throwaway --homedir: `sudo gpg` fails here
# because root has no ~/.gnupg and dirmngr won't launch, and polluting the
# user's real ~/.gnupg with a one-shot repo key is noisy.
REALSENSE_KEY_ID="FB0B24895113F120"
REALSENSE_KEYRING="/etc/apt/keyrings/librealsense.pgp"
sudo mkdir -p /etc/apt/keyrings
GPG_TMP="$(mktemp -d)"
trap 'rm -rf "$GPG_TMP"' EXIT
gpg --homedir "$GPG_TMP" \
    --no-default-keyring \
    --keyring "$GPG_TMP/keyring.gpg" \
    --keyserver hkp://keyserver.ubuntu.com:80 \
    --recv-keys "$REALSENSE_KEY_ID"
sudo install -m 0644 "$GPG_TMP/keyring.gpg" "$REALSENSE_KEYRING"
echo "Installed key $REALSENSE_KEY_ID to $REALSENSE_KEYRING"

# ---- 3. Add apt source ----
echo ""
echo "=== [3/7] Add librealsense apt source ==="
CODENAME="$(lsb_release -cs)"
REPO_LINE="deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo ${CODENAME} main"
echo "$REPO_LINE" | sudo tee /etc/apt/sources.list.d/librealsense.list > /dev/null
sudo apt-get update

# ---- 4. Install SDK packages ----
echo ""
echo "=== [4/7] Install librealsense2 packages ==="
# Purge any DKMS module from earlier runs/manual installs. The out-of-tree
# module tends to fail to build on noble's 6.x kernel, and — more importantly
# for our target machines — it ships unsigned, so Secure Boot rejects it at
# load time. The stock uvcvideo driver already exposes RealSense streams, so
# running without DKMS is the correct default. Keep udev rules — they're what
# actually grants non-root USB access.
if dpkg -l librealsense2-dkms 2>/dev/null | grep -q '^ii'; then
    echo "Purging existing librealsense2-dkms..."
    sudo apt-get purge -y librealsense2-dkms
fi
# Catch a manually-registered DKMS source tree too (dkms install without apt).
if command -v dkms >/dev/null 2>&1; then
    while read -r lrs_ver; do
        [ -z "$lrs_ver" ] && continue
        echo "Removing stray DKMS module: librealsense2/$lrs_ver"
        sudo dkms remove "librealsense2/$lrs_ver" --all || true
    done < <(dkms status 2>/dev/null | awk -F'[,/ :]+' '/^librealsense2/{print $2}' | sort -u)
fi

sudo apt-get install -y \
    librealsense2-udev-rules \
    librealsense2-utils \
    librealsense2-dev

# ---- 5. Reload udev rules ----
echo ""
echo "=== [5/7] Reload udev rules ==="
# librealsense2-udev-rules ships 99-realsense-libusb.rules under
# /lib/udev/rules.d/. Reload + retrigger so an already-plugged camera picks
# up the new permissions without a replug.
sudo udevadm control --reload-rules
sudo udevadm trigger

# ---- 6. Ensure uvcvideo autoloads at boot ----
echo ""
echo "=== [6/7] Ensure uvcvideo autoloads at boot ==="
# Observed failure: after purging librealsense2-dkms on this system, plugging
# in a D435i did NOT cause the kernel to autoload uvcvideo. /dev/video* was
# missing and rs-enumerate-devices returned "No device detected". Running
# `sudo modprobe uvcvideo` manually fixed it every time. The stock uvcvideo
# module is properly signed (Build time autogenerated kernel key), so Secure
# Boot is not the blocker — it's an autoload miss. Force-load explicitly at
# boot via modules-load.d so the fix survives reboots.
UVCVIDEO_CONF="/etc/modules-load.d/uvcvideo.conf"
if [ -f "$UVCVIDEO_CONF" ] && grep -qx 'uvcvideo' "$UVCVIDEO_CONF"; then
    echo "$UVCVIDEO_CONF already contains 'uvcvideo' — leaving as-is."
else
    echo "uvcvideo" | sudo tee -a "$UVCVIDEO_CONF" > /dev/null
    echo "Appended 'uvcvideo' to $UVCVIDEO_CONF"
fi

# Load now so the current session doesn't need a reboot. modprobe is a no-op
# if the module is already in the kernel.
sudo modprobe uvcvideo

if lsmod | grep -q '^uvcvideo'; then
    echo "uvcvideo module is loaded."
else
    echo "WARNING: uvcvideo failed to load after modprobe."
    echo "         Check 'sudo dmesg | grep -i uvcvideo | tail -n 20' for clues."
fi

# ---- 7. Post-install sanity check ----
echo ""
echo "=== [7/7] Post-install sanity check ==="
# Informational only — never fail the script from here, the user may not
# have a camera plugged in yet.
echo ""
echo "--- kernel module (expect: uvcvideo present) ---"
lsmod | grep uvcvideo || echo "(uvcvideo not loaded)"

echo ""
echo "--- video device nodes (empty is OK if no camera is plugged in) ---"
if compgen -G '/dev/video*' > /dev/null; then
    ls -l /dev/video*
else
    echo "(no /dev/video* nodes present)"
fi

echo ""
echo "--- rs-enumerate-devices -s ---"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
    if rs-enumerate-devices -s 2>/dev/null; then
        echo "rs-enumerate-devices: OK"
    else
        echo "rs-enumerate-devices: no device detected (OK if camera is unplugged)"
    fi
else
    echo "rs-enumerate-devices not on PATH — librealsense2-utils install may have failed."
fi

echo ""
echo "============================================================"
echo " RealSense SDK install complete."
echo ""
echo " Next steps:"
echo "   1. Plug a RealSense camera into a USB 3.x port."
echo "   2. Run 'realsense-viewer' to confirm depth + color streams."
echo ""
echo " Using the stock uvcvideo kernel driver (no DKMS, Secure Boot safe)."
echo " If you later need the out-of-tree driver for UVC-metadata features,"
echo " build librealsense from source:"
echo "   https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md"
echo "============================================================"
