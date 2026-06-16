#!/usr/bin/env bash
# Downgrade NVIDIA 595 -> 580-open for Isaac Sim 5.1 GUI on Blackwell (RTX 50xx).
# Run: sudo bash downgrade_nvidia_for_isaac.sh && sudo reboot
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "[ERROR] Run with sudo:"
  echo "  sudo bash $(readlink -f "$0")"
  exit 1
fi

VER="580.159.04-1ubuntu1"

echo "[INFO] Current driver:"
nvidia-smi --query-gpu=driver_version,name --format=csv,noheader 2>/dev/null || true
echo "[INFO] Target: nvidia-driver-580-open ${VER} (DKMS)"

export DEBIAN_FRONTEND=noninteractive
apt-get update -qq

# CUDA repo 580.159.04 set — avoids conflict with ubuntu prebuilt kernel modules
apt-get install -y --allow-change-held-packages \
  nvidia-driver-580-open="${VER}" \
  nvidia-dkms-580-open="${VER}" \
  nvidia-kernel-common-580="${VER}" \
  nvidia-kernel-source-580-open="${VER}" \
  libnvidia-gl-580="${VER}" \
  libnvidia-compute-580="${VER}" \
  libnvidia-decode-580="${VER}" \
  libnvidia-encode-580="${VER}" \
  libnvidia-extra-580="${VER}" \
  libnvidia-fbc1-580="${VER}" \
  libnvidia-cfg1-580="${VER}" \
  nvidia-compute-utils-580="${VER}" \
  nvidia-utils-580="${VER}" \
  xserver-xorg-video-nvidia-580="${VER}"

# Remove 595 metapackage if still present
apt-get remove -y nvidia-driver-595-open 2>/dev/null || true
apt-get autoremove -y

echo ""
echo "[OK] Installed. Reboot required:"
echo "  sudo reboot"
echo ""
echo "After reboot, verify:"
echo "  nvidia-smi"
echo "  ~/sudo_ws/src/isaac_e0509_env/scripts/run_isaac_sim_gui.sh"
