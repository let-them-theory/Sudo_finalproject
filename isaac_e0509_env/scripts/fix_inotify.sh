#!/usr/bin/env bash
# Isaac Sim needs many file watches. Default 65536 is too low → errno=28 + segfault.
# Run once: sudo bash fix_inotify.sh
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  echo "[ERROR] root 권한 필요. 아래처럼 실행하세요:"
  echo "  sudo bash $(readlink -f "$0")"
  exit 1
fi

CONF="/etc/sysctl.d/99-isaac-inotify.conf"
cat > "${CONF}" <<'EOF'
# Isaac Sim / Omniverse Kit file watchers
fs.inotify.max_user_watches=524288
fs.inotify.max_user_instances=512
fs.inotify.max_queued_events=524288
EOF

sysctl --system >/dev/null
echo "[OK] inotify limits applied:"
echo "  max_user_watches=$(cat /proc/sys/fs/inotify/max_user_watches)"
echo "  max_user_instances=$(cat /proc/sys/fs/inotify/max_user_instances)"
echo "  max_queued_events=$(cat /proc/sys/fs/inotify/max_queued_events)"
