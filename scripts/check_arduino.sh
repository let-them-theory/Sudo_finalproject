#!/usr/bin/env bash
# 아두이노 HC-SR04 시리얼 연결 점검
set -euo pipefail

echo "=== Arduino check ==="
echo "groups: $(groups)"
echo ""

if ls /dev/serial/by-id/usb-Arduino* >/dev/null 2>&1; then
  ls -la /dev/serial/by-id/usb-Arduino*
else
  echo "[WARN] /dev/serial/by-id/usb-Arduino* 없음"
fi

PORT="${1:-auto}"
if [[ "$PORT" == "auto" ]]; then
  if ls /dev/serial/by-id/usb-Arduino* >/dev/null 2>&1; then
    PORT=$(readlink -f /dev/serial/by-id/usb-Arduino*)
  elif [[ -e /dev/ttyACM0 ]]; then
    PORT=/dev/ttyACM0
  else
    echo "[FAIL] 아두이노 포트 없음. USB 케이블·전원 확인"
    exit 1
  fi
fi

echo "port: $PORT"
BAUD="${2:-9600}"
echo "baud: $BAUD"
echo ""

python3 - <<PY
import time, serial
port = "$PORT"
baud = int("$BAUD")
ser = serial.Serial(port, baud, timeout=2)
time.sleep(1.5)
ok = False
for i in range(8):
    line = ser.readline().decode("utf-8", errors="ignore").strip()
    if line:
        print(f"  [{i}] {line!r}")
    if "mm" in line.lower() or line.startswith("DIST:"):
        ok = True
ser.close()
if ok:
    print("[OK] 아두이노 거리 데이터 수신됨")
else:
    print("[FAIL] 거리 포맷 미수신 (DIstance:NNNmm 또는 DIST:NN)")
    raise SystemExit(1)
PY

echo ""
echo "ROS 토픽 확인 (스택 실행 중일 때):"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash 2>/dev/null || true
source "$(cd "$(dirname "$0")/.." && pwd)/install/setup.bash" 2>/dev/null || true
if ros2 node list 2>/dev/null | grep -q ultrasonic_node; then
  timeout 4 ros2 topic echo /ultrasonic_range --once && echo "[OK] /ultrasonic_range"
else
  echo "[INFO] ultrasonic_node 미실행 — op 후 다시 확인"
fi
