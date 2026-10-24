#!/usr/bin/env bash
# Vinc demosunu TEMIZ baslatir: eski surecleri kapatir, vcan0'i hazirlar, Gazebo + kontrolcu + CAN koprusunu acar.
# Kullanim (proje klasorunden):   ./crane/start_demo.sh            (kamerasiz: ./crane/start_demo.sh vision:=false)
# Sonra ikinci terminalde:        python3 crane/voice_crane.py
set -e
cd "$(dirname "$0")/.."

echo "[1/4] Eski surecler kapatiliyor..."
killall -9 gzserver gzclient 2>/dev/null || true
for p in crane_controller.py vision_node.py can_bridge.py voice_crane.py motion_node.py voice_node.py; do
  pkill -9 -f "$p" 2>/dev/null || true
done
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

echo "[2/4] CAN hatti (vcan0)..."
if ! ip link show vcan0 >/dev/null 2>&1; then
  ./crane/can/setup_vcan.sh
else
  echo "      vcan0 zaten acik"
fi

echo "[3/4] Kopya node kontrolu..."
if ros2 node list 2>/dev/null | grep -qE "crane_controller|can_bridge"; then
  echo "      UYARI: hala calisan crane node'u var:"; ros2 node list | grep -E "crane_controller|can_bridge"
fi

echo "[4/4] Gazebo + vinc kontrolcusu + CAN koprusu baslatiliyor..."
echo "      Hazir olunca ikinci terminalde: python3 crane/voice_crane.py"
exec ros2 launch crane/launch/crane_can.launch.py "$@"
