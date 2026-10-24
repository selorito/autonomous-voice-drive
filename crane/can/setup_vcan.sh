#!/usr/bin/env bash
# Sanal CAN hatti (vcan0) kurar. Bilgisayar yeniden baslayinca tekrar calistir.
set -e
sudo modprobe vcan
if ! ip link show vcan0 >/dev/null 2>&1; then
  sudo ip link add dev vcan0 type vcan
fi
sudo ip link set up vcan0
ip -details link show vcan0 | head -3
echo "vcan0 hazir. Canli izleme:  candump vcan0 | python3 -m cantools decode crane/can/crane.dbc"
