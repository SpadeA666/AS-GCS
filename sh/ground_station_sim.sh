#!/usr/bin/env bash
set -euo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
trap 'kill 0 2>/dev/null || true' INT TERM EXIT

echo "[ground-station] starting local gateway simulator"
/usr/bin/python3 "$WS/scripts/ground_station_sim.py" &
gateway_pid=$!
cleanup_gateway() { kill "$gateway_pid" 2>/dev/null || true; }
trap 'cleanup_gateway' INT TERM EXIT
sleep 0.5
echo "[ground-station] starting UI at http://127.0.0.1:5173"
cd "$WS/ground_station_ui"
npm run dev
kill "$gateway_pid" 2>/dev/null || true
