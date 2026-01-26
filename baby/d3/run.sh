cat > run.sh << 'SH'
#!/bin/sh
set -eu

UDP_PORT="${1:-5005}"
SOCK_PATH="${2:-/tmp/waypoint.sock}"

LOG_LOGIC="/tmp/logic_latest.log"
LOG_COMM="/tmp/commd.log"
PID_LOGIC="/tmp/logic_latest.pid"
PID_COMM="/tmp/commd.pid"

# cleanup stale
rm -f "$SOCK_PATH"
rm -f "$LOG_LOGIC" "$LOG_COMM"

echo "[run] starting logic_latest..."
stdbuf -oL -eL nohup ./logic_latest "$SOCK_PATH" > "$LOG_LOGIC" 2>&1 &
echo $! > "$PID_LOGIC"

# logic가 소켓 바인드할 시간 아주 조금 줌
sleep 0.05

echo "[run] starting commd (udp:$UDP_PORT -> uds:$SOCK_PATH)..."
stdbuf -oL -eL nohup ./commd "$UDP_PORT" "$SOCK_PATH" > "$LOG_COMM" 2>&1 &
echo $! > "$PID_COMM"

echo "[run] started."
echo "  logic pid: $(cat "$PID_LOGIC")  log: $LOG_LOGIC"
echo "  comm  pid: $(cat "$PID_COMM")   log: $LOG_COMM"
echo "  sock: $SOCK_PATH"

echo "[run] tail logs:"
tail -n 5 "$LOG_LOGIC" 2>/dev/null || true
tail -n 5 "$LOG_COMM"  2>/dev/null || true
SH

chmod +x run.sh
