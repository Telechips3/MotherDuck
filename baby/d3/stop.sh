cat > stop.sh << 'SH'
#!/bin/sh
set -eu

SOCK_PATH="${1:-/tmp/waypoint.sock}"

LOG_LOGIC="/tmp/logic_latest.log"
LOG_COMM="/tmp/commd.log"
PID_LOGIC="/tmp/logic_latest.pid"
PID_COMM="/tmp/commd.pid"

stop_pidfile () {
  name="$1"
  pidfile="$2"
  if [ -f "$pidfile" ]; then
    pid="$(cat "$pidfile" 2>/dev/null || true)"
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "[stop] killing $name pid=$pid"
      kill "$pid" 2>/dev/null || true
      # 조금 기다렸다가 안 죽으면 강제
      sleep 0.05
      kill -9 "$pid" 2>/dev/null || true
    else
      echo "[stop] $name pidfile exists but process not running (pid=$pid)"
    fi
    rm -f "$pidfile"
  else
    echo "[stop] $name pidfile not found: $pidfile"
  fi
}

stop_pidfile "commd" "$PID_COMM"
stop_pidfile "logic_latest" "$PID_LOGIC"

rm -f "$SOCK_PATH"
echo "[stop] removed socket: $SOCK_PATH"

echo "[stop] keep logs by default:"
echo "  $LOG_LOGIC"
echo "  $LOG_COMM"
echo "If you want to delete logs:"
echo "  rm -f $LOG_LOGIC $LOG_COMM"
SH

chmod +x stop.sh
