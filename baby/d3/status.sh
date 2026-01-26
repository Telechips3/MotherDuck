cat > status.sh << 'SH'
#!/bin/sh
set -eu

SOCK_PATH="${1:-/tmp/waypoint.sock}"

LOG_LOGIC="/tmp/logic_latest.log"
LOG_COMM="/tmp/commd.log"
PID_LOGIC="/tmp/logic_latest.pid"
PID_COMM="/tmp/commd.pid"

check_pidfile () {
  name="$1"
  pidfile="$2"
  if [ -f "$pidfile" ]; then
    pid="$(cat "$pidfile" 2>/dev/null || true)"
    if [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null; then
      echo "[status] $name: RUNNING (pid=$pid)"
    else
      echo "[status] $name: NOT RUNNING (pidfile=$pidfile pid=$pid)"
    fi
  else
    echo "[status] $name: pidfile missing ($pidfile)"
  fi
}

echo "=== process ==="
check_pidfile "logic_latest" "$PID_LOGIC"
check_pidfile "commd" "$PID_COMM"

echo
echo "=== socket ==="
if [ -S "$SOCK_PATH" ]; then
  ls -l "$SOCK_PATH"
else
  echo "[status] socket not found: $SOCK_PATH"
fi

echo
echo "=== logs (last 10 lines) ==="
echo "--- $LOG_LOGIC ---"
tail -n 10 "$LOG_LOGIC" 2>/dev/null || echo "(no log)"
echo "--- $LOG_COMM ---"
tail -n 10 "$LOG_COMM" 2>/dev/null || echo "(no log)"
SH

chmod +x status.sh
