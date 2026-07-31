#!/usr/bin/env bash
# Standalone Jetson host service installer (no git clone required).
#
# Copy to the Jetson and run:
#   scp Jetson/jetson-host-install.sh user@jetson:~/
#   ssh user@jetson 'sudo bash ~/jetson-host-install.sh'
#
# Optional: download host_service.py from GitHub instead of the embedded copy:
#   sudo ROBOT_HOST_SERVICE_RAW_URL='https://raw.githubusercontent.com/satomm1/Robot/main/Jetson/host_service.py' \
#     bash ~/jetson-host-install.sh
#
# After install: curl http://localhost:8081/status

set -euo pipefail

INSTALL_DIR="/opt/robot"
SERVICE_NAME="robot-host-service.service"
HOST_SERVICE_PY="${INSTALL_DIR}/host_service.py"

# Override to wget host_service.py (must be raw URL to the .py file)
HOST_SERVICE_RAW_URL="${ROBOT_HOST_SERVICE_RAW_URL:-}"

if [[ "${EUID:-$(id -u)}" -ne 0 ]]; then
  echo "Run as root: sudo bash $0" >&2
  exit 1
fi

if ! command -v python3 >/dev/null 2>&1; then
  echo "python3 is required." >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "Warning: docker not found in PATH. Install Docker before using /docker-start." >&2
fi

# Home directory for volume paths in ROBOT_DOCKER_RUN_CMD (~ does not expand in systemd env)
if [[ -n "${SUDO_USER:-}" ]] && [[ "${SUDO_USER}" != "root" ]]; then
  JETSON_HOME="$(getent passwd "${SUDO_USER}" | cut -d: -f6)"
else
  JETSON_HOME="${HOME}"
fi
JETSON_HOME="${JETSON_HOME:-/root}"

mkdir -p "${INSTALL_DIR}"

substitute_jetson_home_in_py() {
  sed -i "s|__JETSON_HOME__|${JETSON_HOME}|g" "${HOST_SERVICE_PY}"
}

install_host_service_py() {
  if [[ -n "${HOST_SERVICE_RAW_URL}" ]]; then
    echo "Downloading host_service.py from ${HOST_SERVICE_RAW_URL}"
    if command -v wget >/dev/null 2>&1; then
      wget -q -O "${HOST_SERVICE_PY}.tmp" "${HOST_SERVICE_RAW_URL}"
    elif command -v curl >/dev/null 2>&1; then
      curl -fsSL -o "${HOST_SERVICE_PY}.tmp" "${HOST_SERVICE_RAW_URL}"
    else
      echo "wget or curl required for ROBOT_HOST_SERVICE_RAW_URL" >&2
      exit 1
    fi
    mv "${HOST_SERVICE_PY}.tmp" "${HOST_SERVICE_PY}"
    chmod 0755 "${HOST_SERVICE_PY}"
    substitute_jetson_home_in_py
    return
  fi

  echo "Installing embedded host_service.py to ${HOST_SERVICE_PY}"
  cat > "${HOST_SERVICE_PY}" << 'HOST_SERVICE_PY_EOF'
#!/usr/bin/env python3
"""HTTP host service on port 8081. Runs on the Jetson base machine (not inside Docker).

Endpoints: GET /status, /docker-start, /docker-stop, /poweroff, /map; POST /map
Deploy on Jetson: scp Jetson/jetson-host-install.sh and sudo bash ~/jetson-host-install.sh
Keep in sync with the embedded copy in jetson-host-install.sh.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
import subprocess
import time
from urllib.parse import urlparse

# Default map path; __JETSON_HOME__ replaced by jetson-host-install.sh (same as DOCKER_RUN_CMD).
DEFAULT_MAP_JSON_PATH = (
    "__JETSON_HOME__/workspaces/catkin_ws/src/mattbot_mcl/map_json/current_map_mod.json"
)


def _map_json_path():
    override = os.environ.get("ROBOT_MAP_JSON_PATH", "").strip()
    if override:
        return override
    return DEFAULT_MAP_JSON_PATH


HOST_SERVICE_PORT = 8081
DOCKER_CONTAINER = "ros_noetic"
GEMINI_CONTAINER = "gemini"
DOCKER_STOP_TIMEOUT_SEC = 30
CONTAINER_START_WAIT_SEC = 5

# __JETSON_HOME__ is replaced with the install user's home when you run jetson-host-install.sh.
DOCKER_RUN_CMD = (
    "docker run -d --runtime nvidia --network=host "
    "-v __JETSON_HOME__/workspaces/catkin_ws:/workspace/catkin_ws "
    "-v __JETSON_HOME__/gemini_api:/gemini_code "
    "-v /dev/bus/usb:/dev/bus/usb "
    "-v /dev/video0:/dev/video0 -v /dev/video1:/dev/video1 "
    "--device=/dev/ttyUSB0 --device=/dev/spidev0.0 "
    "--rm --privileged --name ros_noetic ghcr.io/satomm1/ml_ros:latest "
    "bash -lc 'python3 /workspace/catkin_ws/src/startup_script.py & exec tail -f /dev/null'"
)

GEMINI_ENV_FILE = "__JETSON_HOME__/gemini_api/.env"

GEMINI_DOCKER_RUN_CMD = (
    "docker run -d --network=host "
    f"--env-file {GEMINI_ENV_FILE} "
    "-v __JETSON_HOME__/gemini_api:/gemini_code "
    "-w /gemini_code --rm --privileged --name gemini ghcr.io/satomm1/gemini:latest "
    "bash -lc '. start_scripts.sh & exec tail -f /dev/null'"
)

HOST_POWEROFF_CMD = (
    "nohup bash -c '/usr/sbin/shutdown -h now' </dev/null >/dev/null 2>&1 &"
)


def _run_cmd(cmd, timeout=None, shell=False):
    try:
        proc = subprocess.run(
            cmd,
            shell=shell,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
        return proc.returncode == 0, (proc.stdout or "").strip(), (proc.stderr or "").strip()
    except subprocess.TimeoutExpired:
        return False, "", f"Command timed out after {timeout}s"
    except OSError as exc:
        return False, "", str(exc)


def _container_running(name):
    ok, stdout, _ = _run_cmd(
        ["docker", "inspect", "-f", "{{.State.Running}}", name],
        timeout=10,
    )
    return ok and stdout.lower() == "true"


def _wait_container_running(name, timeout_sec=CONTAINER_START_WAIT_SEC):
    polls = max(1, int(timeout_sec / 0.5))
    for _ in range(polls):
        if _container_running(name):
            return True
        time.sleep(0.5)
    return False


def _list_running_container_ids():
    ok, stdout, stderr = _run_cmd(["docker", "ps", "-q"], timeout=10)
    if not ok:
        return [], stderr or "docker ps failed"
    ids = [line.strip() for line in stdout.splitlines() if line.strip()]
    return ids, ""


def _stop_container(container_id, timeout_sec):
    ok, _, stderr = _run_cmd(
        ["docker", "stop", "-t", str(timeout_sec), container_id],
        timeout=timeout_sec + 5,
    )
    if ok:
        return True, ""
    kill_ok, _, kill_err = _run_cmd(["docker", "kill", container_id], timeout=15)
    if kill_ok:
        return True, f"kill fallback for {container_id}"
    return False, stderr or kill_err or f"failed to stop {container_id}"


def _stop_all_containers():
    ids, err = _list_running_container_ids()
    if err:
        return False, [], [{"error": err}]

    if not ids:
        return True, [], []

    stopped = []
    errors = []
    for cid in ids:
        ok, note = _stop_container(cid, DOCKER_STOP_TIMEOUT_SEC)
        if ok:
            stopped.append({"id": cid, "note": note or "stopped"})
        else:
            errors.append({"id": cid, "error": note})

    return len(errors) == 0, stopped, errors


def _poweroff_summary():
    all_ok, stopped, errors = _stop_all_containers()
    if stopped:
        message = f"Stopped {len(stopped)} container(s); host shutdown scheduled."
    elif not errors:
        message = "No containers running; host shutdown scheduled."
    elif not all_ok:
        message = "Some containers could not be stopped; host shutdown scheduled anyway."
    else:
        message = "Host shutdown scheduled."

    return {
        "ok": True,
        "message": message,
        "all_stopped": all_ok,
        "containers_stopped": stopped,
        "errors": errors,
    }


def _gemini_env_ready():
    """Gemini is started detached; API_KEY must come from an env file, not host .bashrc."""
    if not os.path.isfile(GEMINI_ENV_FILE):
        return (
            False,
            f"Gemini env file not found: {GEMINI_ENV_FILE}. "
            "Create it with a line like API_KEY=your-key",
        )
    try:
        with open(GEMINI_ENV_FILE, encoding="utf-8") as fh:
            for line in fh:
                stripped = line.strip()
                if not stripped or stripped.startswith("#"):
                    continue
                if stripped.startswith("export "):
                    stripped = stripped[len("export ") :]
                if stripped.startswith("API_KEY=") and stripped != "API_KEY=":
                    return True, ""
    except OSError as exc:
        return False, f"Cannot read {GEMINI_ENV_FILE}: {exc}"
    return False, f"{GEMINI_ENV_FILE} must contain a non-empty API_KEY= line."


def _start_container(name, run_cmd):
    if name == GEMINI_CONTAINER:
        ok, message = _gemini_env_ready()
        if not ok:
            return False, message

    if _container_running(name):
        return True, f"Container {name} is already running."

    if not run_cmd:
        return False, f"Run command for {name} is not configured."

    ok, stdout, stderr = _run_cmd(run_cmd, shell=True, timeout=120)
    if not ok:
        detail = stderr or stdout or "docker run failed"
        return False, f"{name}: {detail}"

    if _wait_container_running(name):
        return True, f"Container {name} started."

    log_hint = ""
    cid = (stdout or "").strip().splitlines()[-1] if stdout else ""
    if cid:
        _, log_out, _ = _run_cmd(["docker", "logs", cid], timeout=10)
        if log_out:
            log_hint = f" Last logs: {log_out[-500:]}"

    return (
        False,
        f"Container {name} exited right after start.{log_hint} "
        "Detached runs need a long-lived main process (use 'tail -f /dev/null' not 'exec bash'). "
        "Check: docker ps -a; journalctl -u robot-host-service",
    )


def _docker_start():
    # Start Gemini before ROS so the API is up when robot nodes launch.
    start_steps = (
        (GEMINI_CONTAINER, GEMINI_DOCKER_RUN_CMD),
        (DOCKER_CONTAINER, DOCKER_RUN_CMD),
    )
    messages = []
    all_ok = True
    for name, run_cmd in start_steps:
        ok, message = _start_container(name, run_cmd)
        messages.append(message)
        if not ok:
            all_ok = False
            break

    return all_ok, " ".join(messages)


def _docker_stop():
    # Stop ROS before Gemini.
    stop_order = (DOCKER_CONTAINER, GEMINI_CONTAINER)
    messages = []
    all_ok = True
    for name in stop_order:
        if not _container_running(name):
            messages.append(f"Container {name} is not running.")
            continue
        ok, note = _stop_container(name, DOCKER_STOP_TIMEOUT_SEC)
        if ok:
            messages.append(f"Container {name} stopped.")
        else:
            all_ok = False
            messages.append(note or f"Failed to stop {name}.")

    return all_ok, " ".join(messages)


def _container_status():
    ros_running = _container_running(DOCKER_CONTAINER)
    gemini_running = _container_running(GEMINI_CONTAINER)
    return {
        "host_service": True,
        "docker_running": ros_running and gemini_running,
        "container": DOCKER_CONTAINER,
        "ros_noetic_running": ros_running,
        "gemini_running": gemini_running,
    }


def _schedule_host_poweroff():
    subprocess.Popen(
        ["bash", "-lc", HOST_POWEROFF_CMD],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def _read_map_json():
    path = _map_json_path()
    if not os.path.isfile(path):
        return None, path, None
    try:
        with open(path, "r", encoding="utf-8") as f:
            return f.read(), path, None
    except OSError as exc:
        return None, path, str(exc)


def _map_json_dir():
    return os.path.dirname(_map_json_path())


def _named_map_path(map_name):
    return os.path.join(_map_json_dir(), f"{map_name}.json")


def _map_name_from_payload(payload):
    name = payload.get("name")
    if not isinstance(name, str) or not name.strip():
        return None, 'missing or empty top-level "name" field'
    if "/" in name or "\\" in name or "\0" in name or name in (".", ".."):
        return None, "invalid map name"
    return name, None


def _write_map_json(raw_text):
    try:
        payload = json.loads(raw_text)
    except json.JSONDecodeError as exc:
        return False, {"ok": False, "error": f"invalid JSON: {exc}"}, 400

    map_name, name_err = _map_name_from_payload(payload)
    if name_err:
        return False, {"ok": False, "error": name_err}, 400

    current_path = os.path.abspath(_map_json_path())
    named_path = os.path.abspath(_named_map_path(map_name))
    same_file = named_path == current_path

    map_dir = _map_json_dir()
    try:
        os.makedirs(map_dir, exist_ok=True)
    except OSError as exc:
        return False, {"ok": False, "error": f"cannot create map directory: {exc}"}, 500

    paths_to_write = [current_path]
    if not same_file:
        paths_to_write.append(named_path)

    written = []
    try:
        for path in paths_to_write:
            with open(path, "w", encoding="utf-8") as f:
                f.write(raw_text)
            written.append(path)
    except OSError as exc:
        for path in written:
            if path != current_path:
                try:
                    os.remove(path)
                except OSError:
                    pass
        return False, {"ok": False, "error": f"map write failed: {exc}"}, 500

    result = {
        "ok": True,
        "name": map_name,
        "current_path": current_path,
    }
    if not same_file:
        result["named_path"] = named_path
    return True, result, 200


class HostServiceHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        if args and str(args[0]).startswith("GET "):
            return
        super().log_message(format, *args)

    def _send_json(self, payload, status=200):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_raw_json(self, raw_text, status=200):
        body = raw_text.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, message, status=200):
        body = message.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        parsed = urlparse(self.path)
        pathname = parsed.path

        if pathname == "/status":
            self._send_json(_container_status())
            return

        if pathname == "/docker-start":
            ok, message = _docker_start()
            self._send_text(message, 200 if ok else 500)
            return

        if pathname == "/docker-stop":
            ok, message = _docker_stop()
            self._send_text(message, 200 if ok else 500)
            return

        if pathname == "/poweroff":
            summary = _poweroff_summary()
            _schedule_host_poweroff()
            self._send_json(summary, 200)
            self.wfile.flush()
            return

        if pathname == "/map":
            raw, path, err = _read_map_json()
            if raw is None and err is None:
                self._send_json(
                    {"ok": False, "error": "map not found", "path": path},
                    status=404,
                )
                return
            if raw is None:
                self._send_json(
                    {"ok": False, "error": f"map read failed: {err}", "path": path},
                    status=500,
                )
                return
            self._send_raw_json(raw, status=200)
            return

        self._send_text("Invalid request.", 404)

    def do_POST(self):
        parsed = urlparse(self.path)
        pathname = parsed.path

        if pathname == "/map":
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0:
                self._send_json({"ok": False, "error": "empty request body"}, status=400)
                return
            try:
                raw = self.rfile.read(length).decode("utf-8")
            except (OSError, UnicodeDecodeError) as exc:
                self._send_json(
                    {"ok": False, "error": f"cannot read request body: {exc}"},
                    status=400,
                )
                return

            ok, payload, status = _write_map_json(raw)
            self._send_json(payload, status=status)
            return

        self._send_text("Invalid request.", 404)


if __name__ == "__main__":
    server = HTTPServer(("0.0.0.0", HOST_SERVICE_PORT), HostServiceHandler)
    print(
        f"Robot host service listening on port {HOST_SERVICE_PORT} "
        "(GET /status, /docker-start, /docker-stop, /poweroff, /map; POST /map)..."
    )
    server.serve_forever()
HOST_SERVICE_PY_EOF
  chmod 0755 "${HOST_SERVICE_PY}"
  substitute_jetson_home_in_py
}

install_systemd_unit() {
  cat > "/etc/systemd/system/${SERVICE_NAME}" << 'SYSTEMD_UNIT_EOF'
[Unit]
Description=Robot host service (Docker control and power off on port 8081)
After=network-online.target docker.service
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/python3 /opt/robot/host_service.py
Restart=on-failure
RestartSec=5
User=root

[Install]
WantedBy=multi-user.target
SYSTEMD_UNIT_EOF
  chmod 0644 "/etc/systemd/system/${SERVICE_NAME}"
}

install_host_service_py
install_systemd_unit

systemctl daemon-reload
systemctl enable "${SERVICE_NAME}"
systemctl restart "${SERVICE_NAME}"

echo ""
echo "Installed ${SERVICE_NAME}."
echo "  Status:  systemctl status ${SERVICE_NAME}"
echo "  Logs:    journalctl -u ${SERVICE_NAME} -f"
echo "  Paths:   Jetson home baked in as ${JETSON_HOME} (edit /opt/robot/host_service.py to change)"
echo "  Test:    curl -s http://127.0.0.1:8081/status"
echo ""
systemctl status "${SERVICE_NAME}" --no-pager || true
