#!/bin/bash
# Robot-side Wi-Fi IP update: rewrite robot_env.sh (ROS_IP) and cyclonedds.xml
# <Peers>, then source robot_env.sh when this script is itself sourced.
#
# Preferred (completes docs steps 6–7 in the current shell):
#   source ./update_wifi_ips.sh
#
# Also works as a subprocess (files only; you must source robot_env.sh yourself):
#   ./update_wifi_ips.sh

# Detect sourced vs executed before set -e (set -e would kill an interactive shell).
_IS_SOURCED=0
if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
  _IS_SOURCED=1
fi

if ((_IS_SOURCED == 0)); then
  set -euo pipefail
else
  set -uo pipefail
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROBOT_ENV_SH="${SCRIPT_DIR}/robot_env.sh"
BACKUP_ENV="${SCRIPT_DIR}/robot_env.sh.bak"
CYCLONEDDS_XML="${SCRIPT_DIR}/cyclonedds.xml"
BACKUP_XML="${SCRIPT_DIR}/cyclonedds.xml.bak"

THIS_ROBOT_IP=""
CENTRAL_IP=""
OTHER_ROBOT_IPS=()

_script_exit() {
  local code="${1:-0}"
  if ((_IS_SOURCED)); then
    return "$code"
  else
    exit "$code"
  fi
}

is_ipv4() {
  local ip="$1"
  [[ "$ip" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]] || return 1
  local IFS=.
  # shellcheck disable=SC2086
  set -- $ip
  for octet in "$@"; do
    ((octet >= 0 && octet <= 255)) || return 1
  done
  return 0
}

detect_wlan0_ip() {
  local ip=""
  if command -v ip >/dev/null 2>&1; then
    ip="$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1 || true)"
  fi
  printf '%s' "$ip"
}

prompt_this_robot_ip() {
  local hint detected ip
  detected="$(detect_wlan0_ip)"
  if [[ -n "$detected" ]]; then
    hint=" [detected wlan0: ${detected}]"
  else
    hint=""
  fi
  while true; do
    read -r -p "This robot IP address${hint}: " ip
    ip="${ip//[[:space:]]/}"
    if [[ -z "$ip" && -n "$detected" ]]; then
      ip="$detected"
    fi
    if [[ -z "$ip" ]]; then
      echo "Error: this robot IP is required." >&2
      continue
    fi
    if ! is_ipv4 "$ip"; then
      echo "Error: '$ip' is not a valid IPv4 address." >&2
      continue
    fi
    THIS_ROBOT_IP="$ip"
    return 0
  done
}

prompt_central_ip() {
  local ip
  while true; do
    read -r -p "Central machine IP address: " ip
    ip="${ip//[[:space:]]/}"
    if [[ -z "$ip" ]]; then
      echo "Error: central machine IP is required." >&2
      continue
    fi
    if ! is_ipv4 "$ip"; then
      echo "Error: '$ip' is not a valid IPv4 address." >&2
      continue
    fi
    if [[ "$ip" == "$THIS_ROBOT_IP" ]]; then
      echo "Error: '$ip' is already this robot's IP. Try again." >&2
      continue
    fi
    CENTRAL_IP="$ip"
    return 0
  done
}

prompt_other_robot_ips() {
  OTHER_ROBOT_IPS=()
  echo "Enter other robot IP addresses (one per line). Press Enter on an empty line when done."
  echo "(You may leave this empty for a single-robot setup.)"
  while true; do
    local ip dup existing
    read -r -p "Other robot IP: " ip
    ip="${ip//[[:space:]]/}"
    if [[ -z "$ip" ]]; then
      break
    fi
    if ! is_ipv4 "$ip"; then
      echo "Error: '$ip' is not a valid IPv4 address. Try again." >&2
      continue
    fi
    if [[ "$ip" == "$CENTRAL_IP" ]]; then
      echo "Error: '$ip' is already the central machine IP. Try again." >&2
      continue
    fi
    if [[ "$ip" == "$THIS_ROBOT_IP" ]]; then
      echo "Error: '$ip' is already this robot's IP. Try again." >&2
      continue
    fi
    dup=0
    for existing in "${OTHER_ROBOT_IPS[@]+"${OTHER_ROBOT_IPS[@]}"}"; do
      if [[ "$existing" == "$ip" ]]; then
        echo "Error: '$ip' was already entered. Try again." >&2
        dup=1
        break
      fi
    done
    if ((dup)); then
      continue
    fi
    OTHER_ROBOT_IPS+=("$ip")
  done
}

confirm_changes() {
  local ip reply
  echo
  echo "Planned changes:"
  echo "  ROS_IP (robot_env.sh): ${THIS_ROBOT_IP}"
  echo "  Peer list (cyclonedds.xml):"
  echo "    - ${CENTRAL_IP}  (central machine)"
  echo "    - ${THIS_ROBOT_IP}  (this robot)"
  for ip in "${OTHER_ROBOT_IPS[@]+"${OTHER_ROBOT_IPS[@]}"}"; do
    echo "    - ${ip}"
  done
  echo
  read -r -p "Apply changes? [y/N] " reply
  case "$reply" in
    y|Y|yes|YES) return 0 ;;
    *)
      echo "Aborted; no files were modified."
      return 1
      ;;
  esac
}

rewrite_robot_env() {
  if [[ ! -f "$ROBOT_ENV_SH" ]]; then
    echo "Error: missing ${ROBOT_ENV_SH}" >&2
    return 1
  fi

  if ! cp "$ROBOT_ENV_SH" "$BACKUP_ENV"; then
    echo "Error: could not write backup ${BACKUP_ENV} (try sudo?)." >&2
    return 1
  fi

  if ! ROBOT_IP="$THIS_ROBOT_IP" ROBOT_ENV_SH="$ROBOT_ENV_SH" python3 - <<'PY'
import os
import re
import sys

path = os.environ["ROBOT_ENV_SH"]
robot_ip = os.environ["ROBOT_IP"]

with open(path, "r", encoding="utf-8") as f:
    text = f.read()

updated, count = re.subn(
    r"^export ROS_IP=.*$",
    f"export ROS_IP={robot_ip}",
    text,
    count=1,
    flags=re.MULTILINE,
)
if count != 1:
    print("Error: could not find a single 'export ROS_IP=...' line in robot_env.sh", file=sys.stderr)
    sys.exit(1)

try:
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(updated)
except OSError as exc:
    print(f"Error: could not write {path}: {exc} (try sudo?)", file=sys.stderr)
    sys.exit(1)
PY
  then
    return 1
  fi
  return 0
}

rewrite_peers() {
  if [[ ! -f "$CYCLONEDDS_XML" ]]; then
    echo "Error: missing ${CYCLONEDDS_XML}" >&2
    return 1
  fi

  local peers_csv="$CENTRAL_IP,$THIS_ROBOT_IP"
  local ip
  for ip in "${OTHER_ROBOT_IPS[@]+"${OTHER_ROBOT_IPS[@]}"}"; do
    peers_csv+=",$ip"
  done

  if ! cp "$CYCLONEDDS_XML" "$BACKUP_XML"; then
    echo "Error: could not write backup ${BACKUP_XML} (try sudo?)." >&2
    return 1
  fi

  if ! PEERS_CSV="$peers_csv" CYCLONEDDS_XML="$CYCLONEDDS_XML" python3 - <<'PY'
import os
import re
import sys

path = os.environ["CYCLONEDDS_XML"]
peers = [p.strip() for p in os.environ["PEERS_CSV"].split(",") if p.strip()]

with open(path, "r", encoding="utf-8") as f:
    text = f.read()

if "<Peers>" not in text or "</Peers>" not in text:
    print("Error: no <Peers>...</Peers> section found in cyclonedds.xml", file=sys.stderr)
    sys.exit(1)

indent_match = re.search(r"^([ \t]*)<Peers>", text, re.MULTILINE)
peer_indent = (indent_match.group(1) if indent_match else "      ") + "  "
block_indent = indent_match.group(1) if indent_match else "      "

peer_lines = "\n".join(f'{peer_indent}<Peer Address="{ip}"/>' for ip in peers)
new_block = f"{block_indent}<Peers>\n{peer_lines}\n{block_indent}</Peers>"

updated, count = re.subn(
    r"[ \t]*<Peers>.*?</Peers>",
    new_block,
    text,
    count=1,
    flags=re.DOTALL,
)
if count != 1:
    print("Error: failed to replace <Peers> section", file=sys.stderr)
    sys.exit(1)

try:
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(updated)
except OSError as exc:
    print(f"Error: could not write {path}: {exc} (try sudo?)", file=sys.stderr)
    sys.exit(1)
PY
  then
    return 1
  fi
  return 0
}

source_robot_env() {
  # shellcheck disable=SC1090
  source "$ROBOT_ENV_SH"
}

print_summary() {
  local ip
  echo
  echo "Updated ${ROBOT_ENV_SH}"
  echo "Backup saved to ${BACKUP_ENV}"
  echo "Updated ${CYCLONEDDS_XML}"
  echo "Backup saved to ${BACKUP_XML}"
  echo
  echo "New peer list:"
  echo "  - ${CENTRAL_IP}  (central machine)"
  echo "  - ${THIS_ROBOT_IP}  (this robot)"
  for ip in "${OTHER_ROBOT_IPS[@]+"${OTHER_ROBOT_IPS[@]}"}"; do
    echo "  - ${ip}"
  done
}

print_checklist() {
  echo
  echo "Still do these manually:"
  echo "  Run this script on every other Jetson with the same peer list"
  echo "  (only this robot's ROS_IP differs per unit)."
}

main() {
  echo "Robot CycloneDDS / ROS_IP Wi-Fi update"
  echo "Editing:"
  echo "  ${ROBOT_ENV_SH}"
  echo "  ${CYCLONEDDS_XML}"
  echo

  prompt_this_robot_ip
  prompt_central_ip
  prompt_other_robot_ips

  if ! confirm_changes; then
    return 1
  fi

  if ! rewrite_robot_env; then
    return 1
  fi
  if ! rewrite_peers; then
    echo "Error: cyclonedds.xml update failed; robot_env.sh may already be updated." >&2
    echo "  Restore from ${BACKUP_ENV} if needed." >&2
    return 1
  fi

  print_summary

  if ((_IS_SOURCED)); then
    source_robot_env
  fi

  print_checklist
  return 0
}

main
_script_exit $?
