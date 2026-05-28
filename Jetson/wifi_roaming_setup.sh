#!/bin/bash

# ==============================================================================
# WiFi Roaming Setup Script
# ------------------------------------------------------------------------------
# This script automates the setup of a custom WiFi connection with roaming
# capabilities on an Ubuntu-based system, as per the provided instructions.
#
# It will:
#   - Disable NetworkManager for the WiFi device.
#   - Configure wpa_supplicant for roaming (per-interface config).
#   - Enable wpa_supplicant@<interface>.service as the sole WiFi owner.
#   - Disable the generic wpa_supplicant.service to avoid conflicts.
#   - Create a DHCP-only boot script and systemd service (custom_wifi).
#
# WARNING: This script modifies critical system network configurations.
# It creates backups of modified files with a .bak extension.
# ==============================================================================

# --- Sanity Checks and Initial Setup ---

# 1. Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root. Please use 'sudo ./setup_roaming_wifi.sh'"
   exit 1
fi

echo "--- WiFi Roaming Setup Script ---"
echo "WARNING: This will modify your system's network configuration."
echo "Backups of modified files will be created with a .bak extension."
read -p "Do you want to continue? (y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborting."
    exit 1
fi

# --- Gather Information ---

# 2. Automatically detect the WiFi interface (wlan0, wlan1, etc.)
WIFI_INTERFACE=$(ls /sys/class/net | grep -E '^wlan' | head -n 1)
if [ -z "$WIFI_INTERFACE" ]; then
    echo "ERROR: Could not automatically detect a WiFi interface. Aborting."
    exit 1
fi
echo "Detected WiFi interface: $WIFI_INTERFACE"

# 3. Get the MAC address for the interface
WIFI_MAC=$(cat "/sys/class/net/$WIFI_INTERFACE/address")
if [ -z "$WIFI_MAC" ]; then
    echo "ERROR: Could not get MAC address for $WIFI_INTERFACE. Aborting."
    exit 1
fi
echo "Detected MAC address: $WIFI_MAC"

# 4. Prompt user for WiFi credentials
read -p "Please enter your WiFi SSID: " WIFI_SSID
read -sp "Please enter your WiFi Password: " WIFI_PSK
echo

if [ -z "$WIFI_SSID" ] || [ -z "$WIFI_PSK" ]; then
    echo "ERROR: SSID and Password cannot be empty. Aborting."
    exit 1
fi

# 5. Prompt for operating country (WiFi regulatory domain)
echo ""
echo "Select the country where this device will operate:"
echo "  1) USA"
echo "  2) South Korea"
while true; do
    read -p "Enter choice [1 or 2]: " COUNTRY_CHOICE
    case "$COUNTRY_CHOICE" in
        1)
            WIFI_COUNTRY="US"
            break
            ;;
        2)
            WIFI_COUNTRY="KR"
            break
            ;;
        *)
            echo "Invalid choice. Please enter 1 or 2."
            ;;
    esac
done
echo "Using WiFi country code: ${WIFI_COUNTRY}"

echo "Configuration complete. Starting setup..."
sleep 1

# --- Configuration Steps ---

# Step 1: Disable NetworkManager for the WiFi interface
echo "[Step 1/5] Configuring NetworkManager..."
NM_CONF="/etc/NetworkManager/NetworkManager.conf"
NM_CONF_BAK="${NM_CONF}.bak"

if [ -f "$NM_CONF" ]; then
    cp "$NM_CONF" "$NM_CONF_BAK"
    echo "  -> Backup created at $NM_CONF_BAK"
fi

# Overwrite NetworkManager.conf with the full roaming setup
cat <<EOF > "$NM_CONF"
[main]
plugins=ifupdown,keyfile

[ifupdown]
managed=false

[device]
wifi.scan-rand-mac-address=no

[keyfile]
unmanaged-devices=mac:${WIFI_MAC}
EOF

echo "  -> Wrote NetworkManager.conf with unmanaged device MAC $WIFI_MAC"
systemctl restart NetworkManager
echo "  -> Restarted NetworkManager service."

# Step 2: Set up wpa_supplicant (systemd per-interface unit)
echo "[Step 2/5] Configuring wpa_supplicant..."
WPA_UNIT="wpa_supplicant@${WIFI_INTERFACE}.service"
WPA_CONF="/etc/wpa_supplicant/wpa_supplicant-${WIFI_INTERFACE}.conf"
WPA_CONF_BAK="${WPA_CONF}.bak"

if ! systemctl cat "$WPA_UNIT" &>/dev/null; then
    echo "ERROR: ${WPA_UNIT} not found. Is wpasupplicant installed?"
    exit 1
fi

if [ -f "$WPA_CONF" ]; then
    cp "$WPA_CONF" "$WPA_CONF_BAK"
    echo "  -> Backup created at $WPA_CONF_BAK"
fi

# wpa_supplicant@<iface> reads wpa_supplicant-<iface>.conf
cat <<EOF > "$WPA_CONF"
ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=${WIFI_COUNTRY}
bgscan="simple:30:-60:600"

network={
    ssid="${WIFI_SSID}"
    psk="${WIFI_PSK}"
    key_mgmt=WPA-PSK
}
EOF

echo "  -> Created ${WPA_CONF} (country=${WIFI_COUNTRY})"

systemctl stop wpa_supplicant.service 2>/dev/null || true
systemctl disable wpa_supplicant.service 2>/dev/null || true
echo "  -> Disabled generic wpa_supplicant.service"

systemctl daemon-reload
systemctl enable "$WPA_UNIT"
systemctl restart "$WPA_UNIT"
echo "  -> Enabled and started ${WPA_UNIT}"


# Step 3: Create the DHCP script (wpa_supplicant is managed by systemd)
echo "[Step 3/5] Creating DHCP script..."
CONNECT_SCRIPT_PATH="/usr/local/bin/custom_wifi.sh"

cat <<EOF > "$CONNECT_SCRIPT_PATH"
#!/bin/bash

INTERFACE="${WIFI_INTERFACE}"

# WiFi association is handled by ${WPA_UNIT}
dhclient \$INTERFACE
EOF

chmod +x "$CONNECT_SCRIPT_PATH"
echo "  -> Script created at $CONNECT_SCRIPT_PATH and made executable."


# Step 4: Create the systemd service
echo "[Step 4/5] Creating systemd service..."
SERVICE_PATH="/etc/systemd/system/custom_wifi.service"

cat <<EOF > "$SERVICE_PATH"
[Unit]
Description=DHCP for WiFi roaming interface (${WIFI_INTERFACE})
After=network.target ${WPA_UNIT}
Wants=${WPA_UNIT}

[Service]
Type=oneshot
ExecStart=/usr/local/bin/custom_wifi.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "  -> Service file created at $SERVICE_PATH"

# Step 5: Enable the DHCP service
echo "[Step 5/5] Enabling DHCP service..."
systemctl daemon-reload
echo "  -> Reloaded systemd daemon."
systemctl enable custom_wifi.service
echo "  -> Enabled custom_wifi.service to start on boot."

# --- Finalization ---

echo ""
echo "--- Setup Complete! ---"
echo "WiFi is managed by ${WPA_UNIT}; DHCP runs via custom_wifi.service on boot."
echo "To apply all changes and test the new configuration, please reboot your system."
echo "Check status: sudo systemctl status ${WPA_UNIT} custom_wifi.service"
echo "You can test DHCP now by running: sudo systemctl start custom_wifi.service"
echo ""
echo "To undo the changes:"
echo "1. sudo systemctl disable --now custom_wifi.service"
echo "2. sudo systemctl disable --now ${WPA_UNIT}"
echo "3. sudo systemctl enable wpa_supplicant.service"
echo "4. sudo rm $SERVICE_PATH $CONNECT_SCRIPT_PATH"
echo "5. sudo mv $WPA_CONF_BAK $WPA_CONF  # if backup exists"
echo "6. sudo mv $NM_CONF_BAK $NM_CONF"
echo "7. Reboot"

exit 0