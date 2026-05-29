#!/bin/bash

# ==============================================================================
# SPI Device (spidev) Setup Script
# ------------------------------------------------------------------------------
# Loads the spidev kernel module automatically on boot by writing to
# /etc/modules-load.d/spidev.conf (Jetson bring-up step 6).
#
# It will:
#   - Create /etc/modules-load.d/spidev.conf with "spidev"
#   - Load the module immediately (no reboot required for testing)
#
# Backups of an existing config file are saved with a .bak extension.
# ==============================================================================

set -e

SPI_CONF="/etc/modules-load.d/spidev.conf"

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root. Please use:"
   echo "  sudo ./spidev_setup.sh"
   exit 1
fi

echo "--- SPI Device (spidev) Setup ---"

if [[ -f "$SPI_CONF" ]] && grep -qx 'spidev' "$SPI_CONF"; then
    echo "spidev is already configured in $SPI_CONF"
else
    if [[ -f "$SPI_CONF" ]]; then
        cp "$SPI_CONF" "${SPI_CONF}.bak"
        echo "Backup created at ${SPI_CONF}.bak"
    fi

    echo 'spidev' > "$SPI_CONF"
    echo "Wrote spidev to $SPI_CONF"
fi

if modprobe spidev 2>/dev/null; then
    echo "Loaded spidev module (available until reboot if this step fails on boot)."
elif lsmod | grep -q '^spidev'; then
    echo "spidev module is already loaded."
else
    echo "Note: spidev could not be loaded now. Reboot after configuring SPI pins (bring-up step 5)."
fi

echo ""
echo "--- Setup complete ---"
echo "Reboot the Jetson so spidev loads automatically on every boot:"
echo "  sudo reboot"
echo ""
echo "After reboot, verify with:"
echo "  ls /dev/spidev*"

exit 0
