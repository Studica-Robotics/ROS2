#!/usr/bin/env bash
# One-time system setup + post-build step to run studica_control without sudo.
#
# The VMX HAL uses pigpio directly, which requires access to /dev/mem.
# setcap grants only the specific Linux capabilities needed so the node
# runs as a normal user without full root.
#
# Usage:
#   ./scripts/setup_permissions.sh              # assumes ~/studica_ws
#   ./scripts/setup_permissions.sh /path/to/ws  # explicit workspace path

set -e

WS="${1:-$HOME/studica_ws}"
EXEC="$WS/install/studica_control/lib/studica_control/manual_composition"

# Verify the executable exists
if [ ! -f "$EXEC" ]; then
    echo "ERROR: Executable not found at $EXEC"
    echo "       Build the package first:"
    echo "         colcon build --packages-select studica_control"
    echo "       Then re-run this script."
    exit 1
fi

# Grant hardware capabilities
# cap_sys_rawio  — access /dev/mem (required by pigpio for GPIO/peripheral registers)
# cap_ipc_lock   — lock pages in memory (required by pigpio for DMA transfers)
# cap_sys_nice   — raise scheduling priority (required by pigpio real-time mode)
sudo setcap cap_sys_rawio,cap_ipc_lock,cap_sys_nice+eip "$EXEC"
echo "OK  capabilities set on $EXEC"

# I2C device access 
if ! groups "$USER" | grep -q '\bi2c\b'; then
    sudo usermod -a -G i2c "$USER"
    echo "OK  added $USER to the i2c group (re-login or run: newgrp i2c)"
else
    echo "OK  $USER already in i2c group"
fi

# udev rule for I2C devices
UDEV_RULE=/etc/udev/rules.d/99-studica.rules
if [ ! -f "$UDEV_RULE" ]; then
    echo 'KERNEL=="i2c-[0-9]*", GROUP="i2c", MODE="0660"' | sudo tee "$UDEV_RULE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "OK  udev rule installed at $UDEV_RULE"
else
    echo "OK  udev rule already present at $UDEV_RULE"
fi

echo ""
echo "Setup complete."
echo ""
echo "  NOTE: setcap is reset every time colcon replaces the executable."
echo "  Re-run this script after each 'colcon build'."
echo ""
echo "  If this is your first time running setup, log out and back in"
echo "  (or run 'newgrp i2c') for group changes to take effect."
