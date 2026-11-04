#!/usr/bin/env bash
# One-time system setup for studica_control.
# Run once after first build, then again after each 'colcon build'.
#
# What this does (one-time setup — does not need to be re-run after builds):
#   - Registers library paths with ldconfig (sudo resets LD_LIBRARY_PATH)
#   - Installs a NOPASSWD sudoers rule so ros2 launch can run the node as
#     root (required by pigpio to access /dev/mem and lock its PID file)
#   - Installs a udev rule for I2C device access
#   - Installs a udev rule that drops a Bluetooth gamepad's link within 1 s of
#     it losing power (the default is 20 s, and until then the joy node keeps
#     repeating the last stick position, so the robot keeps driving)
#
# Usage:
#   ./scripts/setup_permissions.sh              # assumes ~/studica_ws
#   ./scripts/setup_permissions.sh /path/to/ws  # explicit workspace path

set -e

WS="${1:-$HOME/studica_ws}"
EXEC="$WS/install/studica_control/lib/studica_control/manual_composition"

if [ ! -f "$EXEC" ]; then
    echo "ERROR: Executable not found at $EXEC"
    echo "       Build first:  colcon build --packages-select studica_control"
    exit 1
fi

# ldconfig: register all library paths
# 'sudo' resets LD_LIBRARY_PATH, so every library directory must be in
# the ldconfig cache for the root-run process to find them.
LDCONF_FILE="/etc/ld.so.conf.d/studica_ws.conf"
cat <<EOF | sudo tee "$LDCONF_FILE" > /dev/null
# studica_control workspace
$WS/install/studica_control/lib
# ROS2 Humble
/opt/ros/humble/lib
/opt/ros/humble/lib/aarch64-linux-gnu
/opt/ros/humble/opt/rviz_ogre_vendor/lib
EOF
sudo ldconfig
echo "OK  registered workspace and ROS2 libs with ldconfig"

# Sudoers rule
# The launch file runs manual_composition via sudo so pigpio can access
# /dev/mem and lock its PID file. This rule allows that without a prompt.
SUDOERS_FILE=/etc/sudoers.d/studica_control
echo "$USER ALL=(ALL) SETENV: NOPASSWD: $EXEC" | sudo tee "$SUDOERS_FILE" > /dev/null
sudo chmod 0440 "$SUDOERS_FILE"
echo "OK  sudoers rule: $USER may run manual_composition as root without password"

# I2C udev rule
UDEV_RULE=/etc/udev/rules.d/99-studica.rules
if [ ! -f "$UDEV_RULE" ]; then
    echo 'KERNEL=="i2c-[0-9]*", GROUP="root", MODE="0660"' | sudo tee "$UDEV_RULE" > /dev/null
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "OK  udev rule for I2C installed"
else
    echo "OK  I2C udev rule already present"
fi

# Bluetooth gamepad link supervision timeout
# A controller that dies or goes out of range sends no disconnect, so Linux
# keeps the link for its supervision timeout (20 s by default). Meanwhile the
# joy node repeats the last stick position and joy_timeout never fires. The
# timeout is per connection and only the link's central can set it, so udev
# sets it each time a Bluetooth input device connects. BlueZ has no setting
# for it. udev's sandbox blocks Bluetooth sockets, so the rule hands the
# helper to systemd-run instead of running it directly.
BT_HELPER=/usr/local/bin/studica-bt-supervision
cat <<'EOF' | sudo tee "$BT_HELPER" > /dev/null
#!/bin/sh
# Called by udev with a new input device's Bluetooth address. Sets the link
# supervision timeout to 1 s (1600 slots of 0.625 ms).
ADDR="$1"
/usr/bin/hcitool con | grep -qi "$ADDR" || exit 0   # no Bluetooth link: not a Bluetooth device
if /usr/bin/hcitool lst "$ADDR" 1600 > /dev/null 2>&1; then
    logger -t studica-bt "link supervision timeout for $ADDR set to 1 s"
else
    logger -t studica-bt "could not set link supervision timeout for $ADDR (not the link's central?)"
fi
EOF
sudo chmod 0755 "$BT_HELPER"
echo 'ACTION=="add", SUBSYSTEM=="input", KERNEL=="input[0-9]*", ATTR{uniq}=="??:??:??:??:??:??", RUN+="/usr/bin/systemd-run --no-block /usr/local/bin/studica-bt-supervision %s{uniq}"' \
    | sudo tee /etc/udev/rules.d/99-studica-bluetooth.rules > /dev/null
sudo udevadm control --reload-rules
echo "OK  Bluetooth gamepads drop within 1 s of losing power (applies from the next connection)"

echo ""
echo "Setup complete. You can now run:"
echo "  ros2 launch studica_control studica_launch.py"
