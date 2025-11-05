# Laser Scan Merger Installation Guide

Your mapping2_launch.py needs a laser scan merger to combine /scan1 and /scan2 into /scan for SLAM.

## Error You Saw
```
package 'ira_laser_tools' not found
E: Unable to locate package ros-humble-laser-scan-merger
```

## Solution: Build ros2_laser_scan_merger from source

On the VMX robot, run:

```bash
# Navigate to the ros_dependencies folder (already exists in your repo)
cd /home/vmx/ROS2/ros_dependencies/ros2_laser_scan_merger

# Clone the repo
git clone https://github.com/mich1342/ros2_laser_scan_merger.git .

# Build it
cd /home/vmx/ROS2
colcon build --packages-select ros2_laser_scan_merger

# Source the workspace
source install/setup.bash
```

Then verify:
```bash
ros2 pkg list | grep laser
```

The current launch file tries to use this with parameters:
- output_scan_topic:=/scan
- input_topics:=[/scan1,/scan2]
- destination_frame:=base_link

### Option 2: Install ira_laser_tools (Alternative)

```bash
cd ~/ros2_ws/src
git clone https://github.com/iralabdisco/ira_laser_tools.git
cd ~/ros2_ws
colcon build --packages-select ira_laser_tools
source install/setup.bash
```

Then in mapping2_launch.py:
1. Comment out the ExecuteProcess merger (lines ~92-98)
2. Uncomment the Node-based merger (lines ~106-120)

### Option 3: Use laserscan_multi_merger from laser_tools

Some ROS2 distros include this:
```bash
sudo apt install ros-humble-laser-tools
```

## After Installation

1. Rebuild your workspace if you built from source:
   ```bash
   cd ~/ros2_ws  # or ~/ROS2
   colcon build
   source install/setup.bash
   ```

2. Relaunch:
   ```bash
   ros2 launch studica_control mapping2_launch.py
   ```

3. Verify /scan is published:
   ```bash
   ros2 topic list | grep scan
   ros2 topic info /scan
   ros2 topic echo /scan --once
   ```

## Verification

After successful merger launch, you should see:
- `ros2 node list | grep -i merge` shows a merger node
- `ros2 topic info /scan` shows 1 publisher (the merger)
- `ros2 node info /slam_toolbox` shows it subscribes to /scan
- `/map` topic is being published and updated

## Current Status

Your mapping2_launch.py is currently configured to use **ros2_laser_scan_merger** (Option 1).
If that package isn't available, try installing it or switch to ira_laser_tools (Option 2).
