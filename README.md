# robomaster_mujoco
<img width="1294" height="553" alt="robomaster_ep_core" src="https://github.com/user-attachments/assets/674c80aa-6e0c-4c40-9699-57916812746c" />

This repository packages the RoboMaster EP Core for ROS 2 (Humble) and includes support for both the real robot and the MuJoCo simulator. It preserves the official `robomaster_ros` behavior while addressing arm latency by replacing `arm.py` with a queue-friendly version.

## Overview

- **Real robot:** based on `jeguzzi/robomaster_ros` with ROS 2 Humble compatibility and a faster manipulator bridge.
- **Simulator:** a MuJoCo-backed environment that mirrors the physical bot for development without hardware.

## Step-by-step setup

1. **Update system packages and install ROS tooling**

   ```sh
   sudo apt-get update
   sudo apt-get install -y python3-pip
   sudo apt-get install -y ros-humble-ament-cmake-clang-format
   sudo apt-get install -y \
     ros-humble-xacro \
     ros-humble-launch-xml \
     ros-humble-cv-bridge \
     ros-humble-launch-testing-ament-cmake \
     ros-humble-robot-state-publisher \
     ros-humble-joint-state-publisher \
     ros-humble-joint-state-publisher-gui \
     ros-humble-joy \
     ros-humble-teleop-twist-joy \
     ros-humble-joy-linux \
     libopus-dev \
     build-essential \
     cmake \
     pkg-config
   ```

2. **Install MuJoCo and multimedia dependencies**

   ```sh
   sudo apt-get install -y \
     libgl1 libglx-mesa0 libglfw3 libglew2.2 libxrandr2 libxinerama1 libxcursor1 \
     libxi6 libxxf86vm1 libasound2 xsltproc libspdlog-dev libboost-system-dev \
     libboost-thread-dev libavcodec-dev libavformat-dev libavutil-dev libavdevice-dev \
     libx264-dev liblua5.3-dev lua5.3 pkg-config
   ```

3. **Install pip dependencies from `requirements.txt`**

   ```sh
   pip install -r requirements.txt
   ```

4. **Set MuJoCo environment variables**

   ```sh
   export MUJOCO_PATH=/opt/mujoco/mujoco-3.3.5
   export LD_LIBRARY_PATH="$MUJOCO_PATH/lib:$MUJOCO_PATH/bin:$LD_LIBRARY_PATH"
   export DISPLAY="$DISPLAY"
   export XDG_RUNTIME_DIR=/tmp/runtime-root
   ```

5. **Create and build the ROS 2 workspace**

   ```sh
   mkdir -p ~/RM_ws/src
   cd ~/RM_ws/src
   git clone https://github.com/<your-fork>/robomaster_mujoco.git
   cd ~/RM_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

   Replace the clone URL with this repository location. The workspace contains the modified `arm.py`, so sync your fork if you rebased from the upstream repo.

## Running the robot

Connect an Xbox controller (or any joystick mapped to `/joy`) before launching either the real robot or simulator teleop stacks. The commands below were validated with a wired Xbox controller; adjust `serial_number` and network settings for your own robot.

### Real-world teleoperation

```sh
ros2 launch robomaster_ros ep.launch conn_type:=ap serial_number:=3JKCH9A00101RJ log_level:=info
ros2 launch robomaster_ros teleop_linux.launch
```

Start the `ep.launch` file first so the robot establishes coordination with the base station, then start the teleop launch to drive the robot with your controller.

### Simulation teleoperation

```sh
ros2 launch mujoco_rm_bringup teleop_linux.launch.xml start_joy_node:=false joy_topic:=/joy arm_joy_debug_hz:=5.0
```

The MuJoCo simulator publishes camera topics that can be visualized with `rviz2`; once the launch is running, start `rviz2` and add the relevant image/display panels pointing to the simulator topics.

## Notes

- KEEP IN TUNNED: Trying to fix the simulation to real gap differences. 
- The manipulator module ships with a queue-based rewrite of `arm.py` to avoid the arm-state delays reported in the upstream driver.
- Keep pip packages current by re-running `pip install -r requirements.txt` whenever dependencies change.
 
