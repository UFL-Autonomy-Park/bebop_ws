# bebop_ws

Workspace to start up Parrot Bebop 2 drones in the NCR Lab. 

|  Package Name | Description  |  Repository URL   | Acknowledgements | 
| --- | --- | --- | --- |
| `ros2_parrot_arsdk` | Parrot ARSDK | [ros2_parrot_arsdk](https://github.com/UFL-Autonomy-Park/ros2_parrot_arsdk) | Forked from [jeremyfix/ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk) |
| `ros2_bebop_driver` | ROS2 re-write of `bebop_autonomy`, creates publishers and subscribers for core Bebop functionality | [ros2_bebop_driver](https://github.com/UFL-Autonomy-Park/ros2_bebop_driver) | Forked from [jeremyfix/ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver) |
| `vrpn_mocap` | Turns VRPN messages from OptriTrack Motive in ROS 2 messages | [vrpn_mocap](https://github.com/UFL-Autonomy-Park/vrpn_mocap/) | Forked from [alvinsunyixiao/vrpn_mocap](https://github.com/alvinsunyixiao/vrpn_mocap) | 
## Requirements
ROS 2 Humble, Ubuntu 22.04

## Installation

```
mkdir -p <your_ws_name>/src
cd <your_ws_name>/src 
```

```
git clone --recurse-submodules https://github.com/UFL-Autonomy-Park/bebop_ws.git
```

OR 
```
git clone https://github.com/UFL-Autonomy-Park/bebop_ws.git && git submodule update --init --recursive
```

These commands are required for installation of the `ros2_parrot_arsdk`
```
cd ~/<your_ws_name>
sudo apt install ros-humble-camera-info-manager libavdevice-dev
git config --global color.ui auto 
```

Then, you can
```
colcon build
```