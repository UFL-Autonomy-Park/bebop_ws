# bebop_ws

Workspace to start up Parrot Bebop 2 drones in the NCR lab. 

|  Package Name | Description  |  Repository URL   | Acknowledgements | 
| --- | --- | --- | --- |
| `ros2_parrot_arsdk` | Parrot ARSDK | [ros2_parrot_arsdk](https://github.com/UFL-Autonomy-Park/ros2_parrot_arsdk) | Forked from [jeremyfix/ros2_parrot_arsdk](https://github.com/jeremyfix/ros2_parrot_arsdk) |
| `ros2_bebop_driver` | ROS2 re-write of `bebop_autonomy`, creates publishers and subscribers for core Bebop functionality | [ros2_bebop_driver](https://github.com/UFL-Autonomy-Park/ros2_bebop_driver) | Forked from [jeremyfix/ros2_bebop_driver](https://github.com/jeremyfix/ros2_bebop_driver) |

## Requirements
ROS 2 Humble, Ubuntu 22.04

## Installation

```
mkdir -p <your_ws_name>/src
cd <your_ws_name>/src 
```

```
git clone https://github.com/UFL-Autonomy-Park/bebop_ws.git && git submodule update --init --recursive
```

OR 

```
git clone --recurse-submodules https://github.com/UFL-Autonomy-Park/bebop_ws.git
```

These commands are required for installation of the `ros2_parrot_arsdk`
```
cd ~/<your_ws_name>
sudo apt install ros-humble-camera-info-manager libavdevice-dev
git config --global color.ui auto 
```

First, build `ros2_parrot_arsdk`,
```
colcon build --packages-select ros2_parrot_arsdk
```

and then build `ros2_bebop_driver`.

```
colcon build --packages-select ros2_bebop_driver
```

Lastly, build the rest of the packages
```
colcon build --packages-select vrpn_mocap bebop_client bebop_control bebop_server bebop_teleop ncr_lab_viz
```