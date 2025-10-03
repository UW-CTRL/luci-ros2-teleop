# LUCI Basic Teleop Package

The `luci_basic_teleop` package provides functionality to control LUCI, a robotic wheelchair, using either a keyboard or an Xbox controller. This package publishes joystick commands to LUCI and is designed to work on Linux-based systems. It has been tested within the LUCI Docker container and is compatible with the [luci_ros2_sdk](https://github.com/lucimobility/luci-ros2-sdk).

## Overview

### Node Names
- `/keyboard_control_node`: For keyboard-based teleoperation.
- `/controller_control_node`: For controller-based teleoperation.

### Topic
- **Name**: `luci/remote_joystick`
- **Message Type**: `[luci_messages/msg/LuciJoystick]`

### Service Calls
- **On Start**: `/luci/set_shared_remote_input` (`std_srvs/srv/Empty`)
- **On Termination**: `/luci/remove_shared_remote_input` (`std_srvs/srv/Empty`)

For detailed implementation, refer to the [luci_basic_teleop documentation](docs/teleop.md).

---

## How to Get and Build the Package

### Prerequisites
Ensure your ROS2 workspace includes the following packages:
```bash
ros_ws
└── src (Use colcon build at this level)
    └── luci-ros2
        └── src 
            ├── luci-ros2-keyboard-teleop
            ├── luci-ros2-grpc
            └── luci-ros2-msgs
```

If your workspace is missing these packages, follow the instructions in the [LUCI ROS2 SDK repository](https://github.com/lucimobility/luci-ros2-sdk) to install all dependencies.

### LUCI SDK Dependencies
- [luci-ros2-grpc](https://github.com/UW-CTRL/luci-ros2-grpc): Handles gRPC communication with LUCI.
- [luci-ros2-msgs](https://github.com/UW-CTRL/luci-ros2-msgs): Defines ROS2 message types for LUCI.
- [luci-ros2-transforms](https://github.com/lucimobility/luci-ros2-transforms): Provides transformation utilities for LUCI.

### Build Steps
1. Navigate to your ROS2 workspace:
   ```bash
   cd ~/ros_ws
   ```
2. Build the workspace:
   ```bash
   colcon build
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

---

## How to Run the Package

### Keyboard Teleoperation
1. Launch the keyboard teleoperation node using the launch file:
    ```bash
    ros2 launch luci_basic_teleop keyboard_teleop_launch.py
    ```
    This will automatically start the gRPC interface node and set up the necessary connections.
2. Use the arrow keys to control LUCI. Press `Ctrl+C` or `q` to terminate.

### Controller Teleoperation
1. Launch the controller teleoperation node using the launch file:
    ```bash
    ros2 launch luci_basic_teleop controller_teleop_launch.py
    ```
    This will automatically start the gRPC interface node and set up the necessary connections.

3. Use the controller to operate LUCI. Press `Ctrl+C` to terminate.
### Running Without Launch Files

If you prefer to run the nodes without using the provided launch files, follow these steps:

#### Keyboard Teleoperation
1. Open a terminal and source your workspace:
    ```bash
    source ~/ros_ws/install/setup.bash
    ```
2. Start the gRPC interface node with the LUCI IP address:
    ```bash
    ros2 run luci_grpc_interface grpc_interface_node -a <ip_address>
    ```
3. In another terminal, source your workspace:
    ```bash
    source ~/ros_ws/install/setup.bash
    ```
4. Run the keyboard teleoperation node:
    ```bash
    ros2 run luci_basic_teleop keyboard_control_node
    ```
5. Use the arrow keys to control LUCI. Press `Ctrl+C` or `q` to terminate.

#### Controller Teleoperation
1. Open a terminal and source your workspace:
    ```bash
    source ~/ros_ws/install/setup.bash
    ```
2. Start the gRPC interface node with the LUCI IP address:
    ```bash
    ros2 run luci_grpc_interface grpc_interface_node -a <ip_address>
    ```
3. In another terminal, source your workspace:
    ```bash
    source ~/ros_ws/install/setup.bash
    ```
4. Run the controller teleoperation node:
    ```bash
    ros2 run luci_basic_teleop controller_control_node
    ```
5. In a separate terminal, start the joystick driver:
    ```bash
    ros2 run joy joy_node
    ```
6. Use the controller to operate LUCI. Press `Ctrl+C` to terminate.

**Note**: When running with launch files, ensure the IP address is correctly specified for the gRPC interface node within the luci_basic_bringup node.
## Usage Details

### Keyboard Controls
- Use the arrow keys to drive LUCI in a single direction.
- Only one key can be processed at a time (e.g., holding forward and left simultaneously will not turn the chair).
- Terminate the session with `Ctrl+C` or `q`.

### Controller Controls
- Movements mirror the wheelchair's joystick, allowing for multidirectional control.
- Terminate the session with `Ctrl+C`.

**Note**: Using launch files enables receiving additional data from LUCI.
 LUCI Basic Teleop Package

<!-- The luci_basic_teleop package is an example node that publishes messages that commands LUCI to drive by using the `arrow keys` on your keyboard, or an xbox controller. Use `ctrl+c` to terminate.
This package was only tested to be compatible on a Linux OS. If you are running this in our docker container, it will work.

NOTE: This is only a package that is intended to be used with [luci_ros2_sdk](https://github.com/lucimobility/luci-ros2-sdk)

Node name: `/keyboard_control_node` or `/controller_control_node`

Topic name: `luci/remote_joystick`

Topic message type: `[luci_messages/msg/LuciJoystick]`

Service Call when started: `/luci/set_shared_remote_input std_srvs/srv/Empty`

Service Call when terminated: `/luci/remove_shared_remote_input std_srvs/srv/Empty`

More detailed implementation docs can be found here [luci_basic_teleop](docs/teleop.md)

## Usage ##

After correctly sourcing ROS2, Run the following to start the respective launch files:

Keyboard: 

`ros2 launch luci_basic_teleop keyboard_teleop_launch.py`

or 
Controller: 

`ros2 launch luci_basic_teleop controller_teleop_launch.py`

## How to build and run this package ##
Put this package into the src folder of your ros2 workspace. Your workspace should at least have these three packages:
```bash
ros_ws
└── src (Use colcon build at this level)
    └──luci-ros2
        └── src 
            ├── luci-ros2-keyboard-teleop
            ├── luci-ros2-grpc
            └── luci-ros2-msgs
```

If your workspace doesn't look like this, go to https://github.com/lucimobility/luci-ros2-sdk to install all the SDK Dependancies

### LUCI SDK Package Dependencies ###
- [luci-ros2-grpc](https://github.com/UW-CTRL/luci-ros2-grpc)
- [luci-ros2-msgs](https://github.com/UW-CTRL/luci-ros2-msgs)
- [luci-ros2-transforms](https://github.com/lucimobility/luci-ros2-transforms)


### Build Steps ###
1. Navigate to your workspace
2. run `colcon build`
3. run `source install/setup.bash`
4. run `ros2 run luci_grpc_interface grpc_interface_node -a <ip address>` 

1. In another terminal, navigate to your workspace.
2. run `source install/setup.bash`
3. run `ros2 run luci_basic_teleop keyboard_control_node` or `ros2 run luci_basic_teloep controller_control_node`
    - launch files are available but if not in lab, the ip address must be changed in the bringup launch folder
4. Use the arrow keys in this terminal to control the chair.
    - if using the controller you must also open a new terminal and run `ros2 run joy joy_node` 


### Usage ###
Using the arrow keys on your keyboard will drive LUCI in a single direction when a key is held down. This example currently only supports one key at a time, so holding forward and then holding the left arrow while forward is still held down will not make the chair turn left, it will continue to go straight.

To end the keyboard teleoperation press `ctrl-c` or `q` at anytime.

For control with the controller, all movements are mirrored as if they were on the wheelchair's joystick, allowing movements in any direction.

To end the keyboard teleoperation press `ctrl-c` at anytime.

**Using launch files also allows for recieving data.** -->
