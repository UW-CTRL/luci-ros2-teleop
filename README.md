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
└── src 
    ├── luci-ros2-grpc
    ├── luci-ros2-msgs
    ├── luci-ros2-transforms
    ├── luci-ros2-odom
    └── luci-ros2-bringup
```

### LUCI SDK Dependencies
- [luci-ros2-grpc](https://github.com/UW-CTRL/luci-ros2-grpc): Handles gRPC communication with LUCI.
- [luci-ros2-msgs](https://github.com/UW-CTRL/luci-ros2-msgs): Defines ROS2 message types for LUCI.
- [luci-ros2-odom](https://github.com/UW-CTRL/luci-ros2-odom): Creates the odom values from the wheelchair's encoders.
- [luci-ros2-transforms](https://github.com/lucimobility/luci-ros2-transforms): Provides transformation utilities for LUCI.
- [luci-ros2-bringup](https://github.com/UW-CTRL/luci-ros2-bringup): Contains launch files and configurations for LUCI.

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

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.
