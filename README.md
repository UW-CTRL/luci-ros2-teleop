# LUCI Basic Teleop Package

The `luci_basic_teleop` package provides functionality to control LUCI, a robotic wheelchair, using either a keyboard or an Xbox controller. This package publishes joystick commands to LUCI and is designed to work on Linux-based systems. It has been tested within the LUCI Docker container and is compatible with the [luci_ros2_sdk](https://github.com/lucimobility/luci-ros2-sdk).


### Xbox Teleop Node (`xbox_teleop_node.py`)
Translates raw Linux joystick driver events (`/joy`) into `LuciJoystick` formatted commands for manual remote control.

* **Subscribed Topics:**
  * `/joy` (`sensor_msgs/msg/Joy`) — Raw gamepad axis and button events.
  * `/luci/control_state` (`std_msgs/msg/String`) — Monitors global state.
  * `/luci/joystick_position` (`luci_messages/msg/LuciJoystick`) — Monitors physical joystick activity.
  * `/luci/override_button_press_count_data` (`std_msgs/msg/Int32`) — Override button monitoring.
* **Published Topics:**
  * `/luci/remote_joystick` (`luci_messages/msg/LuciJoystick`) — Scaled gamepad control commands.
* **Behavior:**
  * If `current_state != 'TELEOP'`, zeroed joystick commands (`forward_back = 0`, `left_right = 0`) are published to guarantee safe stopping when teleoperation is disengaged.

**TODO:** Update keyboard teleop.


---

## Quickstart & Execution
`ros2 launch luci_basic_teleop xbox_teleop_launch.py`