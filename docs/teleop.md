# luci_basic_teleop

The luci_basic_teleop package is an example node that publishes messages that 
commands LUCI to drive by using the `arrow keys` on your keyboard.
Use `ctrl+c` or `q` to terminate.
This package was only tested to be compatible on a UNIX based Linux OS.
If you are running this in our docker container, it will work.
NOTE: This is only a package that is intended to be used with [luci_ros2_sdk](https://github.com/lucimobility/luci-ros2-sdk)

Node name: `/keyboard_control_node`

Topic name: `luci/remote_joystick`

Topic message type: `[luci_messages/msg/LuciJoystick]`

Service Call when started: `/luci/set_auto_remote_input std_srvs/srv/Empty`

Service Call when terminated: `/luci/remove_auto_remote_input std_srvs/srv/Empty`

## Usage ##

After correctly sourcing ROS2, Run the following to start the node:

`ros2 run luci_basic_teleop keyboard_control_node`

This node was created to communicate with the `luci_grpc_interface`



| Package | Node |
|---------|------|
| luci_basic_teleop | keyboard_control_node |

| Topics | Subscription / Publish | Message Type | Description |
|--------|------------------------|--------------|-------------|
| luci/remote_joystick | publish | luci_messages::msg::LuciJoystick | The remote JS values that are processed by LUCI for remote drive operations


## More Info ##
### Service Calls to Control Joystick Input###
When this node starts, it first sends a ros2 service call to LUCI that enables remote control of the chair.

This call `/luci/set_auto_remote_input std_srvs/srv/Empty` disables the physical joystick on the chair and
only allows for remote control.

When this node is killed it re-enables the physical joystick and disables
remote control by calling `/luci/remove_auto_remote_input std_srvs/srv/Empty`.

If somehow the system crashes or loses connection and the chair gets stuck in a joystick input state that
you don't want, you can always run a service call from the terminal to change the state.

For example:
`ros2 service call /luci/set_auto_remote_input std_srvs/srv/Empty` to enable remote control

or

`ros2 service call /luci/remove_auto_remote_input std_srvs/srv/Empty` to disbale remote control

### LuciJoystick Message Types ###
The LuciJoystick message contains the following data:
| Data Type | Data Range|
|-----------|-----------|
| `int32 forward_back` | [-100, 100] |
| `int32 left_right`   | [-100, 100] |
| `int32 joystick_zone`| [0 - 8] |
| `int32 input_source` | [0 - 4] |

The integers for JoystickZone and InoutSource corespond with the following enums:
enum JoystickZone {
     Front = 0;
     FrontLeft = 1;
     FrontRight = 2;
     Left = 3;
     Right = 4;
     BackLeft = 5;
     BackRight = 6;
     Back = 7;
     Origin = 8;
}

enum InputSource {
     RampAssist = 0;
     Remote = 1;
     WDI = 2;
     ChairVirtual = 3;
     ChairPhysical = 4;
}

More info on the LuciMessages package can be found at: https://github.com/lucimobility/luci-ros2-msgs