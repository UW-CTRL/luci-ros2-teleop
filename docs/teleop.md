# luci_basic_teleop

The luci_basic_teleop package runs a node that publishes messages that can be used to drive LUCI using the arrow keys on your keyboard. ctrl+c or q to terminate. Compatible with Linux.

Node name: `/keyboard_control_node`

Topic name: `luci/remote_joystick`

Topic message type: `[luci_messages/msg/LuciJoystick]`

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

