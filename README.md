# LUCI Basic Teleop Package

The `luci_basic_teleop` package provides user input drivers for controlling LUCI using an Xbox controller or keyboard. This package translates gamepad events into standardized `LuciJoystick` messages published to the central command multiplexer.

It is designed for Linux-based ROS 2 environments (Humble / Jazzy) and works in tandem with `luci_core_control`.

TODO: add keyboard documentation.

---

## Node Documentation

### Xbox Teleop Node (`xbox_teleop_node.py`)
Reads Linux gamepad events on `/joy` and converts stick deflections into scaled percentage values (`-100` to `100`) and directional zones (`JS_FRONT`, `JS_BACK`, `JS_LEFT`, `JS_RIGHT`, `JS_ORIGIN`).

* **Subscribed Topics:**
  * `/joy` (`sensor_msgs/msg/Joy`) — Raw gamepad axis and button events.
* **Published Topics:**
  * `/xbox` (`luci_messages/msg/LuciJoystick`) — Scaled gamepad commands for `CentralController`.

---

## Gamepad Controls

| Control | Action / Mapping |
| :--- | :--- |
| **A Button** (`buttons[0]`) | Transitions system state from **`IDLE` $\rightarrow$ `TELEOP`** |
| **B Button** (`buttons[1]`) | Transitions system state from **`TELEOP` $\rightarrow$ `IDLE`** |
| **Left Stick (Vertical)** | Controls forward/backward velocity percentage (`-100` to `100`) |
| **Left Stick (Horizontal)** | Controls left/right turning percentage (`-100` to `100`) |
