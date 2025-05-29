# LUCI Basic Teleop Package

The luci_basic_teleop package is an example node that publishes messages that commands LUCI to drive by using the `arrow keys` on your keyboard. Use `ctrl+c` or `q` to terminate.
This package was only tested to be compatible on a Linux OS. If you are running this in our docker container, it will work.

NOTE: This is only a package that is intended to be used with [luci_ros2_sdk](https://github.com/lucimobility/luci-ros2-sdk)

Node name: `/keyboard_control_node`

Topic name: `luci/remote_joystick`

Topic message type: `[luci_messages/msg/LuciJoystick]`

Service Call when started: `/luci/set_shared_remote_input std_srvs/srv/Empty`

Service Call when terminated: `/luci/remove_shared_remote_input std_srvs/srv/Empty`

More detailed implementation docs can be found here [luci_basic_teleop](docs/teleop.md)

## Usage ##

After correctly sourcing ROS2, Run the following to start the node:

`ros2 run luci_basic_teleop keyboard_control_node`

## How to build and run this package ##
Put this package into the src folder of your ros2 workspace. Your workspace should at least have these three packages:
```bash
ros_ws
└──luci-ros2
    └── src (Use colcon build at this level)
        ├── luci-ros2-keyboard-teleop
        ├── luci-ros2-grpc
        └── luci-ros2-msgs
```

If your workspace doesn't look like this, go to https://github.com/lucimobility/luci-ros2-sdk to install all the SDK Dependancies

### LUCI SDK Package Dependencies ###
- [luci-ros2-grpc](https://github.com/lucimobility/luci-ros2-grpc)
- [luci-ros2-msgs](https://github.com/lucimobility/luci-ros2-msgs)


### Build Steps ###
1. Navigate to your workspace
2. run `colcon build`
3. run `source install/setup.bash`
4. run `ros2 run luci_grpc_interface grpc_interface_node -a <ip address>`

1. In another terminal, navigate to your workspace.
2. run `source install/setup.bash`
3. run `ros2 run luci_basic_teleop keyboard_control_node`
4. Use the arrow keys in this terminal to control the chair.

### Usage ###
Using the arrow keys on your keyboard will drive LUCI in a single direction when a key is held down. This example currently only supports one key at a time, so holding forward and then holding the left arrow while forward is still held down will not make the chair turn left, it will continue to go straight.

To end the keyboard teleoperation press `ctrl-c` or `q` at anytime.

## Notes for LUCI Devs: ##
NOTE: There is also an automatic build script called `build-package.sh` that can be run to build an installable `.deb` file. This is what the github actions call to automate the release process. You can use it if you want to just put the deb in the ros directory to run when ros is sourced. Another way to do it is just download the debian using apt. 

### Releasing new version ###
When a new version of this package is ready to be released there are a couple steps to follow. It is important to note that most of the process is automated for convenience and the process should be just a couple of button clicks. 

### Steps ### 
1. Update release version
    - This should be its own separate PR and should only update the package.xml `<version> </version>` tag. 
    - LUCI follows [semver](https://semver.org/) style versioning so MAJOR.MINOR.PATCH versions are expected.
    - It is okay to not put out versions until multiple changes have happened to the code. 
2. Once the version increment is merged you simply need to create an official release in github. Make sure you make the release version the same as what is now in `package.xml`. We have chosen to keep github release and package version in sync.
    - This should trigger an action to auto run called `Create and Sign Package` which you can monitor in the github actions panel. This should grab the released code, build it, make an installable .deb file, gdb sign it and push it to jrog artifactory.  

If everything went smoothly congratulations the new package will be released and publicly distributable. 


<b>NOTE: Once a PR is merged into the `main` branch the docs site in the `next` version will update with it that evening.</b>
