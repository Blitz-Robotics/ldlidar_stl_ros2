# LDRobot LiDAR ROS2 Driver

This is a fork from the original [LDRobot LiDAR ROS2 Driver](https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2) repository.

## Installation

In your src folder of your ROS2 workspace, clone this repository:

```bash
git clone https://github.com/Blitz-Robotics/ldlidar_stl_ros2.git
```

In your ROS2 workspace, install the dependencies:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --symlink-install --packages-select ldlidar_stl_ros2
```

or

```bash
colcon build --packages-select ldlidar_stl_ros2
```

## Device Port Configuration (Optional)

The default port used by the driver is `/dev/ldlidar`. When launching the file you
can set the port to use by setting the `port` argument. However, if you want
to create a symbolic link to the device port, you can do that with the
following steps:

1. Unplug the device from the USB port if it is connected.

2. Change directory to scripts:

    ```bash
    cd ldlidar_stl_ros2/scripts
    ```

3. Make the script executable:

    ```bash
    sudo chmod +x create_udev_rules.sh
    ```

4. Run the script:

    ```bash
    sudo ./create_udev_rules.sh
    ```

## Usage

Source the ROS2 workspace from the root of the workspace:

For bash:

```bash
source install/setup.bash
```

For zsh:

```bash
source install/setup.zsh
```

Launch the driver:

```bash
ros2 launch ldlidar_stl_ros2 ld19.launch.py
```

You could also launch the driver node with rviz:

```bash
ros2 launch ldlidar_stl_ros2 viewer_ld19.launch.py
```

## Launch File Arguments

- `namespace` (string, default: `blitzbot`): The namespace to use for the device.
- `port` (string, default: `/dev/ldlidar`): The port to use for the device.
- `lidar_frame` (string, default: `laser_frame`): The frame id to use for the device.
- `scan_direction` (string, default: `ccw`): The scan direction of the lidar. Options: `ccw` and `cw`.
- `angle_crop` (string, default: `false`): Enables angle cropping for the LiDAR.
- `pub_static_tf` (string, default: `true`): Publishes a static transform between the LiDAR and the robot base.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE)
file for details.
