from os.path import join
 
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
 
# CONSTANTS
PACKAGE_NAME = "ldlidar_stl_ros2"
 
 
def generate_launch_description() -> LaunchDescription:
    """
    Launch file to start the LiDAR node with RViz2.
 
    Returns
    -------
    LaunchDescription
        The launch description.
 
    """
 
    # Get the package share directory
    pkg_share = FindPackageShare(package=PACKAGE_NAME).find(PACKAGE_NAME)
 
    rviz2_config = join(pkg_share, "rviz2", "ldlidar.rviz")
 
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_show_ld19",
        arguments=["-d", rviz2_config],
        output="screen",
    )
 
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pkg_share, "/launch/ld19.launch.py"])
    )
 
    return LaunchDescription([lidar_launch, rviz2_node])