from launch_ros.actions import Node
 
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
 
 
def launch_setup(context: LaunchContext) -> list[Node]:
    """
    Launch setup function.
 
    Parameters
    ----------
    context : LaunchContext
        The launch context.
 
    Returns
    -------
    list[Node]
        The list of nodes to launch.
 
    """
    # Get the launch configuration variables
    namespace = LaunchConfiguration("namespace").perform(context)
    lidar_frame = LaunchConfiguration("lidar_frame").perform(context)
    port = LaunchConfiguration("port").perform(context)
    scan_direction = LaunchConfiguration("scan_direction").perform(context) == "ccw"
    angle_crop = LaunchConfiguration("angle_crop").perform(context) == "true"
    pub_static_tf = LaunchConfiguration("pub_static_tf").perform(context)
 
    lidar_node = Node(
        package="ldlidar_stl_ros2",
        executable="ldlidar_stl_ros2_node",
        name="LD19",
        output="screen",
        namespace=namespace,
        parameters=[
            {"product_name": "LDLiDAR_LD19"},
            {"topic_name": "scan"},
            {"frame_id": lidar_frame},
            {"port_name": port},
            {"port_baudrate": 230400},
            {"laser_scan_dir": scan_direction},
            {"enable_angle_crop_func": angle_crop},
            {"angle_crop_min": 135.0},
            {"angle_crop_max": 225.0},
        ],
    )
 
    # base_link to laser_frame tf node
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_base_laser_ld19",
        condition=IfCondition(pub_static_tf),
        arguments=["0.055", "0", "0.08", "0", "0", "0", "base_link", lidar_frame],
    )
 
    return [lidar_node, static_tf_node]
 
 
def generate_launch_description() -> LaunchDescription:
    """
    Launch file to start the LiDAR node.
 
    Returns
    -------
    LaunchDescription
        The launch description.
 
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="blitzbot",
            description="Namespace for the LiDAR node",
        ),
        DeclareLaunchArgument(
            "lidar_frame",
            default_value="laser_frame",
            description="Frame ID for the LiDAR",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ldlidar",
            description="Serial port the LiDAR is connected to",
        ),
        DeclareLaunchArgument(
            "scan_direction",
            default_value="ccw",
            choices=["cw", "ccw"],
            description="Scan direction of the LiDAR",
        ),
        DeclareLaunchArgument(
            "angle_crop",
            default_value="false",
            choices=["true", "false"],
            description="Enable angle cropping for the LiDAR",
        ),
        DeclareLaunchArgument(
            "pub_static_tf",
            default_value="true",
            choices=["true", "false"],
            description="Publish static transform between the LiDAR and the robot base",
        ),
        OpaqueFunction(function=launch_setup),
    ])