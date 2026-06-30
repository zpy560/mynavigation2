from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
                # Velocity command (ROS2 -> IGN)
                '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                # Odometry (IGN -> ROS2)
                '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                # TF (IGN -> ROS2)
                '/odom/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                # Clock (IGN -> ROS2)
                '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                # Joint states (IGN -> ROS2)
                '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # Lidar (IGN -> ROS2)
                '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                # IMU (IGN -> ROS2)
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                # Camera (IGN -> ROS2)
                '/camera/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                '/camera/rgb/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                ],
        remappings=[
            ("/odom/tf", "tf"),
        ],
        output='screen'
    )

    return LaunchDescription([
        bridge,
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
    ])
