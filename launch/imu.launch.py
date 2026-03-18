from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='imu_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='imu_demo',
                plugin='ImuNode',
                name='bmi088_node',
            ),
        ],
        output='screen',
    )

    complementary_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        remappings=[
            ('imu/data_raw', 'imu/data_calibrated'),
        ],
        parameters=[{'publish_debug_topics': True}],
        output='screen',
    )

    return LaunchDescription([container, complementary_filter])
