import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    intra_process = False
    # the more endpoints we have, the more likely the problem happens.
    num_publishers = 10
    num_subscriptions = 32
    string_publish_rate = 100.0

    single_process_nodes = []
    for i in range(num_publishers):
        single_process_nodes.append(
            Node(
                name=f'publisher_{i:05}',
                package='prover_rclcpp',
                executable='composable_node_publisher',
                parameters=[{'publish_rate': string_publish_rate}],
                ros_arguments=['--log-level', 'info'],
            )
        )

    components = []
    for i in range(num_subscriptions):
        components.append(
            ComposableNode(
                name=f'subscription_{i:05}',
                package='prover_rclcpp',
                plugin='prover_rclcpp::ComposableNodeSubscription',
                extra_arguments=[{'use_intra_process_comms': intra_process}],
            )
        )

    component_container = ComposableNodeContainer(
        namespace='',
        name='component_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        ros_arguments=['--log-level', 'info'],
        composable_node_descriptions=components
    )

    return launch.LaunchDescription([
        *single_process_nodes, component_container
    ])
