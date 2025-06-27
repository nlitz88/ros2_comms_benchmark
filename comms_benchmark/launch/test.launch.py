from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    return LaunchDescription([
        # Create a container for composable nodes
        ComposableNodeContainer(
            name='a_buncha_nodes',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            output='log',
            # emulate_tty=True,
            log_cmd=True,
            composable_node_descriptions=[
                ComposableNode(
                    package='comms_benchmark',
                    plugin='comms_benchmark::BenchmarkPublisher',
                    name='benchmark_publisher',
                    extra_arguments=[{
                        'use_intra_process_comms': True,
                        'output': 'own_log',
                        'emulate_tty': False,
                        }],
                    parameters=[],
                ),
                # ComposableNode(
                #     package='comms_benchmark',
                #     plugin='comms_benchmark::BenchmarkPublisher',
                #     name='second_benchmark_publisher',
                #     extra_arguments=[{
                #         'use_intra_process_comms': True,
                #         'output': 'own_log',
                #         'emulate_tty': False,
                #         }],
                #     parameters=[],
                #     remappings=[
                #         ('image', 'image2')
                #     ]
                # ),
                ComposableNode(
                    package='comms_benchmark',
                    plugin='comms_benchmark::BenchmarkSubscriber',
                    name='benchmark_subscriber',
                    extra_arguments=[{
                        'use_intra_process_comms': True,
                        'output': 'own_log',
                        'emulate_tty': False,
                        }],
                    parameters=[],
                ),
            ]
        )
    ])