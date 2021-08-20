from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    param_node = Node(
        package="cpp_parameters_pkg",
        executable="parameter_node",
        name="custom_parameter_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {"my_param": "earth"}
        ]
    )

    return LaunchDescription(
        [
            param_node,
        ]
    )

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package="cpp_parameters_pkg",
#             executable="parameter_node",
#             name="custom_parameter_node",
#             output="screen",
#             emulate_tty=True,
#             parameters=[
#                 {"my_param": "earth"}
#             ]
#         )
#     ])