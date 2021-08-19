#!/usr/bin/env python3

# from typing_extensions import TypeGuard
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    turtle1 = Node(
        package = 'turtlesim',
        executable = 'turtlesim_node',
        name = 'turtle1',
        output = 'screen',
    )

    turtle2 = Node(
        package = 'cpp_a_follow_turtle_pkg',
        executable = 'turtle_spawner',
        name = 'turtle2',
        output = 'screen',
    )

    # turtle_pub = Node(
    #     package = 'cpp_a_follow_turtle_pkg',
    #     executable = 'turtle_pub',
    #     name = 'tf_caster',
    #     output = 'screen',
    # )

    return LaunchDescription([
        turtle1,
        turtle2,
        # turtle_pub,
    ])
