#!/usr/bin/env python3
#
# =======================================================================
#   @file   pimouse_ros2.launch.py
#   @brief
#   @note
#
#   Copyright (C) 2020 Yasushi Oshima (oosmyss@gmail.com)
# =======================================================================

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pimouse_ros2', node_executable='lightsensors', output='screen'),
        launch_ros.actions.Node(
            package='pimouse_ros2', node_executable='motors', output='screen'),
    ])
