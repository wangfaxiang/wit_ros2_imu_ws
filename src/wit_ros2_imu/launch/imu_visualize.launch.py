#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键启动 IMU 驱动 + 三维姿态可视化 + RViz2。

用法：
    ros2 launch wit_ros2_imu imu_visualize.launch.py
    ros2 launch wit_ros2_imu imu_visualize.launch.py start_driver:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkgShare = get_package_share_directory("wit_ros2_imu")
    defaultRvizConfig = os.path.join(pkgShare, "rviz", "imu_view.rviz")

    startDriverArg = DeclareLaunchArgument(
        "start_driver",
        default_value="true",
        description="是否启动 IMU 串口驱动节点",
    )
    serialPortArg = DeclareLaunchArgument(
        "port",
        default_value="/dev/imu_usb",
        description="IMU 串口设备",
    )
    baudrateArg = DeclareLaunchArgument(
        "baudrate",
        default_value="921600",
        description="串口波特率",
    )
    imuTopicArg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/data",
        description="要可视化的 IMU 话题",
    )
    useRvizArg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="是否启动 RViz2",
    )
    rvizConfigArg = DeclareLaunchArgument(
        "rviz_config",
        default_value=defaultRvizConfig,
        description="RViz2 配置文件路径",
    )

    driverNode = Node(
        package="wit_ros2_imu",
        executable="wit_ros2_imu",
        name="imuDriverNode",
        output="screen",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
        }],
        condition=IfCondition(LaunchConfiguration("start_driver")),
    )

    visualizerNode = Node(
        package="wit_ros2_imu",
        executable="imu_visualizer",
        name="imu_visualizer_node",
        output="screen",
        parameters=[{
            "imu_topic": LaunchConfiguration("imu_topic"),
        }],
    )

    rvizNode = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return LaunchDescription([
        startDriverArg,
        serialPortArg,
        baudrateArg,
        imuTopicArg,
        useRvizArg,
        rvizConfigArg,
        driverNode,
        visualizerNode,
        rvizNode,
    ])
