#!/usr/bin/env python3

# Copyright 2022 Agricultural-Robotics-Bonn
# Copyright 2026 Agroecology Lab Ltd
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# crop_row_nav.launch.py
# ======================
# Derived from:
#   visual-multi-crop-row-navigation
#   https://github.com/Agricultural-Robotics-Bonn/visual-multi-crop-row-navigation
#   Authors: Alireza Ahmadi, Michael Halstead, Chris McCool (Agricultural-Robotics-Bonn)
#   ROS2 port: Agroecology Lab CIC
#   Reference: Ahmadi et al., "Towards Autonomous Crop-Agnostic Visual Navigation
#              in Arable Fields", arXiv:2109.11936, 2021.
#              Ahmadi et al., ICRA 2020.
#
# Adapted for caatinga_vision / Sowbot by Agroecology Lab Ltd
#
# Launches the USB camera (usb_cam_node_exe) AND crop_row_node together in the
# same launch session, so they share one DDS graph. crop_row_node subscribes to
# the camera topic published by usb_cam — they are separate processes, not a
# single node opening the camera directly.
#
# Why usb_cam is launched here:
#   Previously the camera had to be started by hand in a separate terminal
#   (`ros2 run usb_cam usb_cam_node_exe ...`). On loopback-only CycloneDDS a
#   separately-started process can land in a different discovery context, so
#   crop_row_node saw zero publishers and the feed never appeared. Launching
#   both from one launch file guarantees they are in the same session.
#
# QoS note:
#   crop_row_node subscribes to the image topic RELIABLE. usb_cam defaults to
#   the sensor convention (BEST_EFFORT), which will NOT connect to a RELIABLE
#   subscriber. We therefore force usb_cam to publish RELIABLE here via the
#   qos_overrides parameter, so publisher and subscriber match. (The cleaner
#   long-term fix is to make crop_row_node subscribe BEST_EFFORT with
#   qos_profile_sensor_data — see node TODO — but matching at the publisher
#   keeps this launch self-contained.)
#
# Topics published:
#   /devkit/camera/image_raw            Raw camera frames (from usb_cam)
#   /caatinga_vision/row_nav/debug_image  Annotated frames (view in Foxglove)
#   /aoc/conditions/row_offset          Normalised lateral offset [-1,1]
#   /aoc/conditions/row_heading_error   Heading error in radians
#   /aoc/heartbeat/neo_vision           Perception heartbeat for M10
#   /cmd_vel                            Always published; gated by /row_follow/enable service
#
# Usage:
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py video_device:=/dev/video2
#   ros2 launch sowbot_row_follow crop_row_nav.launch.py use_camera:=false   # subscribe only (sim / external cam)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Base params from file — hardware-specific values live here, not in this launch file.
    # Edit caatinga_vision/config/crop_row_params.yaml to set camera_height_m, camera_tilt_deg etc.
    params_file = PathJoinSubstitution(
        [FindPackageShare("sowbot_row_follow"), "config", "crop_row_params.yaml"]
    )

    image_topic = LaunchConfiguration("image_topic")

    return LaunchDescription([

        # ------------------------------------------------------------------
        # Launch arguments
        # ------------------------------------------------------------------
        DeclareLaunchArgument("camera_index", default_value="0"),
        DeclareLaunchArgument(
            "image_topic", default_value="/devkit/camera/image_raw",
            description="Image topic crop_row_node subscribes to and usb_cam publishes. "
                        "Default: /devkit/camera/image_raw. "
                        "Override for sim: /caatinga_vision/row_nav/image_raw",
        ),
        DeclareLaunchArgument(
            "video_device", default_value="/dev/video0",
            description="V4L2 device node for the USB camera.",
        ),
        DeclareLaunchArgument(
            "use_camera", default_value="true",
            description="If true, launch usb_cam here. Set false when the camera is "
                        "provided elsewhere (sim, external bringup, or a remote Neo).",
        ),

        # ------------------------------------------------------------------
        # USB camera — publishes the image topic crop_row_node consumes.
        # Remapped so usb_cam's default /image_raw becomes image_topic.
        # QoS forced RELIABLE to match crop_row_node's subscription.
        # ------------------------------------------------------------------
        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="usb_cam",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_camera")),
            parameters=[{
                "video_device": LaunchConfiguration("video_device"),
                "image_width": 640,
                "image_height": 480,
                "pixel_format": "yuyv",
                "framerate": 30.0,
                "camera_name": "default_cam",
                # Force the published image stream to RELIABLE so it connects to
                # crop_row_node's RELIABLE subscription. Without this, usb_cam
                # publishes BEST_EFFORT and the subscriber silently gets nothing.
                "qos_overrides./devkit/camera/image_raw.publisher.reliability": "reliable",
            }],
            remappings=[
                ("/image_raw", image_topic),
            ],
        ),

        # ------------------------------------------------------------------
        # Crop-row perception / control node
        # ------------------------------------------------------------------
        Node(
            package="sowbot_row_follow",
            executable="crop_row_node",
            name="crop_row_node",
            output="screen",
            parameters=[
                # 1. Load all defaults from the hardware params file
                params_file,
                # 2. Allow launch-arg overrides
                {
                    "camera_index": LaunchConfiguration("camera_index"),
                    "image_topic":  image_topic,
                },
            ],
        ),
    ])
