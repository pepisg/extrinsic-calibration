# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import  Node
from launch.conditions import UnlessCondition

# add launch path to  import helper functions in this folder
extrinsic_calibration_dir = get_package_share_directory("extrinsic_calibration")
launch_dir = os.path.join(extrinsic_calibration_dir, 'launch')
sys.path.append(launch_dir)
from launch_utils.apriltag_parser import read_apriltag_from_yaml
from launch_utils.camera_parser import read_camera_from_yaml


def generate_launch_description():
    world = os.path.join(extrinsic_calibration_dir, "worlds", "camera_calibration_world.sdf")
    models_dir = os.path.join(extrinsic_calibration_dir, "models")
    config_file = os.path.join(extrinsic_calibration_dir, "config", "world_setup.yaml")

    headless = LaunchConfiguration('headless')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    

    if 'GAZEBO_MODEL_PATH' in os.environ:
        gazebo_model_path = os.environ['GAZEBO_MODEL_PATH'] + \
            os.pathsep + models_dir
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", gazebo_model_path)
    else:
        set_gazebo_model_path_cmd = SetEnvironmentVariable(
            "GAZEBO_MODEL_PATH", models_dir)


    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world, "--verbose"],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], 
        output='both',
        condition=UnlessCondition(headless)
    )


    # Create the launch description and populate
    ld = LaunchDescription()

    # launch configurations
    ld.add_action(declare_simulator_cmd)

    # Set gazebo up to find models properly
    ld.add_action(set_gazebo_model_path_cmd)

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # aprltag spawner
    # for apriltag in read_apriltag_from_yaml(config_file):
    #     ld.add_action(apriltag.get_launch_descriptor())

    for camera in read_camera_from_yaml(config_file):
        ld.add_action(camera.get_launch_descriptor())

    return ld