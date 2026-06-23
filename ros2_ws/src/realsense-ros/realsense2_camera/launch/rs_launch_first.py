# Copyright 2023 Intel Corporation. All Rights Reserved.
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

"""Launch realsense2_camera node for camera2 (241222061237)."""
import os
import yaml
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


configurable_parameters = [{'name': 'camera_name',                  'default': 'camera2', 'description': 'camera unique name'},
                           {'name': 'camera_namespace',             'default': 'camera', 'description': 'namespace for camera'},
                           {'name': 'serial_no',                    'default': "'241222061237'", 'description': 'choose device by serial number'},
                           {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
                           {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
                           {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
                           {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
                           {'name': 'initial_reset',                'default': 'true', 'description': "''"},
                           {'name': 'accelerate_gpu_with_glsl',     'default': "false", 'description': 'enable GPU acceleration with GLSL'},
                           {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
                           {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
                           {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
                           {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
                           {'name': 'rgb_camera.color_profile',     'default': '640,480,15', 'description': 'color stream profile'},
                           {'name': 'rgb_camera.color_format',      'default': 'RGB8', 'description': 'color stream format'},
                           {'name': 'rgb_camera.enable_auto_exposure', 'default': 'true', 'description': 'enable/disable auto exposure for color image'},
                           {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
                           {'name': 'enable_infra',                 'default': 'false', 'description': 'enable infra0 stream'},
                           {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
                           {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
                           {'name': 'depth_module.depth_profile',   'default': '640,480,15', 'description': 'depth stream profile'},
                           {'name': 'depth_module.depth_format',    'default': 'Z16', 'description': 'depth stream format'},
                           {'name': 'enable_sync',                  'default': 'false', 'description': "'enable sync mode'"},
                           {'name': 'pointcloud.enable',            'default': 'true', 'description': ''},
                           {'name': 'align_depth.enable',           'default': 'true', 'description': 'enable align depth filter'},
                           {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
                           {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},
                          ]

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def yaml_to_dict(path_to_yaml):
    with open(path_to_yaml, "r") as f:
        return yaml.load(f, Loader=yaml.SafeLoader)

def launch_setup(context, params, param_name_suffix=''):
    _config_file = LaunchConfiguration('config_file' + param_name_suffix).perform(context)
    params_from_file = {} if _config_file == "''" else yaml_to_dict(_config_file)

    return [
        launch_ros.actions.Node(
            package='realsense2_camera',
            namespace=LaunchConfiguration('camera_namespace' + param_name_suffix),
            name=LaunchConfiguration('camera_name' + param_name_suffix),
            executable='realsense2_camera_node',
            parameters=[params, params_from_file],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level' + param_name_suffix)],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    return LaunchDescription(declare_configurable_parameters(configurable_parameters) + [
        OpaqueFunction(function=launch_setup, kwargs = {'params' : set_configurable_parameters(configurable_parameters)})
    ])