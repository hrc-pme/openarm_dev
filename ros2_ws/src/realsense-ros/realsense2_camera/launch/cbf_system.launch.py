import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    """
    主啟動腳本，一次性啟動整個 CBF 避障系統。
    1. 啟動頂視 RealSense 相機。
    2. 啟動 C++ 點雲預處理伺服器。
    3. 啟動 Python CBF 主控制節點。
    """
    # 找到 realsense2_camera 套件的 launch 文件夾路徑
    realsense_launch_dir = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch')

    # 我們的頂視相機啟動腳本
    top_camera_launch_file = os.path.join(realsense_launch_dir, 'rs_launch_top.py')

    return LaunchDescription([
        # 1. 包含並啟動 RealSense 相機
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(top_camera_launch_file)
        ),

        # 2. 啟動 C++ 點雲預處理伺服器
        Node(
            package='openarm_cbf',
            executable='koch_vamp_server',
            name='pointcloud_preprocessor_node',
            output='screen',
            emulate_tty=True,
        ),

        # 3. 啟動 Python CBF 主控制節點
        Node(
            package='openarm_cbf',
            executable='open_CBF.py',
            name='cbf_control_node',
            output='screen',
            emulate_tty=True,
        ),
    ])