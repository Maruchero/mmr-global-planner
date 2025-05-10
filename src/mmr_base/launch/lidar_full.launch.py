import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource




workspace_path = get_package_share_directory('mmr_base')
sh_path = os.path.join(workspace_path.split("/install")[0],'useful_scripts','lidar_on.sh')
os.system(sh_path)


def generate_launch_description():
    node = IncludeLaunchDescription(
             PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('velodine'),'launch', 'velodyne-all-nodes-VLP32C-launch.py')))


    return LaunchDescription([
        node,

    ])
