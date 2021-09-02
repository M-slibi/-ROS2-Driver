import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition, LaunchConfigurationNotEquals

parameters_file_name = "default.yaml"


def generate_launch_description():
    # get current path and go one level up
    gad_dir = get_package_share_directory("oxts_gad")

    gad_param_path = os.path.join(gad_dir, "config", parameters_file_name)
    with open(gad_param_path, "r") as f:
        gad_params = yaml.safe_load(f)["oxts_gad"]["ros__parameters"]

    # declare launch arguments (this exposes the argument
    # to IncludeLaunchDescriptionand to the command line)
    declare_use_sim_time = DeclareLaunchArgument("use_sim_time", default_value="False")

    oxts_gad_node = Node(
        package="oxts_gad",
        executable="oxts_gad",
        name="oxts_gad",
        output="screen",
        parameters=[
            gad_params,
        ],
    )

    # create launch descroption and populate
    ld = LaunchDescription()
    # launch options
    # (none currently)
    # launch nodes
    ld.add_action(oxts_gad_node)

    return ld
