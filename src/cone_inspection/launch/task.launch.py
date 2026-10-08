import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('cone_inspection')
    xacro_file = os.path.join(pkg, 'urdf', 'ur3_cell.urdf.xacro')
    rviz_file = os.path.join(pkg, 'rviz', 'demo.rviz')

    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file, ' ur_type:=ur3']),
        value_type=str,
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        # task_node now publishes /joint_states itself -- no joint_state_publisher_gui here.
        # It needs its OWN plain-urdf copy (see note below) since it loads the model
        # with the roboticstoolbox URDF parser, not via the xacro Command above.
        Node(
            package='cone_inspection',
            executable='task_node',
            parameters=[{'urdf_path': '/tmp/ur3_cell.urdf'}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
        ),
    ])