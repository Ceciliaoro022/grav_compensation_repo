from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    default_urdf_path = '/home/user/10April_kuka_mini/src/grav_compensation_repo/pinocchio_/urdf/rrbot_1.urdf'
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
    )

    gravity_compensation_pinocchio = Node(
        package='pinocchio_',
        executable='gravity_compensation_2',
        name='gravity_compensation_2',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path')
        }]
    )

    impedance_node_ = Node(
        package='impedance_node',
        executable='impedance_node_free',
        name='impedance_node_free',
        output='screen',
    )
    
    return LaunchDescription([
        urdf_path_arg,
        gravity_compensation_pinocchio,
        impedance_node_
    ])