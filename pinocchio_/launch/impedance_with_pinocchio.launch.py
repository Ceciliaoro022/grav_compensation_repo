from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    default_urdf_path = '/tmp/kuka_mini_combined.urdf '
    
    urdf_path_arg = DeclareLaunchArgument(
        'urdf_path',
        default_value=default_urdf_path,
    )
      
    tf2_trial = Node(
        package='pinocchio_',
        executable='kinematics',
        name='kinematics',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path')
        }]
    )

    gravity_compensation_pinocchio = Node(
        package='pinocchio_',
        executable='gravity_compensation',
        name='gravity_compensation',
        output='screen',
        parameters=[{
            'urdf_path': LaunchConfiguration('urdf_path')
        }]
    )

    impedance_node_ = Node(
        package='impedance_node',
        executable='impedance_node',
        name='impedance_node',
        output='screen',
    )
    
    return LaunchDescription([
        urdf_path_arg,
        tf2_trial,
        gravity_compensation_pinocchio,
        impedance_node_
    ])