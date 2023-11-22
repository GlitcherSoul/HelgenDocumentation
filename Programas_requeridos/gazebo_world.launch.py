# ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.3}}"
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

package_name = 'gazebo_world_launch'
robot_model='caster_diffbot'
pose = ['0.0', '0.0', '0.0', '0.0']
gz_robot_name = robot_model

#Launch argument
world_file = 'almace.world'

def generate_launch_description():

    pkg_robot_simulation = get_package_share_directory(package_name)

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(pkg_robot_simulation, 'worlds', world_file), ''],
        description='Custom SDF world file')    

    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')

    robot_description_path =  os.path.join( pkg_robot_simulation, "urdf", robot_model + '.xacro', )
    
    robot_description = {"robot_description": xacro.process_file(robot_description_path).toxml()}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join( get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'),
        )
    )

    robot_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", "-entity", gz_robot_name, 
            "-x", pose[0], "-y", pose[1], "-z", pose[2], "-Y", pose[3] ])
    
    red = Node(
        package="gazebo_world_launch",
        executable="red.py",
        output="screen",
    )
    
    rqt_view = Node(
        package="rqt_image_view",
        executable="rqt_image_view",
        output="screen",
    )
    
    rqt_move = Node(
        package="rqt_robot_steering",
        executable="rqt_robot_steering",
        output="screen",
    )
    
    data = Node(
        package="gazebo_world_launch",
        executable="data.py",
        output="screen",
    )

    print("STARTING ALL NODES ...")

    return LaunchDescription([
        world_arg,
        simu_time,
        gazebo_node,
        robot_state_publisher_node,
        robot_spawner,
        rqt_view,
        rqt_move,
        red,
        #data,
    ])
