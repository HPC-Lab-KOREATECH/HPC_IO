from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    print("bringup start!!!")
    
    #ugv_script_path = "/home/nvidia/ros2_ws/src/HPC_IO/ugv_sdk/scripts"
    os.system('cd /home/nvidia/ros2_ws/src/HPC_IO/ugv_sdk/scripts && bash bringup_can2usb_500k.bash')

    model_name = 'tracer_v1.xacro'
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("hpc_io_description"), "urdf", model_name]
        ),
    ])

    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                                description='Flag to enable joint_state_publisher_gui')

    #base_path = os.path.realpath(get_package_share_directory('package')) # also tried without realpath

    hpc_io_bringup_dir = get_package_share_directory('hpc_io_bringup')
    rviz_path = hpc_io_bringup_dir+'/rviz/hpc_io_bringup.rviz'
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content
        }]
    )

    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d' + str(rviz_path)],
    )


    tracer_base_dir = get_package_share_directory('tracer_base')
    tracer_base_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                tracer_base_dir + '/launch/tracer_base.launch.py')
                )

    velodyne_driver_dir = get_package_share_directory('velodyne_driver')
    velodyne_driver_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                velodyne_driver_dir + '/launch/velodyne_driver_node-VLP16-launch.py')
                )

    velodyne_pointcloud_dir = get_package_share_directory('velodyne_pointcloud')
    velodyne_pointcloud_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                velodyne_pointcloud_dir + '/launch/velodyne_convert_node-VLP16-composed-launch.py')
                )
                
    velodyne_laserscan_dir = get_package_share_directory('velodyne_laserscan')
    velodyne_laserscan_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                velodyne_laserscan_dir + '/launch/velodyne_laserscan_node-composed-launch.py')
                )           

    realsense2_dir = get_package_share_directory('realsense2_camera')
    realsense2_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                realsense2_dir + '/launch/rs_launch.py')
                )
                #hy
    return LaunchDescription([
        gui_arg,
        # model_arg,
        # rviz_arg,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        tracer_base_launch,
        velodyne_driver_launch,
        velodyne_pointcloud_launch,
        velodyne_laserscan_launch,
        realsense2_launch,
        rviz_node,
    ])
