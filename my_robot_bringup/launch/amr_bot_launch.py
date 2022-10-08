from launch import LaunchDescription
import launch_ros
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # pkg_share = launch_ros.substitutions.FindPackageShare(package='pumpkin_bot_description').find('pumpkin_bot_description')
    # default_model_path = os.path.join(pkg_share, 'src/description/pumpkin_bot_description.urdf')
    # default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    # serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    # frame_id = LaunchConfiguration('frame_id', default='laser')
    # inverted = LaunchConfiguration('inverted', default='false')
    # angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    # robot_localization_file_path = os.path.join(pkg_share, 'config/ekf.yaml') 

    # robot_ui_node = launch_ros.actions.Node(
    #     package='pumpkin_py_pkg',
    #     executable='robot_ui',
    #     name='robot_ui',
    #     output='screen'
    # )

    # odometry_node = launch_ros.actions.Node(
    #     package='pumpkin_cpp_pkg',
    #     executable='odometry',
    #     name='odometry'
    # )

    # pumpkin_clock_node = launch_ros.actions.Node(
    #     package='pumpkin_cpp_pkg',
    #     executable='pumpkin_clock',
    #     name='pumpkin_clock',
    #     parameters = [
    #         {"publish_frequency": 100.0}
    #     ]
    # )

    ctre_node = launch_ros.actions.Node(
        package='my_cpp_pkg',
        executable='ctre',
        name='ctre'
    )

    usb_cam_node = launch_ros.actions.Node(
        package='image_tools',
        executable='cam2image',
        name='cam2image',
        parameters = [
            {"width": 640},
            {"height": 480}
        ]
    )

    # usb_cam_node = launch_ros.actions.Node(
    #     package='usb_cam',
    #     executable='usb_cam_node_exe',
    #     name='usb_cam_node_exe',
    #     parameters = [
    #         {"params-file": '/home/ubuntu/ros2_ws/src/camera_config/ms_lifecam/params.yaml'}
    #     ]
    # )

    # robot_state_publisher_node = launch_ros.actions.Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    # )

    # rviz_node = launch_ros.actions.Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', LaunchConfiguration('rvizconfig')],
    # )

    # robot_localization_node = launch_ros.actions.Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[robot_localization_file_path, 
    #     {'use_sim_time': False}])

    return LaunchDescription([
        # DeclareLaunchArgument(name='model', default_value=default_model_path,
        #                       description='Absolute path to robot urdf file'),
        # DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        #                       description='Absolute path to rviz config file'),
        # DeclareLaunchArgument(
        #     'serial_port',
        #     default_value=serial_port,
        #     description='Specifying usb port to connected lidar'),

        # DeclareLaunchArgument(
        #     'serial_baudrate',
        #     default_value=serial_baudrate,
        #     description='Specifying usb port baudrate to connected lidar'),

        # DeclareLaunchArgument(
        #     'frame_id',
        #     default_value=frame_id,
        #     description='Specifying frame_id of lidar'),

        # DeclareLaunchArgument(
        #     'inverted',
        #     default_value=inverted,
        #     description='Specifying whether or not to invert scan data'),

        # DeclareLaunchArgument(
        #     'angle_compensate',
        #     default_value=angle_compensate,
        #     description='Specifying whether or not to enable angle_compensate of scan data'),

        # pumpkin_clock_node,

        #launch_ros.actions.Node(
        #     package='rplidar_ros',
        #     executable='rplidar_composition',
        #     name='rplidar_composition',
        #     parameters=[{'serial_port': serial_port, 
        #                  'serial_baudrate': serial_baudrate, 
        #                  'frame_id': frame_id,
        #                  'inverted': inverted, 
        #                  'angle_compensate': angle_compensate}],
        #     output='screen'),

        # robot_ui_node,
        # robot_state_publisher_node,
        # robot_localization_node,
        # odometry_node,
        ctre_node,
        #usb_cam_node
        #rviz_node
    ])