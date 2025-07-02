import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )

    return LaunchDescription([
    wheeltec_camera,
    launch_ros.actions.Node(
            package='aruco_ros', 
            executable='single', 
            parameters=[
                {'image_is_rectified': True},
                {'marker_size': 0.1},
                {'marker_id': 582},
                {'reference_frame':'camera_link'},
                {'camera_frame': 'camera_link'},
                {'marker_frame': 'aruco_marker_frame'},
                {'corner_refinement':'LINES'}
                ],
       	    remappings=[('/camera_info', '/camera/color/camera_info'),
                    ('/image', '/camera/color/image_raw')],
            output='screen',
            ),]
            
    )
    
    
