from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Parameters for RTAB-Map
    parameters = [{
        'frame_id': 'camera_link',
        'subscribe_depth': True,
        'subscribe_odom_info': True,
        'approx_sync': True
    }]

    # Topic remappings
    remappings = [
        ('rgb/image', '/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/color/camera_info'),
        ('depth/image', '/camera/depth/image_raw')
    ]

    return LaunchDescription([
        # Optical rotation for the camera
        Node(
            package='tf2_ros', executable='static_transform_publisher', output='screen',
            arguments=["0", "0", "0", "-1.57", "0", "-1.57", "camera_link", "camera_rgb"]
        ),

        # RTAB-Map Odometry
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"
        ),

        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d'],
            namespace="rtabmap"
        ),

        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings,
            namespace="rtabmap"
        ),
    ])

if __name__ == '__main__':
    from launch import LaunchService

    ls = LaunchService()
    ls.include_launch_description(generate_launch_description())
    ls.run()

