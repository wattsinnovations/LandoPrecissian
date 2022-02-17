from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    aruco_node = Node(
        package="ros2_aruco",
        executable="aruco_node",
        remappings=[("/arucotag/image", "/camera/color/image_raw"), ("/arucotag/camera_info", "/camera/color/camera_info")],
    )
    lando_node = Node(
        package="lando",
        executable="app"
    )
    ld.add_action(aruco_node)
    ld.add_action(lando_node)
    return ld
