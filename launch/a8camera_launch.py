from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


_PACKAGE_NAME = 'a8camera'
_CAMERA_CONTROLLER_NAME = 'a8mini_controller_node'
_CAMERA_STREAM_NODE_NAME = 'a8mini_stream_node'
_CAMERA_CONTROLLER_EXECUTABLE = 'controller'
_CAMERA_STREAM_EXECUTABLE = 'stream'

def generate_launch_description() -> LaunchDescription:
    system_id = LaunchConfiguration('system_id')
    camera_server_ip = LaunchConfiguration('camera_server_ip')

    return LaunchDescription([
        Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_CONTROLLER_EXECUTABLE,
            name=_CAMERA_CONTROLLER_NAME,
            parameters=[{
                'system_id': system_id,
                'camera_server_ip': camera_server_ip
            }]),

        Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_STREAM_EXECUTABLE,
            name=_CAMERA_STREAM_NODE_NAME,
            parameters=[{
                'system_id': system_id,
                'camera_server_ip': camera_server_ip
            }])
  ])