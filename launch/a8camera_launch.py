from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


_PACKAGE_NAME = 'a8camera'
_CAMERA_CONTROLLER_NAME = 'a8mini_controller_node'
_CAMERA_STREAM_NODE_NAME = 'a8mini_stream_node'
_CAMERA_CONTROLLER_EXECUTABLE = 'controller'
_CAMERA_STREAM_EXECUTABLE = 'stream'

def generate_launch_description() -> LaunchDescription:
    camera_server_ip_arg = DeclareLaunchArgument(
        'camera_server_ip',
        default_value='10.41.10.3',
        description='System ID to be passed to nodes'
    )

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=''
    )

    return LaunchDescription([
        camera_server_ip_arg,
        namespace_arg,

        Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_CONTROLLER_EXECUTABLE,
            name=_CAMERA_CONTROLLER_NAME,
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'camera_server_ip': LaunchConfiguration('camera_server_ip')
            }]),

        Node(
            package=_PACKAGE_NAME,
            executable=_CAMERA_STREAM_EXECUTABLE,
            name=_CAMERA_STREAM_NODE_NAME,
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'camera_server_ip': LaunchConfiguration('camera_server_ip')
            }])
  ])