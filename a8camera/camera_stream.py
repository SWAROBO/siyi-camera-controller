import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

_CAM_NODE_NAME = "a8_mini_stream_node"
_CAM_PUB_TOPIC = "a8_mini/camera_stream"
_CAM_FRAME_ID = "a8_mini"
_QUEUE_SIZE = 1
_PUBLISH_PERIOD_SEC = 0.05


class CameraStreamNode(Node):
    def __init__(self, node_name: str = _CAM_NODE_NAME, pub_period: float = _PUBLISH_PERIOD_SEC) -> None:
        super().__init__(node_name)

        camera_server_ip = self.declare_parameter('camera_server_ip', "192.168.144.25").value
        stream_port = self.declare_parameter('stream_port', 8554).value

        gst_str = (
            f"rtspsrc location=rtsp://{camera_server_ip}:{stream_port}/main.264 latency=0 ! "
            f"rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! "
            f"appsink drop=1 sync=false max-buffers=1"
        )

        self.capture = cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            self.get_logger().error("Failed to open camera stream.")
            raise RuntimeError("Camera open failed")

        self.bridge = CvBridge()

        # define video feed publish topic
        self.get_logger().info(f"topic: __ns/{_CAM_PUB_TOPIC}")
        self.publisher = self.create_publisher(Image, _CAM_PUB_TOPIC, _QUEUE_SIZE)

        # define publishing frequency and callback function
        self.timer_ = self.create_timer(pub_period, self.capture_image_callback)
        self.i = 0

    def capture_image_callback(self) -> None:
        """
        Captures an image from the camera via RTSP and publishes it as a ROS Image message.
        """
        ret, frame = self.capture.read()
        if not ret or frame is None:
            self.get_logger().warn("Failed to capture frame from camera stream.")
            return

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.frame_id = _CAM_FRAME_ID
            msg.header.stamp = Node.get_clock(self).now().to_msg()

            self.publisher.publish(msg)
            self.i += 1

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def destroy_node(self):
        self.capture.release()
        self.get_logger().info('Streaming Node being destroyed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraStreamNode()

    rclpy.spin(camera_publisher)

    camera_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
