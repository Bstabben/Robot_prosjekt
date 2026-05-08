import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)

        device_id = self.get_parameter('device_id').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('frame_width').value
        height = self.get_parameter('frame_height').value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, 'camera/image_raw', 10)

        self._cap = cv2.VideoCapture(device_id)
        if not self._cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device_id}')
            raise RuntimeError(f'Cannot open camera device {device_id}')

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        period = 1.0 / frame_rate
        self._timer = self.create_timer(period, self._timer_callback)
        self.get_logger().info(
            f'Camera node started: device={device_id}, '
            f'{int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
            f'{int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {frame_rate} Hz'
        )

    def _timer_callback(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame')
            return
        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
