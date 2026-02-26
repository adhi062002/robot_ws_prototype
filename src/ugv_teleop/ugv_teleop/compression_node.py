import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np


class ImageTrackRepublisher(Node):

    def __init__(self):
        super().__init__('image_track_republisher')

        # Subscribe to raw image
        self.subscription = self.create_subscription(
            Image,
            '/image_track',
            self.image_callback,
            10
        )

        # Publisher (still called /track/compressed but type = Image)
        self.publisher = self.create_publisher(
            CompressedImage,
            '/image_track/compressed',
            10
        )

        self.get_logger().info("Republishing /image_track → /track/compressed (Image msg)")


    def image_callback(self, msg: Image):
        # Convert ROS Image → OpenCV
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3)
            )
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Compress to JPEG
        ok, encoded = cv2.imencode('.jpeg', img, [cv2.IMWRITE_JPEG_QUALITY, 95])
        if not ok:
            self.get_logger().error("JPEG encoding failed")
            return

        # Prepare ROS Image msg
        out = CompressedImage()
        out.header = msg.header
        out.format = 'jpeg'
        out.data = encoded.tobytes()

        # Publish Image message
        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ImageTrackRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
