import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class RGBDViewer(Node):
    def __init__(self):
        super().__init__('rgbd_viewer')
        self.bridge = CvBridge()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.rgb_callback,
            qos_profile_sensor_data)
        
        # self.depth_sub = self.create_subscription(
        #     Image,
        #     '/camera/camera/depth/image_rect_raw',
        #     self.depth_callback,
        #     10)

        self.rgb_image = None
        self.depth_image = None

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.show_images()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.show_images()

    def show_images(self):
        if self.rgb_image is not None or self.depth_image is not None:
            cv2.imshow("RGB Image", self.rgb_image)
            
            # Normalize depth for visualization
            # depth_vis = cv2.normalize(self.depth_image, None, 0, 255, cv2.NORM_MINMAX)
            # depth_vis = depth_vis.astype('uint8')
            # cv2.imshow("Depth Image", depth_vis)

            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = RGBDViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
