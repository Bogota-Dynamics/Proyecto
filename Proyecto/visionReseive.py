import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VisionReseive(Node):

    def __init__(self):
        super().__init__('robot_bot_control')
        self.subscription = self.create_subscription(
            Image,
            'Vision',
            self.listener_callback,
            10)    
        
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image  = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow('Hola', cv_image)
        cv2.waitKey(3)
        self.get_logger().info('Received an image')

        


def main(args=None):
    rclpy.init(args=args)
    interface = VisionReseive()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()