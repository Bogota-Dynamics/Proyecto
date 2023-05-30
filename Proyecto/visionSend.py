import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np

class VisionSend(Node):
      def __init__(self):
         super().__init__('vision_send')
         self.publisher_ = self.create_publisher(Image, 'Vision', 20)

         timer_period = 0.1  # seconds
         self.timer = self.create_timer(timer_period, self.timer_callback)
         
         self.cap = cv2.VideoCapture(0)
         self.bridge = CvBridge()

         if not self.cap.isOpened():
            self.get_logger().error("Unable to open camera.")
            exit(1)

      def timer_callback(self):
         ret, frame = self.cap.read()

         if not ret:
            self.get_logger().error("Failed to capture frame from camera.")
            return
         
         image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
         self.publisher_.publish(image_msg)
         self.get_logger().info('Publishing an image')

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = VisionSend()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
   main()