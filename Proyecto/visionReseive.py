import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge

class VisionReseive(Node):

    def __init__(self):
        super().__init__('robot_bot_control')
        self.subscription = self.create_subscription(
            Image,
            'Vision',
            self.listener_callback,
            20)    
        
        self.bridge = CvBridge()

        self.boundaries = [
	                        ([17, 15, 100], [50, 56, 200]),
	                        ([86, 31, 4], [220, 88, 50]),
	                        ([25, 146, 190], [62, 174, 250]),
	                        ([103, 86, 65], [145, 133, 128])
]

    def listener_callback(self, msg):
        cv_image  = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        ##Detectar formas


        # Convertir imagen a grayscales
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        #blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        #_, threshold = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        edges = cv2.Canny(gray, 100, 200, L2gradient = True)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
  
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Adjust the minimum area threshold as needed
                filtered_contours.append(contour)


        i = 0

        for contour in filtered_contours:
            # here we are ignoring first counter because 
            # findcontour function detects whole image as shape
            if i == 0:
                i = 1
                continue
            
            # cv2.approxPloyDP() function to approximate the shape
            approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)

            # using drawContours() function
            cv2.drawContours(cv_image, [approx], 0, (0, 0, 255), 5)

            # finding center point of shapecircl
            x=0
            y=0
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])

            # putting shape name at center of each shape
            if len(approx) == 3:
                cv2.putText(cv_image, 'Triangle', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 4:
                cv2.putText(cv_image, 'Quadrilateral', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 5:
                cv2.putText(cv_image, 'Pentagon', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            elif len(approx) == 6:
                cv2.putText(cv_image, 'Hexagon', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            else:
                cv2.putText(cv_image, 'circle', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


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