import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
import easyocr

class VisionReseive(Node):

    def __init__(self):
        super().__init__('robot_bot_control')
        self.subscription = self.create_subscription(
            Image,
            'Vision',
            self.listener_callback,
            20)    
        
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['es'])


    def listener_callback(self, msg):
        cv_image_original  = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        

        ##Detectar formas
        cv_shape= cv_image_original[3:180, 250:500]

        # Convertir imagen a grayscales
        gray = cv2.cvtColor(cv_shape, cv2.COLOR_BGR2GRAY)
        #_, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

        edges = cv2.Canny(gray, 127, 200, L2gradient = True)

        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
  
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
            approx = cv2.approxPolyDP(contour, 0.1 * cv2.arcLength(contour, True), True)

            # using drawContours() function
            cv2.drawContours(cv_shape, [approx], 0, (0, 0, 255), 2)

            # finding center point of shapecircl
            x=0
            y=0
            M = cv2.moments(contour)
            if M['m00'] != 0.0:
                x = int(M['m10']/M['m00'])
                y = int(M['m01']/M['m00'])

            # putting shape name at center of each shape
            if len(approx) == 3:
                cv2.putText(cv_shape, 'TRIANGULO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            elif len(approx) == 4:
                cv2.putText(cv_shape, 'CUADRADO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            elif len(approx) == 5:
                cv2.putText(cv_shape, 'PENTAGONO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            elif len(approx) == 6:
                cv2.putText(cv_shape, 'HEXAGONO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            else:
                cv2.putText(cv_shape, 'CIRCULO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        #Deteccion palabras

        cv_palabras = cv_image_original[165:280, 240:510]

        results = self.reader.readtext(cv_palabras)

        for res in results:
            top_left = tuple(res[0][0]) # top left coordinates as tuple
            bottom_right = tuple(res[0][2]) # bottom right coordinates as tuple
            # draw rectangle on image
            cv2.rectangle(cv_palabras, top_left, bottom_right, (0, 255, 0), 2) 
            # write recognized text on image (top_left) minus 10 pixel on y
            cv2.putText(cv_palabras, res[1], (top_left[0], top_left[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)


        #Deteccion color
        cv_colores = cv_image_original[310:, 240:490]

        hsv_frame = cv2.cvtColor(cv_colores, cv2.COLOR_BGR2HSV)

        height, width, _ = hsv_frame.shape
        cx = int(width / 2)
        cy = int(height / 2)
        # Pick pixel value
        pixel_center = hsv_frame[cy, cx]
        hue_value = pixel_center[0]


        color = "Undefined"
        if hue_value < 5:
            color = "RED"
        elif hue_value < 22:
            color = "ORANGE"
        elif hue_value < 33:
            color = "YELLOW"
        elif hue_value < 78:
            color = "GREEN"
        elif hue_value < 131:
            color = "BLUE"
        elif hue_value < 170:
            color = "VIOLET"
        else:
            color = "RED"
        
        cv2.putText(cv_colores, color, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.rectangle(cv_colores, (0, 0), (width, height), (255, 0, 0), 2) 

        #Mostrar todo
        cv2.imshow('Imagen final', cv_image_original)
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