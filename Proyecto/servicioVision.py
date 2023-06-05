import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_msgs.msg import Banner
from my_msgs.srv import StartPerceptionTest
from cv_bridge import CvBridge
import cv2
import easyocr
from os.path import abspath
from geometry_msgs.msg import Twist
import time
import threading

class ServicioVision(Node):

    def __init__(self):
        super().__init__('persepcion')
        # crear un servicio que recree los movimientos guardados en un archivo txt
        self.service = self.create_service(StartPerceptionTest, '/group_7/start_perception_test_srv', self.persepction_test_callback)

        #Crear un suscriptor para poder obtener la imagen del robot
        self.subscription = self.create_subscription(Image, 'Vision', self.listener_callback,20)    
        self.imagen = 0
        self.bridge = CvBridge()
        self.reader = easyocr.Reader(['es'])

        #Crear un publicador para llegar a las zonas de vision
        self.publisher_ = self.create_publisher(Twist, 'robot_cmdVel', 10)

        #Crear publicador para resultado banner
        self.publisher2 = self.create_publisher(Banner, '/vision/banner_group_7', 10)

    def listener_callback(self, msg):
        self.imagen  = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def service_method_thread(self, request):
        #Recibir el id de los banners que toca realizar la vision
        banner_a = request.banner_a
        banner_b = request.banner_b

        #Ir al primero
        self.get_logger().info('Navegando a banner: ' + str(banner_a))
        self.recrear_recorrido(str(banner_a))


        #Dar tiempo para procesar la imagen antes de ir al siguiente 
        self.get_logger().info('Procesando imagen')
        time.sleep(2)
        newImage1 = self.imagen.copy()
        status = cv2.imwrite('src/Proyecto/fotos/primera.jpg', newImage1) 
        figura1, palabra1, color1 = self.persepcion(newImage1)
        ban1 = Banner()
        ban1.banner = banner_a
        ban1.figure = figura1
        ban1.word = palabra1
        ban1.color = color1

        self.publisher2.publish(ban1)


        #Ir al segundo
        self.get_logger().info('Navegando a banner: ' + str(banner_b))
        nombre = str(banner_a)+str(banner_b)
        self.recrear_recorrido(nombre)
    

        #Dar tiempo para procesar la imagen antes de ir al siguiente 
        self.get_logger().info('Procesando imagen')
        time.sleep(2)
        newImage2 = self.imagen.copy()
        figura2, palabra2, color2 = self.persepcion(newImage2)
        ban2 = Banner()
        ban2.banner = banner_b
        ban2.figure = figura2
        ban2.word = palabra2
        ban2.color = color2

        self.publisher2.publish(ban2)

        a = "Resultados banner 1, figura: " + figura1 + " , palabra: " + palabra1 + " , color: " + color1 + ". Resultados banner 2, figura: " + figura2 + " , palabra: " + palabra2 + " , color: " + color2
        self.get_logger().info(a)
        self.get_logger().info('Terminado')
    


    def persepction_test_callback(self, request, response):
        thread = threading.Thread(target=lambda : self.service_method_thread(request))
        thread.start()
        response.answer = "Finalizado"

        return response
        


    def recrear_recorrido(self, file):

        # leer el archivo y publicar los movimientos
         # leer el archivo y publicar los movimientos
        filename = 'src/Proyecto/motion/' + file + '.txt'
        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        self.publisher_.publish(msg)   

        msg_viejo = 0   

        with open(filename, 'r') as f:
            linear = 5.0
            angular = 5.0
            # 2. Las siguientes lineas son los movimientos
            #    TriggerR: adelante
            #    TriggerL: atras
            #    Izquierda: izquierda
            #    Derecha: derecha
            #    QUIETO: pausa, separada por '=' del tiempo de pausa
            for line in f:
                start = time.time()
                line = line.strip()
                msg = Twist()
                if line == 'TriggerR':
                    msg.linear.x = linear
                elif line == 'TriggerL':
                    msg.linear.x = -linear
                elif line == 'Izquierda':
                    msg.angular.z = angular
                elif line == 'Derecha':
                    msg.angular.z = -angular
                elif line == 'QUIETO':
                    msg.linear.x=0.0
                    msg.angular.z=0.0

                # Los mensajes se publican cada 0.04 segundos aproximadamente
                time.sleep(0.05)
                if (msg_viejo != msg):
                    self.publisher_.publish(msg)
                    msg_viejo = msg

        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        self.publisher_.publish(msg)        
        # retornar el path global del archivo


    def persepcion(self, cv_image_original):

        res_forma = "no se"
        res_palabra = "no se"
        color = "no se"

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
                res_forma = "TRIANGULO"

            elif len(approx) == 4:
                cv2.putText(cv_shape, 'CUADRADO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                res_forma = "CUADRADO"

            elif len(approx) == 5:
                cv2.putText(cv_shape, 'PENTAGONO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                res_forma = "PENTAGONO"

            elif len(approx) == 6:
                cv2.putText(cv_shape, 'HEXAGONO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                res_forma = "HEXAGONO"

            else:
                cv2.putText(cv_shape, 'CIRCULO', (x, y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                res_forma = "CIRCULO"

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

            res_palabra = res[1]


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

        #print(res_forma + ", " + res_palabra + ", " + color)

        return  res_forma, res_palabra, color

def main(args=None):
    rclpy.init(args=args)
    persepcion = ServicioVision()
    rclpy.spin(persepcion)
    persepcion.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()