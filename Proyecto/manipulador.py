import rclpy
import time
from rclpy.node import Node
from my_msgs.srv import StartManipulationTest
from geometry_msgs.msg import Twist

class ManipuladorNode(Node):

    def __init__(self):
        super().__init__('manipulador')
        self.srv = self.create_service(
            StartManipulationTest,
            '/group_x/start_manipulation_test_srv',
            self.start_manipulation_test_callback
        )
        self.get_logger().info('Manipulation Test service is ready')

    def start_manipulation_test_callback(self, request, response):
        # Obtener los valores de la solicitud
        platform = request.platform
        x = request.x

        # Realizar las operaciones necesarias para determinar la plataforma de origen y destino
        # y construir el mensaje de respuesta
        origin_platform = 'platform_1' if platform == 'platform_2' else 'platform_2'
        destination_platform = 'platform_1' if platform == 'platform_1' else 'platform_2'
        response.answer = f"La ficha de tipo {x} se encuentra en la plataforma {origin_platform} y la llevaré a la plataforma {destination_platform}"
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

def main(args=None):
    rclpy.init(args=args)
    manipulador_node = ManipuladorNode()
    rclpy.spin(manipulador_node)
    manipulador_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
