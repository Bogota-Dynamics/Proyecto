import rclpy
import time
from rclpy.node import Node
from my_msgs.srv import StartManipulationTest
from geometry_msgs.msg import Twist

class ManipuladorNode(Node):

    def __init__(self):
        super().__init__('robot_manipulador')
        self.service = self.create_service(
            StartManipulationTest,
            '/group_7/start_manipulation_test_srv',
            self.manipulation_test_callback
        )
        self.publisher_ = self.create_publisher(Twist, 'robot_cmdVel', 10)
        self.get_logger().info('Manipulation Test service is ready')

    def manipulation_test_callback(self, request, response):
        # Obtener los valores de la solicitud
        platform = request.platform
        x = request.x

        if platform == 'platform_1':
            print('Moviendo el robot a la plataforma 1')
            self.recrear_recorrido('manipulation_01')
            time.sleep(5)
            print('Tomando la ficha')
            self.enviar_manipulador(1.0)
            time.sleep(5)
            print('Moviendo el robot a la plataforma 2')
            self.recrear_recorrido('manipulation_12')
            time.sleep(5)
            print('Dejando la ficha')
            self.enviar_manipulador(2.0)

        elif platform == 'platform_2':
            print('Moviendo el robot a la plataforma 2')
            self.recrear_recorrido('manipulation_02')
            time.sleep(5)
            print('Tomando la ficha')
            self.enviar_manipulador(2.0)
            time.sleep(5)
            print('Moviendo el robot a la plataforma 1')
            self.recrear_recorrido('manipulation_21')
            time.sleep(5)
            print('Dejando la ficha')
            self.enviar_manipulador(1.0)

        # Realizar las operaciones necesarias para determinar la plataforma de origen y destino
        # y construir el mensaje de respuesta

        origin_platform = 'platform_1' if platform == 'platform_1' else 'platform_2'
        destination_platform = 'platform_2' if platform == 'platform_1' else 'platform_1'
        response.answer = f"La ficha de tipo {x} se encuentra en la plataforma {origin_platform} y la llevaré a la plataforma {destination_platform}"
        print(response.answer)
        
        return response
    
    def enviar_manipulador(self, v):
        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        msg.linear.y = v
        self.publisher_.publish(msg)

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
                line = line.strip()
                print(line)
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
