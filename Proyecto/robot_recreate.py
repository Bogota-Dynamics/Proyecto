import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class robot_recreate(Node):

    def __init__(self):
        super().__init__('robot_recreate')
        self.publisher_ = self.create_publisher(Twist, 'robot_cmdVel', 10)
        self.recrear_recorrido('instructions')

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
    recreate = robot_recreate()
    rclpy.spin(recreate)
    recreate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

