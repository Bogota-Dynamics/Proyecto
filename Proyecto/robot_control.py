import rclpy
import serial
import serial.tools.list_ports
from rclpy.node import Node
from geometry_msgs.msg import Twist

class robot_bot_control(Node):

    def __init__(self):
        super().__init__('robot_bot_control')
        self.subscription = self.create_subscription(
            Twist,
            'robot_cmdVel',
            self.listener_callback,
            10)
    

        #Encontrar puerto Automaticamente
        ports = list(serial.tools.list_ports.comports())
        arduino_port = ports[0].device

        self.arduino = serial.Serial(port=arduino_port, baudrate=250000,timeout=.1)
    

    def listener_callback(self, msg):

        x = msg.linear.x
        z = msg.angular.z
        manipulator = msg.linear.y

        """if msg.linear.y != 0:
            mensaje = f'{x},{z},{manipulator}'
        else:
            mensaje = f'{x},{z}'"""

        mensaje = f'{x},{z},{manipulator}'
        self.write_read(mensaje)
        
    def write_read(self, x):
        print(f'writing {x}')
        self.arduino.write(bytes(x, 'utf-8'))


def main(args=None):
    rclpy.init(args=args)
    interface = robot_bot_control()
    rclpy.spin(interface)
    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()