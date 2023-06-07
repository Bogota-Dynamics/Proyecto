import rclpy
import pygame
from rclpy.node import Node
from geometry_msgs.msg import Twist

class robot_teleop(Node):
    def __init__(self):
        super().__init__('robot_teleop')
        self.publisher_ = self.create_publisher(Twist, 'robot_cmdVel', 10)
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.timer = self.create_timer(0.1, self.timer_callback)

        #Encontrar el control para input
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No se detectaron dispositivos de joystick.")
            pygame.quit()
            exit()
        else:
            print("Hay" + str(joystick_count))
        
        #Inicializar variables para publicar controles
        self.linear = 5.0
        self.angular = 5.0
        self.msg_viejo = 0
        self.cambio_angulo = 1.0

    def timer_callback_motor(self):
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: 
                pygame.quit()
                quit()
        
        x_axis_left = self.joystick.get_axis(0)
        triggerR = self.joystick.get_axis(4)
        triggerL = self.joystick.get_axis(5)

        minVal = 0.1 # si el valor leído es menor a 0.1, no lo lee

        mov = 'QUIETO'
        if (abs(x_axis_left) > minVal):
            mov = ("Izquierda" if x_axis_left < 0 else "Derecha")
        if (triggerL > -1):
            mov = ("TriggerL")
        if (triggerR > -1):
            mov = ("TriggerR")

        msg = Twist()
        if 'TriggerR' == mov:
            msg.linear.x = self.linear
        elif 'TriggerL' == mov:
            msg.linear.x = -self.linear
        elif 'Izquierda' == mov:
            msg.angular.z = self.angular
        elif 'Derecha' == mov:  
            msg.angular.z = -self.angular
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0 # ?

        print(msg)
        if (self.msg_viejo!=msg): 
            self.publisher_.publish(msg)

        self.msg_viejo = msg

    def timer_callback_manipulator(self):
        for event in pygame.event.get(): 
            if event.type == pygame.QUIT: 
                pygame.quit()
                quit()

        servo1 = self.joystick.get_axis(1)
        servo2 = self.joystick.get_axis(3)
        abrir = self.joystick.get_button(6)
        cerrar = self.joystick.get_button(7)

        minVal = 0.5 # si el valor leído es menor a 0.1, no lo lee

        mov = []
        if (abs(servo1) > minVal):
            mov.append(("Ser1Ari" if servo1 < 0 else "Ser1Aba"))
        if (abs(servo2) > minVal):
            mov.append(("Ser2Ari" if servo2 < 0 else "Ser2Aba"))
        if (abrir):
            mov.append("Abrir")
        if (cerrar):
            mov.append("Cerrar")

        msg = Twist()
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        if 'Ser1Ari' in mov:
            msg.linear.y = self.cambio_angulo
        if 'Ser1Aba' in mov:
            msg.linear.y = -self.cambio_angulo
        if 'Ser2Ari' in mov:
            msg.linear.z = self.cambio_angulo
        if 'Ser2Aba' in mov:
            msg.linear.z = -self.cambio_angulo
        if 'Abrir' in mov:
            msg.angular.x = self.cambio_angulo
        if 'Cerrar' in mov:
            msg.angular.x = -self.cambio_angulo


        if (self.msg_viejo!=msg): 
            print(msg)
            self.publisher_.publish(msg)

        self.msg_viejo = msg

    def timer_callback(self):
        self.timer_callback_motor()
        self.timer_callback_manipulator()


def main(args=None):
    rclpy.init(args=args)
    pygame.init()
    teleop = robot_teleop()
    rclpy.spin(teleop)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
