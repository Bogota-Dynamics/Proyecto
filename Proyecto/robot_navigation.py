import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_msgs.srv import StartNavigationTest
import time
import board
import busio
import adafruit_vl53l0x

import heapq
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

from functools import wraps

class M: None

""" CONSTANTES DE PRUEBA """
NUM_QUIETOS = 10 # Número de veces que se repite QUIETO entre instrucciones
FACTOR_TRIGGERR = 4.6 # Factor por el que se divide el número de veces que se repite TriggerR entre instrucciones
TIME_GIRO_90G = 0.75 # Número de veces que se repite una instruccion de giro para girar 90 grados
SECURE_RANGE = 170 # Rango de seguridad en mm (distancia a la que se detiene el robot)
TIME_TO_CORRECT = 10 # Tiempo que se espera un objeto dinamico para corregir la trayectoria (asumirlo estatico)
TIME_REVERSE = 0.5 # Tiempo durante el que retrocede para corregir la trayectoria (asumirlo estatico)

class robot_navigation(Node):
    def __init__(self):
        super().__init__('robot_navigation')
        self.service = self.create_service(StartNavigationTest, '/group_7/start_navigation_test_srv', self.navigation_test_callback)
        self.publisher_ = self.create_publisher(Twist, 'robot_cmdVel', 10)
        # Initialize I2C bus and sensor.
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.vl53 = adafruit_vl53l0x.VL53L0X(i2c)
            print("Sensor VL53L0X inicializado correctamente")
        except:
            self.vl53 = M()
            setattr(self.vl53, 'range', 1000)
            print("Error al inicializar el sensor VL53L0X")

    def navigation_test_callback(self, request, response):
        start_x = 1650 # request.start_x
        start_y = 945 # request.start_y
        goal_x = int(request.x)
        goal_y = int(request.y)

        self.get_logger().info('Calculando ruta desde: ' + str(start_x) + ',' + str(start_y) + ' hasta: ' + str(goal_x) + ',' + str(goal_y))
        get_path((start_y, start_x), (goal_y, goal_x), True, True)
        self.get_logger().info('¡Ruta Calculada! Robot en movimiento')

        self.recrear_recorrido('instructions')

        response.answer = "Finalizado"
        return response

    def recrear_recorrido(self, file):
        # leer el archivo y publicar los movimientos
            # leer el archivo y publicar los movimientos
        filename = 'src/Proyecto/navigation/' + file + '.txt'
        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        self.publisher_.publish(msg)

        msg_viejo = 0

        with open(filename, 'r') as f:
            linear = 5.0
            angular = 5.0
            data = f.readlines()
            i = 0
            # 2. Las siguientes lineas son los movimientos
            #    TriggerR: adelante
            #    TriggerL: atras
            #    Izquierda: izquierda
            #    Derecha: derecha
            #    QUIETO: pausa
            last_movement_time = time.time()
            while i < len(data):
                line = data[i]
                line = line.strip()
                msg = Twist()
                
                range = self.vl53.range # Rango en mm medido por el sensor
                if range > SECURE_RANGE:
                    last_movement_time = time.time()

                    if line == 'TriggerR':
                        msg.linear.x = linear
                    elif line == 'TriggerL':
                        msg.linear.x = -linear
                    elif line == 'Izquierda':
                        msg.angular.z = angular
                        time.sleep(0.05)
                        self.publisher_.publish(msg)
                        time.sleep(TIME_GIRO_90G)
                        msg.angular.z = 0.0
                        self.publisher_.publish(msg)
                        msg_viejo = msg
                        i += 1
                        continue
                    elif line == 'Derecha':
                        msg.angular.z = -angular
                        time.sleep(0.05)
                        self.publisher_.publish(msg)
                        time.sleep(TIME_GIRO_90G)
                        msg.angular.z = 0.0
                        self.publisher_.publish(msg)
                        msg_viejo = msg
                        i += 1
                        continue
                    elif line == 'QUIETO':
                        msg.linear.x=0.0
                        msg.angular.z=0.0
                else:
                    time_since_last_movement = time.time() - last_movement_time
                    if time_since_last_movement > TIME_TO_CORRECT:
                        msg.linear.x = -linear
                        time.sleep(0.05)
                        self.publisher_.publish(msg)
                        time.sleep(TIME_REVERSE)
                        msg.linear.x = 0.0
                        self.publisher_.publish(msg)
                    else:
                        msg.linear.x=0.0
                        msg.angular.z=0.0
                        time.sleep(0.05)
                        if (msg_viejo != msg):
                            self.publisher_.publish(msg)
                            msg_viejo = msg
                        continue # No sigue enviando instrucciones hasta que el rango sea mayor a SECURE_RANGE

                # Los mensajes se publican cada 0.05 segundos aproximadamente
                time.sleep(0.05)
                if (msg_viejo != msg):
                    self.publisher_.publish(msg)
                    msg_viejo = msg
                
                i += 1

        msg = Twist()
        msg.linear.x=0.0
        msg.angular.z=0.0
        self.publisher_.publish(msg)
        # retornar el path global del archivo

def main(args=None):
    rclpy.init(args=args)
    navigation = robot_navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


""" PATH PLANNING """

def timer(func):
    @wraps(func)
    def wrapper(*args, **kwargs):
        print(f"Executing function '{func.__name__}'...")
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"Function '{func.__name__}' executed in {execution_time:.6f} seconds.")
        return result
    return wrapper

@timer
def widen_grid(grid, N, wide_factor = 1):
    expanded_grid = np.copy(grid)

    for i in range(grid.shape[0]):
        for j in range(grid.shape[1]):
            if grid[i, j] < 255:
                for dx in range(-N, N + 1):
                    for dy in range(-N, N + 1):
                        nx, ny = i + dx, j + dy
                        if (
                            0 <= nx < grid.shape[0]
                            and 0 <= ny < grid.shape[1]
                            and expanded_grid[nx, ny] == 255
                        ):
                            expanded_grid[nx, ny] = grid[i, j] + wide_factor

    return expanded_grid

@timer
def astar(grid, start, goal, heuristic):
    # Initialize open and closed lists
    open_list = []
    closed_set = set()

    # Create dictionaries to store cost and parent information
    g_cost = {start: 0}
    f_cost = {start: heuristic(start, goal)}
    parent = {}

    heapq.heappush(open_list, (f_cost[start], start))

    while open_list:
        # Get the node with the lowest f_cost
        current = heapq.heappop(open_list)[1]

        if current == goal:
            # Reached the goal, reconstruct the path
            path = []
            while current in parent:
                path.append(current)
                current = parent[current]
            path.append(start)
            path.reverse()
            return path

        closed_set.add(current)

        # Explore neighbors
        for neighbor in get_neighbors(grid, current):
            if neighbor in closed_set:
                continue

            g = g_cost[current] + 1

            if neighbor not in g_cost or g < g_cost[neighbor]:
                parent[neighbor] = current
                g_cost[neighbor] = g
                f_cost[neighbor] = g + heuristic(neighbor, goal)
                heapq.heappush(open_list, (f_cost[neighbor], neighbor))

    return None

# Define the heuristic (Manhattan distance)
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

# Get valid neighbors for a given position
def get_neighbors(grid, position):
    x, y = position
    neighbors = []
    for dx, dy in [(-1, 0), (0, -1), (1, 0), (0, 1)]:
        nx, ny = x + dx, y + dy
        if (
            0 <= nx < grid.shape[0]
            and 0 <= ny < grid.shape[1]
            and grid[nx, ny] == 255
        ):
            neighbors.append((nx, ny))
    return neighbors

# Define the path-planning algorithm
def path_planning(grid, start, goal, wide_grid=None, go_just_closest=False):
    # Perform path planning with widened grid
    if wide_grid is None:
        widened_grid = grid
    else:
        widened_grid = wide_grid
    closest_goal = find_closest_goal(widened_grid, goal)
    path_to_closest_goal = astar(widened_grid, start, closest_goal, heuristic)

    plt.plot(start[1], start[0], "bo", markersize=8, label="Start")
    plt.plot(goal[1], goal[0], "go", markersize=8, label="Goal")
    plt.plot(closest_goal[1], closest_goal[0], "yo", markersize=8, label="C. Goal")

    if path_to_closest_goal is None:
        return None

    if go_just_closest:
        return path_to_closest_goal
    else:
        # Calculate the point closest to the goal in the widened grid
        closest_point = path_to_closest_goal[-1]

        # Perform path planning from the closest point to the actual goal
        path_to_goal = astar(grid, closest_point, goal, heuristic)

        if path_to_goal is None:
            return None

        # Combine the two paths
        combined_path = path_to_closest_goal[:-1] + path_to_goal

        return combined_path

def find_closest_goal(grid, goal):
    # Get all neighbors for a given position, including obstacles
    def get_all_neighbors(grid, position):
        x, y = position
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx, ny = x + dx, y + dy
                if (
                    nx >= 0
                    and ny >= 0
                    and nx < grid.shape[0]
                    and ny < grid.shape[1]
                ):
                    neighbors.append((nx, ny))
        return neighbors

    # Check if the goal itself is a non-obstacle point
    if grid[goal] == 255:
        return goal

    # Initialize variables
    queue = [goal]
    visited = set([goal])
    obstacle_count = 0

    while queue:
        current = queue.pop(0)

        # Check if the current point is a non-obstacle point
        if grid[current] == 255:
            return current

        # Count the number of obstacle neighbors
        neighbors = get_all_neighbors(grid, current)
        for neighbor in neighbors:
            if neighbor not in visited:
                queue.append(neighbor)
                visited.add(neighbor)
                if grid[neighbor] == 0:
                    obstacle_count += 1

    # Check if all points in the grid are obstacles
    if obstacle_count == len(grid):
        return None

    return None

# Load the PGM image and convert it to a grid
def load_pgm_image(filename, filename_wide = None):
    image = Image.open(filename).convert("L")
    grid = np.array(image)
    if filename_wide:
        image_wide = Image.open(filename_wide).convert("L")
        wide_grid = np.array(image_wide)
    else:
        px_ensanchamiento = 75
        wide_grid = widen_grid(grid, px_ensanchamiento, 10)
        wide_grid_image = Image.fromarray(wide_grid)
        wide_grid_image.save(f"wide_grid_{px_ensanchamiento}px.pgm")
    return grid, wide_grid


def get_path(start_positionyx, goal_positionyx, showplot=False, go_just_closest=True):
    # Define the start and goal positions
    #start_position = (455, 900) # (y, x)
    #goal_position = (500, 500) # (y, x)
    #goal_position = (615, 725) # (y, x)
    #goal_position = (521, 658) # (y, x)
    #goal_position = (744, 494) # (y, x)
    #goal_position = (638, 297) # (y, x)
    #goal_position = (786, 1005) # (y, x), OUT OF RANGE
    #goal_position = (757, 101) # (y, x)
    #goal_position = (661, 321) # (y, x)
    #goal_position = (654, 313) # (y, x)
    #goal_position = (631, 477) # (y, x)

    # Load the PGM image
    grid, wide_grid = load_pgm_image("src/Proyecto/navigation/MapaRobotica.pgm", "src/Proyecto/navigation/wide_grid_75px.pgm")

    # Plot the grid and path
    plt.figure(figsize=(8, 8))

    # Plot the grid (flipped vertically)
    plt.imshow(grid, cmap="gray", origin="upper")

    # Call the path planning function
    path = path_planning(grid, start_positionyx, goal_positionyx, wide_grid, go_just_closest)
    # NOTA: go_just_closest=True para ir solo al punto más cercano al objetivo y no al objetivo directamente, por si el robot es muy grande y no cabe por el camino al objetivo

    plt.legend()
    plt.title("Path Planning")
    plt.axis("off")

    # Plot the path
    if path:
        path = np.array(path)
        plt.plot(path[:, 1], path[:, 0], color="red", linewidth=2)

        # Guardar la ruta en un archivo 'path.txt'
        #path_save = path[::-1] # Invertir el orden de los puntos para que coincida con el orden de las coordenadas del mapa
        np.savetxt('src/Proyecto/navigation/path.txt', path, fmt='%d', delimiter=',', header='y,x', comments='')
        print(f"Path saved to 'path.txt'")

        # Guardar el plot en un archivo 'path.png'
        plt.savefig('src/Proyecto/navigation/path.png')
        print(f"Plot saved to 'path.png'")

        to_instructions("src/Proyecto/navigation/path.txt", "src/Proyecto/navigation/instructions.txt")
    
    if showplot:
        print(f"Showing plot...")
        plt.show()

""" GUARDAR RUTA A INSTRUCCIONES EN UN TXT """

def get_coordinates(coords_path):
    with open(coords_path, "r") as f:
        f.readline() # Skip the first line
        data = []
        for line in f:
            data.append(line.strip().split(","))

    return np.array(data, dtype=np.uint8)

def get_instructions(data):
    """
    TriggerR: Ir Adelante
    TriggerL: Ir Atras
    Derecha: Girar Derecha
    Izquierda: Girar Izquierda
    """
    instructions = ["TriggerR"]

    y1, x1 = data[0]
    y2, x2 = data[1]

    if x2 == x1 and y2 > y1:
        state = "y"
    elif x2 == x1 and y2 < y1:
        state = "-y"
    elif x2 > x1 and y2 == y1:
        state = "x"
    elif x2 < x1 and y2 == y1:
        state = "-x"

    for i in range(2, data.shape[0] - 1):
        y1, x1 = data[i]
        y2, x2 = data[i + 1]

        if state == "y":
            if x2 < x1:
                v = "Derecha"
                state = "-x"
            elif x2 > x1:
                v = "Izquierda"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "-y":
            if x2 < x1:
                v = "Izquierda"
                state = "-x"
            elif x2 > x1:
                v = "Derecha"
                state = "x"
            else:
                v = "TriggerR"
        elif state == "x":
            if y2 < y1:
                v = "Izquierda"
                state = "-y"
            elif y2 > y1:
                v = "Derecha"
                state = "y"
            else:
                v = "TriggerR"
        elif state == "-x":
            if y2 < y1:
                v = "Derecha"
                state = "-y"
            elif y2 > y1:
                v = "Izquierda"
                state = "y"
            else:
                v = "TriggerR"

        # Si el último movimiento guardado no es TriggerR y el movimiento a guardar tampoco, entonces se elimina el último movimiento guardado y se guarda el nuevo movimiento
        if instructions[-1] != "TriggerR" and v != "TriggerR":
            instructions.pop()
            instructions.append(v)
        else:
            instructions.append(v)

    return instructions

def save_instructions(instructions, instructions_path):
    with open(instructions_path, "w") as f:
        count = 0
        for instruction in instructions:
            if instruction == "TriggerR":
                count += 1
            else:
                f.write(("QUIETO\n")*NUM_QUIETOS)
                f.write(("TriggerR\n")*(int(count//(FACTOR_TRIGGERR))))
                f.write(("QUIETO\n")*NUM_QUIETOS)
                count = 0
                if instruction == "Derecha":
                    instruction = "Izquierda"
                elif instruction == "Izquierda":
                    instruction = "Derecha"
                f.write((instruction + "\n")*1)
        
        f.write(("QUIETO\n")*NUM_QUIETOS)
        f.write(("TriggerR\n")*(int(count//(FACTOR_TRIGGERR))))
        f.write(("QUIETO\n")*NUM_QUIETOS)

def to_instructions(coords_path, instructions_path):
    print(coords_path, instructions_path)
    data = get_coordinates(coords_path)
    instructions = get_instructions(data)
    save_instructions(instructions, instructions_path)
