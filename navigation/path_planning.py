import heapq
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt

import time
from functools import wraps
from path_to_inst import to_instructions

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
        px_ensanchamiento = 35
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
    grid, wide_grid = load_pgm_image("MapaRobotica.pgm", "wide_grid_35px.pgm")

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
        np.savetxt('path.txt', path, fmt='%d', delimiter=',', header='y,x', comments='')
        print(f"Path saved to 'path.txt'")

        # Guardar el plot en un archivo 'path.png'
        plt.savefig('path.png')
        print(f"Plot saved to 'path.png'")

        to_instructions("path.txt", "instructions.txt")
    
    if showplot:
        plt.show()

if __name__ == "__main__":
    get_path((455, 900), (657, 886), showplot=True, go_just_closest=True)
