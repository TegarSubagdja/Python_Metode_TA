import pygame
import numpy as np
import heapq
import math
import time

# Pygame Initialization
pygame.init()

# Grid settings
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 5, 5
TILE_WIDTH = WIDTH // COLS
TILE_HEIGHT = HEIGHT // ROWS

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREY = (200, 200, 200)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)

# Representasi peta: 2 = Start, 3 = Goal, 1 = Obstacle, 0 = Free space
map_grid = np.array([
    [2, 0, 0, 0, 0],
    [0, 1, 1, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 1, 1, 3, 0],
    [0, 0, 0, 0, 0]
])

# Fungsi untuk menghitung jarak Euclidean
def euclidean_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

# Fungsi untuk menghitung jarak ke guideline
def guideline_cost(node, start, goal):
    x, y = node
    x_start, y_start = start
    x_goal, y_goal = goal
    numerator = abs((y_goal - y_start) * x - (x_goal - x_start) * y + (x_goal * y_start - y_goal * x_start))
    denominator = math.sqrt((x_goal - x_start)**2 + (y_goal - y_start)**2)
    return numerator / denominator

# Cari koordinat dari nilai tertentu dalam grid
def find_coordinates(grid, value):
    result = np.argwhere(grid == value)
    return tuple(result[0]) if result.size > 0 else None

# A* Algorithm dengan guideline cost
def a_star_search(grid, draw_func, delay=0.5):
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: euclidean_distance(start, goal) + guideline_cost(start, start, goal)}
    closed_list = set()
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while open_list:
        current = heapq.heappop(open_list)[1]
        closed_list.add(current)
        
        draw_func(current, 'close')  # Animasi untuk close list
        time.sleep(delay)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for offset in neighbors:
            neighbor = (current[0] + offset[0], current[1] + offset[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list:
                    continue
                
                g_new = g_score[current] + (euclidean_distance(current, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                h_new = euclidean_distance(neighbor, goal)
                c_new = guideline_cost(neighbor, start, goal)
                f_new = g_new + h_new + c_new

                if neighbor not in g_score or g_new < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = g_new
                    f_score[neighbor] = f_new
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_new, neighbor))
                        draw_func(neighbor, 'open')  # Animasi untuk open list
                        time.sleep(delay)
    return None

# Fungsi untuk menggambar grid
def draw_grid(screen, grid, open_nodes, close_nodes, path_nodes):
    for row in range(ROWS):
        for col in range(COLS):
            x, y = col * TILE_WIDTH, row * TILE_HEIGHT
            value = grid[row, col]
            if value == 1:  # Obstacle
                pygame.draw.rect(screen, BLACK, (x, y, TILE_WIDTH, TILE_HEIGHT))
            elif value == 2:  # Start
                pygame.draw.rect(screen, GREEN, (x, y, TILE_WIDTH, TILE_HEIGHT))
            elif value == 3:  # Goal
                pygame.draw.rect(screen, RED, (x, y, TILE_WIDTH, TILE_HEIGHT))
            else:  # Free space
                pygame.draw.rect(screen, WHITE, (x, y, TILE_WIDTH, TILE_HEIGHT))
            pygame.draw.rect(screen, GREY, (x, y, TILE_WIDTH, TILE_HEIGHT), 1)
    for node in open_nodes:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, BLUE, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in close_nodes:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, YELLOW, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in path_nodes:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, CYAN, (x, y, TILE_WIDTH, TILE_HEIGHT))

# Fungsi utama animasi
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("A* Pathfinding Visualization")
    clock = pygame.time.Clock()
    running = True
    open_nodes, close_nodes, path_nodes = [], [], []

    def draw_func(node, state):
        if state == 'open':
            open_nodes.append(node)
        elif state == 'close':
            close_nodes.append(node)
        elif state == 'path':
            path_nodes.append(node)
        screen.fill(WHITE)
        draw_grid(screen, map_grid, open_nodes, close_nodes, path_nodes)
        pygame.display.flip()

    path = a_star_search(map_grid, draw_func, delay=0.5)
    if path:
        for node in path:
            draw_func(node, 'path')
            time.sleep(0.2)

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        clock.tick(60)
    pygame.quit()

if __name__ == "__main__":
    main()
