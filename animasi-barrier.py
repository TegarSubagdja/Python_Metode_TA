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

# Cari koordinat dari nilai tertentu dalam grid
def find_coordinates(grid, value):
    result = np.argwhere(grid == value)
    return tuple(result[0]) if result.size > 0 else None

# Fungsi untuk menghitung Barrier Raster Coefficient (P)
def compute_barrier_coefficient(current, goal, grid):
    x1, y1 = current
    x2, y2 = goal
    x_min, x_max = min(x1, x2), max(x1, x2)
    y_min, y_max = min(y1, y2), max(y1, y2)
    
    obstacle_count = np.sum(grid[x_min:x_max + 1, y_min:y_max + 1] == 1)
    total_area = (x_max - x_min + 1) * (y_max - y_min + 1)
    
    P = obstacle_count / total_area if total_area > 0 else 0.01  # Avoid division by zero
    return max(P, 0.01)  # Ensure P is non-zero to avoid log issues

# A* Algorithm with Barrier Raster Coefficient and Turn Penalty
def a_star_search(grid, draw_func, turn_penalty_coefficient=1.0, delay=0.5):
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))  # Priority queue (f_score, node)
    came_from = {}  # Untuk melacak jalur

    # G-score (biaya dari start ke node saat ini)
    g_score = {start: 0}
    # F-score (g_score + heuristic)
    f_score = {start: euclidean_distance(start, goal)}
    
    closed_list = set()  # Closed list to track processed nodes
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    while open_list:
        current = heapq.heappop(open_list)[1]
        closed_list.add(current)  # Tambahkan ke closed list
        
        # Animasi untuk closed list
        draw_func(current, 'close')
        time.sleep(delay)
        
        # Jika goal tercapai, rekonstruksi jalur
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Balikkan jalur
        
        for offset in neighbors:
            neighbor = (current[0] + offset[0], current[1] + offset[1])
            
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list:
                    continue  # Skip jika sudah di closed list
                
                tentative_g_score = g_score[current] + (euclidean_distance(current, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                
                # Hitung Barrier Raster Coefficient (P)
                P = compute_barrier_coefficient(current, goal, grid)
                h = (1 - math.log(P)) * euclidean_distance(neighbor, goal)
                
                # Hitung Turn Penalty
                if current in came_from:
                    prev = came_from[current]
                    dx1, dy1 = goal[0] - current[0], goal[1] - current[1]
                    dx2, dy2 = neighbor[0] - current[0], neighbor[1] - current[1]
                    turn_penalty = abs(dx1 * dy2 - dx2 * dy1) * turn_penalty_coefficient
                else:
                    turn_penalty = 0
                
                # Total cost
                f = tentative_g_score + h + turn_penalty
                
                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = f

                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_score[neighbor], neighbor))
                        draw_func(neighbor, 'open')  # Animasi untuk open list
                        time.sleep(delay)
    
    return None  # Tidak ada jalur ditemukan

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
            
            pygame.draw.rect(screen, GREY, (x, y, TILE_WIDTH, TILE_HEIGHT), 1)  # Grid lines
    
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
    pygame.display.set_caption("A* Pathfinding Visualization with Barrier Raster")
    clock = pygame.time.Clock()
    
    running = True
    open_nodes = []
    close_nodes = []
    path_nodes = []

    def draw_func(node, state):
        if state == 'open':
            open_nodes.append(node)
        elif state == 'close':
            close_nodes.append(node)
        screen.fill(WHITE)
        draw_grid(screen, map_grid, open_nodes, close_nodes, path_nodes)
        pygame.display.flip()
    
    # Jalankan A* dan dapatkan path
    path = a_star_search(map_grid, draw_func, turn_penalty_coefficient=1.0, delay=0.5)
    
    if path:
        path_nodes = path
        for node in path_nodes:
            draw_func(node, 'path')
            time.sleep(0.2)
    
    # Tunggu hingga window ditutup
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        clock.tick(60)
    
    pygame.quit()

# Jalankan program
if __name__ == "__main__":
    main()
