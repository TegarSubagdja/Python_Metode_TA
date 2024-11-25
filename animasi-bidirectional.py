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
MAGENTA = (255, 0, 255)

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

# Bidirectional A* Algorithm
def bidirectional_a_star(grid, draw_func, delay=1):
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    rows, cols = grid.shape
    
    # Open lists and closed lists for both directions
    open_list_start = []
    open_list_goal = []
    heapq.heappush(open_list_start, (0, start))
    heapq.heappush(open_list_goal, (0, goal))
    closed_list_start = set()
    closed_list_goal = set()

    # G-score and F-score for both directions
    g_score_start = {start: 0}
    g_score_goal = {goal: 0}
    f_score_start = {start: euclidean_distance(start, goal)}
    f_score_goal = {goal: euclidean_distance(start, goal)}

    # Came-from for path reconstruction
    came_from_start = {}
    came_from_goal = {}

    # Set neighbor offsets (horizontal, vertical, diagonal)
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    def reconstruct_path(meeting_point):
        # Rekonstruksi jalur dari start ke goal
        path_start = []
        current = meeting_point
        while current in came_from_start:
            path_start.append(current)
            current = came_from_start[current]
        path_start = path_start[::-1]

        path_goal = []
        current = meeting_point
        while current in came_from_goal:
            path_goal.append(current)
            current = came_from_goal[current]
        
        return path_start + path_goal[1:]  # Hindari duplikasi titik meeting point
    
    while open_list_start and open_list_goal:
        # Proses dari arah start
        current_start = heapq.heappop(open_list_start)[1]
        closed_list_start.add(current_start)
        draw_func(current_start, 'close_start')  # Animasi untuk closed list (start)
        time.sleep(delay)
        
        # Proses dari arah goal
        current_goal = heapq.heappop(open_list_goal)[1]
        closed_list_goal.add(current_goal)
        draw_func(current_goal, 'close_goal')  # Animasi untuk closed list (goal)
        time.sleep(delay)
        
        # Jika kedua sisi bertemu
        if current_start in closed_list_goal or current_goal in closed_list_start:
            meeting_point = current_start if current_start in closed_list_goal else current_goal
            return reconstruct_path(meeting_point)

        # Proses neighbors untuk arah start
        for offset in neighbors:
            neighbor = (current_start[0] + offset[0], current_start[1] + offset[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list_start:
                    continue
                
                tentative_g_score = g_score_start[current_start] + (euclidean_distance(current_start, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                if neighbor not in g_score_start or tentative_g_score < g_score_start[neighbor]:
                    came_from_start[neighbor] = current_start
                    g_score_start[neighbor] = tentative_g_score
                    f_score_start[neighbor] = tentative_g_score + euclidean_distance(neighbor, goal)
                    
                    if not any(neighbor == item[1] for item in open_list_start):
                        heapq.heappush(open_list_start, (f_score_start[neighbor], neighbor))
                        draw_func(neighbor, 'open_start')  # Animasi untuk open list (start)
                        time.sleep(delay)
        
        # Proses neighbors untuk arah goal
        for offset in neighbors:
            neighbor = (current_goal[0] + offset[0], current_goal[1] + offset[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list_goal:
                    continue
                
                tentative_g_score = g_score_goal[current_goal] + (euclidean_distance(current_goal, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                if neighbor not in g_score_goal or tentative_g_score < g_score_goal[neighbor]:
                    came_from_goal[neighbor] = current_goal
                    g_score_goal[neighbor] = tentative_g_score
                    f_score_goal[neighbor] = tentative_g_score + euclidean_distance(neighbor, start)
                    
                    if not any(neighbor == item[1] for item in open_list_goal):
                        heapq.heappush(open_list_goal, (f_score_goal[neighbor], neighbor))
                        draw_func(neighbor, 'open_goal')  # Animasi untuk open list (goal)
                        time.sleep(delay)
    
    return None  # Tidak ada jalur ditemukan

# Fungsi untuk menggambar grid
def draw_grid(screen, grid, open_nodes_start, open_nodes_goal, close_nodes_start, close_nodes_goal, path_nodes):
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
            
            # Garis grid
            pygame.draw.rect(screen, GREY, (x, y, TILE_WIDTH, TILE_HEIGHT), 1)
    
    # Tampilkan node open, close, dan path
    for node in open_nodes_start:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, BLUE, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in open_nodes_goal:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, MAGENTA, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in close_nodes_start:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, YELLOW, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in close_nodes_goal:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, CYAN, (x, y, TILE_WIDTH, TILE_HEIGHT))
    for node in path_nodes:
        x, y = node[1] * TILE_WIDTH, node[0] * TILE_HEIGHT
        pygame.draw.rect(screen, RED, (x, y, TILE_WIDTH, TILE_HEIGHT))

# Fungsi utama animasi
def main():
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Bidirectional A* Pathfinding Visualization")
    clock = pygame.time.Clock()
    
    running = True
    open_nodes_start = []
    open_nodes_goal = []
    close_nodes_start = []
    close_nodes_goal = []
    path_nodes = []

    def draw_func(node, state):
        if state == 'open_start':
            open_nodes_start.append(node)
        elif state == 'open_goal':
            open_nodes_goal.append(node)
        elif state == 'close_start':
            close_nodes_start.append(node)
        elif state == 'close_goal':
            close_nodes_goal.append(node)
        screen.fill(WHITE)
        draw_grid(screen, map_grid, open_nodes_start, open_nodes_goal, close_nodes_start, close_nodes_goal, path_nodes)
        pygame.display.flip()
    
    # Jalankan Bidirectional A* dan dapatkan path
    path = bidirectional_a_star(map_grid, draw_func, delay=0.5)
    
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
