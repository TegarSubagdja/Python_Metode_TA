import numpy as np
import heapq
import math

# Representasi peta: 2 = Start, 3 = Goal, 1 = Obstacle, 0 = Free space
map_grid = np.array([
    [2, 0, 0, 0, 0],
    [0, 0, 1, 0, 0],
    [0, 1, 1, 1, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 3]
])

# Fungsi untuk menghitung jarak Euclidean
def euclidean_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

# Cari koordinat dari nilai tertentu dalam grid
def find_coordinates(grid, value):
    result = np.argwhere(grid == value)
    return tuple(result[0]) if result.size > 0 else None

# Bidirectional A* Algorithm
def bidirectional_a_star(grid):
    # Temukan titik start dan goal
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    # Inisialisasi struktur data
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
        
        # Proses dari arah goal
        current_goal = heapq.heappop(open_list_goal)[1]
        closed_list_goal.add(current_goal)
        
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
    
    return None  # Tidak ada jalur ditemukan

# Fungsi untuk mengganti path dengan nilai 5
def mark_path_on_map(grid, path):
    output_grid = grid.copy()
    for step in path:
        if output_grid[step] not in (2, 3):  # Jangan ubah start (2) dan goal (3)
            output_grid[step] = 5
    return output_grid

# Jalankan algoritma A*
path = bidirectional_a_star(map_grid)

# Tampilkan hasil
if path:
    print("Path found:")
    print([tuple(map(int, step)) for step in path])  # Format koordinat sebagai angka biasa
    # Tandai path pada peta
    result_grid = mark_path_on_map(map_grid, path)
    print("\nMap with path marked as 5:")
    print(result_grid)
else:
    print("No path found.")
