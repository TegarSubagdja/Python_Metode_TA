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

# A* Algorithm dengan guideline
def a_star_with_guideline(grid):
    # Temukan titik start dan goal
    start = find_coordinates(grid, 2)
    goal = find_coordinates(grid, 3)
    
    if not start or not goal:
        raise ValueError("Start or Goal node not found in the grid.")
    
    # Inisialisasi struktur data
    rows, cols = grid.shape
    open_list = []
    heapq.heappush(open_list, (0, start))  # Priority queue (f_score, node)
    came_from = {}  # Untuk melacak jalur

    # G-score (biaya dari start ke node saat ini)
    g_score = {start: 0}
    # F-score (g_score + heuristic + guideline_cost)
    f_score = {start: euclidean_distance(start, goal) + guideline_cost(start, start, goal)}
    
    # Closed list untuk melacak node yang sudah diproses
    closed_list = set()

    # Set neighbor offsets (horizontal, vertical, diagonal)
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    step = 1
    while open_list:
        # Ambil node dengan f_score terendah
        current = heapq.heappop(open_list)[1]
        closed_list.add(current)  # Tambahkan ke closed list
        
        # Jika goal tercapai, rekonstruksi jalur
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]  # Balikkan jalur
        
        print(f"Step {step}: Processing node {current}")
        step += 1
        
        # Proses semua neighbor
        for offset in neighbors:
            neighbor = (current[0] + offset[0], current[1] + offset[1])
            
            # Pastikan neighbor valid (dalam grid dan bukan halangan)
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 1:
                if neighbor in closed_list:
                    continue

                # Hitung G, H, dan C
                g_new = g_score[current] + (euclidean_distance(current, neighbor) if offset[0] != 0 and offset[1] != 0 else 1)
                h_new = euclidean_distance(neighbor, goal)
                c_new = guideline_cost(neighbor, start, goal)

                f_new = g_new + h_new + c_new  # Total evaluasi F

                if neighbor not in g_score or g_new < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = g_new
                    f_score[neighbor] = f_new
                    if not any(neighbor == item[1] for item in open_list):
                        heapq.heappush(open_list, (f_new, neighbor))
                        print(f"Added to open list: {neighbor}")
                
                # Tampilkan perhitungan g, h, f
                print(f"Neighbor: {neighbor}")
                print(f"  g = {g_new:.2f}")
                print(f"  h = {h_new:.2f}")
                print(f"  c = {c_new:.2f}")
                print(f"  f = {f_new:.2f}")
        
        print(f"Closed list: {list(closed_list)}")
        print('=========================================')
    
    return None  # Tidak ada jalur ditemukan

# Fungsi untuk mengganti path dengan nilai 5
def mark_path_on_map(grid, path):
    output_grid = grid.copy()
    for step in path:
        if output_grid[step] not in (2, 3):  # Jangan ubah start (2) dan goal (3)
            output_grid[step] = 5
    return output_grid

# Jalankan algoritma A*
path = a_star_with_guideline(map_grid)

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
